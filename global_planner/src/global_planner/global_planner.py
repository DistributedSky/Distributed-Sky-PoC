import json
import ipfsapi
import rospy
from collections import namedtuple
from urllib.parse import urlparse
from shapely.geometry import LineString
from web3 import Web3, HTTPProvider

from std_srvs.srv import Empty
from geographic_msgs.msg import GeoPath, GeoPoint, GeoPoseStamped

from robonomics_lighthouse.msg import Ask, Bid, Result

from distributed_sky_uav.msg import RouteConfirmationRequest, RouteConfirmationResponse

from dsky_main.connectors import IPFSFileRegistry, IPFSConnector
from global_planner.converters import GeoConverter
from global_planner.segmenter import Segmenter

RouteSegment = namedtuple('RouteSegment', ['route', 'model'])


class GlobalPlanner:
    REGIONAL_CONFIRM_SERVICE_NAME = "regional_asp_route_confirm"
    SERVICES_SECTION = "services"

    # Here just for prototyping
    REGIONAL_ASK_DEADLINE = 100
    REGIONAL_ASK_COST = 1

    def __init__(self):
        rospy.init_node('global_planner', anonymous=True)

        self.token_address = rospy.get_param('~token_address')
        self.lighthouse_fee = rospy.get_param('~lighthouse_fee')

        self.bid_topic = rospy.Publisher('~remote/sending/bid', Bid, queue_size=10)

        web3_provider = rospy.get_param('~web3_http_provider')
        self.web3 = Web3(HTTPProvider(web3_provider))

        ipfs_provider = urlparse(rospy.get_param('~ipfs_http_provider')).netloc.split(':')
        self.ipfs = ipfsapi.connect(ipfs_provider[0], int(ipfs_provider[1]))

        self.asp_registry_abi = rospy.get_param('~asp_registry_abi')
        self.global_registry_abi = rospy.get_param('~global_registry_abi')
        self.asp_manifest_abi = rospy.get_param('~asp_manifest_abi')

        self.global_registry_address = rospy.get_param('~global_registry_address')

        self.pending_requests = []

        self.asp_registry = IPFSFileRegistry(self.asp_registry_abi, self.web3, self.ipfs)
        self.asp_manifest = IPFSFileRegistry(self.asp_manifest_abi, self.web3, self.ipfs)
        self.global_registry = IPFSFileRegistry(self.global_registry_abi, self.web3, self.ipfs)
        self.ipfs_connector = IPFSConnector(self.ipfs)

        self.segmenter = Segmenter(self.asp_registry, self.global_registry, self.global_registry_address)

        rospy.Subscriber('~remote/incoming/ask', Ask, self.on_ask)
        rospy.Subscriber('~local/incoming/route_response', RouteConfirmationResponse, self.on_route_response)
        rospy.Subscriber('~rosbag/route_request', GeoPath, self.on_route_request)

        self.regional_request_topic = rospy.Publisher('~remote/sending/regional_confirm_request', RouteConfirmationRequest,
                                                      queue_size=10)
        self.confirmed_route_topic = rospy.Publisher('~local/sending/confirmation_result', RouteConfirmationResponse,
                                                     queue_size=10)
        self.liability_finish_service = rospy.ServiceProxy('~liability_finish', Empty)

        self.delay = rospy.get_param('~delay', 0)

        self.service_id = None
        self.service_name = rospy.get_param('~announced_service_name')
        self.pending_regional_route_requests = []
        self.current_route = None
        self.announce_service()

    def announce_service(self):
        manifest = {"service_id": self.service_name}
        self.service_id = self.ipfs_connector.add_to_ipfs(json.dumps(manifest))
        rospy.loginfo("Announced service id is %s", self.service_id)

    def on_ask(self, ask):
        rospy.logdebug("GP found an Ask")
        if ask.model == self.service_id:
            rospy.loginfo("GP found an Ask with its service id")
            rospy.sleep(self.delay)
            bid = self.prepare_bid(ask.model, ask.cost, ask.deadline)
            self.bid_topic.publish(bid)

    def send_response(self, result: bool):
        rospy.sleep(self.delay)
        confirmation = RouteConfirmationResponse()
        confirmation.route = self.current_route
        confirmation.isConfirmed = result
        self.confirmed_route_topic.publish(confirmation)
        self.liability_finish_service()
        self.current_route = None
        rospy.loginfo("Global planner sent confirmation with result: %s", result)

    def on_route_response(self, response: RouteConfirmationResponse):
        rospy.loginfo("Global planner got response from regional ASP")
        if response.isConfirmed:
            if not self.proceed_regional_route_requests():
                self.send_response(True)
        else:
            self.send_response(False)

    def proceed_regional_route_requests(self):
        if len(self.pending_regional_route_requests) > 0:
            segment_to_serve = self.pending_regional_route_requests.pop()
            request = RouteConfirmationRequest()
            request.route = segment_to_serve.route
            request.model = segment_to_serve.model
            request.cost = self.REGIONAL_ASK_COST
            request.deadline = self.web3.eth.getBlock('latest')['number'] + self.REGIONAL_ASK_DEADLINE
            self.regional_request_topic.publish(request)
            rospy.loginfo("Global planner sent request for %s", segment_to_serve.model)
            return True
        else:
            return False

    def on_route_request(self, route: GeoPath):
        rospy.loginfo("Global planner got new route request")
        self.current_route = route
        route_line_string = GeoConverter.geo_path_to_line_string(route)
        self.pending_regional_route_requests = []
        try:
            self.pending_regional_route_requests = self.segment_route(route_line_string)
        except Exception:
            rospy.logwarn("Exception during route handling")
        if len(self.pending_regional_route_requests) == 0 or not self.proceed_regional_route_requests():
            self.send_response(False)

    def segment_route(self, route: LineString):
        segments_with_models = []
        segments = self.segmenter.segments(route)
        for segment in segments:
            asp_manifest = json.loads(self.asp_manifest.get_file_content(segment.service_provider.contract_address))
            rospy.logdebug("Got asp_manifest: {}".format(asp_manifest))
            try:
                services_section = asp_manifest[self.SERVICES_SECTION]
                regional_confirm_service = services_section[self.REGIONAL_CONFIRM_SERVICE_NAME]
                route_with_model = RouteSegment(GeoConverter.line_string_to_geo_path(segment.route),
                                                regional_confirm_service)
                segments_with_models.append(route_with_model)
                output = "{} to {}".format(segment.route, regional_confirm_service)
                rospy.logdebug(output)
            except KeyError:
                pass
        return segments_with_models

    def prepare_bid(self, model, cost, deadline):
        msg = Bid()
        msg.model = model
        msg.token = self.token_address
        msg.cost = cost
        msg.count = 1
        msg.lighthouseFee = self.lighthouse_fee
        msg.salt = [0]
        msg.signature = [0]
        msg.deadline = deadline
        return msg

    def spin(self):
        rospy.spin()
