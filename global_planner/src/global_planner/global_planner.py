import json
from urllib.parse import urlparse

import ipfsapi
import rospy
from shapely.geometry import LineString, Point

from robonomics_lighthouse.msg import Ask, Bid, Result
from web3 import Web3, HTTPProvider

from dsky_main.connectors import IPFSFileRegistry
from global_planner.segmenter import Segmenter


class GlobalPlanner:

    REGIONAL_CONFIRM_SERVICE_NAME = "regional_asp_route_confirm"

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

        self.segmenter = Segmenter(self.asp_registry, self.global_registry_address, self.global_registry_address)

        rospy.Subscriber('~remote/incoming/ask', Ask, self.on_ask)
        self.service_id = None

    def on_ask(self, ask):
        rospy.logdebug("GP found an Ask")
        if ask.model == self.service_id:
            rospy.logdebug("GP found an Ask with its service id")
            bid = self.prepare_bid(ask.model, ask.cost, ask.deadline)
            self.bid_topic.publish(bid)

    def set_response(self):
        pass
        # confirmation = RouteConfirmationResponse()
        # confirmation.route = route
        # confirmation.isConfirmed = True
        # #self.confirmed_route_topic.publish(confirmation)
        # #self.liability_finish_service()
        # rospy.loginfo("ASP sent confirmation")

    def on_route_request(self, route):
        rospy.loginfo("ASP got new route request")
        route_line_string = self._route_to_line_string(route)
        self.segment_route(route_line_string)

    def _route_to_line_string(self, route):
        points = []
        for pose_wrapper in route.poses:
            pose = pose_wrapper.pose
            position = pose.position
            point = Point(position.longitude, position.latitude, position.altitude)
            points.append(point)
        return LineString(points)

    def segment_route(self, route: LineString):
        segments = self.segmenter.segments(route)
        for segment in segments:
            asp_manifest = json.loads(self.asp_manifest.get_file_content(segment.service_provider.contract_address))
            if self.REGIONAL_CONFIRM_SERVICE_NAME in asp_manifest:
                output = "{} to {}".format(segment.route, asp_manifest[self.REGIONAL_CONFIRM_SERVICE_NAME])
                rospy.loginfo(output)

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
