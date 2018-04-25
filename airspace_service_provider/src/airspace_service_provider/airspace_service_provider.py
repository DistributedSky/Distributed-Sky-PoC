from urllib.parse import urlparse

import rosbag, rospy
import ipfsapi

import geographic_msgs.msg

from std_srvs.srv import Empty
from std_msgs.msg import String
from robonomics_lighthouse.msg import Ask, Bid, Result
from distributed_sky_uav.msg import RouteConfirmationRequest, RouteConfirmationResponse


class AirspaceServiceProvider:

    def __init__(self):
        rospy.init_node('airspace_service_provider', anonymous=True)

        self._current_request = None

        self.token_address = rospy.get_param('~token_address')
        self.lighthouse_fee = rospy.get_param('~lighthouse_fee')

        ipfs_provider = urlparse(rospy.get_param('~ipfs_http_provider')).netloc.split(':')
        self.ipfs = ipfsapi.connect(ipfs_provider[0], int(ipfs_provider[1]))

        rospy.Subscriber('~rosbag/route_request', geographic_msgs.msg.GeoPath, self.on_route_request)
        rospy.Subscriber('~remote/incoming/ask', Ask, self.on_ask)

        self.liability_finish_service = rospy.ServiceProxy('~liability_finish', Empty)

        self.bid_topic = rospy.Publisher('~remote/sending/bid', Bid, queue_size=10)
        self.confirmed_route_topic = rospy.Publisher('~local/sending/confirmation_result', RouteConfirmationResponse,
                                                     queue_size=10)

        self.asp_service_id = None
        rospy.Subscriber('~local/asp_service_id', String, self.on_new_service_id)

    def on_new_service_id(self, msg):
        rospy.loginfo("Got new service id: %s", msg.data)
        self.asp_service_id = msg.data

    def on_route_request(self, route):
        rospy.loginfo("ASP got new route request")
        confirmation = RouteConfirmationResponse()
        confirmation.route = route
        confirmation.isConfirmed = True
        self.confirmed_route_topic.publish(confirmation)
        self.liability_finish_service()
        rospy.loginfo("ASP sent confirmation")

    def on_ask(self, ask):
        rospy.logdebug("ASP found an Ask")
        if ask.model == self.asp_service_id:
            rospy.logdebug("ASP found an Ask with its service id")
            bid = self.prepare_bid(ask.model, ask.cost, ask.deadline)
            self.bid_topic.publish(bid)

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
