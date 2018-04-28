import os
from tempfile import TemporaryDirectory
from urllib.parse import urlparse

import rosbag, rospy
import ipfsapi

import std_msgs.msg
import geographic_msgs.msg

from robonomics_liability.msg import Liability
from robonomics_lighthouse.msg import Ask, Bid, Result
from web3 import Web3, HTTPProvider

from distributed_sky_uav.msg import RouteConfirmationRequest, RouteConfirmationResponse
from dsky_main import utils


class RouteConfirmer:

    def __init__(self):
        rospy.init_node('route_confirmer_node', anonymous=True)

        web3_provider = rospy.get_param('~web3_http_provider')
        self.web3 = Web3(HTTPProvider(web3_provider))

        self._current_request = None
        self._current_liability = None

        self.token_address = rospy.get_param('~token_address')
        self.validator_fee = rospy.get_param('~validator_fee')
        self.validator_address = rospy.get_param('~validator_address')
        self.rosbag_response_topic_name = rospy.get_param('~response_rosbag_topic')

        self.account = rospy.get_param('~eth_account_address')
        self.account = self.web3.eth.accounts[0] if len(self.account) == 0 else self.account

        ipfs_provider = urlparse(rospy.get_param('~ipfs_http_provider')).netloc.split(':')
        self.ipfs = ipfsapi.connect(ipfs_provider[0], int(ipfs_provider[1]))

        rospy.Subscriber('~local/route_request', RouteConfirmationRequest, self.on_route_request)
        rospy.Subscriber('~remote/incoming/objective_ipfs_hash', std_msgs.msg.String, self.on_objective_ipfs_hash)
        rospy.Subscriber('~remote/incoming/result', Result, self.on_result)
        rospy.Subscriber('~remote/incoming/liability', Liability, self.on_liability)

        self.request_ipfs_hash_topic = rospy.Publisher('~remote/sending/to_ipfs', geographic_msgs.msg.GeoPath,
                                                       queue_size=10)
        self.ask_topic = rospy.Publisher('~remote/sending/ask_publish', Ask, queue_size=10)
        self.confirmed_route_topic = rospy.Publisher('~local/route_response', RouteConfirmationResponse,
                                                     queue_size=10)

    def on_route_request(self, msg):
        if self._current_request is None:
            rospy.loginfo("Got route request, handling...")
            self._current_request = msg
            self.request_ipfs_hash_topic.publish(msg.route)
        else:
            rospy.logwarn("Got route request, when previous one was not handled")

    def on_liability(self, liability):
        if liability.promisee == self.account:
            if self._current_liability is None:
                rospy.loginfo("Request is in work. Liability: %s", liability.address)
                self._current_liability = liability.address
            else:
                rospy.logwarn("New liability, when the old one is in process: %s", liability.address)

    def on_objective_ipfs_hash(self, msg):
        if self._current_request is not None:
            rospy.loginfo("Got IPFS hash for objective, handling...")
            ask = self.prepare_ask(self._current_request.model,
                                   msg.data,
                                   self._current_request.cost,
                                   self._current_request.deadline)
            self.ask_topic.publish(ask)
        else:
            rospy.logwarn("Got IPFS hash, when no request was pending")

    def on_result(self, msg):
        rospy.logdebug('Got result')
        if msg.liability == self._current_liability:
            if self._current_request is not None:
                with TemporaryDirectory() as tmpdir:
                    os.chdir(tmpdir)
                    self.ipfs.get(msg.result)
                    bag_path = os.path.join(tmpdir, msg.result)
                    rospy.logdebug('Result is written to %s', bag_path)
                    result = utils.get_message_from_rosbag(bag_path, self.rosbag_response_topic_name)
                    self._current_request = None
                    self._current_liability = None
                    if result:
                        rospy.loginfo("Route request handled, result: %s", result.isConfirmed)
                        self.confirmed_route_topic.publish(result)
                    else:
                        rospy.logwarn("Received rosbag does not have Route Confirmation messages")
            else:
                rospy.logwarn("Got result, when no request was pending")

    def get_message_from_rosbag(self, bag_path, requested_topic):
        with rosbag.Bag(bag_path) as bag:
            for topic, msg, t in bag.read_messages():
                if topic == requested_topic:
                    return msg

    def prepare_ask(self, model, objective, cost, deadline):
        msg = Ask()
        msg.model = model
        msg.objective = objective
        msg.token = self.token_address
        msg.cost = cost
        msg.count = 1
        msg.validator = self.validator_address
        msg.validatorFee = self.validator_fee
        msg.salt = [0]
        msg.signature = [0]
        msg.deadline = deadline
        return msg

    def spin(self):
        rospy.spin()
