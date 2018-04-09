import os
from tempfile import TemporaryDirectory
from urllib.parse import urlparse

import rosbag, rospy
import ipfsapi

import std_msgs.msg
import geographic_msgs.msg

from robonomics_lighthouse.msg import Ask, Bid, Result
from distributed_sky_uav.msg import RouteConfirmationRequest


class RouteConfirmer:

    def __init__(self):
        rospy.init_node('route_confirmer_node', anonymous=True)

        self._current_request = None

        self.token_address = rospy.get_param('~token_address')
        self.validator_fee = rospy.get_param('~validator_fee')
        self.validator_address = rospy.get_param('~validator_address')

        ipfs_provider = urlparse(rospy.get_param('~ipfs_http_provider')).netloc.split(':')
        self.ipfs = ipfsapi.connect(ipfs_provider[0], int(ipfs_provider[1]))

        rospy.Subscriber('~local/incoming/route_request', RouteConfirmationRequest, self.on_route_request)
        rospy.Subscriber('~remote/incoming/objective_ipfs_hash', std_msgs.msg.String, self.on_objective_ipfs_hash)
        rospy.Subscriber('~remote/incoming/result', Result, self.on_result)

        self.request_ipfs_hash_topic = rospy.Publisher('~remote/sending/to_ipfs', geographic_msgs.msg.GeoPath,
                                                       queue_size=10)
        self.ask_topic = rospy.Publisher('~remote/sending/ask_publish', Ask, queue_size=10)
        self.confirmed_route_topic = rospy.Publisher('~local/sending/confirmation_result', geographic_msgs.msg.GeoPath,
                                                     queue_size=10)

    def on_route_request(self, msg):
        if self._current_request is None:
            rospy.loginfo("Got route request, handling...")
            self._current_request = msg
            self.request_ipfs_hash_topic.publish(msg.route)
        else:
            rospy.logwarn("Got route request, when previous one was not handled")

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
        if msg.liability == self._current_request.model:
            if self._current_request is not None:
                with TemporaryDirectory() as tmpdir:
                    self.ipfs.get(msg.objective)
                    bag_path = os.path.join(tmpdir, msg.objective)
                    rospy.logdebug('Objective is written to %s', bag_path)
                    msg = self.get_message_from_rosbag(bag_path)
                    self._current_request = None
            else:
                rospy.logwarn("Got result, when no request was pending")

    def get_message_from_rosbag(self, bag_path):
        with rosbag.Bag('input.bag') as bag:
            for topic, msg, t in bag.read_messages():
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
