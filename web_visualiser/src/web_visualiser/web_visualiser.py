import json
import os
from tempfile import TemporaryDirectory
from urllib.parse import urlparse

import geojson
import ipfsapi
import rospy
import hashlib
from flask import Flask, render_template
from flask_socketio import SocketIO

from robonomics_liability.msg import Liability
from robonomics_lighthouse.msg import Ask, Bid, Result
from robonomics_lighthouse.signer import bidhash, askhash, reshash
from web3 import Web3, HTTPProvider
from web3.auto import w3

from dsky_main import utils
from dsky_main.connectors import IPFSConnector
from dsky_main.utils import eth_msg_hash
from global_planner.converters import GeoConverter


class WebVisualiser:

    def __init__(self):
        rospy.init_node('web_visualiser', anonymous=True)

        template_folder_path = rospy.get_param('~flask_template_folder')
        static_folder_path = rospy.get_param('~flask_static_folder')

        self.rosbag_response_topic_name = rospy.get_param('~response_rosbag_topic')
        self.rosbag_request_topic_name = rospy.get_param('~request_rosbag_topic')

        self.app = Flask(__name__, template_folder=template_folder_path, static_folder=static_folder_path)
        self.socketio = SocketIO(self.app, async_mode='threading')
        self.web_views()

        ipfs_provider = urlparse(rospy.get_param('~ipfs_http_provider')).netloc.split(':')
        self.ipfsapi = ipfsapi.connect(ipfs_provider[0], int(ipfs_provider[1]))
        self.ipfs_connector = IPFSConnector(self.ipfsapi)

        http_provider = rospy.get_param('~web3_http_provider')
        self.web3 = Web3(HTTPProvider(http_provider))
        w3.eth.enable_unaudited_features()

        self.regional_service_name = rospy.get_param('~regional_service_name')
        self.global_service_name = rospy.get_param('~global_service_name')

        rospy.Subscriber('~bids', Bid, self.on_bid)
        rospy.Subscriber('~asks', Ask, self.on_ask)
        rospy.Subscriber('~results', Result, self.on_result)
        rospy.Subscriber('~liabilities', Liability, self.on_liability)

        self.liabilites = {}

    def spin(self):
        self.socketio.run(self.app, host='0.0.0.0')

    def extract_msg(self, ipfs_file_hash, topic_name):
        with TemporaryDirectory() as tmpdir:
            workdir = os.getcwd()
            try:
                os.chdir(tmpdir)
                self.ipfsapi.get(ipfs_file_hash)
                return utils.get_message_from_rosbag(ipfs_file_hash, topic_name)
            finally:
                os.chdir(workdir)

    def get_service_id_from_model(self, model):
        service_manifest_raw = self.ipfs_connector.get_from_ipfs(model)
        try:
            service_manifest = json.loads(service_manifest_raw)
            return service_manifest['service_id']
        except Exception:
            pass

    def check_service(self, model):
        service_id = self.get_service_id_from_model(model)
        if service_id in [self.regional_service_name, self.global_service_name]:
            return service_id

    def ecrecover(self, msg_hash, signature):
        return w3.eth.account.recoverHash(eth_msg_hash(msg_hash), signature=signature)

    def route_hash(self, route):
        return hashlib.sha256(route.wkb_hex.encode('utf-8')).hexdigest()

    def on_ask(self, ask):
        rospy.loginfo("Got ask")
        service_id = self.check_service(ask.model)
        if service_id:
            rospy.loginfo("Got service id: %s", service_id)
            request_msg = self.extract_msg(ask.objective, self.rosbag_request_topic_name)
            route = request_msg
            route_line_string = GeoConverter.geo_path_to_line_string(route)
            sender = self.ecrecover(askhash(ask), ask.signature)

            service_type = {
                self.global_service_name: 'global',
                self.regional_service_name: 'regional'
            }[service_id]

            msg = {
                'serviceId': service_id,
                'routeHash': self.route_hash(route_line_string),
                'sender': sender,
                'serviceType': service_type,
                'route': geojson.Feature(geometry=route_line_string)
            }
            self.socketio.emit('ask', msg)

    def on_bid(self, bid):
        rospy.loginfo("Got bid")
        service_id = self.check_service(bid.model)
        if service_id:
            rospy.loginfo("Got service id: %s", service_id)
            sender = self.ecrecover(bidhash(bid), bid.signature)
            msg = {
                'serviceId': service_id,
                'sender': sender
            }
            self.socketio.emit('bid', msg)

    def on_liability(self, liability):
        rospy.loginfo("Got liability")
        service_id = self.check_service(liability.model)
        if service_id:
            rospy.loginfo("Got service id: %s", service_id)
            request_msg = self.extract_msg(liability.objective, self.rosbag_request_topic_name)
            route = request_msg
            route_line_string = GeoConverter.geo_path_to_line_string(route)
            msg = {
                'serviceId': service_id,
                'promisor': liability.promisor,
                'promisee': liability.promisee,
                'liability': liability.address,
                'routeHash': self.route_hash(route_line_string)
            }
            self.liabilites[liability.address] = liability
            self.socketio.emit('liability', msg)

    def on_result(self, result):
        rospy.loginfo("Got result")
        if result.liability in self.liabilites:
            result_msg = self.extract_msg(result.result, self.rosbag_response_topic_name)
            route = result_msg.route
            route_line_string = GeoConverter.geo_path_to_line_string(route)
            msg = {
                'isConfirmed': result_msg.isConfirmed,
                'liability': result.liability,
                'routeHash': self.route_hash(route_line_string)
            }
            self.socketio.emit('result', msg)

    def web_views(self):
        @self.app.route('/')
        def hello_world():
            return render_template('index.html')
