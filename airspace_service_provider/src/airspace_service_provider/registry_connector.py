import json
from collections import OrderedDict
from urllib.parse import urlparse

import ipfsapi
import rospy
from web3 import Web3, HTTPProvider

from std_msgs.msg import String
from dsky_main.connectors import IPFSFileRegistry
from dsky_main.resolvers import RegionResolver


class RegistryConnector:

    def __init__(self):
        rospy.init_node('registry_connector', anonymous=True)

        web3_provider = rospy.get_param('~web3_http_provider')
        ipfs_provider = urlparse(rospy.get_param('~ipfs_http_provider')).netloc.split(':')

        self.web3 = Web3(HTTPProvider(web3_provider))
        self.ipfs = ipfsapi.connect(ipfs_provider[0], int(ipfs_provider[1]))

        self.asp_registry_address = rospy.get_param('~asp_registry_address')
        self.asp_manifest_address = rospy.get_param('~asp_manifest_address')

        self.asp_registry_abi = rospy.get_param('~asp_registry_abi')
        self.asp_manifest_abi = rospy.get_param('~asp_manifest_abi')

        self.account = rospy.get_param('~eth_account_address', self.web3.eth.accounts[0])

        self.asp_registry = IPFSFileRegistry(self.asp_registry_abi, self.web3, self.ipfs, self.account)
        self.asp_manifest = IPFSFileRegistry(self.asp_manifest_abi, self.web3, self.ipfs, self.account)

        self.asp_service_id_topic = rospy.Publisher('~asp_service_id', String, latch=True)

        self.current_region = None
        self.load_info()

    def load_info(self):
        asp_registry_json = self.asp_registry.get_file_content(self.asp_registry_address)
        resolver = RegionResolver(json.loads(asp_registry_json))
        self.current_region = resolver.get_by_address(self.asp_manifest_address)
        if self.current_region:
            rospy.loginfo("%s got assigned region.", self.asp_manifest_address)
            service = self.create_confirm_route_service()
            self.publish_manifest({'regional_asp_route_confirm': service})
            self.asp_service_id_topic.publish(String(service))
        else:
            rospy.logwarn("%s has no assigned region", self.asp_manifest_address)

    def create_confirm_route_service(self):
        confirm_route_service = OrderedDict()
        confirm_route_service['service_id'] = 'regional_asp_route_confirm'
        confirm_route_service['contract_address'] = self.asp_manifest_address
        return self.asp_manifest.add_to_ipfs(json.dumps(confirm_route_service))

    def publish_manifest(self, services):
        manifest = {'services': services}
        self.asp_manifest.post_file_content(self.asp_manifest_address, json.dumps(manifest))
        rospy.loginfo("%s has just updated the manifest", self.asp_manifest_address)

    def spin(self):
        rospy.spin()
