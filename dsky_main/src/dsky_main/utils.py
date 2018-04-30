import rosbag
from web3 import Web3

ETHEREUM_MSG_PREFIX = "\u0019Ethereum Signed Message:\n32".encode('utf-8')

def get_message_from_rosbag(bag_path, requested_topic):
    with rosbag.Bag(bag_path) as bag:
        for topic, msg, t in bag.read_messages():
            if topic == requested_topic:
                return msg


def eth_msg_hash(msg):
    return Web3.soliditySha3(['bytes', 'bytes32'], [ETHEREUM_MSG_PREFIX, msg])
