import os
from tempfile import TemporaryDirectory

import rosbag
import ipfsapi

import std_msgs.msg
from rospy.msg import AnyMsg


class MessageToBagToIPFS:

    def __init__(self):
        rospy.init_node('rosbag_ipfs_pack_node', anonymous=True)
        ipfs_provider = urlparse(rospy.get_param('~ipfs_http_provider')).netloc.split(':')
        self.ipfs = ipfsapi.connect(ipfs_provider[0], int(ipfs_provider[1]))

        rospy.Subscriber('input', AnyMsg, self.on_message)
        self.generated_ipfs_hash_topic = rospy.Publisher('output', std_msgs.msg.String, queue_size=10)

    def on_message(self, msg):
        with TemporaryDirectory() as tmpdir:
            rospy.logdebug('Temporary directory created: %s', tmpdir)
            bag_file = os.path.join(tmpdir, 'bag.bag')
            bag = rosbag.Bag(bag_file, 'w')
            bag.write('channel', msg)
            bag.close()
            self.generated_ipfs_hash_topic.publish(self.ipfs.add(bag_file)['Hash'])

    def spin(self):
        rospy.spin()
