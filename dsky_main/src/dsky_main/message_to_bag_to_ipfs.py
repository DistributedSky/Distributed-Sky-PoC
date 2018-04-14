import os
from importlib import import_module
from tempfile import TemporaryDirectory
from urllib.parse import urlparse

import rosbag, rospy
import ipfsapi

import std_msgs.msg
import rospy.msg


class MessageToBagToIPFS:

    def __init__(self):
        rospy.init_node('message_to_bag_to_ipfs_node', anonymous=True)

        self.topic = rospy.get_param('~rosbag_topic')
        ipfs_provider = urlparse(rospy.get_param('~ipfs_http_provider')).netloc.split(':')
        self.ipfs = ipfsapi.connect(ipfs_provider[0], int(ipfs_provider[1]))

        rospy.Subscriber('~input', rospy.msg.AnyMsg, self.on_message)
        self.generated_ipfs_hash_topic = rospy.Publisher('~output', std_msgs.msg.String, queue_size=10)

    def convert_message(self, msg):
        connection_header = msg._connection_header['type'].split('/')
        ros_pkg = connection_header[0] + '.msg'
        msg_type = connection_header[1]
        msg_class = getattr(import_module(ros_pkg), msg_type)
        return msg_class().deserialize(msg._buff)

    def on_message(self, msg):
        with TemporaryDirectory() as tmpdir:
            rospy.logdebug('Temporary directory created: %s', tmpdir)
            bag_file = os.path.join(tmpdir, 'bag.bag')
            bag = rosbag.Bag(bag_file, 'w')
            bag.write(self.topic, self.convert_message(msg))
            bag.close()

            file_hash = self.ipfs.add(bag_file)['Hash']
            rospy.loginfo('Published rosbag to: %s', file_hash)
            self.generated_ipfs_hash_topic.publish(file_hash)

    def spin(self):
        rospy.spin()
