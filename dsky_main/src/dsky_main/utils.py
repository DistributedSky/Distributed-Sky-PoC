import rosbag


def get_message_from_rosbag(bag_path, requested_topic):
    with rosbag.Bag(bag_path) as bag:
        for topic, msg, t in bag.read_messages():
            if topic == requested_topic:
                return msg
