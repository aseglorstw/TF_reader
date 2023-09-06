import rosbag
import rospy
import tf2_ros
from rosbag import ROSBagException
from tqdm import tqdm
from sensor_msgs.msg import PointCloud2
import numpy as np
from sensor_msgs.point_cloud2 import read_points
from ros_numpy import numpify
from tf2_ros import ExtrapolationException


def read_point_cloud():
    path = 'path/to/your/bag_file'
    bag = rosbag.Bag(path)
    buffer = load_buffer(bag)
    point_cloud = []
    for msg_number, (topic, msg, time) in enumerate(bag.read_messages(topics=['/points'])):
        if msg_number % 20 == 0:
            try:
                msg = PointCloud2(*slots(msg))
                cloud = np.array(list(read_points(msg)))
                transform_map_lidar = buffer.lookup_transform_full("map", time, msg.header.frame_id, time,
                                                                        "map", rospy.Duration(1))
                matrix = numpify(transform_map_lidar.transform)
                vectors = np.array([cloud[::200, 0], cloud[::200, 1], cloud[::200, 2]])
                transformed_vectors = matrix[:3, :3] @ vectors + matrix[:3, 3:4]
                point_cloud.append(transformed_vectors)
            except ExtrapolationException:
                continue


def load_buffer(bag):
    tf_topics = ['/tf', '/tf_static']
    buffer = tf2_ros.Buffer(rospy.Duration(3600 * 3600))
    try:
        for topic, msg, stamp in tqdm(bag.read_messages(topics=tf_topics),
                                      total=bag.get_message_count(topic_filters=tf_topics)):
            if topic == '/tf':
                for tf in msg.transforms:
                    buffer.set_transform(tf, 'bag')
            elif topic == '/tf_static':
                for tf in msg.transforms:
                    buffer.set_transform_static(tf, 'bag')
    except ROSBagException:
        print('Could not read')
    return buffer


def slots(msg):
    return [getattr(msg, var) for var in msg.__slots__]
