
"""
camera_shim.py
The webots controller automatically creates topics for each of the sensors. However,
these topics do not match the name or the message type of what is created by the
deepracer hardware. This module will be responsible for combining the camera topics
from webots into the single camera image expected by the rest of the platform/

Webots publishes these topics
  - /Agent/zed_camera_left_sensor: sensor_msgs/msg/Image
  - /Agent/zed_camera_right_sensor: sensor_msgs/msg/Image

The Deepracer platform expects the following topics:
  - /camera_pkg/video_mpej: CameraMsg
    - This is just a fancy messge that is a list of all the images recorded by the cameras
  - /camera_okg/display_mpej: Image
    - Just the image to display in the GUI
"""

from typing import List
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile,
                        QoSHistoryPolicy,
                        QoSReliabilityPolicy)

from sensor_msgs.msg import Image
from deepracer_interfaces_pkg.msg import CameraMsg

class CameraShim(Node):

    def __init__(self, qos_profile):
        super().__init__('camera_shim_node')
        self.get_logger().info('camera_shim_node started')

        self.camera_msg = CameraMsg()
        self.camera_subs = []

        self.declare_parameter('camera_topics', [])
        camera_topics = self.get_parameter('camera_topics').get_parameter_value().string_array_value

        joiner = ', '
        self.get_logger().info(f'camera_shim_node -  Attaching to topics:  { joiner.join(camera_topics) }')

        for index, topic in enumerate(camera_topics):
            sub = self.create_subscription(Image, topic, self.make_webots_camera_cb(topic, index), qos_profile)
            self.camera_subs.append(sub)

            self.camera_msg.images.append(Image())

        self.video_publisher = self.create_publisher(CameraMsg, '/camera_pkg/video_mpeg', qos_profile)
        self.display_publisher = self.create_publisher(Image, '/camera_pkg/display_mpeg', qos_profile)
        
    def make_webots_camera_cb(self, topic_name, index):

        def webots_camera_cb(image: Image):

            self.get_logger().debug(f'Received frame from ({ index }) { topic_name }')

            self.camera_msg.images[index] = image
            
            if index == 0:
                self.display_publisher.publish(image)

            if index == (len(self.camera_subs) - 1):
                self.get_logger().debug('Publishing on video topic!')
                self.video_publisher.publish(self.camera_msg)

        return webots_camera_cb

def main(args=None):
    rclpy.init(args=args)
    qos = QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                     depth=1,
                     history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST)
    
    camera_shim_node = CameraShim(qos)
    rclpy.spin(camera_shim_node)

    camera_shim_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
