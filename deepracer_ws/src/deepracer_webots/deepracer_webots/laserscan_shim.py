from numpy import angle
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile,
                        QoSHistoryPolicy,
                        QoSReliabilityPolicy)

from sensor_msgs.msg import LaserScan

class LaserScanShim(Node):

    def __init__(self):
        super().__init__('odometry_broadcaster_node')
        self.get_logger().info('odometry_broadcaster_node started')

        self.publisher = self.create_publisher(LaserScan, '/scan', 1)
        self.subscription = self.create_subscription(LaserScan, '/Agent/hokuyo_sensor', self.webots_cb, 1)

    def webots_cb(self, msg: LaserScan):
        msg.angle_increment = (msg.angle_max - msg.angle_min) / (len(msg.ranges) - 1)
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = LaserScanShim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()