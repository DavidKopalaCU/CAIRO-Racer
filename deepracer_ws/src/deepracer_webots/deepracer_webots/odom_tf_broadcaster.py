import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile,
                        QoSHistoryPolicy,
                        QoSReliabilityPolicy)

# from nav_.msg import OdometryMsg 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose, TwistWithCovariance, Twist, TransformStamped
from tf2_ros import TransformBroadcaster

class OdomBroadcaster(Node):

    def __init__(self):
        super().__init__('odometry_broadcaster_node')
        self.get_logger().info('odometry_broadcaster_node started')

        self.br = TransformBroadcaster(self)

        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_cb, 1)

    def odom_cb(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = msg.child_frame_id
        
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        
        t.transform.rotation = msg.pose.pose.orientation

        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = OdomBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()