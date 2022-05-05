import math
import numpy as np

import rclpy

from std_msgs.msg import Header
from geometry_msgs.msg import Twist, Point, TwistWithCovariance, PoseWithCovariance
from nav_msgs.msg import Odometry

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    Stolen from: https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[3] = cy * cp * cr + sy * sp * sr
    q[0] = cy * cp * sr - sy * sp * cr
    q[1] = sy * cp * sr + cy * sp * cr
    q[2] = sy * cp * cr - cy * sp * sr

    return q

class MyRobotDriver:
    def init(self, webots_node, properties) -> None:
        self.__robot = webots_node.robot

        self.wheel_rf = self.__robot.getDevice("right_front_wheel_joint")
        self.wheel_lf = self.__robot.getDevice("left_front_wheel_joint")
        self.wheel_rb = self.__robot.getDevice("right_rear_wheel_joint")
        self.wheel_lb = self.__robot.getDevice("right_rear_wheel_joint")

        self.wheels = [self.wheel_rf, self.wheel_lf, self.wheel_rb, self.wheel_lb]

        # Using velocity control
        for wheel in self.wheels:
            wheel.setPosition(math.inf)
            wheel.setVelocity(0)
            wheel.setAcceleration(math.inf)
            wheel.setAvailableTorque(10)

        self.steer_left = self.__robot.getDevice('left_steering_hinge_joint')
        self.steer_right = self.__robot.getDevice('right_steering_hinge_joint')

        self.steering = [self.steer_left, self.steer_right]

        # Use position control
        for steer in self.steering:
            steer.setPosition(0)
            steer.setVelocity(100)

        self.camera_left = self.__robot.getDevice("zed_camera_left_sensor")
        self.camera_right = self.__robot.getDevice("zed_camera_right_sensor")

        self.cameras = [self.camera_left, self.camera_right]

        for camera in self.cameras:
            camera.enable(1000 // 15)

        self.lidar = self.__robot.getDevice("hokuyo_sensor")
        self.lidar.enable(1000 // 10)
        self.lidar.enablePointCloud()

        self.__target_twist = Twist()
        self.__state = [0, 0, 0] # X, Y, Theta

        rclpy.init(args=None)
        
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.get_logger().info('started my_robot_driver node')
        self.__odom_pub = self.__node.create_publisher(Odometry, '/odom', 1)
        # self.__timer = self.__node.create_timer(0.1, self.timer_cb)
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist: Twist):
        self.__target_twist = twist

    def update_odom(self, dtime):
        # target_twsist.linear.x is in rad/s
        #   / (2 * pi) -> rotations / second
        #   * diameter -> meters / s
        #   * period -> meters
        scale = (1 / (2 * math.pi)) * (2 * math.pi * 0.03) * dtime * 0.6
        
        # Pull the speed and angle from the twist
        speed = self.__target_twist.linear.x
        angle = self.__target_twist.angular.y
        
        # Clip the speed and the angle
        speed = max(-100, min(100, speed))
        angle = max(-1, min(1, angle))
        self.__state = [
            self.__state[0] + scale * (speed * math.cos(self.__state[2])),
            self.__state[1] + scale * (speed * math.sin(self.__state[2])),
            self.__state[2] + scale * (speed / 0.16 * math.tan(angle))
        ]

        odom_msg = Odometry()
        odom_msg.header = Header(frame_id='odom')
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose = PoseWithCovariance()
        odom_msg.pose.pose.position = Point(x=self.__state[0], y=self.__state[1], z=0.0)

        quat = quaternion_from_euler(0, 0, self.__state[2])
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        odom_msg.pose.covariance = np.diag([0.001, 0.001, 0.001, 0.001, 0.001, 0.01]).flatten()

        odom_msg.twist = TwistWithCovariance()
        odom_msg.twist.twist = self.__target_twist
        odom_msg.twist.covariance = np.diag([0.001, 0.001, 0.001, 0.001, 0.001, 0.01]).flatten()

        self.__odom_pub.publish(odom_msg)


    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        self.update_odom(self.__robot.getBasicTimeStep() / 1000)

        if self.__target_twist is not None:
            speed = self.__target_twist.linear.x
            angle = self.__target_twist.angular.y

            speed = max(-100, min(100, speed))
            angle = max(-1, min(1, angle))

            for wheel in self.wheels:
                wheel.setVelocity(speed)

            for steer in self.steering:
                steer.setPosition(angle)
