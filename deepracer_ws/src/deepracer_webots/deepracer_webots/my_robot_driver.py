import math
import rclpy
from geometry_msgs.msg import Twist

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

        self.steer_left = self.__robot.getDevice('left_steering_hinge_joint')
        self.steer_right = self.__robot.getDevice('right_steering_hinge_joint')

        self.steering = [self.steer_left, self.steer_right]

        # Use position control
        for steer in self.steering:
            steer.setPosition(0)
            steer.setVelocity(2)

        self.camera_left = self.__robot.getDevice("zed_camera_left_sensor")
        self.camera_right = self.__robot.getDevice("zed_camera_right_sensor")

        self.cameras = [self.camera_left, self.camera_right]

        for camera in self.cameras:
            camera.enable(100)

        self.lidar = self.__robot.getDevice("hokuyo_sensor")
        self.lidar.enable(100)
        self.lidar.enablePointCloud()

        self.__target_twist = None

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist: Twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        if self.__target_twist is not None:
            speed = self.__target_twist.linear.x
            angle = self.__target_twist.angular.y

            speed = max(-100, min(100, speed))
            angle = max(-1, min(1, angle))

            for wheel in self.wheels:
                wheel.setVelocity(speed)

            for steer in self.steering:
                steer.setPosition(angle)
