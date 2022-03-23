import rclpy
from geometry_msgs.msg import Twist

class MyRobotDriver:
    def init(self, webots_node, properties) -> None:
        self.__robot = webots_node.robot

        self.__device_count = self.__robot.getNumberOfDevices()
        for idx in range(self.__device_count):
            device = self.__robot.getDeviceByIndex(idx)

            print(f'\t{idx:02d}: {device.getName()}')

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist: Twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        print('step')