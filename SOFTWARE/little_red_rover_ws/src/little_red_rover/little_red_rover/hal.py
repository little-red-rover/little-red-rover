import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg._joint_state import JointState
from sensor_msgs.msg._laser_scan import LaserScan
from geometry_msgs.msg._twist import Twist

import socket

# LRR Hardware Abstraction Layer (HAL)


class HAL(Node):
    def __init__(self):
        super().__init__("HAL")
        self.subscription = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, qos_profile_sensor_data
        )

        self.joint_state_publisher = self.create_publisher(
            JointState, "joint_states", qos_profile_sensor_data
        )
        self.scan_publisher = self.create_publisher(
            LaserScan, "scan", qos_profile_sensor_data
        )

    def cmd_vel_callback(self, msg: JointState):
        self.get_logger().info("Got joint state message: %f" % msg.position[0])


def main(args=None):
    rclpy.init(args=args)

    hal_node = HAL()

    rclpy.spin(hal_node)

    hal_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
