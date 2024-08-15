import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg._joint_state import JointState
from sensor_msgs.msg._laser_scan import LaserScan
from geometry_msgs.msg._twist import Twist

import threading
import socket
import msgpack

# LRR Hardware Abstraction Layer (HAL)

robot_ip = "192.168.4.1"


class HAL(Node):
    def __init__(self):
        super().__init__("hal")

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("0.0.0.0", 8001))

        self.packer = msgpack.Packer()
        self.unpacker = msgpack.Unpacker()

        self.subscription = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, qos_profile_sensor_data
        )

        self.joint_state_publisher = self.create_publisher(
            JointState, "joint_states", qos_profile_sensor_data
        )
        self.scan_publisher = self.create_publisher(
            LaserScan, "scan", qos_profile_sensor_data
        )

        threading.Thread(target=self.run_loop).start()

    def run_loop(self):
        while True:
            data = self.socket.recv(1500)
            self.unpacker.feed(data)
            for o in self.unpacker:
                # if o[0]
                pass

            print("received message: %s" % data)

    def cmd_vel_callback(self, msg: Twist):
        self.socket.sendto(
            self.packer.pack([msg.linear.x, msg.angular.z]),
            ("192.168.4.1", 8001),
        )


def main(args=None):
    rclpy.init(args=args)

    hal_node = HAL()

    rclpy.spin(hal_node)

    hal_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
