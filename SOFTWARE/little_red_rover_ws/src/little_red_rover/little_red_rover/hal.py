import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg._joint_state import JointState
from sensor_msgs.msg._laser_scan import LaserScan
from geometry_msgs.msg._twist import Twist
from std_msgs.msg._header import Header

import little_red_rover.pb.messages_pb2 as messages

import threading
import socket

# LRR Hardware Abstraction Layer (HAL)

robot_ip = "192.168.4.1"


class HAL(Node):
    def __init__(self):
        super().__init__("hal")

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("0.0.0.0", 8001))

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
            packet = messages.UdpPacket()
            packet.FromString(data)
            print(packet)
            if packet.HasField("laser"):
                msg = LaserScan()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "lidar_frame"
                msg.angle_min = packet.laser.angle_min
                msg.angle_max = packet.laser.angle_max
                msg.range_min = packet.laser.range_min
                msg.range_max = packet.laser.range_max
                msg.time_increment = packet.laser.time_increment
                msg.angle_increment = packet.laser.angle_increment
                msg.ranges = packet.laser.ranges
                msg.intensities = packet.laser.intensities
                self.scan_publisher.publish(msg)

    def cmd_vel_callback(self, msg: Twist):
        packet = messages.UdpPacket()
        packet.cmd_vel.v = msg.linear.x
        packet.cmd_vel.w = msg.angular.z

        self.socket.sendto(
            packet.SerializeToString(),
            ("192.168.4.1", 8001),
        )
        # self.get_logger().info("Got cmd vel msg %f, %f" % (msg.linear.x, msg.angular.z))


def main(args=None):
    rclpy.init(args=args)

    hal_node = HAL()

    rclpy.spin(hal_node)

    hal_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
