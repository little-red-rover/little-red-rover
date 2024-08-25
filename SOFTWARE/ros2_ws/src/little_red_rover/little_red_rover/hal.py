from math import floor, inf, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg._joint_state import JointState
from sensor_msgs.msg._laser_scan import LaserScan
from geometry_msgs.msg._twist import Twist

import little_red_rover.pb.messages_pb2 as messages

import threading
import socket

# LRR Hardware Abstraction Layer (HAL)

robot_ip = "192.168.4.1"


class HAL(Node):
    def __init__(self):
        super().__init__("hal")

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
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
        self.laser_msg = LaserScan()
        self.laser_msg.header.frame_id = "lidar"
        self.laser_msg.range_min = 0.1
        self.laser_msg.range_max = 8.0
        self.laser_msg.angle_min = 0.0
        self.laser_msg.angle_max = 2.0 * pi
        self.laser_msg.ranges.fromlist([0.0] * 720)
        self.laser_msg.intensities.fromlist([0.0] * 720)
        self.laser_msg.angle_increment = (2 * pi) / (len(self.laser_msg.ranges))

        threading.Thread(target=self.run_loop).start()

    def run_loop(self):
        while True:
            data = self.socket.recv(1500)
            packet = messages.UdpPacket()
            try:
                packet.ParseFromString(data)
            except Exception as e:
                print(e)
                continue

            if packet.HasField("laser"):
                self.handle_laser_scan(packet.laser)
            elif packet.HasField("joint_states"):
                self.handle_joint_states(packet.joint_states)

    def handle_joint_states(self, packet: messages.JointStates):
        msg = JointState()
        msg.header.frame_id = "robot_body"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = packet.name
        msg.effort = packet.effort
        msg.position = packet.position
        msg.velocity = packet.velocity
        self.joint_state_publisher.publish(msg)

    def handle_laser_scan(self, packet: messages.LaserScan):
        break_in_packet = False

        for i in range(len(packet.ranges)):
            angle = packet.angle_min + (packet.angle_max - packet.angle_min) * (
                i / (len(packet.ranges) - 1)
            )
            index = int(((angle % (2.0 * pi)) / (2.0 * pi)) * 720.0)

            if angle > pi * 2.0 and not break_in_packet:
                self.laser_msg.time_increment = packet.time_increment
                self.laser_msg.scan_time = packet.scan_time

                break_in_packet = True
                self.scan_publisher.publish(self.laser_msg)
                self.laser_msg.ranges = [0.0] * 720
                self.laser_msg.intensities = [0.0] * 720

                self.laser_msg.header.stamp = self.get_clock().now().to_msg()

            self.laser_msg.ranges[index] = packet.ranges[i]
            self.laser_msg.intensities[index] = packet.intensities[i]

            if (
                self.laser_msg.ranges[index] > 8.0
                or self.laser_msg.ranges[index] < 0.1
                or self.laser_msg.intensities[index] == 0
            ):
                self.laser_msg.ranges[index] = inf
                self.laser_msg.intensities[index] = 0
                pass

    def cmd_vel_callback(self, msg: Twist):
        packet = messages.UdpPacket()
        packet.cmd_vel.v = msg.linear.x
        packet.cmd_vel.w = msg.angular.z

        self.socket.sendto(
            packet.SerializeToString(),
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
