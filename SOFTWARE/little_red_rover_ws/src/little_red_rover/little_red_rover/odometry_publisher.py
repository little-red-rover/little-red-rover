import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg._joint_state import JointState
from geometry_msgs.msg import TwistWithCovarianceStamped

# LRR Odometry Publisher
# Takes in wheel position / velocity information from JointState messages on /joint_states
# Outputs integrated odometry as Odometry message on /odom
# https://control.ros.org/master/doc/ros2_controllers/doc/mobile_robot_kinematics.html#differential-drive-robot

wheel_track = 0.11439
wheel_diameter = 0.060960


class OdomPublisher(Node):
    def __init__(self):
        super().__init__("odom_publisher")
        self.subscription = self.create_subscription(
            JointState, "joint_states", self.listener_callback, qos_profile_sensor_data
        )
        self.subscription  # prevent unused variable warning

        self.publisher = self.create_publisher(
            TwistWithCovarianceStamped, "twist", qos_profile_sensor_data
        )

    def listener_callback(self, msg: JointState):
        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header.frame_id = "base_link"
        twist_msg.header.stamp = msg.header.stamp

        omega_left = msg.velocity[0]  # forward = +
        omega_right = -msg.velocity[1]  # forward = -, so flip sign

        twist_msg.twist.twist.linear.x = (
            (omega_right + omega_left) * (wheel_diameter / 2)
        ) / 2.0
        twist_msg.twist.twist.angular.z = (
            (omega_right - omega_left) * (wheel_diameter / 2)
        ) / wheel_track

        # Todo: these are based off of nothing
        twist_msg.twist.covariance = sum(
            [
                [0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.1],
            ],
            [],
        )

        self.publisher.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)

    odom_publisher = OdomPublisher()

    rclpy.spin(odom_publisher)

    odom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
