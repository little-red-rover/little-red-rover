import rclpy
from rclpy.node import Node

# from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32

class LaserScanPlotter(Node):
    def __init__(self):
        super().__init__('laser_scan_plotter');
        # self.subscription = self.create_subscription(LaserScan, 'lidar_scans', self.listener_callback, 10)
        self.subscription = self.create_subscription(Int32, 'lidar_scans', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info('Number: %d'% msg.data);

def main(args=None):
    rclpy.init(args=args)

    laser_scan_plotter = LaserScanPlotter()

    rclpy.spin(laser_scan_plotter)

    laser_scan_plotter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
