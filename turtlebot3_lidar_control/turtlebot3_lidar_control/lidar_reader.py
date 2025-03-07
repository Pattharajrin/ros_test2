import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarReader(Node):
    def __init__(self):
        super().__init__('lidar_reader')
        self.subscription = self.create_subscription(
            LaserScan,
            '/rplidar_scan',
            self.lidar_callback,
            10)

    def lidar_callback(self, msg):
        ranges = msg.ranges
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min

        min_distance = float('inf')
        min_angle = 0

        for i, distance in enumerate(ranges):
            if 0.2 <= distance <= 0.4:  # เฉพาะค่าที่อยู่ในช่วง 20-40 ซม.
                if distance < min_distance:  # เลือกค่าที่ใกล้ที่สุด
                    min_distance = distance
                    min_angle = math.degrees(angle_min + (i * angle_increment))

        if min_distance == float('inf'):
            self.get_logger().info('No hand detected in range.')
        else:
            if min_angle > 180:
                min_angle -= 360  # ปรับองศาให้เป็น ±180°
            self.get_logger().info(f'Hand at: {min_angle:.2f}°, Distance: {min_distance*100:.2f} cm')

def main(args=None):
    rclpy.init(args=args)
    node = LidarReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
