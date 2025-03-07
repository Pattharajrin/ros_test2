import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
from std_msgs.msg import Bool  # ใช้สำหรับ Publisher
import math

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        self.service = self.create_service(SetBool, 'stop_robot', self.handle_stop_service)
        self.publisher = self.create_publisher(Bool, '/stop_signal', 10)  # Publisher สำหรับส่งสัญญาณหยุด
        self.stop_robot = False

    def lidar_callback(self, msg):
        """ ตรวจจับสิ่งกีดขวางและส่งสัญญาณหยุด """
        min_distance = float('inf')
        
        for distance in msg.ranges:
            if 0.1 < distance < 0.15:  # ตรวจจับสิ่งกีดขวางในช่วง 10 - 15 ซม.
                min_distance = min(min_distance, distance)

        if min_distance < 0.15:
            if not self.stop_robot:
                self.get_logger().warn(f'🚨 Obstacle detected! Distance: {min_distance*100:.2f} cm')
            self.stop_robot = True
        else:
            self.stop_robot = False

        # ส่งสัญญาณหยุดไปที่ /stop_signal
        stop_msg = Bool()
        stop_msg.data = self.stop_robot
        self.publisher.publish(stop_msg)

    def handle_stop_service(self, request, response):
        """ ใช้ service เพื่อตรวจสอบหรือเปลี่ยนค่า stop_robot """
        if request.data:
            self.stop_robot = True
            response.message = "Robot stopped due to obstacle."
        else:
            self.stop_robot = False
            response.message = "Robot resumed movement."
        
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
