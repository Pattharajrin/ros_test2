import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from std_msgs.msg import Bool  # ใช้สำหรับ Subscribe /stop_signal
import math

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/rplidar_scan',
            self.lidar_callback,
            10)
        self.stop_subscription = self.create_subscription(
            Bool,
            '/stop_signal',
            self.stop_callback,
            10)  # Subscribe จาก /stop_signal
        self.client = self.create_client(SetBool, 'stop_robot')
        self.max_speed = 0.1
        self.stop_robot = False  # ตัวแปรเก็บสถานะการหยุด

        # ตั้ง Timer ให้แน่ใจว่าหยุดหุ่นยนต์เมื่อปิดโปรแกรม
        self.create_timer(1.0, self.ensure_stop_on_shutdown)
        self.get_logger().info("✅ TurtleBot Controller Node started.")

    def stop_callback(self, msg):
        """ Callback สำหรับรับสัญญาณหยุดจาก /stop_signal """
        self.stop_robot = msg.data
        if self.stop_robot:
            self.get_logger().warn('🚨 Stop signal received! Stopping TurtleBot.')
            self.publish_stop()

    def lidar_callback(self, msg):
        """ Callback สำหรับ Lidar """
        if self.stop_robot:
            self.publish_stop()
            return

        ranges = msg.ranges
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min

        closest_distance = float('inf')
        closest_angle = None

        for i, distance in enumerate(ranges):
            if 0.1 <= distance <= 0.4 and distance < closest_distance:
                closest_distance = distance
                closest_angle = math.degrees(angle_min + (i * angle_increment))

        # ✅ ถ้าพบมืออยู่ต่ำกว่า 20 ซม. → หยุดการเคลื่อนที่
        if closest_distance < 0.2:
            self.get_logger().warn("🚫 มืออยู่ใกล้เกินไป (<20 ซม.) หยุดหุ่นยนต์")
            self.publish_stop()
            return

        if closest_angle is None:
            self.get_logger().info("No hand detected, stopping movement.")
            self.publish_stop()
            return

        if closest_angle > 180:
            closest_angle -= 360  

        speed = (0.4 - closest_distance) / 0.2 * self.max_speed

        cmd = Twist()
        direction_log = ""  # เก็บข้อความสถานะทิศทาง

        if -45 <= closest_angle <= 45:
            cmd.linear.x = speed
            direction_log = f"🚶‍♂️ พบมือด้านหน้า หุ่นเดินหน้า (Speed: {speed:.2f} m/s)"
        elif 135 <= closest_angle or closest_angle <= -135:
            cmd.linear.x = -speed
            direction_log = f"🔙 พบมือด้านหลัง หุ่นกำลังถอยหลัง (Speed: {speed:.2f} m/s)"
        elif 45 < closest_angle < 135:
            cmd.angular.z = -speed
            direction_log = f"🔄 พบมือด้านซ้าย หุ่นหมุนทวนเข็มนาฬิกา (Speed: {speed:.2f} rad/s)"
        elif -135 < closest_angle < -45:
            cmd.angular.z = speed
            direction_log = f"🔄 พบมือด้านขวา หุ่นหมุนตามเข็มนาฬิกา (Speed: {speed:.2f} rad/s)"

        self.publisher.publish(cmd)

        if direction_log:
            self.get_logger().info(direction_log)  # แสดง Log ทิศทาง

    def publish_stop(self):
        """ หยุดการเคลื่อนที่ของหุ่นยนต์ """
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.publisher.publish(cmd)

    def ensure_stop_on_shutdown(self):
        """ ฟังก์ชันส่งคำสั่งหยุดทุกครั้งเมื่อ Node ปิด """
        self.get_logger().info("🛑 Ensuring TurtleBot stops before shutdown.")
        self.publish_stop()

    def destroy_node(self):
        """ เมื่อ Node ถูกปิด, หยุดหุ่นยนต์ก่อน """
        self.get_logger().info("🔴 Stopping TurtleBot before shutdown...")
        self.publish_stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Keyboard Interrupt (Ctrl+C) received. Stopping...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
