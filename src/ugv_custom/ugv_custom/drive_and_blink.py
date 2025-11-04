#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import time

class DriveAndBlink(Node):
    def __init__(self):
        super().__init__('drive_and_blink')
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_led = self.create_publisher(Float32MultiArray, '/ugv/led_ctrl', 10)
        self.get_logger().info('UGV Drive & Blink node started.')

        self.linear_speed = 0.2
        self.distance = 0.5
        self.drive_duration = self.distance / self.linear_speed

        self.timer = self.create_timer(0.1, self.drive_forward)
        self.start_time = time.time()
        self.driving_done = False
        self.blink_count = 0

    def drive_forward(self):
        if not self.driving_done:
            elapsed = time.time() - self.start_time
            if elapsed < self.drive_duration:
                twist = Twist()
                twist.linear.x = self.linear_speed
                self.pub_cmd.publish(twist)
            else:
                # stop rover
                twist = Twist()
                self.pub_cmd.publish(twist)
                self.driving_done = True
                self.start_time = time.time()
                self.get_logger().info('Drive complete')
        else:
            stop_twist = Twist()
            self.pub_cmd.publish(stop_twist)
            self.blink_leds()

    def blink_leds(self):
        elapsed = time.time() - self.start_time
        if self.blink_count < 6:  # blink 3 times
            led_msg = Float32MultiArray()

            if int(elapsed * 2) % 2 == 0:
                led_msg.data = [9.0, 0.0]  # ON
            else:
                led_msg.data = [0.0, 0.0]  # OFF
            self.pub_led.publish(led_msg)
            self.blink_count += 0.1
        else:
            led_msg.data = [0.0, 0.0]  # OFF
            self.pub_led.publish(led_msg)
            self.get_logger().info('Blink complete')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DriveAndBlink()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
