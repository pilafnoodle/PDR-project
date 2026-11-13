import sys
import termios
import tty

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import Header

class TapNode(Node):

    def __init__(self):
        super().__init__('tap_node')
        self.publisher_ = self.create_publisher(Header, 'tap', 10)
        self.tap_count = 0
        
    def send_tap(self):
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frame_id = 'left_foot'
        self.publisher_.publish(msg)
        self.tap_count += 1
        self.get_logger().info('Tap #%d' % self.tap_count)
        
def main(args=None):
    rclpy.init(args=args)
    tap_node = TapNode()
    tap_node.get_logger().info('Press SPACE on every LEFT heel strike. Press q to quit.')
    
    old_settings = termios.tcgetattr(sys.stdin)
    
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            key = sys.stdin.read(1)
            if key == ' ':
                tap_node.send_tap()
            elif key == 'q':
                break
    
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print('Complete. Total taps: %d' % tap_node.tap_count)
        tap_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
                    
if __name__ == '__main__':
    main()
