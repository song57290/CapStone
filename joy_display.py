import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoyDisplay(Node):
    def __init__(self):
        super().__init__('joy_display')
        self.subscription = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        self.get_logger().info('한글로 조이스틱 데이터 표시 시작!')

    def joy_callback(self, msg):
        print("---")
        print(f"X축: {msg.axes[0]:.2f}")  # Axis 0
        print(f"Y축: {msg.axes[1]:.2f}")  # Axis 1 (반전)
        print(f"Z축: {msg.axes[2]:.2f}")  # Axis 4 (반전)
        print(f"잡기 버튼(A): {msg.buttons[0]}")
        print(f"놓기 버튼(B): {msg.buttons[1]}")
        print("---")

def main(args=None):
    rclpy.init(args=args)
    joy_display = JoyDisplay()
    try:
        rclpy.spin(joy_display)
    except KeyboardInterrupt:
        joy_display.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()