import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoyDisplay(Node):
    def __init__(self):
        super().__init__('joy_display')
        self.subscription = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        self.get_logger().info('조이스틱 데이터 표시 시작!')

    def joy_callback(self, msg):
        print("---")
        print(f"X축: {msg.axes[0]:.2f}")  # 왼쪽 스틱 X축
        print(f"Y축: {msg.axes[1]:.2f}")  # 왼쪽 스틱 Y축
        print(f"Z축: {msg.axes[2]:.2f}")  # 오른쪽 스틱 X축
        print(f"잡기 버튼(LB): {msg.buttons[0]}")
        print(f"놓기 버튼(RB): {msg.buttons[1]}")
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
