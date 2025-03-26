import pygame
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoystickPublisher(Node):
    def __init__(self):
        super().__init__('joystick_publisher')
        self.publisher_ = self.create_publisher(Joy, '/joy', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        pygame.init()
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f'조이스틱 이름: {self.joystick.get_name()}')

    def timer_callback(self):
        pygame.event.pump()
        axis_x = self.joystick.get_axis(0)    # X축: Axis 0
        axis_y = -self.joystick.get_axis(1)   # Y축: Axis 1 (반전)
        axis_z = -self.joystick.get_axis(4)   # Z축: Axis 4 (반전)
        grip_button = self.joystick.get_button(0)  # 잡기: Button 0
        release_button = self.joystick.get_button(1)  # 놓기: Button 1

        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.axes = [axis_x, axis_y, axis_z]  # X, Y, Z만 포함
        joy_msg.buttons = [grip_button, release_button]

        self.publisher_.publish(joy_msg)

def main(args=None):
    rclpy.init(args=args)
    joystick_publisher = JoystickPublisher()
    try:
        rclpy.spin(joystick_publisher)
    except KeyboardInterrupt:
        joystick_publisher.destroy_node()
        rclpy.shutdown()
        pygame.quit()

if __name__ == '__main__':
    main()