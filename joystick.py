import pygame
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoystickPublisher(Node):
    def __init__(self):
        super().__init__('joystick_publisher')
        self.publisher_ = self.create_publisher(Joy, '/joy', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)

        # Pygame 초기화
        pygame.init()
        pygame.joystick.init()

        # 조이스틱 초기화
        if pygame.joystick.get_count() == 0:
            self.get_logger().error("조이스틱이 연결되지 않았습니다!")
            raise Exception("No joystick found")
        
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f'조이스틱 이름: {self.joystick.get_name()}')
        self.get_logger().info(f'축 개수: {self.joystick.get_numaxes()}')
        self.get_logger().info(f'버튼 개수: {self.joystick.get_numbuttons()}')

    def timer_callback(self):
        pygame.event.pump()
        pygame.event.clear()

        # 조정된 축/버튼 번호 사용
        axis_x = self.joystick.get_axis(0)  # 왼쪽 스틱 X축
        axis_y = -self.joystick.get_axis(1)  # 왼쪽 스틱 Y축 (반전)
        axis_z = -self.joystick.get_axis(4)   # 오른쪽 스틱 Y축 (반전)
        grip_button = self.joystick.get_button(4)  # LB 버튼
        release_button = self.joystick.get_button(5)  # RB 버튼

        # 디버깅 로그
        self.get_logger().info(f"Axis X: {axis_x:.2f}")
        self.get_logger().info(f"Axis Y: {axis_y:.2f}")
        self.get_logger().info(f"Axis Z: {axis_z:.2f}")
        self.get_logger().info(f"Button Grip (LB): {grip_button}")
        self.get_logger().info(f"Button Release (RB): {release_button}")

        # ROS 2 메시지 생성 및 퍼블리시
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.axes = [axis_x, axis_y, axis_z]
        joy_msg.buttons = [int(grip_button), int(release_button)]
        self.publisher_.publish(joy_msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        joystick_publisher = JoystickPublisher()
        rclpy.spin(joystick_publisher)
    except Exception as e:
        print(f"에러 발생: {e}")
    finally:
        rclpy.shutdown()
        pygame.quit()

if __name__ == '__main__':
    main()