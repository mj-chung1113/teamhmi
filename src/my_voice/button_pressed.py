import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import curses  # 터미널 키보드 입력 감지 라이브러리

class KeyboardButtonPublisher(Node):
    def __init__(self, stdscr):
        super().__init__('keyboard_button_publisher')
        self.publisher_ = self.create_publisher(Bool, '/talkbutton_state', 10)
        self.is_pressed = False  # 버튼 상태 추적
        self.get_logger().info("Keyboard Button Publisher Node has started.")
        self.run_keyboard_listener(stdscr)

    def run_keyboard_listener(self, stdscr):
        stdscr.nodelay(True)  # 입력 대기 없이 동작
        self.get_logger().info("Press and hold '1' to simulate pressing. Release to simulate release. Press 'q' to quit.")

        while rclpy.ok():
            try:
                key = stdscr.getch()  # 키 입력 읽기
                if key == ord('1'):  # '1' 키를 누름
                    if not self.is_pressed:
                        self.publish_button_state(True)
                        self.is_pressed = True
                elif self.is_pressed:  # '1' 키를 뗌
                    self.publish_button_state(False)
                    self.is_pressed = False

                if key == ord('q'):  # 'q' 키를 눌러 종료
                    self.get_logger().info("Exiting keyboard listener.")
                    break

            except Exception as e:
                self.get_logger().error(f"Error in keyboard listener: {e}")
                break

    def publish_button_state(self, state):
        msg = Bool()
        msg.data = state
        self.publisher_.publish(msg)
        self.get_logger().info(f"Button state: {'Pressed' if state else 'Released'}")

def main(args=None):
    rclpy.init(args=args)
    curses.wrapper(lambda stdscr: KeyboardButtonPublisher(stdscr))  # curses 실행

    try:
        rclpy.spin_once()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
