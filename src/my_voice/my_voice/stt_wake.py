import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pvcheetah import Cheetah
import pvcheetah
class SpeechToTextNode(Node):
    def __init__(self):
        super().__init__('speech_to_text_node')

        # 설정: Cheetah 모델 경로와 키
        access_key = "7xMkWa/RTl5/JP8Ofri6PRc1MRnHAF2Y1hQwFm0aeVUIbN9KwH3AQg==" # Picovoice에서 발급받은 키를 사용
        library_path = None# 라이브러리 경로를 지정하지 않으면 기본값 사용
        model_path = "~/Download/porcupine_params_ko.pv" # 영어 모델 사용
        # model_path = ""  # 한국어 모델 사용

        # Cheetah 초기화
        try:
            self.cheetah = pvcheetah.create(
                access_key=access_key,
                library_path=library_path,
                model_path=model_path
            )
            self.get_logger().info("Cheetah initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Cheetah: {e}")
            exit(1)

        # ROS2 Publisher
        self.publisher_ = self.create_publisher(String, '/stt', 10)

        # 타이머로 음성 처리
        self.timer = self.create_timer(0.1, self.process_audio_callback)

        # 녹음 데이터를 위한 버퍼
        self.audio_buffer = []

        # ROS2 Subscriber (오디오 데이터 받기)
        self.audio_subscriber = self.create_subscription(
            String, 'audio_input', self.audio_callback, 10)

    def audio_callback(self, msg):
        # 오디오 데이터를 버퍼에 추가
        self.audio_buffer.append(msg.data)

    def process_audio_callback(self):
        if not self.audio_buffer:
            return

        try:
            # 버퍼의 오디오 데이터를 처리하여 텍스트로 변환
            pcm_audio = b''.join(self.audio_buffer)
            transcript, is_endpoint = self.cheetah.process(pcm_audio)

            # 결과를 ROS2 토픽으로 퍼블리시
            if transcript:
                #self.publisher_.publish(String(data=transcript))
                self.get_logger().info(f"Transcript: {transcript}")

            if is_endpoint:
                self.audio_buffer = []  # 새로운 문장을 위한 버퍼 초기화
        except Exception as e:
            self.get_logger().error(f"Error during speech-to-text: {e}")

    def destroy_node(self):
        # 리소스 정리
        if self.cheetah:
            self.cheetah.delete()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = SpeechToTextNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
