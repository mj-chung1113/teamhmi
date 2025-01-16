import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int16, Float32
import pyaudio
import pvrhino
import webrtcvad
import numpy as np

class VoiceRecognitionNode(Node):
    def __init__(self):
        super().__init__('voice_recognition_node')

        # ROS2 Subscriber for button state
        self.subscription = self.create_subscription(
            Bool, '/button_state', self.button_callback, 10
        )
        self.dst_publisher = self.create_publisher(Int16, '/dst', 10)
        # Variables for button state and recording
        self.button_pressed = False
        self.recording = False

        # PyAudio and WebRTC VAD setup
        self.audio = pyaudio.PyAudio()
        self.stream = None
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(3)  # Aggressive mode for noise robustness

        # Rhino Speech-to-Intent setup
        self.rhino = pvrhino.create(access_key="7xMkWa/RTl5/JP8Ofri6PRc1MRnHAF2Y1hQwFm0aeVUIbN9KwH3AQg==",
                                    context_path="/home/jmj/voice_ws/src/my_voice/test_ko_linux_v3_0_0.rhn", 
                                    model_path="/home/jmj/voice_ws/src/my_voice/rhino_params_ko.pv", 
                                    sensitivity= 0.5, 
                                    endpoint_duration_sec= 1.,
                                    require_endpoint= True
                                    )#pv
        self.slot_to_value = {
                "후문": 1,
                "학생회관": 2,
                "새천년관": 3,
                "신공학관": 4,
                "공대": 5,
                "공학관": 5,
            }
        # Parameters
        self.rate = 16000
        self.chunk = 320  # 20ms chunks at 16kHz
        self.silence_timeout = 2.0  # Max silence duration in seconds
        self.silence_frames = int(self.silence_timeout * 1000 / 20)
        self.silence_count = 0

        self.get_logger().info("Voice Recognition Node Initialized")

    def button_callback(self, msg: Bool):
        self.button_pressed = msg.data
        if self.button_pressed and not self.recording:
            self.start_recording()
        elif not self.button_pressed and self.recording:
            self.stop_recording()

    def start_recording(self):
        self.get_logger().info("Button pressed, starting recording...")
        self.recording = True
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )
        self.process_audio()

    def stop_recording(self):
        self.get_logger().info("Stopping recording...")
        self.recording = False
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
            self.stream = None

    def process_audio(self):
        audio_buffer = b""
        self.silence_count = 0

        while self.recording:
            audio_data = self.stream.read(self.chunk, exception_on_overflow=False)
            audio_buffer += audio_data

            # VAD check for speech
            is_speech = self.vad.is_speech(audio_data, self.rate)
            if is_speech:
                self.silence_count = 0
            else:
                self.silence_count += 1

            # Check for silence or button release
            if self.silence_count > self.silence_frames or not self.button_pressed:
                self.get_logger().info("Silence detected or button released.")
                self.stop_recording()
                self.process_intent(audio_buffer)
                break

    def process_intent(self, audio_buffer):
        # Convert audio to Rhino-compatible format
        audio_samples = np.frombuffer(audio_buffer, dtype=np.int16).tolist()
        is_finalized = self.rhino.process(audio_samples)
        if is_finalized: #추출 완료 시 
            inference = self.rhino.get_inference() #추출된 결과 객체 
            if inference.is_understood(): #추출된 결과 이해함
                if inference.intent is not None:
                    self.get_logger().info(f"Intent Detected: {inference.intent}, slots: {inference.slots}")
                    self.shoot_intent(inference.slots)
                else:
                    self.get_logger().info("No intent detected")
                    self.dst_publisher.publish(Int16(data=-1))
        # Cleanup
        self.rhino.delete()
        self.audio.terminate()
        super().destroy_node()
'''
    def shoot_intent(self, slots):
        value = self.slot_to_value.get(slots)
            if value is not None:
                self.dst_publisher.publish(Int16(data=value))
        elif intents == None: 
            self.get_logger().info("Intent not recognized")
            self.dst_publisher.publish(Int16(data=-1))
'''
def main(args=None):
    rclpy.init(args=args)
    node = VoiceRecognitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
