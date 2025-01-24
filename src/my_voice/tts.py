import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, String, Bool
from gtts import gTTS
import playsound
import threading
from collections import deque
import time

class TTS(Node):
    def __init__(self):
        super().__init__('text_to_speech_node')

        # 상태 변수
        self.state = 0  # 주행 상태
        self.current_position = "Unknown"  # 현재 위치
        self.fin_goal = "Unknown"  # 최종 목적지
        self.speed = 0  # 현재 속도
        self.obstacle = False  # 장애물 감지 메시지
        self.handle_status = 0  # 손잡이 상태 (0:둘 다 떨어짐 1:하나만 잡힘 2:둘 다 잡힘)
        self.last_handle_update = time.time()
        self.next_node = "Unknown"  # 다음 노드
        self.meter_to_node = 0
        self.meter_to_dst = 0
        self.hmi_stop = False 
        # 딕셔너리: 상태별 출력 문구
        self.output_text = {
            0: ', 이해하지 못했습니다. 다시 말씀해주시겠어요?',
            1: ', 안내사항',
            2: ', 버튼을 누르고 목적지를 말씀해주세요',
            3: f', 목적지를 { self.fin_goal }로 설정할까요?',
            4: f', 안내 서비스를 시작합니다. 예상 소요 시간은 약 {self.meter_to_dst/self.speed/60  }분 입니다.',
            5: ', 목적지 변경, 현재 위치 확인, 정지, 중 말씀해주세요',
            6: f' 현재 위치는 {self.next_node} 의 { self.meter_to_node} 미터 앞 입니다.',
            7: ', 속도 조작에 따라 가속하겠습니다.',
            8: ', 속도 조작에 따라 감속하겠습니다.',
            9: ', 보행 중입니다. 주의하여주세요.',
            10: ', 양쪽 손잡이를 꼭 잡아주세요.',
            11: ', 잠시 정지하겠습니다. 주행을 시작하려면 손잡이를 바르게 잡아주세요.',
            12: ', 비상정지합니다. 비상 정지합니다.',
            13: ', 주행이 완료되었습니다. 주차구역에서 대기하겠습니다.',
            14: ', 듣고있어요'
        }

        # 큐 생성
        self.request_queue = deque()

        # 재생 스레드 시작
        self.playback_thread = threading.Thread(target=self.process_queue)
        self.playback_thread.daemon = True
        self.playback_thread.start()
        
        # 비상 정지 이벤트 설정
        self.emergency_stop_event = threading.Event()
        self.talkbutton_on = threading.Event()
        self.is_playing = False  # 현재 음성이 재생 중인지 여부
        self.stop_playing = threading.Event()  # 음성을 중단하는 이벤트

        # 구독자 설정
        self.create_subscription(UInt8, '/driving_status', self.status_callback, 10)
        self.create_subscription(Bool, '/obstacle_warning', self.obstacle_callback, 10)
        self.create_subscription(String, '/handle_status', self.handle_callback, 10)
        self.create_subscription(UInt8, '/speed_change', self.speed_callback, 10)
        self.create_subscription(String, '/current_position', self.position_callback, 10)
        self.create_subscription(String, '/meter_to_node', self.next_node_callback, 10)
        self.create_subscription(String, '/meter_to_dst', self.meter_to_dst_callback, 10)
        self.create_subscription(String, '/final_goal', self.final_goal_callback, 10)
        self.create_subscription(Bool, '/emergency_stop', self.emergency_stop_callback, 10)
        self.create_subscription(Bool, '/talkbutton_state', self.talkbutton_callback, 10)
        self.hmi_stop_pub = self.create_publisher(Bool, '/hmi_stop', 10)
        

    def final_goal_callback(self, msg):
        """최종 목적지 업데이트"""
        self.fin_goal = msg.data
        self.get_logger().info(f"Final goal updated to: {self.fin_goal}")
        self.request_queue.append(self.output_text[3])

    def status_callback(self, msg):
        """주행 상태 업데이트"""
        self.state = msg.data
        self.get_logger().info(f"Updated driving status to: {self.state}")

    def obstacle_callback(self, msg):
        """장애물 감지 상태 업데이트"""
        self.obstacle = msg.data
        self.get_logger().info(f"Obstacle detected: {self.obstacle}")
        if self.state == 1 and self.obstacle:
            self.request_queue.append(self.output_text[9])

    def handle_callback(self, msg):
        """손잡이 상태 업데이트"""
        self.handle_status = msg.data
        self.last_handle_update = time.time()  # 상태 업데이트 시간 기록
        self.get_logger().info(f"Handle status updated to: {self.handle_status}")

    def check_handle_status(self):
        """손잡이 상태를 모니터링하여 알림"""
        elapsed_time = time.time() - self.last_handle_update

        if self.handle_status == 0 and elapsed_time >= 1:
            # 1초 동안 손잡이가 떨어져 있으면 알림
            self.request_queue.append(self.output_text[11])
            self.hmi_stop_pub.publish(True)
            
        elif self.handle_status == 1 and elapsed_time >= 3:
            # 3초 동안 손잡이가 하나만 잡혀 있으면 알림
            self.request_queue.append(self.output_text[10])
            self.hmi_stop_pub.publish(False)
        else:
            self.hmi_stop_pub.publish(False)
    def speed_callback(self, msg):
        """속도 업데이트"""
        if self.speed < msg.data:
            self.request_queue.append(self.output_text[7])
        elif self.speed > msg.data:
            self.request_queue.append(self.output_text[8])
        self.speed = msg.data
        self.get_logger().info(f"Speed updated to: {self.speed}")
    
    #- planning : 이거 메시지로 받아서 처리 추천 
    def position_callback(self, msg):
        """현재 위치 업데이트"""
        self.current_position = msg.data
        self.get_logger().info(f"Current position updated to: {self.current_position}")
        self.request_queue.append(f"현재 위치는 {self.current_position}입니다.")
    
    def meter_to_dst_callback(self, msg):
        """목적지까지 거리 업데이트"""  
        self.meter_to_dst = msg.data
        self.get_logger().info(f"Meter to destination updated to: {self.meter_to_dst}")
        self.request_queue.append(self.output_text[4])
    
    def next_node_callback(self, msg):
        """다음 노드 업데이트"""
        self.next_node = msg.data
        self.get_logger().info(f"Next node updated to: {self.next_node}")
        self.request_queue.append(self.output_text[6]) 
 
    def emergency_stop_callback(self, msg):
        """비상 정지 메시지 수신"""
        if msg.data:
            self.get_logger().info("Emergency stop received, pausing queue processing.")
            self.emergency_stop_event.set()  # 비상 정지 시 이벤트 설정
            self.stop_playing.set()  # 현재 음성을 중단
            self.request_queue.appendleft(self.output_text[12])  # 비상 정지 메시지 출력
            time.sleep(5)  # 비상 정지 후 잠시 대기
            self.stop_playing.clear()  # 비상 정지가 해제된 후 다시 큐 처리 가능
            self.emergency_stop_event.clear()

    def talkbutton_callback(self, msg: Bool):
        """Talk button 상태 처리"""
        if msg.data:
            self.talkbutton_on.set()
            self.get_logger().info("Talk button pressed")
            self.stop_playing.set()  # 현재 음성을 중단
            self.request_queue.appendleft(self.output_text[14])
            self.talkbutton_on.clear()

    def process_queue(self):
        """큐에서 요청을 꺼내 순차적으로 처리"""
        while True:
            if not self.emergency_stop_event.is_set() and not self.stop_playing.is_set():
                if self.request_queue:
                    text = self.request_queue.popleft()  # 큐에서 요청 꺼내기
                    self.text_to_speech(text)

    def text_to_speech(self, text, delay=0.4):
            """텍스트를 음성으로 변환하고 출력, 앞에 임의의 공백 시간(delay)을 추가"""
            try:
                # 임의의 공백 시간만큼 대기
                if delay > 0:
                    print(f"Waiting for {delay} seconds before speaking...")
                    time.sleep(delay)  # delay 초만큼 대기

                file_name = 'output.mp3'
                tts = gTTS(text=text, lang='ko')
                tts.save(file_name)

                print(f"Playing: {text}")
                playsound.playsound(file_name, True)
            except Exception as e:
                print(f"Error in text-to-speech: {e}")
def main(args=None):
    rclpy.init(args=args)

    node = TTS()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
