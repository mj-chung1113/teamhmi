import threading
import time
from collections import deque
from gtts import gTTS
import playsound

class RealTimeTTS:
    def __init__(self):
        self.test = "unknown"
        # 상태별 출력 문구
        self.output_text = {
            
            0: ', 이해하지 못했습니다. 다시 말씀해주시겠어요?',
            1: ', 안내사항',
            2: ', botton을 누르고 목적지를 말씀해주세요',
            3: f', 목적지를 { self.test }로 설정할까요?',
            4: f', 안내 서비스를 시작합니다. 예상 소요 시간은 약 {10}분 입니다.',
            5: ', 목적지 변경, 현재 위치 확인, 정지, 중 말씀해주세요',
            6: f' 현재 위치는 { "다음 노드" } 의 { 100 } 미터 앞 입니다.',
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

    def process_queue(self):
        """큐에서 요청을 꺼내 순차적으로 처리"""
        while True:
            if self.request_queue:
                text = self.request_queue.popleft()  # 큐에서 요청 꺼내기
                self.text_to_speech(text)

    def text_to_speech(self, text, delay=0.3):
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
                
    def add_to_queue(self, message_id):
        """큐에 메시지 추가"""
        if message_id in self.output_text:
            # 13번과 14번 메시지는 큐 앞에 추가
            if message_id in [13, 14]:
                self.request_queue.appendleft(self.output_text[message_id])
            else:
                self.request_queue.append(self.output_text[message_id])
        else:
            print("Invalid message ID")

def main():
    tts_system = RealTimeTTS()
    
    while True:
        # 사용자 입력 받기
        try:
            message_id = int(input("메시지 번호를 입력하세요 (0-14): "))
            tts_system.add_to_queue(message_id)
        except ValueError:
            print("유효하지 않은 입력입니다. 숫자를 입력하세요.")

if __name__ == '__main__':
    main()
