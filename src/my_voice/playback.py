import pyaudio
import wave
from scipy.signal import butter, lfilter
import numpy as np
# 기본 설정
CHUNK = 1024  # 버퍼 크기
FORMAT = pyaudio.paInt16  # 오디오 포맷 (16비트 음성 데이터)
CHANNELS = 1  # 모노
RATE = 44100  # 샘플링 레이트 (Hz)
RECORD_SECONDS = 5  # 녹음 시간 (초)
OUTPUT_FILENAME = "output.wav"  # 저장할 파일 이름
# 필터 설계 함수
def butter_lowpass(cutoff, fs, order=5):
    nyquist = 0.5 * fs  # 나이퀴스트 주파수
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def apply_filter(data, cutoff=8000, fs=44100, order=5):
    """데이터에 저역 통과 필터 적용"""
    b, a = butter_lowpass(cutoff, fs, order=order)
    return lfilter(b, a, data)

def record_audio():
    """오디오를 녹음하고 파일로 저장"""
    p = pyaudio.PyAudio()

    # 입력 스트림 (마이크)
    input_stream = p.open(format=FORMAT,
                          channels=CHANNELS,
                          rate=RATE,
                          input=True,
                          frames_per_buffer=CHUNK)

    print("Recording...")
    frames = []

    # 녹음 데이터 수집
    for _ in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
        data = input_stream.read(CHUNK)
        audio_data = np.frombuffer(data, dtype=np.int16)
        filtered_data = apply_filter(audio_data, cutoff=8000, fs=RATE)
        filtered_data = np.array(filtered_data, dtype=np.int16)
        frames.append(filtered_data.tobytes())

    print("Recording complete. Saving to file...")

    # 스트림 종료
    input_stream.stop_stream()
    input_stream.close()
    p.terminate()

    # 녹음 데이터를 파일로 저장
    with wave.open(OUTPUT_FILENAME, 'wb') as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))

    print(f"Audio saved to {OUTPUT_FILENAME}")

def play_audio():
    """저장된 오디오 파일 재생"""
    p = pyaudio.PyAudio()

    # 파일 열기
    with wave.open(OUTPUT_FILENAME, 'rb') as wf:
        # 출력 스트림 (스피커)
        output_stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                               channels=wf.getnchannels(),
                               rate=wf.getframerate(),
                               output=True)

        print("Playing audio...")
        # 파일에서 데이터 읽고 출력
        data = wf.readframes(CHUNK)
        while data:
            output_stream.write(data)
            data = wf.readframes(CHUNK)

        # 스트림 종료
        output_stream.stop_stream()
        output_stream.close()

    p.terminate()
    print("Playback complete.")

if __name__ == "__main__":
    record_audio()  # 음성 녹음 및 파일 저장
    play_audio()    # 저장된 파일 재생
