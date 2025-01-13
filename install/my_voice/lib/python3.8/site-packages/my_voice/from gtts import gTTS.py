from gtts import gTTS
import playsound 

test = "안녕하세요"

file = "test.mp3"

tts = gTTS(text=test, lang='ko')

playsound.playsound(file, True)

