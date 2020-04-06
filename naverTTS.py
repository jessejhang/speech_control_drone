import os
import sys
import urllib.request

client_id = "your_id"
client_secret = "your_pw"

url = "https://openapi.naver.com/v1/voice/tts.bin"

speakers = [
    'mijin',     #Korean - female
    'jinho',     #Korean - male
    'clara',     #American English - female
    'matt',      #American English - male
    'yuri',      #Japanese - female
    'shinji',    #Japanese - male
    'meimei',    #Chinese - female
    'liangliang',#Chinese - male
    'jose',      #Spanish - female
    'carmen'     #Spanish - male
    ]


tmpPlayPath = './tmp.mp3'

class NaverTTS():
    def __init__(self, speaker=2, speed=0):
        self.speaker = speakers[speaker]
        self.speed=str(speed)
    def play(self, txt):
        encText = urllib.parse.quote(txt)
        data = "speaker=" + self.speaker + "&speed=" + self.speed + "&text=" + encText;

        request = urllib.request.Request(url)
        request.add_header("X-Naver-Client-Id",client_id)
        request.add_header("X-Naver-Client-Secret",client_secret)
        response = urllib.request.urlopen(request, data=data.encode('utf-8'))
        rescode = response.getcode()
        if(rescode==200):
            response_body = response.read()
            with open(tmpPlayPath, 'wb') as f:
                f.write(response_body)

            #vlc
            #os.system('cvlc ' + tmpPlayPath + ' --play-and-exit')

            #omx
            os.system('omxplayer ' + tmpPlayPath)
        else:
            print("Error Code:" + rescode)


def main():
    tts = NaverTTS()
    tts.play("hello")

if __name__ == '__main__':
    main()
