# -*- coding: utf-8 -*-

from __future__ import division

import re
import sys
import os
import RPi.GPIO as GPIO
import time


GPIO.setmode(GPIO.BOARD)

Modeswitch=33   #0
Roll=35         #1
Yaw=36          #2
Pitch=37        #3
Throttle=38     #4


GPIO.setup(Roll,GPIO.OUT)
GPIO.setup(Pitch,GPIO.OUT)
GPIO.setup(Throttle,GPIO.OUT)
GPIO.setup(Yaw,GPIO.OUT)
GPIO.setup(Modeswitch,GPIO.OUT)

THROTTLE=GPIO.PWM(Throttle,50)
YAW=GPIO.PWM(Yaw,50)
ROLL=GPIO.PWM(Throttle,50)
PITCH=GPIO.PWM(Yaw,50)

GPIO.output(Modeswitch,True)

ROLL.start(7.5)
PITCH.start(7.5)
THROTTLE.start(5)
YAW.start(7.5)

from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types
import pyaudio
from six.moves import queue

import naverTTS


tts = naverTTS.NaverTTS()
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = "/home/pi/your_API_json"

cmdLists = [
        #Me                         Drone                       Return
        [u'hello',                  'Nice to meet you.',         1],
        [u'stand by',               'Okay, Standby.',           2],
        [u'take off',                'Okay, Take off.',            3],
        [u'go straight',            'Okay, Go straight.',      4],
        [u'go backward',              'Okay, Go backward.',          5],
        [u'turn left', 'Okay, Turn left.', 6],
        [u'turn right', 'Okay, Turn right.', 7],
        [u'land', 'Okay, Land.', 8],
        [u'turn up', 'Okay, Turn up.', 9],
        [u'turn down', 'Okay, Turn down.', 10],
        [u'dismiss', 'See you next time.', 11]]


# Audio recording parameters
RATE = 48000
CHUNK = int(RATE / 30)  # 100ms

class MicrophoneStream(object):
    def __init__(self, rate, chunk):
        self._rate = rate
        self._chunk = chunk
        
        # Create a thread-safe buffer of audio data
        self._buff = queue.Queue()
        self.closed = True
        self.isPause = False

    def __enter__(self):
        self._audio_interface = pyaudio.PyAudio()
        self._audio_stream = self._audio_interface.open(
            format=pyaudio.paInt16,
            # The API currently only supports 1-channel (mono) audio
            # https://goo.gl/z757pE
            channels=1, rate=self._rate,
            input=True, frames_per_buffer=self._chunk,
            # Run the audio stream asynchronously to fill the buffer object.
            # This is necessary so that the input device's buffer doesn't
            # overflow while the calling thread makes network requests, etc.
            stream_callback=self._fill_buffer,
        )

        self.closed = False

        return self

    def __exit__(self, type, value, traceback):
        self._audio_stream.stop_stream()
        self._audio_stream.close()


        self.closed = True
        # Signal the generator to terminate so that the client's
        # streaming_recognize method will not block the process termination.
        self._buff.put(None)
        self._audio_interface.terminate()

    def _fill_buffer(self, in_data, frame_count, time_info, status_flags):
        self._buff.put(in_data)
        return None, pyaudio.paContinue

    def generator(self):
        while not self.closed:
            # Use a blocking get() to ensure there's at least one chunk of
            # data, and stop iteration if the chunk is None, indicating the
            # end of the audio stream.
            chunk = self._buff.get()
            if chunk is None:
                return

            if self.isPause:
                continue

            data = [chunk]

            # Now consume whatever other data's still buffered.
            while True:
                try:
                    
                    chunk = self._buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except queue.Empty:
                    break

            yield b''.join(data)


    def pause(self):
        self.isPause = True


    def restart(self):
        self.isPause = False
# [END audio_stream]


def CommandProc(stt):
    cmd = stt.strip()

    print('ME : ' + str(cmd))

   
    for cmdList in cmdLists:
    
        if str(cmd) == cmdList[0]:

            tts.play(cmdList[1])
            print ('E2D : ' + cmdList[1])


            duty_MAX = 10
            duty_MIN = 5
            duty_MID = 7.5
            duty4hover_THRO = 7.5
            duty4hover_YAW = 7.5
            duty4hover_PITCH = 7.5

            global duty_THRO
            global duty_YAW
            global duty_PITCH
            global duty_ROLL

            if cmdList[2] == 1: #Hello

                # Control
                duty_THRO = duty_MIN
                duty_YAW = duty_MAX
                duty_PITCH = duty_MID

                THROTTLE.ChangeDutyCycle(duty_THRO)
                time.sleep(1)
                YAW.ChangeDutyCycle(duty_YAW)

                duty_YAW = duty4hover_YAW
                time.sleep(3)
                YAW.ChangeDutyCycle(duty_YAW)

                # Print
                print('Cmd Index =', cmdList[2])
                print('Current Throttle Duty =', duty_THRO)
                print('Current duty =', duty_YAW)
                print('Current duty =', duty_PITCH)

                # Not Hovering

            elif cmdList[2] == 2: #Standby

                # Control
                inc = 1.5
                ms = 1000
                for a in range(ms):
                    duty_THRO = duty_THRO + inc/ms
                    THROTTLE.ChangeDutyCycle(duty_THRO)
                    YAW.ChangeDutyCycle(duty_YAW)
                    PITCH.ChangeDutyCycle(duty_PITCH)

                # Result
                print('Cmd Index =', cmdList[2])
                print('Current Throttle Duty =', duty_THRO)
                print('Current duty =', duty_YAW)
                print('Current duty =', duty_PITCH)

                # Not Hovering

            elif cmdList[2] == 3: #Take off

                # Control
                inc = 1.5
                dec = 0.5
                ms = 300
                for a in range(ms):
                    duty_THRO  += inc/ms
                    duty_YAW   = duty4hover_YAW
                    duty_PITCH = duty4hover_PITCH
                for a in range(ms):
                    duty_THRO  -= dec/ms
                    duty_YAW   = duty4hover_YAW
                    duty_PITCH = duty4hover_PITCH

                duty_THRO  = duty4hover_THRO
                duty_YAW   = duty4hover_YAW
                duty_PITCH = duty4hover_PITCH
                THROTTLE.ChangeDutyCycle(duty_THRO)
                YAW.ChangeDutyCycle(duty_YAW)
                PITCH.ChangeDutyCycle(duty_PITCH)

                # Result
                print('Cmd Index =', cmdList[2])
                print('Current Throttle Duty =', duty_THRO)
                print('Current duty =', duty_YAW)
                print('Current duty =', duty_PITCH)

                # Hovering
                ms = 200
                for a in range(ms):
                    duty_THRO = duty_THRO + (duty4hover_THRO-duty_THRO)/200
                    duty_YAW = duty_YAW + (duty4hover_YAW-duty_YAW)/200
                    duty_PITCH = duty_PITCH + (duty4hover_PITCH-duty_PITCH)/200

                duty_THRO  = duty4hover_THRO
                duty_YAW   = duty4hover_YAW
                duty_PITCH = duty4hover_PITCH
                THROTTLE.ChangeDutyCycle(duty_THRO)
                YAW.ChangeDutyCycle(duty_YAW)
                PITCH.ChangeDutyCycle(duty_PITCH)

                # Hovering Result
                print('Cmd Index =', cmdList[2])
                print('Current Throttle Duty =', duty_THRO)
                print('Current duty =', duty_YAW)
                print('Current duty =', duty_PITCH)


            elif cmdList[2] == 4: #Go straight

                # Control
                inc = 0.3
                ms = 200
                for a in range(ms):
                    duty_PITCH =  duty_PITCH + inc/ms
                    THROTTLE.ChangeDutyCycle(duty_THRO)
                    YAW.ChangeDutyCycle(duty_YAW)
                    PITCH.ChangeDutyCycle(duty_PITCH)

                time.sleep(3)

                # Result
                print('Cmd Index =', cmdList[2])
                print('Current Throttle Duty =', duty_THRO)
                print('Current duty =', duty_YAW)
                print('Current duty =', duty_PITCH)

                # Hovering
                ms = 200
                for a in range(ms):
                    duty_THRO = duty_THRO + (duty4hover_THRO-duty_THRO)/ms
                    duty_YAW = duty_YAW + (duty4hover_YAW-duty_YAW)/ms
                    duty_PITCH = duty_PITCH + (duty4hover_PITCH-duty_PITCH)/ms

                duty_THRO  = duty4hover_THRO
                duty_YAW   = duty4hover_YAW
                duty_PITCH = duty4hover_PITCH
                THROTTLE.ChangeDutyCycle(duty_THRO)
                YAW.ChangeDutyCycle(duty_YAW)
                PITCH.ChangeDutyCycle(duty_PITCH)
                
                # Hovering Result
                print('Cmd Index =', cmdList[2])
                print('Current Throttle Duty =', duty_THRO)
                print('Current duty =', duty_YAW)
                print('Current duty =', duty_PITCH)


            elif cmdList[2] == 5: #Go backward

                # Control
                inc = -0.3
                ms = 200
                for a in range(ms):
                    duty_PITCH = duty_PITCH + inc / ms
                    THROTTLE.ChangeDutyCycle(duty_THRO)
                    YAW.ChangeDutyCycle(duty_YAW)
                    PITCH.ChangeDutyCycle(duty_PITCH)

                time.sleep(3)

                # Result
                print('Cmd Index =', cmdList[2])
                print('Current Throttle Duty =', duty_THRO)
                print('Current duty =', duty_YAW)
                print('Current duty =', duty_PITCH)

                # Hovering
                ms = 200
                for a in range(ms):
                    duty_THRO = duty_THRO + (duty4hover_THRO-duty_THRO)/ms
                    duty_YAW = duty_YAW + (duty4hover_YAW-duty_YAW)/ms
                    duty_PITCH = duty_PITCH + (duty4hover_PITCH-duty_PITCH)/ms

                duty_THRO  = duty4hover_THRO
                duty_YAW   = duty4hover_YAW
                duty_PITCH = duty4hover_PITCH
                THROTTLE.ChangeDutyCycle(duty_THRO)
                YAW.ChangeDutyCycle(duty_YAW)
                PITCH.ChangeDutyCycle(duty_PITCH)
                
                # Hovering Result
                print('Cmd Index =', cmdList[2])
                print('Current Throttle Duty =', duty_THRO)
                print('Current duty =', duty_YAW)
                print('Current duty =', duty_PITCH)

            elif cmdList[2] == 6: #Turn left

                # Control
                inc = (duty_MAX - duty_MIN ) / 4
                ms = 200
                for a in range(ms):
                    duty_YAW = duty_YAW + inc / ms
                    THROTTLE.ChangeDutyCycle(duty_THRO)
                    YAW.ChangeDutyCycle(duty_YAW)
                    PITCH.ChangeDutyCycle(duty_PITCH)

                dec = -inc
                ms = 200
                for a in range(ms):
                    duty_YAW = duty_YAW + dec / ms
                    THROTTLE.ChangeDutyCycle(duty_THRO)
                    YAW.ChangeDutyCycle(duty_YAW)
                    PITCH.ChangeDutyCycle(duty_PITCH)

                time.sleep(2)
                
                for a in range(ms):
                    duty_YAW = duty_YAW + (duty4hover_YAW - duty_YAW) / ms
                    THROTTLE.ChangeDutyCycle(duty_THRO)
                    YAW.ChangeDutyCycle(duty_YAW)
                    PITCH.ChangeDutyCycle(duty_PITCH)

                for a in range(ms):
                    duty_PITCH = duty_PITCH + inc / ms
                    THROTTLE.ChangeDutyCycle(duty_THRO)
                    YAW.ChangeDutyCycle(duty_YAW)
                    PITCH.ChangeDutyCycle(duty_PITCH)

                time.sleep(3)

                # Result
                print('Cmd Index =', cmdList[2])
                print('Current Throttle Duty =', duty_THRO)
                print('Current duty =', duty_YAW)
                print('Current duty =', duty_PITCH)

                # Hovering
                ms = 200
                for a in range(ms):
                    duty_THRO = duty_THRO + (duty4hover_THRO-duty_THRO)/ms
                    duty_YAW = duty_YAW + (duty4hover_YAW-duty_YAW)/ms
                    duty_PITCH = duty_PITCH + (duty4hover_PITCH-duty_PITCH)/ms
                
                duty_THRO  = duty4hover_THRO
                duty_YAW   = duty4hover_YAW
                duty_PITCH = duty4hover_PITCH
                THROTTLE.ChangeDutyCycle(duty_THRO)
                YAW.ChangeDutyCycle(duty_YAW)
                PITCH.ChangeDutyCycle(duty_PITCH)

                # Hovering Result
                print('Cmd Index =', cmdList[2])
                print('Current Throttle Duty =', duty_THRO)
                print('Current duty =', duty_YAW)
                print('Current duty =', duty_PITCH)



            elif cmdList[2] == 7: #Turn right

                # Control
                inc = -(duty_MAX - duty_MIN) / 4
                ms = 200
                for a in range(ms):
                    duty_YAW = duty_YAW + inc / ms
                    THROTTLE.ChangeDutyCycle(duty_THRO)
                    YAW.ChangeDutyCycle(duty_YAW)
                    PITCH.ChangeDutyCycle(duty_PITCH)

                dec = -inc
                for a in range(ms):
                    duty_YAW = duty_YAW + dec / ms
                    THROTTLE.ChangeDutyCycle(duty_THRO)
                    YAW.ChangeDutyCycle(duty_YAW)
                    PITCH.ChangeDutyCycle(duty_PITCH)

                time.sleep(3)

                for a in range(ms):
                    duty_YAW = duty_YAW + (duty4hover_YAW - duty_YAW) / ms
                    THROTTLE.ChangeDutyCycle(duty_THRO)
                    YAW.ChangeDutyCycle(duty_YAW)
                    PITCH.ChangeDutyCycle(duty_PITCH)

                for a in range(ms):
                    duty_PITCH = duty_PITCH + inc / ms
                    THROTTLE.ChangeDutyCycle(duty_THRO)
                    YAW.ChangeDutyCycle(duty_YAW)
                    PITCH.ChangeDutyCycle(duty_PITCH)

                time.sleep(3)

                # Result
                print('Cmd Index =', cmdList[2])
                print('Current Throttle Duty =', duty_THRO)
                print('Current duty =', duty_YAW)
                print('Current duty =', duty_PITCH)

                # Hovering
                ms = 200
                for a in range(ms):
                    duty_THRO = duty_THRO + (duty4hover_THRO-duty_THRO)/ms
                    duty_YAW = duty_YAW + (duty4hover_YAW-duty_YAW)/ms
                    duty_PITCH = duty_PITCH + (duty4hover_PITCH-duty_PITCH)/ms


                duty_THRO  = duty4hover_THRO
                duty_YAW   = duty4hover_YAW
                duty_PITCH = duty4hover_PITCH
                THROTTLE.ChangeDutyCycle(duty_THRO)
                YAW.ChangeDutyCycle(duty_YAW)
                PITCH.ChangeDutyCycle(duty_PITCH)
                
                # Hovering Result
                print('Cmd Index =', cmdList[2])
                print('Current Throttle Duty =', duty_THRO)
                print('Current duty =', duty_YAW)
                print('Current duty =', duty_PITCH)


            elif cmdList[2] == 8: #land

                # Control
                dec = -1.5
                ms = 1000
                for a in range(ms):
                    duty_THRO = duty_THRO + dec / ms
                    THROTTLE.ChangeDutyCycle(duty_THRO)
                    YAW.ChangeDutyCycle(duty_YAW)
                    PITCH.ChangeDutyCycle(duty_PITCH)

                    THROTTLE.ChangeDutyCycle(duty_THRO)
                    YAW.ChangeDutyCycle(duty_YAW)
                    PITCH.ChangeDutyCycle(duty_PITCH)
                    
                # Result
                print('Cmd Index =', cmdList[2])
                print('Current Throttle Duty =', duty_THRO)
                print('Current duty =', duty_YAW)
                print('Current duty =', duty_PITCH)

                # Not Hovering


            elif cmdList[2] == 9: #Turn up
                inc = 1
                ms = 100
                for a in range(ms):
                    duty_THRO = duty_THRO + inc/ms
                    THROTTLE.ChangeDutyCycle(duty_THRO)

                # Result
                print('Cmd Index =', cmdList[2])
                print('Current Throttle Duty =', duty_THRO)
                print('Current duty =', duty_YAW)
                print('Current duty =', duty_PITCH)

                # Hovering
                for a in range(ms):
                    duty_THRO = duty_THRO + (duty4hover_THRO-duty_THRO)/ms
                    duty_YAW = duty_YAW + (duty4hover_YAW-duty_YAW)/ms
                    duty_PITCH = duty_PITCH + (duty4hover_PITCH-duty_PITCH)/ms
                    
                duty_THRO  = duty4hover_THRO
                duty_YAW   = duty4hover_YAW
                duty_PITCH = duty4hover_PITCH
                THROTTLE.ChangeDutyCycle(duty_THRO)
                YAW.ChangeDutyCycle(duty_YAW)
                PITCH.ChangeDutyCycle(duty_PITCH)

                # Not Hovering

            elif cmdList[2] == 10: #Turn down
                dec = -1
                ms = 100
                for a in range(ms):
                    duty_THRO = duty_THRO + dec / ms
                    THROTTLE.ChangeDutyCycle(duty_THRO)

                # Result
                print('Cmd Index =', cmdList[2])
                print('Current Throttle Duty =', duty_THRO)
                print('Current duty =', duty_YAW)
                print('Current duty =', duty_PITCH)

                # Not Hovering


            elif cmdList[2] == 11:  # Dismiss

                THROTTLE.ChangeDutyCycle(duty_MIN)
                YAW.ChangeDutyCycle(duty_MIN)
                PITCH.ChangeDutyCycle(duty_MID)

                # Result
                print('Cmd Index =', cmdList[2])
                print('Current Throttle Duty =', duty_THRO)
                print('Current duty =', duty_YAW)
                print('Current duty =', duty_PITCH)

                # Not Hovering

            return cmdList[2]

    # If not command is in unicode
    # Error
    print ("E2D : Sorry, I don't understand")

    tts.play("Sorry, I don't understand")
    return 1


def listen_print_loop(responses, mic):
    
    num_chars_printed = 0
    for response in responses:
        if not response.results:
            continue

        # There could be multiple results in each response.
        result = response.results[0]
        if not result.alternatives:
            continue

        # Display the transcription of the top alternative.
        transcript = result.alternatives[0].transcript

        # Display interim results, but with a carriage return at the end of the
        # line, so subsequent lines will overwrite them.
        
        # If the previous result was longer than this one, we need to print
        # some extra spaces to overwrite the previous result
        overwrite_chars = ' ' * (num_chars_printed - len(transcript))

        if not result.is_final:

            #sys.stdout.write('나 : ')
            #sys.stdout.write(transcript + overwrite_chars + '\r')
            #sys.stdout.flush()
            num_chars_printed = len(transcript)

        else:

            mic.pause()

            if CommandProc(transcript) == 0:
                break;

            mic.restart()
           
            num_chars_printed = 0

def main():
    # See http://g.co/cloud/speech/docs/languages
    # for a list of supported languages.
    language_code = 'en-US'  # a BCP-47 language tag
    #language_code = 'ko-KR'  # a BCP-47 language tag

    client = speech.SpeechClient()
    config = types.RecognitionConfig(
        encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=RATE,
        language_code=language_code)
    streaming_config = types.StreamingRecognitionConfig(
        config=config,
        interim_results=True)

    with MicrophoneStream(RATE, CHUNK) as stream:
        audio_generator = stream.generator()
        requests = (types.StreamingRecognizeRequest(audio_content=content)
                    for content in audio_generator)

        responses = client.streaming_recognize(streaming_config, requests)

        # Now, put the transcription responses to use.
        listen_print_loop(responses, stream)

if __name__ == '__main__':
    main()
