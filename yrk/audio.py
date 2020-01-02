#!/usr/bin/python
#
# York Robotics Kit Python API
# Version 0.1
# Functions for audio output
# James Hilder, York Robotics Laboratory, Oct 2019

"""
.. module:: audio
   :synopsis: Functions for using the onboard amplifier\speaker

.. moduleauthor:: James Hilder <github.com/jah128>

The YRL040 PCB contains a TPA2005 class-D 1.4W mono audio amplifier.  The audio
output from the Raspberry Pi is sent as a PWM signal over GPIO pin 12 and a
logic-high on GPIO pin 16 enables the amplifier.   PWM audio is relatively poor
in audio quality and inherently noisy, so it is generally preferable to disable
the output when audio is not being played.

The audio module is designed to run continuously and uses a seperate execution
thread to process a queue of stored audio commands.  The audio setup is started
by calling the ``setup_audio`` function.  To add a sound to the playback queue
either ``play_audio_file``, which plays a .mp3 or .wav file, or ``say``, which
uses the ``espeak`` program to read a message.

TPA2005 Data Sheet:
https://www.ti.com/lit/ds/symlink/tpa2005d1.pdf
"""

import RPi.GPIO as GPIO, threading, logging, subprocess, os, time
import yrk.settings as s
from queue import *

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(s.AUDIO_ON_GPIO_PIN,GPIO.OUT,initial=GPIO.LOW)

q=Queue()
running_process = None
audio_killed = False

def set_volume(volume):
    """A function that sets the [PWM] volume output

    Args:
        volume (int): The percentage volume [0-100]

    """

    logging.debug("Setting volume to %d%%" % (volume))
    subprocess.call(["amixer","-q","sset","PCM,0","%d%%" % (volume)])  #Set audio_volume_string to 96 for 100%


def audio_queue_thread():
    """Thread loop for handling queued audio files"""
    global audio_killed
    logging.debug("Audio Queue Thread started")
    while True :
        while not q.empty():
          #Uncomment following to purge all but most recent item in audio queue
          #while q.qsize() > 1:
            #next_function=q.get()
            #q.task_done()
          next_function = q.get()
          func = next_function[0]
          args = next_function[1:]
          logging.debug("Handling next audio job from queue: %s %s" % (func.__name__ ,args , ))
          audio_killed = False
          unmute()
          func(*args)
          while not audio_killed and running_process.poll() is None: time.sleep(0.01)
          logging.debug("Audio task finished - muting audio")
          q.task_done()
          mute()
    time.sleep(0.01)

def start_audio_thread():
    """Function to start the audio thread"""
    audio_thread.start()

def setup_audio():
    """Function to setup the audio system: sets volume, mutes output, starts thread"""
    set_volume(s.AUDIO_VOLUME)
    mute()
    start_audio_thread()

def kill_audio():
    """Function to kill all running audio processes"""
    global audio_killed
    global running_process
    if not running_process is None:
        if running_process.poll() is None:
            logging.info("Killing audio process %d" % running_process.pid)
            running_process.kill()
    audio_killed = True
    running_process = None

def mute():
    """This function mutes the audio output"""
    GPIO.output(s.AUDIO_ON_GPIO_PIN,GPIO.LOW)


def unmute():
    """This function unmutes the audio output.

    Note that audio out uses PWM GPIO which is inherently noisy, CPU activity
    noise should be expected when audio is unmuted.  Where possible, use of the
    play_audio_file() function and queue system is recommended as this mutes
    audio when the queue is empty"""
    GPIO.output(s.AUDIO_ON_GPIO_PIN,GPIO.HIGH)

def IF_say(message):
    subprocess.call(["espeak",message])

def IF_play_audio_file(file):
    global running_process
    filename,ext=os.path.splitext(file)
    #print ext
    if ext=='.mp3': running_process = subprocess.Popen(["mpg123","-q",file])
    else:  running_process = subprocess.Popen(["aplay","-q",file])

def say(message):
    """Function to add a spoken message to the audio Queue

    Args:
        message (str): The message to read out [using e-speak]

    """

    q.put((IF_say,message))

def play_audio_file(file):
    """Function to add an audio file to the audio Queue

    Args:
        file (str): The filename for the [.mp3] or [.wav] audio file

    """

    q.put((IF_play_audio_file,file))

def play_audio_file_background(file):
  t=threading.Thread(target=play_audio_file,args=(file,))
  t.daemon=True
  t.start()

def say_ip():
    """This function will speak IP address using hostname subprocess and
    stored audio files"""
    cmd = "hostname -I | cut -d\' \' -f1"
    IP = subprocess.check_output(cmd, shell = True ).strip()
    ip_string = IP.decode()
    play_audio_file(s.AUDIO_FILEPATH+"ip_address.wav")
    for char in ip_string:
        if char == '1': play_audio_file(s.AUDIO_FILEPATH+"1.wav")
        elif char == '2': play_audio_file(s.AUDIO_FILEPATH+"2.wav")
        elif char == '3': play_audio_file(s.AUDIO_FILEPATH+"3.wav")
        elif char == '4': play_audio_file(s.AUDIO_FILEPATH+"4.wav")
        elif char == '5': play_audio_file(s.AUDIO_FILEPATH+"5.wav")
        elif char == '6': play_audio_file(s.AUDIO_FILEPATH+"6.wav")
        elif char == '7': play_audio_file(s.AUDIO_FILEPATH+"7.wav")
        elif char == '8': play_audio_file(s.AUDIO_FILEPATH+"8.wav")
        elif char == '9': play_audio_file(s.AUDIO_FILEPATH+"9.wav")
        elif char == '0': play_audio_file(s.AUDIO_FILEPATH+"0.wav")
        elif char == '.': play_audio_file(s.AUDIO_FILEPATH+"dot.wav")
    return IP.decode()


audio_thread=threading.Thread(target=audio_queue_thread)
audio_thread.daemon=True

#Command line test [will run when audio.py is run directly]
if __name__ == "__main__":
 s.init()
 s.setup_logger("audio")
 logging.info("YRK Audio Test")
 setup_audio()
 play_audio_file(s.AUDIO_FILEPATH+"york-robotics-kit.wav")
 time.sleep(1.2)
 say_ip()
 time.sleep(10)
 kill_audio()
 os._exit(1)
