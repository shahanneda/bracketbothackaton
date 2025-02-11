import os
import io
import wave
import threading
import pyaudio
from openai import OpenAI
from dotenv import load_dotenv
from ctypes import *
from contextlib import contextmanager
import paho.mqtt.client as mqtt
import time
import random
import datetime

# Load environment variables
load_dotenv()

# OpenAI client setup
client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

# MQTT setup
MQTT_BROKER_ADDRESS = "localhost"
MQTT_DRIVE_TOPIC = "robot/drive"
MQTT_SPEAK_TOPIC = "robot/speak"
MQTT_MODE_TOPIC = "robot/mode"

mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqtt_client.connect(MQTT_BROKER_ADDRESS)
mqtt_client.loop_start()

word_bank = ["pineapple", "strawberry", "grapefruit", "mandarin", "tangerine", "persimmon"]

indexes = list(range(len(word_bank)))
random.shuffle(indexes)

keywords = []
for i in range(4):
    keywords.append(word_bank[indexes[i]])

mqtt_client.publish(MQTT_SPEAK_TOPIC, "Forward command is {}".format(keywords[0]))
mqtt_client.publish(MQTT_SPEAK_TOPIC, "Backward command is {}".format(keywords[1]))
mqtt_client.publish(MQTT_SPEAK_TOPIC, "Left spin command is {}".format(keywords[2]))
mqtt_client.publish(MQTT_SPEAK_TOPIC, "Right spin command is {}".format(keywords[3]))

# ALSA error suppression
ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)

def py_error_handler(filename, line, function, err, fmt):
    pass

c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)

@contextmanager
def noalsaerr():
    asound = cdll.LoadLibrary("libasound.so")
    asound.snd_lib_error_set_handler(c_error_handler)
    try:
        yield
    finally:
        asound.snd_lib_error_set_handler(None)

# Add this at the top level with other global variables
last_transcription = ""

def process_command(command):
    """Process voice commands and respond with speech"""
    command = command.lower()
    print("command is", command)
    
    if keywords[0] in command:
        mqtt_client.publish(MQTT_MODE_TOPIC, "forward")
        mqtt_client.publish(MQTT_SPEAK_TOPIC, "Switching to forward mode")
        print("forward mode")
    elif keywords[1] in command:
        mqtt_client.publish(MQTT_MODE_TOPIC, "backward")
        mqtt_client.publish(MQTT_SPEAK_TOPIC, "Switching to backward mode")
        print("backward mode")
    elif keywords[2] in command:
        mqtt_client.publish(MQTT_MODE_TOPIC, "left")
        mqtt_client.publish(MQTT_SPEAK_TOPIC, "Switching to left mode")
        print("left mode")
    elif keywords[3] in command:
        mqtt_client.publish(MQTT_MODE_TOPIC, "right")
        mqtt_client.publish(MQTT_SPEAK_TOPIC, "Switching to right mode")
        print("right mode")
    elif "stop" in command or "quit" in command or "abort" in command:
        mqtt_client.publish(MQTT_DRIVE_TOPIC, "stop")
        mqtt_client.publish(MQTT_SPEAK_TOPIC, "Stopping now sir")
        print("stop")
    elif "selfie" in command:
        mqtt_client.publish(MQTT_SPEAK_TOPIC, "its photo time, set your camera for a delay and put your phone in the box")
        time.sleep(10)
        mqtt_client.publish(MQTT_MODE_TOPIC, "selfie")
        print("test before sleep")
        print(datetime.datetime.now())
        time.sleep(5)
        print(datetime.datetime.now())
        print("test after sleep")
        mqtt_client.publish(MQTT_MODE_TOPIC, "forward")
        mqtt_client.publish(MQTT_SPEAK_TOPIC, "done")




def frames_to_wav_bytes(frames, channels, sample_rate, sample_width):
    buffer = io.BytesIO()
    with wave.open(buffer, 'wb') as wf:
        wf.setnchannels(channels)
        wf.setsampwidth(sample_width)
        wf.setframerate(sample_rate)
        wf.writeframes(b''.join(frames))
    buffer.seek(0)
    buffer.name = "audio.wav"
    return buffer

def transcribe_audio(frames, channels, sample_rate, sample_width):
    global last_transcription
    audio_file = frames_to_wav_bytes(frames, channels, sample_rate, sample_width)
    
    print(keywords)

    try:
        result = client.audio.transcriptions.create(
            file=audio_file,
            model="whisper-1",
            response_format="text"
        )
        transcription = result.strip() if isinstance(result, str) else result.text.strip()
        # Only process the command if transcription is not empty and different from last
        if transcription and len(transcription) > 0:
            print("Detected speech:", transcription)
            # Calculate similarity (simple check if the transcription is exactly the same)
            if transcription.lower() != last_transcription.lower():
                process_command(transcription)
                last_transcription = transcription.lower()
            else:
                print("Ignoring repeated command")
        else:
            print("No speech detected")
    except Exception as e:
        print("Error during transcription:", e)

def record_audio():
    CHUNK = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 16000
    RECORD_SECONDS = 3

    with noalsaerr():
        p = pyaudio.PyAudio()
    sample_width = p.get_sample_size(FORMAT)

    stream = p.open(format=FORMAT,
                   channels=CHANNELS,
                   rate=RATE,
                   input=True,
                   frames_per_buffer=CHUNK)

    print("Listening for commands... (Press Ctrl+C to stop)")
    try:
        while True:
            frames = []
            for _ in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
                data = stream.read(CHUNK, exception_on_overflow=False)
                frames.append(data)
            threading.Thread(target=transcribe_audio,
                           args=(frames, CHANNELS, RATE, sample_width)).start()
    except KeyboardInterrupt:
        print("Stopping recording...")
    finally:
        stream.stop_stream()
        stream.close()
        p.terminate()

if __name__ == "__main__":
    try:
        record_audio()
    finally:
        mqtt_client.publish(MQTT_DRIVE_TOPIC, "stop")
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        print("MQTT client shutdown complete.")
