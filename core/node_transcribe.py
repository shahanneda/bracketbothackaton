#!/usr/bin/env python3

import sys
import os
import io
import wave
import pyaudio
import numpy as np
from dotenv import load_dotenv
from openai import OpenAI
import paho.mqtt.client as mqtt
import time

# Load environment variables
load_dotenv()

# ------------------------------------------------------------------------------------
# Constants & Setup
# ------------------------------------------------------------------------------------
MQTT_BROKER_ADDRESS = "localhost"
MQTT_TOPIC = "robot/drive"

# Create MQTT client
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.connect(MQTT_BROKER_ADDRESS)
client.loop_start()

# Initialize OpenAI client
openai_client = OpenAI()

# Audio recording parameters
CHUNK = 1024 * 4
FORMAT = pyaudio.paFloat32
CHANNELS = 1
RATE = 16000
RECORD_SECONDS = 3

# Initialize PyAudio
audio = pyaudio.PyAudio()

def process_command(command):
    command = command.lower().strip()
    if "forward" in command or "ahead" in command:
        print("Moving forward...")
        # client.publish(MQTT_TOPIC, "forward")
    elif "back" in command or "backward" in command:
        print("Moving backward...")
        # client.publish(MQTT_TOPIC, "back")
    elif "left" in command:
        print("Turning left...")
        # client.publish(MQTT_TOPIC, "left")
        # Stop after a brief turn
        time.sleep(2.0)
        # client.publish(MQTT_TOPIC, "stop")
    elif "right" in command:
        print("Turning right...")
        # client.publish(MQTT_TOPIC, "right")
        # Stop after a brief turn
        time.sleep(2)
        print("Stopping...")
        # client.publish(MQTT_TOPIC, "stop")
    elif "stop" in command or "halt" in command:
        print("Stopping...")
        # client.publish(MQTT_TOPIC, "stop")
    elif "quit" in command or "exit" in command or "end" in command:
        print("Stopping...")
        print("\nStopping program...")
        raise KeyboardInterrupt

def record_audio():
    stream = audio.open(format=FORMAT, channels=CHANNELS,
                       rate=RATE, input=True,
                       frames_per_buffer=CHUNK)
    
    print("Listening...", end="\r")
    frames = []
    
    for _ in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
        data = stream.read(CHUNK)
        frames.append(np.frombuffer(data, dtype=np.float32))
    
    stream.stop_stream()
    stream.close()
    
    # Convert float32 array to int16 array
    audio_data = np.concatenate(frames, axis=0)
    audio_data = (audio_data * 32767).astype(np.int16)
    
    # Create a temporary file
    temp_file = io.BytesIO()
    with wave.open(temp_file, 'wb') as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(2)  # 2 bytes for int16
        wf.setframerate(RATE)
        wf.writeframes(audio_data.tobytes())
    
    # Prepare file for OpenAI API
    temp_file.seek(0)
    return {"file": temp_file, "name": "audio.wav"}

try:
    print("Voice commands activated...")
    print("Say 'forward', 'back', 'left', 'right', or 'stop'")
    
    while True:
        # Record audio
        audio_data = record_audio()
        
        # Transcribe with OpenAI Whisper API
        try:
            transcription = openai_client.audio.transcriptions.create(
                model="whisper-1",
                file=("audio.wav", audio_data["file"], "audio/wav"),
                response_format="text"
            )
            
            sys.stdout.write("\033[K")  # Clear the line
            print(f"Heard: {transcription}", end="\r")
            process_command(transcription)
            print()  # Move to next line when chunk is complete
            
        except Exception as e:
            print(f"Transcription error: {e}")

except KeyboardInterrupt:
    print("\nStopped by user")
except Exception as e:
    print(f"Error: {e}")
finally:
    # Clean up
    audio.terminate()
    client.publish(MQTT_TOPIC, "stop")
    client.loop_stop()
    client.disconnect()
    print("Shutdown complete.")
