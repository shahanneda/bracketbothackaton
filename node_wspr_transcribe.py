#!/usr/bin/env python3

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import paho.mqtt.client as mqtt
from transformers import pipeline
from transformers.pipelines.audio_utils import ffmpeg_microphone_live

# ------------------------------------------------------------------------------------
# Constants & Setup
# ------------------------------------------------------------------------------------
MQTT_BROKER_ADDRESS = "localhost"
MQTT_TOPIC = "robot/drive"

# Create MQTT client
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.connect(MQTT_BROKER_ADDRESS)
client.loop_start()

# Initialize speech recognition
try:
    model = "openai/whisper-tiny.en"
    transcriber = pipeline(
        "automatic-speech-recognition",
        model=model,
        device='cpu',
        model_kwargs={"local_files_only": False}
    )
except Exception as e:
    print(f"Error loading speech model: {e}")
    print("Make sure you have an internet connection and the required packages installed:")
    print("pip install transformers torch")
    sys.exit(1)

def process_command(command):
    command = command.lower().strip()
    if "forward" in command or "ahead" in command:
        client.publish(MQTT_TOPIC, "forward")
    elif "back" in command or "backward" in command:
        client.publish(MQTT_TOPIC, "back")
    elif "left" in command:
        client.publish(MQTT_TOPIC, "left")
    elif "right" in command:
        client.publish(MQTT_TOPIC, "right")
    elif "stop" in command or "halt" in command:
        client.publish(MQTT_TOPIC, "stop")

try:
    print("Voice commands activated...")
    print("Say 'forward', 'back', 'left', 'right', or 'stop'")
    
    sampling_rate = transcriber.feature_extractor.sampling_rate
    mic = ffmpeg_microphone_live(
        sampling_rate=sampling_rate,
        chunk_length_s=5.0,
        stream_chunk_s=2.0,
    )

    while True:
        for item in transcriber(mic, generate_kwargs={"max_new_tokens": 128}):
            sys.stdout.write("\033[K")  # Clear the line
            print(f"Heard: {item['text']}", end="\r")
            if not item["partial"][0]:
                process_command(item["text"])
                print()  # Move to next line when chunk is complete

except KeyboardInterrupt:
    print("\nStopped by user")
except Exception as e:
    print(f"Error: {e}")
finally:
    # Clean up
    client.publish(MQTT_TOPIC, "stop")
    client.loop_stop()
    client.disconnect()
    print("Shutdown complete.")
