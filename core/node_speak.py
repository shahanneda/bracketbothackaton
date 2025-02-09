#!/usr/bin/env python3

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from dotenv import load_dotenv
from elevenlabs.client import ElevenLabs
from elevenlabs import play
import paho.mqtt.client as mqtt

# Load environment variables and setup ElevenLabs client
load_dotenv()
eleven_labs = ElevenLabs(api_key=os.getenv("ELEVENLABS_API_KEY"))

# MQTT Setup
MQTT_BROKER_ADDRESS = "localhost"
MQTT_TOPIC = "robot/speak"  # Topic to listen for speech messages

# Callback when a message is received
def on_message(client, userdata, message):
    text = message.payload.decode()
    print(f"Speaking: {text}")
    audio = eleven_labs.text_to_speech.convert(
        text=text,
        voice_id="JBFqnCBsd6RMkjVDRZzb",
        model_id="eleven_multilingual_v2",
        output_format="mp3_44100_128",
    )
    play(audio)

# Create and configure MQTT client
mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqtt_client.on_message = on_message
mqtt_client.connect(MQTT_BROKER_ADDRESS)
mqtt_client.subscribe(MQTT_TOPIC)

try:
    print(f"Listening for messages on {MQTT_TOPIC}...")
    mqtt_client.loop_forever()
except KeyboardInterrupt:
    print("\nShutting down...")
finally:
    mqtt_client.disconnect()
    print("Shutdown complete.")