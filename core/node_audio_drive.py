#!/usr/bin/env python3

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import paho.mqtt.client as mqtt
import json

# MQTT Settings
MQTT_BROKER_ADDRESS = "localhost"
MQTT_AUDIO_TOPIC = "robot/audio_data"
MQTT_DRIVE_TOPIC = "robot/drive"

# Audio thresholds
VOLUME_THRESHOLD = -30  # dB
PITCH_THRESHOLD_LOW = 200   # Hz
PITCH_THRESHOLD_HIGH = 800  # Hz

def on_connect(client, userdata, flags, rc, properties=None):
    print("Connected to MQTT broker")
    client.subscribe(MQTT_AUDIO_TOPIC)

def on_message(client, userdata, msg):
    try:
        # Parse the JSON message
        data = json.loads(msg.payload.decode())
        volume = data['volume']
        pitch = data['pitch']

        # Decision logic for driving
        if volume > VOLUME_THRESHOLD:
            if pitch < PITCH_THRESHOLD_LOW:
                client.publish(MQTT_DRIVE_TOPIC, "left")
                print(f"LEFT  - Volume: {volume:.1f}dB, Pitch: {pitch:.1f}Hz")
            elif pitch > PITCH_THRESHOLD_HIGH:
                client.publish(MQTT_DRIVE_TOPIC, "right")
                print(f"RIGHT - Volume: {volume:.1f}dB, Pitch: {pitch:.1f}Hz")
            else:
                client.publish(MQTT_DRIVE_TOPIC, "forward")
                print(f"FWD   - Volume: {volume:.1f}dB, Pitch: {pitch:.1f}Hz")
        else:
            client.publish(MQTT_DRIVE_TOPIC, "stop")
            print(f"STOP  - Volume: {volume:.1f}dB, Pitch: {pitch:.1f}Hz", end="\r")

    except json.JSONDecodeError:
        print("Error decoding JSON message")
    except KeyError:
        print("Missing expected data in message")

def main():
    # Setup MQTT client
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        client.connect(MQTT_BROKER_ADDRESS)
        print("Listening for audio data... (Ctrl+C to quit)")
        print("Volume > -30dB: Activate movement")
        print("Pitch < 200Hz: Turn Left")
        print("Pitch > 800Hz: Turn Right")
        print("Pitch 200-800Hz: Go Forward")
        client.loop_forever()

    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        client.publish(MQTT_DRIVE_TOPIC, "stop")
        client.disconnect()
        print("Shutdown complete.")

if __name__ == "__main__":
    main()
