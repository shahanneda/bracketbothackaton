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
MQTT_MODE_TOPIC = "robot/mode"  # New topic for mode changes

# Audio thresholds
VOLUME_THRESHOLD = -30  # dB
PITCH_THRESHOLD_LOW = 200   # Hz
PITCH_THRESHOLD_HIGH = 800  # Hz

# Add this after MQTT settings
current_mode = "forward"  # Default mode

def on_connect(client, userdata, flags, rc, properties=None):
    print("Connected to MQTT broker")
    client.subscribe(MQTT_AUDIO_TOPIC)
    client.subscribe(MQTT_MODE_TOPIC)  # Subscribe to mode topic instead of drive topic

def on_message(client, userdata, msg):
    global current_mode
    
    if msg.topic == MQTT_MODE_TOPIC:  # Changed from MQTT_DRIVE_TOPIC
        # Handle mode changes
        new_mode = msg.payload.decode()
        if new_mode in ["forward", "backward", "left", "right", "selfie"]:

            current_mode = new_mode
            print(f"\nMode changed to: {current_mode}")
        return

    try:
        # Parse the JSON message
        data = json.loads(msg.payload.decode())
        volume = data['volume']

        if current_mode == "selfie":
            client.publish(MQTT_DRIVE_TOPIC, "back")  # Changed to match node_drive.py's command
            print(f"BACK  - SELFIE", end="\r")
        # Decision logic based on current mode and volume
        elif volume > VOLUME_THRESHOLD:
            if current_mode == "forward":
                client.publish(MQTT_DRIVE_TOPIC, "forward")
                print(f"FWD   - Volume: {volume:.1f}dB, Mode: {current_mode}", end="\r")
            elif current_mode == "backward":
                client.publish(MQTT_DRIVE_TOPIC, "back")  # Changed to match node_drive.py's command
                print(f"BACK  - Volume: {volume:.1f}dB, Mode: {current_mode}", end="\r")
            elif current_mode == "left":
                client.publish(MQTT_DRIVE_TOPIC, "left")
                print(f"LEFT  - Volume: {volume:.1f}dB, Mode: {current_mode}", end="\r")
            elif current_mode == "right":
                client.publish(MQTT_DRIVE_TOPIC, "right")
                print(f"RIGHT - Volume: {volume:.1f}dB, Mode: {current_mode}", end="\r")
        else:
            client.publish(MQTT_DRIVE_TOPIC, "stop")
            print(f"STOP  - Volume: {volume:.1f}dB, Mode: {current_mode}", end="\r")

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
        print("Volume > -30dB: Activate movement in current mode")
        print(f"Current mode: {current_mode}")
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
