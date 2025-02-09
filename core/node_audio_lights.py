#!/usr/bin/env python3

import paho.mqtt.client as mqtt
import json
from pi5neo import Pi5Neo
import time

# LED Configuration
MAX_LEDS = 15
LED_DEVICE_PATH = "/dev/spidev0.0"
LED_BAUDRATE = 800

# MQTT Configuration
MQTT_BROKER_ADDRESS = "localhost"
MQTT_TOPIC = "robot/audio_data"

# Visualization parameters
ALPHA = 0.4  # Smoothing factor for volume changes

class AudioLightDisplay:
    def __init__(self):
        self.neo = Pi5Neo(LED_DEVICE_PATH, MAX_LEDS, LED_BAUDRATE)
        self.smoothed_volume = -60  # Start at -60 dB (very quiet)
        
        # Setup MQTT client
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_message = self.on_message
        self.client.connect(MQTT_BROKER_ADDRESS)
        self.client.subscribe(MQTT_TOPIC)

    def on_message(self, client, userdata, message):
        try:
            # Parse the JSON message
            data = json.loads(message.payload.decode())
            volume_db = data['volume']
            
            # Smooth the volume changes
            self.smoothed_volume = (ALPHA * volume_db + 
                                  (1 - ALPHA) * self.smoothed_volume)
            
            # Map volume to LED display (-60dB to -20dB range to 0-15 LEDs)
            volume_normalized = (self.smoothed_volume + 60) / 40  # Normalize to 0-1 (40dB range)
            volume_normalized = max(0, min(1, volume_normalized))  # Clamp to 0-1
            num_leds = int(volume_normalized * MAX_LEDS)
            
            # Update LED strip
            self.neo.clear_strip()
            for i in range(num_leds):
                # Create a color gradient from green to red
                if i < MAX_LEDS * 0.6:  # First 60% are green
                    color = (0, 255, 0)
                elif i < MAX_LEDS * 0.8:  # Next 20% are yellow
                    color = (255, 255, 0)
                else:  # Last 20% are red
                    color = (255, 0, 0)
                self.neo.set_led_color(i, *color)
            self.neo.update_strip()
            
        except json.JSONDecodeError:
            print("Error decoding JSON message")
        except KeyError:
            print("Missing volume data in message")

    def run(self):
        print("Starting audio light display... Press Ctrl+C to stop")
        self.client.loop_forever()

    def cleanup(self):
        self.neo.clear_strip()
        self.neo.update_strip()
        self.client.disconnect()

def main():
    display = AudioLightDisplay()
    try:
        display.run()
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        display.cleanup()

if __name__ == "__main__":
    main()
