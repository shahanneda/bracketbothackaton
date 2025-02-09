#!/usr/bin/env python3

import pyaudio
import numpy as np
from scipy.signal import find_peaks
import paho.mqtt.client as mqtt  # Add MQTT import

# Audio parameters (similar to node_transcribe.py)
CHUNK = 2048  # Larger chunk for better frequency resolution
FORMAT = pyaudio.paFloat32
CHANNELS = 1
RATE = 16000

# MQTT settings
MQTT_BROKER_ADDRESS = "localhost"
MQTT_TOPIC = "robot/audio_data"  # Changed topic to reflect audio data

def get_pitch(audio_data, rate):
    """Estimate fundamental frequency using autocorrelation"""
    corr = np.correlate(audio_data, audio_data, mode='full')
    corr = corr[len(corr)//2:]
    
    # Find peaks in correlation
    peaks, _ = find_peaks(corr, distance=20)
    if len(peaks) > 1:
        # Convert sample distance to frequency
        fundamental_freq = rate / (peaks[1] - peaks[0])
        return fundamental_freq
    return 0

def main():
    # Setup MQTT client
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.connect(MQTT_BROKER_ADDRESS)
    client.loop_start()
    
    audio = pyaudio.PyAudio()
    
    stream = audio.open(
        format=FORMAT,
        channels=CHANNELS,
        rate=RATE,
        input=True,
        frames_per_buffer=CHUNK
    )
    
    print("Listening for audio... (Press Ctrl+C to stop)")
    
    try:
        while True:
            # Read audio data
            data = np.frombuffer(stream.read(CHUNK), dtype=np.float32)
            
            # Calculate volume (RMS)
            volume = np.sqrt(np.mean(data**2))
            volume_db = 20 * np.log10(volume + 1e-10)  # Convert to decibels
            
            # Calculate pitch
            pitch = get_pitch(data, RATE)
            
            # Publish audio data as JSON-formatted string
            audio_data = f'{{"volume": {volume_db:.1f}, "pitch": {pitch:.1f}}}'
            client.publish(MQTT_TOPIC, audio_data)
            
            # Create visual representations
            volume_bars = "█" * int((volume_db + 60) // 3)  # Scale to reasonable range
            
            # Clear line and print status
            print("\033[K", end="")  # Clear line
            print(f"Volume: {volume_bars} ({volume_db:.1f} dB) | Pitch: {pitch:.1f} Hz", end="\r")
            
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        # Cleanup
        stream.stop_stream()
        stream.close()
        audio.terminate()
        client.loop_stop()
        client.disconnect()

if __name__ == "__main__":
    main()
