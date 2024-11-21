import time
from threading import Thread
import paho.mqtt.client as mqtt
from elevenlabs.client import ElevenLabs
import os
from io import BytesIO
import sounddevice as sd
import soundfile as sf
import numpy as np
from scipy import signal
import dotenv

dotenv.load_dotenv()

class SpeakThread(Thread):
    def __init__(self, broker_address="localhost", topic="robot/speak"):
        super().__init__()
        self.broker_address = broker_address
        self.topic = topic
        self.client = mqtt.Client()
        
        # Initialize ElevenLabs
        api_key = os.getenv("ELEVENLABS_API_KEY")
        self.elevenlabs = ElevenLabs(api_key=api_key)
        
        # Set up audio device
        self.device_name = "UACDemoV1.0"
        self.device_info = sd.query_devices(self.device_name, 'output')
        self.device_id = self.device_info['index']
        self.device_sample_rate = self.device_info['default_samplerate']

    def set_volume(self, volume_level):
        # Volume level should be between 0 and 100
        os.system(f"amixer sset Master {volume_level}%")

    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected to MQTT broker with result code {rc}")
        client.subscribe(self.topic)

    def on_message(self, client, userdata, msg):
        try:
            text = msg.payload.decode()
            print(f"Speaking: {text}")
            
            # Generate audio
            audio = self.elevenlabs.generate(
                text=text,
                voice="Brian",
                model="eleven_multilingual_v2"
            )
            
            # Convert to playable format
            audio_data = b''.join(audio)
            data, sample_rate = sf.read(BytesIO(audio_data), dtype='float32')

            # Resample if needed
            if sample_rate != self.device_sample_rate:
                number_of_samples = int(round(len(data) * float(self.device_sample_rate) / sample_rate))
                data = signal.resample(data, number_of_samples)
                sample_rate = self.device_sample_rate

            # Play audio
            self.set_volume(100)
            # data = data*4
            sd.play(data, samplerate=sample_rate, device=self.device_id)
            sd.wait()
            
        except Exception as e:
            print(f"Error processing speech: {e}")

    def run(self):
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(self.broker_address)
        self.client.loop_forever()

if __name__ == "__main__":
    speaker = SpeakThread()
    speaker.start()
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping speaker thread...")
