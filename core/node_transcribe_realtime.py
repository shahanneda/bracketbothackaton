import os
import json
import base64
import asyncio
import websockets
import pyaudio
import wave
import struct
from dotenv import load_dotenv
import sys
import subprocess
import select
import time

# Load the .env file into memory so the code has access to the key
load_dotenv()

# PyAudio configuration
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100

def move_robot(direction):
    print(f"Moving {direction}")
    # Implement the actual movement logic here

def start_recording(debug_save=True):
    print("Speak to send a message to the assistant. Press Enter when done.")
    p = pyaudio.PyAudio()
    stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)
    
    frames = []
    try:
        print("Recording... (Press Enter to stop)")
        while True:
            data = stream.read(CHUNK)
            frames.append(data)
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                line = input()
                break
    finally:
        stream.stop_stream()
        stream.close()
        p.terminate()
    
    audio_data = b''.join(frames)
    
    if debug_save:
        # Save the audio to a file
        debug_file = "debug_audio.wav"
        with wave.open(debug_file, 'wb') as wf:
            wf.setnchannels(CHANNELS)
            wf.setsampwidth(p.get_sample_size(FORMAT))
            wf.setframerate(RATE)
            wf.writeframes(audio_data)
        print(f"Debug: Audio saved to {debug_file}")
        print(f"Debug: Audio file size: {os.path.getsize(debug_file)} bytes")
        print("Playing back recorded audio...")
        
        # Play the audio using aplay
        try:
            subprocess.run(["aplay", debug_file], check=True)
            print("Playback finished.")
        except subprocess.CalledProcessError as e:
            print(f"Error playing audio: {e}")

    # Ensure audio_data is in little-endian format
    if sys.byteorder == 'big':
        audio_data = audio_data[::-1]
    
    base64_audio = base64.b64encode(audio_data).decode('utf-8')
    print("Finished recording.")
    return base64_audio


async def process_response(message):
    data = json.loads(message)
    if "type" in data and data["type"] == "conversation.item.created":
        response_text = data.get("item", {}).get("text", "").lower()
        if "forward" in response_text:
            move_robot("forward")
        elif "backward" in response_text:
            move_robot("backward")
        elif "left" in response_text:
            move_robot("left")
        elif "right" in response_text:
            move_robot("right")

async def main():
    url = "wss://api.openai.com/v1/realtime?model=gpt-4o-realtime-preview-2024-10-01"
    headers = {
        "Authorization": f"Bearer {os.getenv('OPENAI_API_KEY')}",
        "OpenAI-Beta": "realtime=v1"
    }

    async with websockets.connect(url, additional_headers=headers) as ws:
        while True:
            base64_audio_data = start_recording(debug_save=False)

            create_conversation_event = {
                "type": "conversation.item.create",
                "item": {
                    "type": "message",
                    "role": "user",
                    "content": [
                        {"type": "input_audio", "audio": base64_audio_data}
                    ],
                }
            }
            print("Sending audio data...")
            await ws.send(json.dumps(create_conversation_event))
            print("Audio data sent.")

            async for message in ws:
                await process_response(message)
                break

    print("Conversation ended.")

if __name__ == "__main__":
    asyncio.run(main())
