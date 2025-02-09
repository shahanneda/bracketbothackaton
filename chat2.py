import os
import json
import base64
import asyncio
import websockets
import pyaudio
import wave
import tempfile
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

class AudioPlayer:
    def __init__(self, channels=1, rate=16000, width=2):
        self.p = pyaudio.PyAudio()
        self.channels = channels
        self.rate = rate
        self.width = width
        self.stream = None
        self.buffer = b''

    def play(self, audio_chunk):
        if self.stream is None:
            self.stream = self.p.open(format=self.p.get_format_from_width(self.width),
                                      channels=self.channels,
                                      rate=self.rate,
                                      output=True,
                                      stream_callback=self._callback)
            self.stream.start_stream()

        self.buffer += audio_chunk

    def _callback(self, in_data, frame_count, time_info, status):
        data = self.buffer[:frame_count*self.channels*self.width]
        self.buffer = self.buffer[frame_count*self.channels*self.width:]
        return (data, pyaudio.paContinue)

    def close(self):
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        self.p.terminate()

    def get_buffer_duration(self):
        return len(self.buffer) / (self.channels * self.width * self.rate)

# --- Function to be called by GPT when user requests movement ---
def move(direction: str):
    # In a real scenario, you'd interface with your robot's movement logic here.
    # For this example, let's just print a placeholder.
    print(f"\n=== MOVING ROBOT {direction.upper()} ===\n")

async def main():
    url = "wss://api.openai.com/v1/realtime?model=gpt-4o-realtime-preview-2024-10-01"
    headers = {
        "Authorization": f"Bearer {os.getenv('OPENAI_API_KEY')}",
        "OpenAI-Beta": "realtime=v1"
    }

    async with websockets.connect(
        url,
        additional_headers=headers
    ) as ws:
        while True:
            base64_audio_data = start_recording(debug_save=False)

            create_conversation_event = {
                "type": "conversation.item.create",
                "item": {
                    "type": "message",
                    "role": "user",
                    "content": [
                        {
                            "type": "input_audio",
                            "audio": base64_audio_data,
                        }
                    ],
                }
            }
            print("Sending audio data...")
            await ws.send(json.dumps(create_conversation_event))
            print("Audio data sent.")

            # Wait for server response that the item was created
            async for message in ws:
                data = json.loads(message)
                if data["type"] == "conversation.item.created":
                    break

            # Now request a response with function calling capability
            create_response_event = {
                "type": "response.create",
                "response": {
                    "modalities": ["text", "audio"],
                    "instructions": "You are neda-bot, a helpful home robot. You are currently in a room with a user. You are tasked with helping the user with their request. Keep your responses concise and to the point. Be a little sarcastic and funny, but not too much. Dont say that you are sarcastic. Make sure to say your name in your introduction.",
                },
                "functions": [
                    {
                        "name": "move",
                        "description": "Moves the robot in one of four directions.",
                        "parameters": {
                            "type": "object",
                            "properties": {
                                "direction": {
                                    "type": "string",
                                    "description": "The direction to move in.",
                                    "enum": ["forward", "backward", "left", "right"]
                                }
                            },
                            "required": ["direction"]
                        }
                    }
                ]
            }
            print("Requesting response...")
            await ws.send(json.dumps(create_response_event))

            audio_player = AudioPlayer(channels=1, rate=32000, width=2)
            try:
                total_audio_duration = 0
                start_time = None

                async for message in ws:
                    data = json.loads(message)

                    if data["type"] == "response.audio.delta":
                        # This is part of the assistant's audio output.
                        audio_chunk = base64.b64decode(data["delta"])
                        if start_time is None:
                            start_time = time.time()
                        audio_player.play(audio_chunk)
                        chunk_duration = len(audio_chunk) / (audio_player.channels * audio_player.width * audio_player.rate)
                        total_audio_duration += chunk_duration

                    elif data["type"] == "response.audio.done":
                        # The assistant has finished sending audio
                        break

                    elif data["type"] == "response.function_call":
                        # The assistant wants to call a function.
                        function_call_name = data["function_call"]["name"]
                        arguments = data["function_call"].get("arguments", {})

                        if function_call_name == "move":
                            direction = arguments.get("direction")
                            move(direction)  # Call our Python function.

                            # Then we can send the result back to GPT if we like.
                            # For now, let's keep it simple and not send a separate function result.

                # Calculate leftover playback time.
                if start_time is not None:
                    elapsed_time = time.time() - start_time
                    remaining_playback_time = max(0, total_audio_duration - elapsed_time)
                    sleep_time = remaining_playback_time * 1.07
                    await asyncio.sleep(sleep_time)

            finally:
                audio_player.close()

    print("Conversation ended.")

if __name__ == "__main__":
    asyncio.run(main())
