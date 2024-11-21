import sys
import json
import paho.mqtt.client as mqtt

class VelocityPublisher:
    def __init__(self, broker_address="localhost", velocity_topic="robot/velocity", speak_topic="robot/speak"):
        self.broker_address = broker_address
        self.velocity_topic = velocity_topic
        self.speak_topic = speak_topic
        self.client = mqtt.Client()

    def connect(self):
        self.client.connect(self.broker_address)

    def publish_velocity(self, linear_vel, angular_vel):
        payload = json.dumps({"linear": linear_vel, "angular": angular_vel})
        self.client.publish(self.velocity_topic, payload)

    def publish_speech(self, text):
        self.client.publish(self.speak_topic, text)

    def run(self):
        self.connect()
        while True:
            try:
                # Read input from stdin
                input_line = sys.stdin.readline().strip()
                
                # Check if input is a speech command
                if input_line.startswith("say "):
                    speech_text = input_line[4:]  # Remove "say " prefix
                    self.publish_speech(speech_text)
                else:
                    # Parse the input to get linear and angular velocities
                    linear_vel, angular_vel = map(float, input_line.split())
                    # Publish the velocity
                    self.publish_velocity(linear_vel, angular_vel)
            except ValueError:
                print("Invalid input. For velocity, enter two numbers separated by space.")
                print("For speech, start with 'say ' followed by your message.")
            except KeyboardInterrupt:
                print("Publisher stopped by user.")
                break

if __name__ == "__main__":
    velocity_publisher = VelocityPublisher(broker_address='localhost')
    velocity_publisher.run()