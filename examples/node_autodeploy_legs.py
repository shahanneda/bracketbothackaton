import time
import os
from rpi_hardware_pwm import HardwarePWM
import paho.mqtt.client as mqtt
from threading import Thread

# Initialize hardware PWM
pwm = HardwarePWM(pwm_channel=0, hz=50, chip=2)  # GPIO 12, 50 Hz frequency for servo

# Calculate duty cycles for angles:
# Legs out = 10.5% duty cycle  
# Legs in = 5% duty cycle
error_duty = 10.5    # Legs out position
ok_duty = 5.0       # Legs in position

# Track current servo state to avoid unnecessary updates
current_state = None  # Start with no state

class WatchdogSubscriber(Thread):
    def __init__(self, broker_address="localhost", topic="balance_watchdog"):
        super().__init__()
        self.broker_address = broker_address
        self.topic = topic
        self.last_timestamp = 0
        self.client = mqtt.Client()

    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected with result code {rc}")
        client.subscribe(self.topic)

    def on_message(self, client, userdata, msg):
        try:
            self.last_timestamp = float(msg.payload.decode())
        except ValueError as e:
            print(f"Error decoding timestamp: {e}")

    def run(self):
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(self.broker_address)
        self.client.loop_forever()

def duty_cycle_to_pulse_width(duty_cycle):
    # Convert duty cycle percentage to pulse width in milliseconds
    pulse_width = (duty_cycle / 100) / 50 * 1000  # Convert to ms
    return pulse_width

def set_servo_positions(duty_cycle):
    global current_state
    if current_state != duty_cycle:
        # Only change position if needed
        pwm.change_duty_cycle(duty_cycle)
        current_state = duty_cycle
        pulse_width = duty_cycle_to_pulse_width(duty_cycle)
        print(f"Set servo to {duty_cycle}% = {pulse_width:.2f}ms pulse width")

def check_watchdog(watchdog):
    current_time = time.monotonic()
    if current_time - watchdog.last_timestamp > 0.5:  # Half second delay before deploying legs
        set_servo_positions(error_duty)  # Legs out
    else:
        set_servo_positions(ok_duty)  # Legs in

# Initialize MQTT subscriber
watchdog = WatchdogSubscriber()
watchdog.start()

# Start PWM and set initial position
pwm.start(0)
set_servo_positions(error_duty)

while True:
    check_watchdog(watchdog)
    time.sleep(0.05)  # Check every 50ms