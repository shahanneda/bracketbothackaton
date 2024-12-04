import time
from RPi import GPIO

# GPIO setup for servo control
GPIO.setmode(GPIO.BCM)
GPIO.setup(6, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
pwm1 = GPIO.PWM(6, 50)  # 50 Hz frequency for servos
pwm2 = GPIO.PWM(13, 50)  # Second servo

# Calculate duty cycles for angles:
# 90 degrees = 6.5% duty cycle
# 0 degrees = 2.5% duty cycle
legs_out = 6.5     # 90 degrees - legs out
legs_in = 2.5      # 0 degrees - legs in

def set_servo_positions(duty_cycle):
    pwm1.start(duty_cycle)
    pwm2.start(duty_cycle)
    print(f"Set servos to {duty_cycle}%")
    time.sleep(1.5)  # Wait for servos to reach position
    pwm1.stop()
    pwm2.stop()

# Test servo positions
print("Moving legs out...")
set_servo_positions(legs_out)

time.sleep(1)  # Pause between movements

print("Moving legs in...")
set_servo_positions(legs_in)

GPIO.cleanup()
