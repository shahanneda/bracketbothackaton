import time
from RPi import GPIO

# GPIO setup for servo control
GPIO.setmode(GPIO.BCM)
GPIO.setup(6, GPIO.OUT)
pwm = GPIO.PWM(6, 50)  # 50 Hz frequency for servos
# MG90S servo range is typically 0-180 degrees
# For 50Hz signal (20ms period):
# 500μs pulse = 2.5% duty cycle = 0 degrees
# 2500μs pulse = 12.5% duty cycle = 180 degrees
min_duty = 2.5    # 500μs pulse (0 degrees)
mid_duty = 6.5    # 1500μs pulse (90 degrees)
max_duty = 12.5   # 2500μs pulse (180 degrees)

pwm.start(max_duty)
time.sleep(2)  # Let servo get to initial position

pwm.ChangeDutyCycle(mid_duty)
time.sleep(2)  # Let servo get to initial position

pwm.ChangeDutyCycle(max_duty)
time.sleep(2)  # Let servo get to initial position

# for i in range(100):
#     pwm.ChangeDutyCycle(max_duty - (i * 0.05))
#     time.sleep(0.05)

# try:
#     while True:
#         # Sweep from min to max
#         for duty in range(int(min_duty * 10), int(max_duty * 10), 10):
#             pwm.ChangeDutyCycle(duty / 10.0)
#             time.sleep(0.25)
            
#         # Sweep from max to min
#         for duty in range(int(max_duty * 10), int(min_duty * 10), -10):
#             pwm.ChangeDutyCycle(duty / 10.0)
#             time.sleep(0.25)

# except KeyboardInterrupt:
#     pwm.stop()
#     GPIO.cleanup()
