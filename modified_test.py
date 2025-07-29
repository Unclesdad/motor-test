import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
# No pull-ups this time
GPIO.setup(2, GPIO.IN)
GPIO.setup(3, GPIO.IN)

print("Testing without pull-ups...")
print("Turn motor shaft VERY slowly and watch for changes...")

prev_a = None
prev_b = None

for i in range(100):
    val_a = GPIO.input(2)
    val_b = GPIO.input(3)
    
    # Only print when values change
    if val_a != prev_a or val_b != prev_b:
        print(f"CHANGE! GPIO 2: {val_a}, GPIO 3: {val_b}")
        prev_a = val_a
        prev_b = val_b
    
    time.sleep(0.1)

GPIO.cleanup()