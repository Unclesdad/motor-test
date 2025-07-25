#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(2, GPIO.IN)  # Channel A
GPIO.setup(3, GPIO.IN)  # Channel B

print("Reading encoder pins for 10 seconds...")
print("Turn the motor shaft by hand and watch for changes")
print("Pin 2 (Ch A) | Pin 3 (Ch B)")

try:
    for i in range(100):
        val_a = GPIO.input(2)
        val_b = GPIO.input(3)
        print(f"    {val_a}      |     {val_b}")
        time.sleep(0.1)
except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()