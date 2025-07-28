import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)
GPIO.setup(19, GPIO.OUT)

print("Testing GPIO outputs...")
for i in range(5):
    GPIO.output(18, GPIO.HIGH)
    GPIO.output(19, GPIO.LOW)
    print(f"IN1: {GPIO.input(18)}, IN2: {GPIO.input(19)}")
    time.sleep(1)
    
    GPIO.output(18, GPIO.LOW)  
    GPIO.output(19, GPIO.HIGH)
    print(f"IN1: {GPIO.input(18)}, IN2: {GPIO.input(19)}")
    time.sleep(1)

GPIO.cleanup()