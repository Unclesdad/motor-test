import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)
GPIO.setup(19, GPIO.OUT)

# Test 1: IN1 high, IN2 low (should spin one direction)
GPIO.output(18, GPIO.HIGH)
GPIO.output(19, GPIO.LOW)
time.sleep(2)

# Test 2: IN1 low, IN2 high (should spin other direction)  
GPIO.output(18, GPIO.LOW)
GPIO.output(19, GPIO.HIGH)
time.sleep(2)

# Stop
GPIO.output(18, GPIO.LOW)
GPIO.output(19, GPIO.LOW)

GPIO.cleanup()