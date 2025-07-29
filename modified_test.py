import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(3, GPIO.IN, pull_up_down=GPIO.PUD_UP)

print("Testing with pull-ups enabled...")
for i in range(50):
    val_a = GPIO.input(2)
    val_b = GPIO.input(3)
    print(f"GPIO 2: {val_a}, GPIO 3: {val_b}")
    time.sleep(0.2)

GPIO.cleanup()