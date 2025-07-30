#!/usr/bin/env python3
"""
Simple test to check if GPIO3 can detect signals
"""

import RPi.GPIO as GPIO
import time

def test_gpio3():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Setup both pins for comparison
    GPIO.setup(2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(3, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    def callback_gpio2(channel):
        print(f"GPIO2 signal detected! State: {GPIO.input(2)}")
    
    def callback_gpio3(channel):
        print(f"GPIO3 signal detected! State: {GPIO.input(3)}")
    
    GPIO.add_event_detect(2, GPIO.BOTH, callback=callback_gpio2, bouncetime=5)
    GPIO.add_event_detect(3, GPIO.BOTH, callback=callback_gpio3, bouncetime=5)
    
    print("Testing GPIO2 and GPIO3...")
    print("Current states:")
    print(f"GPIO2: {GPIO.input(2)}")
    print(f"GPIO3: {GPIO.input(3)}")
    print()
    print("Now manually turn the motor shaft slowly...")
    print("You should see signals from both GPIOs")
    print("Press Ctrl+C to stop...")
    
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nTest complete")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    test_gpio3()