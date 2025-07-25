#!/usr/bin/env python3
"""
DRV8833 Dual Motor Controller Script for Raspberry Pi 5
Compatible with RPi.GPIO (no pigpio required)
"""

import RPi.GPIO as GPIO
import time

class DRV8833Controller:
    def __init__(self, ain1_pin, ain2_pin, bin1_pin, bin2_pin, pwm_freq=1000):
        """
        Initialize DRV8833 motor controller
        
        Args:
            ain1_pin: GPIO pin for motor A input 1
            ain2_pin: GPIO pin for motor A input 2  
            bin1_pin: GPIO pin for motor B input 1
            bin2_pin: GPIO pin for motor B input 2
            pwm_freq: PWM frequency in Hz (default 1000)
        """
        self.ain1 = ain1_pin
        self.ain2 = ain2_pin
        self.bin1 = bin1_pin
        self.bin2 = bin2_pin
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.ain1, self.ain2, self.bin1, self.bin2], GPIO.OUT)
        
        # Setup PWM for speed control
        self.pwm_a1 = GPIO.PWM(self.ain1, pwm_freq)
        self.pwm_a2 = GPIO.PWM(self.ain2, pwm_freq)
        self.pwm_b1 = GPIO.PWM(self.bin1, pwm_freq)
        self.pwm_b2 = GPIO.PWM(self.bin2, pwm_freq)
        
        # Start PWM with 0% duty cycle (stopped)
        self.pwm_a1.start(0)
        self.pwm_a2.start(0)
        self.pwm_b1.start(0)
        self.pwm_b2.start(0)
    
    def motor_a_forward(self, speed=100):
        """Move motor A forward at specified speed (0-100%)"""
        speed = max(0, min(100, speed))  # Clamp between 0-100
        self.pwm_a1.ChangeDutyCycle(speed)
        self.pwm_a2.ChangeDutyCycle(0)
    
    def motor_a_backward(self, speed=100):
        """Move motor A backward at specified speed (0-100%)"""
        speed = max(0, min(100, speed))
        self.pwm_a1.ChangeDutyCycle(0)
        self.pwm_a2.ChangeDutyCycle(speed)
    
    def motor_a_stop(self):
        """Stop motor A"""
        self.pwm_a1.ChangeDutyCycle(0)
        self.pwm_a2.ChangeDutyCycle(0)
    
    def motor_a_brake(self):
        """Brake motor A (both pins high)"""
        self.pwm_a1.ChangeDutyCycle(100)
        self.pwm_a2.ChangeDutyCycle(100)
    
    def motor_b_forward(self, speed=100):
        """Move motor B forward at specified speed (0-100%)"""
        speed = max(0, min(100, speed))
        self.pwm_b1.ChangeDutyCycle(speed)
        self.pwm_b2.ChangeDutyCycle(0)
    
    def motor_b_backward(self, speed=100):
        """Move motor B backward at specified speed (0-100%)"""
        speed = max(0, min(100, speed))
        self.pwm_b1.ChangeDutyCycle(0)
        self.pwm_b2.ChangeDutyCycle(speed)
    
    def motor_b_stop(self):
        """Stop motor B"""
        self.pwm_b1.ChangeDutyCycle(0)
        self.pwm_b2.ChangeDutyCycle(0)
    
    def motor_b_brake(self):
        """Brake motor B (both pins high)"""
        self.pwm_b1.ChangeDutyCycle(100)
        self.pwm_b2.ChangeDutyCycle(100)
    
    def move_forward(self, speed=100):
        """Move both motors forward (robot moves forward)"""
        self.motor_a_forward(speed)
        self.motor_b_forward(speed)
    
    def move_backward(self, speed=100):
        """Move both motors backward (robot moves backward)"""
        self.motor_a_backward(speed)
        self.motor_b_backward(speed)
    
    def turn_left(self, speed=100):
        """Turn left (right motor forward, left motor backward)"""
        self.motor_a_backward(speed)  # Assuming A is left motor
        self.motor_b_forward(speed)   # Assuming B is right motor
    
    def turn_right(self, speed=100):
        """Turn right (left motor forward, right motor backward)"""
        self.motor_a_forward(speed)   # Assuming A is left motor
        self.motor_b_backward(speed)  # Assuming B is right motor
    
    def stop_all(self):
        """Stop both motors"""
        self.motor_a_stop()
        self.motor_b_stop()
    
    def brake_all(self):
        """Brake both motors"""
        self.motor_a_brake()
        self.motor_b_brake()
    
    def cleanup(self):
        """Clean up GPIO resources"""
        self.stop_all()
        self.pwm_a1.stop()
        self.pwm_a2.stop()
        self.pwm_b1.stop()
        self.pwm_b2.stop()
        GPIO.cleanup()

def demo_sequence(controller):
    """Demonstration sequence showing various motor movements"""
    print("Starting motor demo sequence...")
    
    try:
        # Forward
        print("Moving forward...")
        controller.move_forward(70)
        time.sleep(2)
        
        # Stop
        print("Stopping...")
        controller.stop_all()
        time.sleep(1)
        
        # Backward
        print("Moving backward...")
        controller.move_backward(70)
        time.sleep(2)
        
        # Turn left
        print("Turning left...")
        controller.turn_left(60)
        time.sleep(1.5)
        
        # Turn right
        print("Turning right...")
        controller.turn_right(60)
        time.sleep(1.5)
        
        # Speed ramping demo
        print("Speed ramping demo...")
        for speed in range(0, 101, 10):
            print(f"Speed: {speed}%")
            controller.move_forward(speed)
            time.sleep(0.3)
        
        # Gradual stop
        for speed in range(100, -1, -10):
            controller.move_forward(speed)
            time.sleep(0.2)
        
        print("Demo complete!")
        
    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
    finally:
        controller.stop_all()

if __name__ == "__main__":
    # Define your GPIO pin connections
    # Adjust these pins according to your wiring
    AIN1_PIN = 18  # Motor A input 1
    AIN2_PIN = 19  # Motor A input 2
    BIN1_PIN = 20  # Motor B input 1
    BIN2_PIN = 21  # Motor B input 2
    
    # Create controller instance
    motors = DRV8833Controller(AIN1_PIN, AIN2_PIN, BIN1_PIN, BIN2_PIN)
    
    try:
        # Run demo sequence
        demo_sequence(motors)
        
        # Interactive control example
        print("\nInteractive control (press Ctrl+C to exit):")
        print("Commands: w=forward, s=backward, a=left, d=right, space=stop")
        
        while True:
            cmd = input("Enter command: ").lower().strip()
            
            if cmd == 'w':
                motors.move_forward(80)
                print("Moving forward")
            elif cmd == 's':
                motors.move_backward(80)
                print("Moving backward")
            elif cmd == 'a':
                motors.turn_left(70)
                print("Turning left")
            elif cmd == 'd':
                motors.turn_right(70)
                print("Turning right")
            elif cmd == ' ' or cmd == 'stop':
                motors.stop_all()
                print("Stopped")
            elif cmd == 'quit' or cmd == 'exit':
                break
            else:
                print("Unknown command. Use w/s/a/d/stop/quit")
                
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        motors.cleanup()
        print("GPIO cleaned up. Goodbye!")
