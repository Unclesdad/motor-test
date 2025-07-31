#!/usr/bin/env python3
"""
Dual Motor Control with TB6612FNG Driver
Simple, reliable motor control for two motors
"""

import RPi.GPIO as GPIO
import time

# GPIO Pin definitions for TB6612FNG
# Motor A (Left motor)
MOTOR_A_PWM = 18   # PWMA pin
MOTOR_A_IN1 = 19   # AIN1 pin
MOTOR_A_IN2 = 20   # AIN2 pin

# Motor B (Right motor)  
MOTOR_B_PWM = 12   # PWMB pin
MOTOR_B_IN1 = 13   # BIN1 pin
MOTOR_B_IN2 = 16   # BIN2 pin

# Standby pin (shared)
MOTOR_STBY = 21    # STBY pin

class DualMotorController:
    def __init__(self):
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Setup motor A pins
        GPIO.setup(MOTOR_A_PWM, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(MOTOR_A_IN1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(MOTOR_A_IN2, GPIO.OUT, initial=GPIO.LOW)
        
        # Setup motor B pins
        GPIO.setup(MOTOR_B_PWM, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(MOTOR_B_IN1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(MOTOR_B_IN2, GPIO.OUT, initial=GPIO.LOW)
        
        # Setup standby pin
        GPIO.setup(MOTOR_STBY, GPIO.OUT, initial=GPIO.HIGH)
        
        # Setup PWM for both motors
        self.pwm_a = GPIO.PWM(MOTOR_A_PWM, 1000)  # 1kHz
        self.pwm_b = GPIO.PWM(MOTOR_B_PWM, 1000)  # 1kHz
        self.pwm_a.start(0)
        self.pwm_b.start(0)
        
        # Stop all motors initially
        self.stop_all()
        
        print("Dual motor controller initialized")
        print(f"Motor A: PWM={MOTOR_A_PWM}, IN1={MOTOR_A_IN1}, IN2={MOTOR_A_IN2}")
        print(f"Motor B: PWM={MOTOR_B_PWM}, IN1={MOTOR_B_IN1}, IN2={MOTOR_B_IN2}")
        print(f"Standby: {MOTOR_STBY}")
    
    # Motor A controls
    def motor_a_forward(self, speed=50):
        """Run motor A forward"""
        GPIO.output(MOTOR_STBY, GPIO.HIGH)
        GPIO.output(MOTOR_A_IN1, GPIO.HIGH)
        GPIO.output(MOTOR_A_IN2, GPIO.LOW)
        self.pwm_a.ChangeDutyCycle(speed)
        print(f"Motor A: Forward at {speed}%")
    
    def motor_a_reverse(self, speed=50):
        """Run motor A reverse"""
        GPIO.output(MOTOR_STBY, GPIO.HIGH)
        GPIO.output(MOTOR_A_IN1, GPIO.LOW)
        GPIO.output(MOTOR_A_IN2, GPIO.HIGH)
        self.pwm_a.ChangeDutyCycle(speed)
        print(f"Motor A: Reverse at {speed}%")
    
    def motor_a_stop(self):
        """Stop motor A"""
        GPIO.output(MOTOR_A_IN1, GPIO.HIGH)
        GPIO.output(MOTOR_A_IN2, GPIO.HIGH)
        self.pwm_a.ChangeDutyCycle(0)
        print("Motor A: Stopped")
    
    # Motor B controls
    def motor_b_forward(self, speed=50):
        """Run motor B forward"""
        GPIO.output(MOTOR_STBY, GPIO.HIGH)
        GPIO.output(MOTOR_B_IN1, GPIO.HIGH)
        GPIO.output(MOTOR_B_IN2, GPIO.LOW)
        self.pwm_b.ChangeDutyCycle(speed)
        print(f"Motor B: Forward at {speed}%")
    
    def motor_b_reverse(self, speed=50):
        """Run motor B reverse"""
        GPIO.output(MOTOR_STBY, GPIO.HIGH)
        GPIO.output(MOTOR_B_IN1, GPIO.LOW)
        GPIO.output(MOTOR_B_IN2, GPIO.HIGH)
        self.pwm_b.ChangeDutyCycle(speed)
        print(f"Motor B: Reverse at {speed}%")
    
    def motor_b_stop(self):
        """Stop motor B"""
        GPIO.output(MOTOR_B_IN1, GPIO.HIGH)
        GPIO.output(MOTOR_B_IN2, GPIO.HIGH)
        self.pwm_b.ChangeDutyCycle(0)
        print("Motor B: Stopped")
    
    # Combined controls
    def both_forward(self, speed=50):
        """Both motors forward"""
        self.motor_a_forward(speed)
        self.motor_b_forward(speed)
        print(f"Both motors: Forward at {speed}%")
    
    def both_reverse(self, speed=50):
        """Both motors reverse"""
        self.motor_a_reverse(speed)
        self.motor_b_reverse(speed)
        print(f"Both motors: Reverse at {speed}%")
    
    def turn_left(self, speed=50):
        """Turn left (A reverse, B forward)"""
        self.motor_a_reverse(speed)
        self.motor_b_forward(speed)
        print(f"Turning left at {speed}%")
    
    def turn_right(self, speed=50):
        """Turn right (A forward, B reverse)"""
        self.motor_a_forward(speed)
        self.motor_b_reverse(speed)
        print(f"Turning right at {speed}%")
    
    def stop_all(self):
        """Stop both motors"""
        self.motor_a_stop()
        self.motor_b_stop()
        print("All motors stopped")
    
    def standby(self):
        """Put driver in standby mode"""
        GPIO.output(MOTOR_STBY, GPIO.LOW)
        self.pwm_a.ChangeDutyCycle(0)
        self.pwm_b.ChangeDutyCycle(0)
        print("Driver in standby mode")
    
    def cleanup(self):
        """Clean up GPIO"""
        print("Stopping motors and cleaning up...")
        self.stop_all()
        time.sleep(0.5)
        
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()
        print("GPIO cleaned up")

def main():
    """Interactive motor control"""
    motors = DualMotorController()
    
    try:
        print("\n🚗 DUAL MOTOR CONTROL 🚗")
        print("\nCommands:")
        print("w - Both forward")
        print("s - Both reverse")
        print("a - Turn left")
        print("d - Turn right")
        print("q - Stop all")
        print("x - Standby mode")
        print()
        print("Individual motor controls:")
        print("1 - Motor A forward")
        print("2 - Motor A reverse")
        print("3 - Motor A stop")
        print("4 - Motor B forward")
        print("5 - Motor B reverse")
        print("6 - Motor B stop")
        print()
        print("Speed controls:")
        print("7,8,9 - Set speed to 30%, 60%, 90%")
        print("e - Exit")
        print("\nStarting motor control...")
        
        speed = 50  # Default speed
        
        while True:
            command = input(f"\nCommand (speed: {speed}%): ").lower().strip()
            
            # Movement commands
            if command == 'w':
                motors.both_forward(speed)
            elif command == 's':
                motors.both_reverse(speed)
            elif command == 'a':
                motors.turn_left(speed)
            elif command == 'd':
                motors.turn_right(speed)
            elif command == 'q':
                motors.stop_all()
            elif command == 'x':
                motors.standby()
            
            # Individual motor commands
            elif command == '1':
                motors.motor_a_forward(speed)
            elif command == '2':
                motors.motor_a_reverse(speed)
            elif command == '3':
                motors.motor_a_stop()
            elif command == '4':
                motors.motor_b_forward(speed)
            elif command == '5':
                motors.motor_b_reverse(speed)
            elif command == '6':
                motors.motor_b_stop()
            
            # Speed controls
            elif command == '7':
                speed = 30
                print(f"Speed set to {speed}%")
            elif command == '8':
                speed = 60
                print(f"Speed set to {speed}%")
            elif command == '9':
                speed = 90
                print(f"Speed set to {speed}%")
            
            elif command == 'e':
                break
            else:
                print("Invalid command")
    
    except KeyboardInterrupt:
        print("\nProgram interrupted")
    
    finally:
        motors.cleanup()
        print("Program ended")

def demo():
    """Automated demo sequence"""
    motors = DualMotorController()
    
    try:
        print("🎬 Running automated demo...")
        
        print("\n1. Both motors forward for 2 seconds...")
        motors.both_forward(60)
        time.sleep(2)
        
        print("\n2. Stop for 1 second...")
        motors.stop_all()
        time.sleep(1)
        
        print("\n3. Turn right for 1 second...")
        motors.turn_right(50)
        time.sleep(1)
        
        print("\n4. Turn left for 1 second...")
        motors.turn_left(50)
        time.sleep(1)
        
        print("\n5. Both reverse for 2 seconds...")
        motors.both_reverse(60)
        time.sleep(2)
        
        print("\n6. Final stop...")
        motors.stop_all()
        
        print("Demo complete!")
        
    except KeyboardInterrupt:
        print("Demo interrupted")
    
    finally:
        motors.cleanup()

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "demo":
        demo()
    else:
        main()