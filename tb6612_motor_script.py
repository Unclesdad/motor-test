#!/usr/bin/env python3
"""
N20 Motor Control with TB6612FNG Driver and Encoder Recording
Hardware setup:
- Motor control: PWMA=GPIO18, AIN1=GPIO19, AIN2=GPIO20, STBY=GPIO21
- Encoder signals: Yellow=GPIO2, Green=GPIO3
- TB6612FNG motor driver
"""

import RPi.GPIO as GPIO
import time
import threading
from datetime import datetime

# GPIO Pin definitions for TB6612FNG
MOTOR_PWM = 18   # PWMA pin - speed control
MOTOR_AIN1 = 19  # AIN1 pin - direction control 1
MOTOR_AIN2 = 20  # AIN2 pin - direction control 2
MOTOR_STBY = 21  # STBY pin - standby control (must be HIGH)

ENCODER_A = 2    # Yellow wire (Channel A)
ENCODER_B = 3    # Green wire (Channel B)

# Global variables for encoder counting
encoder_count = 0
encoder_lock = threading.Lock()
encoder_log = []

class TB6612MotorEncoder:
    def __init__(self):
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Setup motor control pins
        GPIO.setup(MOTOR_PWM, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(MOTOR_AIN1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(MOTOR_AIN2, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(MOTOR_STBY, GPIO.OUT, initial=GPIO.HIGH)  # Enable driver
        
        # Setup PWM for speed control
        self.pwm = GPIO.PWM(MOTOR_PWM, 1000)  # 1kHz frequency
        self.pwm.start(0)  # Start with 0% duty cycle
        
        # Setup encoder pins with pull-up resistors
        GPIO.setup(ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ENCODER_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Wait a moment for pins to stabilize
        time.sleep(0.1)
        
        # Setup interrupt handlers for encoder
        GPIO.add_event_detect(ENCODER_A, GPIO.RISING, callback=self.encoder_callback_a, bouncetime=5)
        GPIO.add_event_detect(ENCODER_B, GPIO.RISING, callback=self.encoder_callback_b, bouncetime=5)
        
        # Initialize motor to stopped state
        self.stop_motor()
        
        print("TB6612FNG Motor and encoder initialized")
        print(f"Motor pins: PWM={MOTOR_PWM}, AIN1={MOTOR_AIN1}, AIN2={MOTOR_AIN2}, STBY={MOTOR_STBY}")
        print(f"Encoder pins: A={ENCODER_A}, B={ENCODER_B}")
    
    def encoder_callback_a(self, channel):
        """Callback for encoder channel A"""
        global encoder_count, encoder_log
        try:
            with encoder_lock:
                encoder_count += 1
                timestamp = time.time()
                encoder_log.append({
                    'time': timestamp,
                    'count': encoder_count,
                    'channel': 'A'
                })
                print(f"Encoder A pulse: {encoder_count}")
        except Exception as e:
            print(f"Encoder A callback error: {e}")
    
    def encoder_callback_b(self, channel):
        """Callback for encoder channel B"""
        global encoder_count, encoder_log
        try:
            with encoder_lock:
                encoder_count += 1
                timestamp = time.time()
                encoder_log.append({
                    'time': timestamp,
                    'count': encoder_count,
                    'channel': 'B'
                })
                print(f"Encoder B pulse: {encoder_count}")
        except Exception as e:
            print(f"Encoder B callback error: {e}")
    
    def motor_forward(self, speed=50):
        """Run motor forward with speed control (0-100%)"""
        GPIO.output(MOTOR_STBY, GPIO.HIGH)  # Enable driver
        GPIO.output(MOTOR_AIN1, GPIO.HIGH)
        GPIO.output(MOTOR_AIN2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(speed)
        print(f"Motor: Forward at {speed}% speed")
    
    def motor_reverse(self, speed=50):
        """Run motor reverse with speed control (0-100%)"""
        GPIO.output(MOTOR_STBY, GPIO.HIGH)  # Enable driver
        GPIO.output(MOTOR_AIN1, GPIO.LOW)
        GPIO.output(MOTOR_AIN2, GPIO.HIGH)
        self.pwm.ChangeDutyCycle(speed)
        print(f"Motor: Reverse at {speed}% speed")
    
    def stop_motor(self):
        """Stop the motor (short brake)"""
        GPIO.output(MOTOR_AIN1, GPIO.HIGH)
        GPIO.output(MOTOR_AIN2, GPIO.HIGH)
        self.pwm.ChangeDutyCycle(0)
        print("Motor: Stopped (short brake)")
    
    def coast_motor(self):
        """Coast the motor (both pins low)"""
        GPIO.output(MOTOR_AIN1, GPIO.LOW)
        GPIO.output(MOTOR_AIN2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)
        print("Motor: Coasting")
    
    def standby_mode(self):
        """Put driver in standby (disables all outputs)"""
        GPIO.output(MOTOR_STBY, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)
        print("Motor: Standby mode")
    
    def get_encoder_count(self):
        """Get current encoder count thread-safely"""
        with encoder_lock:
            return encoder_count
    
    def reset_encoder_count(self):
        """Reset encoder count to zero"""
        global encoder_count, encoder_log
        with encoder_lock:
            encoder_count = 0
            encoder_log.clear()
        print("Encoder count reset")
    
    def print_status(self):
        """Print current status"""
        count = self.get_encoder_count()
        revolutions = count / 7.0  # 7 pulses per revolution according to spec
        print(f"Encoder count: {count}, Revolutions: {revolutions:.2f}")
    
    def save_encoder_log(self, filename=None):
        """Save encoder log to file"""
        if filename is None:
            filename = f"encoder_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        
        with encoder_lock:
            with open(filename, 'w') as f:
                f.write("timestamp,count,channel\n")
                for entry in encoder_log:
                    f.write(f"{entry['time']:.6f},{entry['count']},{entry['channel']}\n")
        
        print(f"Encoder log saved to {filename}")
    
    def cleanup(self):
        """Clean up GPIO and ensure motor is stopped"""
        print("Stopping motor and cleaning up...")
        self.standby_mode()
        time.sleep(0.5)
        
        # Stop PWM
        self.pwm.stop()
        
        # Remove interrupt handlers
        try:
            GPIO.remove_event_detect(ENCODER_A)
            GPIO.remove_event_detect(ENCODER_B)
        except:
            pass
        
        # Clean up GPIO
        GPIO.cleanup()
        print("GPIO cleaned up - motor should be stopped")

def main():
    """Main function with interactive motor control"""
    motor = TB6612MotorEncoder()
    
    try:
        print("\nTB6612FNG Motor Control Commands:")
        print("f - Forward (50% speed)")
        print("F - Forward (75% speed)")
        print("r - Reverse (50% speed)") 
        print("R - Reverse (75% speed)")
        print("s - Stop (short brake)")
        print("c - Coast")
        print("x - Standby mode")
        print("1-9 - Set speed (10%-90%) then use f/r")
        print("p - Print status")
        print("z - Reset encoder count")
        print("l - Save encoder log")
        print("q - Quit")
        print("\nStarting motor control loop...")
        
        speed = 50  # Default speed
        
        while True:
            command = input(f"\nEnter command (current speed: {speed}%): ").lower().strip()
            
            if command == 'f':
                motor.motor_forward(speed)
            elif command == 'F':
                motor.motor_forward(75)
            elif command == 'r':
                motor.motor_reverse(speed)
            elif command == 'R':
                motor.motor_reverse(75)
            elif command == 's':
                motor.stop_motor()
            elif command == 'c':
                motor.coast_motor()
            elif command == 'x':
                motor.standby_mode()
            elif command in '123456789':
                speed = int(command) * 10
                print(f"Speed set to {speed}%")
            elif command == 'p':
                motor.print_status()
            elif command == 'z':
                motor.reset_encoder_count()
            elif command == 'l':
                motor.save_encoder_log()
            elif command == 'q':
                break
            else:
                print("Invalid command")
            
            # Show current status after motor commands
            if command in 'fFrRscx':
                time.sleep(0.1)
                motor.print_status()
    
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    
    finally:
        motor.stop_motor()
        motor.save_encoder_log()
        motor.cleanup()
        print("Program ended")

def auto_test():
    """Automated test sequence"""
    motor = TB6612MotorEncoder()
    
    try:
        print("Starting TB6612FNG automated test sequence...")
        
        # Test forward
        print("\n1. Testing forward rotation for 3 seconds at 60%...")
        motor.reset_encoder_count()
        motor.motor_forward(60)
        time.sleep(3)
        motor.stop_motor()
        motor.print_status()
        
        time.sleep(1)
        
        # Test reverse
        print("\n2. Testing reverse rotation for 3 seconds at 60%...")
        motor.reset_encoder_count()
        motor.motor_reverse(60)
        time.sleep(3)
        motor.stop_motor()
        motor.print_status()
        
        # Test coast
        print("\n3. Testing coast...")
        motor.motor_forward(40)
        time.sleep(1)
        motor.coast_motor()
        time.sleep(2)
        motor.print_status()
        
        # Save results
        motor.save_encoder_log("tb6612_test_results.csv")
        
    except KeyboardInterrupt:
        print("\nTest interrupted")
    
    finally:
        motor.standby_mode()
        motor.cleanup()

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "test":
        auto_test()
    else:
        main()