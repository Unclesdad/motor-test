#!/usr/bin/env python3
"""
N20 Motor Control with Encoder Recording
Hardware setup:
- Motor control: IN1=GPIO18, IN2=GPIO19
- Encoder signals: Yellow=GPIO2, Green=GPIO3
- DRV8833 motor driver
"""

import RPi.GPIO as GPIO
import time
import threading
from datetime import datetime

# GPIO Pin definitions
MOTOR_IN1 = 18  # Motor control pin 1
MOTOR_IN2 = 19  # Motor control pin 2
ENCODER_A = 2   # Yellow wire (Channel A)
ENCODER_B = 3   # Green wire (Channel B)

# Global variables for encoder counting
encoder_count = 0
encoder_lock = threading.Lock()
encoder_log = []

class MotorEncoder:
    def __init__(self):
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Setup motor control pins as outputs, initially LOW
        GPIO.setup(MOTOR_IN1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(MOTOR_IN2, GPIO.OUT, initial=GPIO.LOW)
        
        # Setup PWM for speed control
        self.pwm1 = GPIO.PWM(MOTOR_IN1, 1000)  # 1kHz frequency
        self.pwm2 = GPIO.PWM(MOTOR_IN2, 1000)  # 1kHz frequency
        self.pwm1.start(0)  # Start with 0% duty cycle
        self.pwm2.start(0)
        
        # Setup encoder pins with pull-up resistors
        GPIO.setup(ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ENCODER_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Wait a moment for pins to stabilize
        time.sleep(0.1)
        
        # Setup interrupt handlers for encoder with longer bouncetime
        GPIO.add_event_detect(ENCODER_A, GPIO.RISING, callback=self.encoder_callback_a, bouncetime=5)
        GPIO.add_event_detect(ENCODER_B, GPIO.RISING, callback=self.encoder_callback_b, bouncetime=5)
        
        # Ensure motor is stopped
        self.stop_motor()
        
        print("Motor and encoder initialized with PWM control")
        print(f"Motor pins: IN1={MOTOR_IN1}, IN2={MOTOR_IN2}")
        print(f"Encoder pins: A={ENCODER_A}, B={ENCODER_B}")
        print("Motor should be stopped now")
    
    def encoder_callback_a(self, channel):
        """Callback for encoder channel A - simplified counting"""
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
        """Callback for encoder channel B - simplified counting"""
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
        """Run motor forward with PWM speed control"""
        self.pwm1.ChangeDutyCycle(0)
        self.pwm2.ChangeDutyCycle(speed)
        print(f"Motor: Forward at {speed}% speed")
    
    def motor_reverse(self, speed=50):
        """Run motor reverse with PWM speed control"""
        self.pwm1.ChangeDutyCycle(speed)
        self.pwm2.ChangeDutyCycle(0)
        print(f"Motor: Reverse at {speed}% speed")
    
    def stop_motor(self):
        """Stop the motor"""
        self.pwm1.ChangeDutyCycle(0)
        self.pwm2.ChangeDutyCycle(0)
        print("Motor: Stopped")
    
    def brake_motor(self):
        """Brake the motor (both pins high)"""
        self.pwm1.ChangeDutyCycle(100)
        self.pwm2.ChangeDutyCycle(100)
        print("Motor: Braking")
    
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
                f.write("timestamp,count,direction,channel,a_state,b_state\n")
                for entry in encoder_log:
                    f.write(f"{entry['time']:.6f},{entry['count']},{entry['direction']},{entry['channel']},{entry['a_state']},{entry['b_state']}\n")
        
        print(f"Encoder log saved to {filename}")
    
    def cleanup(self):
        """Clean up GPIO and ensure motor is stopped"""
        print("Stopping motor and cleaning up...")
        self.stop_motor()
        time.sleep(0.5)  # Give time for motor to stop
        
        # Stop PWM
        self.pwm1.stop()
        self.pwm2.stop()
        
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
    motor = MotorEncoder()
    
    try:
        print("\nMotor Control Commands:")
        print("f - Forward (50% speed)")
        print("F - Forward (75% speed)")
        print("r - Reverse (50% speed)") 
        print("R - Reverse (75% speed)")
        print("s - Stop")
        print("b - Brake")
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
            elif command == 'b':
                motor.brake_motor()
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
            
            # Always show current status after command
            if command not in 'pzl123456789':
                time.sleep(0.1)  # Small delay
                motor.print_status()
    
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    
    finally:
        motor.stop_motor()
        motor.save_encoder_log()  # Auto-save log on exit
        motor.cleanup()
        print("Program ended")

# Auto-test function
def auto_test():
    """Automated test sequence"""
    motor = MotorEncoder()
    
    try:
        print("Starting automated test sequence...")
        
        # Test forward
        print("\n1. Testing forward rotation for 3 seconds...")
        motor.reset_encoder_count()
        motor.motor_forward()
        time.sleep(3)
        motor.stop_motor()
        motor.print_status()
        
        time.sleep(1)
        
        # Test reverse
        print("\n2. Testing reverse rotation for 3 seconds...")
        motor.reset_encoder_count()
        motor.motor_reverse()
        time.sleep(3)
        motor.stop_motor()
        motor.print_status()
        
        # Save results
        motor.save_encoder_log("auto_test_results.csv")
        
    except KeyboardInterrupt:
        print("\nTest interrupted")
    
    finally:
        motor.stop_motor()
        motor.cleanup()

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "test":
        auto_test()
    else:
        main()