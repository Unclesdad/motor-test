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
        
        # Setup motor control pins
        GPIO.setup(MOTOR_IN1, GPIO.OUT)
        GPIO.setup(MOTOR_IN2, GPIO.OUT)
        
        # Setup encoder pins with pull-up resistors
        GPIO.setup(ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ENCODER_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Setup interrupt handlers for encoder
        GPIO.add_event_detect(ENCODER_A, GPIO.BOTH, callback=self.encoder_callback_a, bouncetime=1)
        GPIO.add_event_detect(ENCODER_B, GPIO.BOTH, callback=self.encoder_callback_b, bouncetime=1)
        
        # Initialize motor to stopped state
        self.stop_motor()
        
        print("Motor and encoder initialized")
        print(f"Motor pins: IN1={MOTOR_IN1}, IN2={MOTOR_IN2}")
        print(f"Encoder pins: A={ENCODER_A}, B={ENCODER_B}")
    
    def encoder_callback_a(self, channel):
        """Callback for encoder channel A"""
        global encoder_count, encoder_log
        with encoder_lock:
            # Read both channels
            a_state = GPIO.input(ENCODER_A)
            b_state = GPIO.input(ENCODER_B)
            
            # Determine direction (simple quadrature decoding)
            if a_state == b_state:
                encoder_count += 1
                direction = "CW"
            else:
                encoder_count -= 1
                direction = "CCW"
            
            # Log the event
            timestamp = time.time()
            encoder_log.append({
                'time': timestamp,
                'count': encoder_count,
                'direction': direction,
                'channel': 'A',
                'a_state': a_state,
                'b_state': b_state
            })
    
    def encoder_callback_b(self, channel):
        """Callback for encoder channel B"""
        global encoder_count, encoder_log
        with encoder_lock:
            # Read both channels
            a_state = GPIO.input(ENCODER_A)
            b_state = GPIO.input(ENCODER_B)
            
            # Determine direction (simple quadrature decoding)
            if a_state != b_state:
                encoder_count += 1
                direction = "CW"
            else:
                encoder_count -= 1
                direction = "CCW"
            
            # Log the event
            timestamp = time.time()
            encoder_log.append({
                'time': timestamp,
                'count': encoder_count,
                'direction': direction,
                'channel': 'B',
                'a_state': a_state,
                'b_state': b_state
            })
    
    def motor_forward(self):
        """Run motor forward"""
        GPIO.output(MOTOR_IN1, GPIO.HIGH)
        GPIO.output(MOTOR_IN2, GPIO.LOW)
        print("Motor: Forward")
    
    def motor_reverse(self):
        """Run motor in reverse"""
        GPIO.output(MOTOR_IN1, GPIO.LOW)
        GPIO.output(MOTOR_IN2, GPIO.HIGH)
        print("Motor: Reverse")
    
    def stop_motor(self):
        """Stop the motor"""
        GPIO.output(MOTOR_IN1, GPIO.LOW)
        GPIO.output(MOTOR_IN2, GPIO.LOW)
        print("Motor: Stopped")
    
    def brake_motor(self):
        """Brake the motor (both pins high)"""
        GPIO.output(MOTOR_IN1, GPIO.HIGH)
        GPIO.output(MOTOR_IN2, GPIO.HIGH)
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
        """Clean up GPIO"""
        GPIO.cleanup()
        print("GPIO cleaned up")

def main():
    """Main function with interactive motor control"""
    motor = MotorEncoder()
    
    try:
        print("\nMotor Control Commands:")
        print("f - Forward")
        print("r - Reverse") 
        print("s - Stop")
        print("b - Brake")
        print("p - Print status")
        print("z - Reset encoder count")
        print("l - Save encoder log")
        print("q - Quit")
        print("\nStarting motor control loop...")
        
        while True:
            command = input("\nEnter command: ").lower().strip()
            
            if command == 'f':
                motor.motor_forward()
            elif command == 'r':
                motor.motor_reverse()
            elif command == 's':
                motor.stop_motor()
            elif command == 'b':
                motor.brake_motor()
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