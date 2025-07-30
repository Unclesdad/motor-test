#!/usr/bin/env python3
"""
Debug script to test motor behavior step by step
"""

import RPi.GPIO as GPIO
import time

# GPIO Pin definitions
MOTOR_IN1 = 18
MOTOR_IN2 = 19
ENCODER_A = 2
ENCODER_B = 3

def setup_gpio():
    """Initialize GPIO pins"""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Setup motor pins with explicit initial state
    GPIO.setup(MOTOR_IN1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(MOTOR_IN2, GPIO.OUT, initial=GPIO.LOW)
    
    # Setup encoder pins
    GPIO.setup(ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(ENCODER_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    print("GPIO initialized")
    print(f"IN1 (GPIO{MOTOR_IN1}): {GPIO.input(MOTOR_IN1)}")
    print(f"IN2 (GPIO{MOTOR_IN2}): {GPIO.input(MOTOR_IN2)}")

def test_motor_states():
    """Test different motor states"""
    print("\n=== MOTOR STATE TESTS ===")
    
    # Test 1: Both LOW (should be stopped)
    print("\n1. Both pins LOW (STOP)")
    GPIO.output(MOTOR_IN1, GPIO.LOW)
    GPIO.output(MOTOR_IN2, GPIO.LOW)
    print(f"IN1: {GPIO.input(MOTOR_IN1)}, IN2: {GPIO.input(MOTOR_IN2)}")
    input("Press Enter when you've observed the motor behavior...")
    
    # Test 2: Forward
    print("\n2. IN1=HIGH, IN2=LOW (FORWARD)")
    GPIO.output(MOTOR_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IN2, GPIO.LOW)
    print(f"IN1: {GPIO.input(MOTOR_IN1)}, IN2: {GPIO.input(MOTOR_IN2)}")
    input("Press Enter when you've observed the motor behavior...")
    
    # Test 3: Stop again
    print("\n3. Both pins LOW (STOP)")
    GPIO.output(MOTOR_IN1, GPIO.LOW)
    GPIO.output(MOTOR_IN2, GPIO.LOW)
    print(f"IN1: {GPIO.input(MOTOR_IN1)}, IN2: {GPIO.input(MOTOR_IN2)}")
    input("Press Enter when you've observed the motor behavior...")
    
    # Test 4: Reverse
    print("\n4. IN1=LOW, IN2=HIGH (REVERSE)")
    GPIO.output(MOTOR_IN1, GPIO.LOW)
    GPIO.output(MOTOR_IN2, GPIO.HIGH)
    print(f"IN1: {GPIO.input(MOTOR_IN1)}, IN2: {GPIO.input(MOTOR_IN2)}")
    input("Press Enter when you've observed the motor behavior...")
    
    # Test 5: Stop
    print("\n5. Both pins LOW (STOP)")
    GPIO.output(MOTOR_IN1, GPIO.LOW)
    GPIO.output(MOTOR_IN2, GPIO.LOW)
    print(f"IN1: {GPIO.input(MOTOR_IN1)}, IN2: {GPIO.input(MOTOR_IN2)}")
    input("Press Enter when you've observed the motor behavior...")
    
    # Test 6: Brake
    print("\n6. Both pins HIGH (BRAKE)")
    GPIO.output(MOTOR_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IN2, GPIO.HIGH)
    print(f"IN1: {GPIO.input(MOTOR_IN1)}, IN2: {GPIO.input(MOTOR_IN2)}")
    input("Press Enter when you've observed the motor behavior...")
    
    # Test 7: Final stop
    print("\n7. Both pins LOW (FINAL STOP)")
    GPIO.output(MOTOR_IN1, GPIO.LOW)
    GPIO.output(MOTOR_IN2, GPIO.LOW)
    print(f"IN1: {GPIO.input(MOTOR_IN1)}, IN2: {GPIO.input(MOTOR_IN2)}")

def test_encoder_readings():
    """Test encoder signal readings"""
    print("\n=== ENCODER TESTS ===")
    print("Reading encoder signals for 10 seconds...")
    print("Manually turn the motor shaft and observe readings:")
    
    start_time = time.time()
    last_a = GPIO.input(ENCODER_A)
    last_b = GPIO.input(ENCODER_B)
    
    print(f"Initial: A={last_a}, B={last_b}")
    
    while time.time() - start_time < 10:
        current_a = GPIO.input(ENCODER_A)
        current_b = GPIO.input(ENCODER_B)
        
        if current_a != last_a or current_b != last_b:
            print(f"Change: A={current_a}, B={current_b}")
            last_a = current_a
            last_b = current_b
        
        time.sleep(0.01)

def check_pin_states_on_boot():
    """Check what the pin states are when script starts"""
    print("\n=== INITIAL PIN STATES ===")
    print("Before GPIO setup:")
    # This might cause warnings, but helps debug
    try:
        GPIO.setmode(GPIO.BCM)
        print(f"GPIO{MOTOR_IN1} state: {GPIO.input(MOTOR_IN1)}")
        print(f"GPIO{MOTOR_IN2} state: {GPIO.input(MOTOR_IN2)}")
    except:
        print("Could not read pin states before setup")

def main():
    try:
        check_pin_states_on_boot()
        setup_gpio()
        
        print("\nThis script will test motor behavior step by step.")
        print("Observe the motor at each step and report what happens.")
        
        test_motor_states()
        
        choice = input("\nTest encoder readings? (y/n): ")
        if choice.lower() == 'y':
            test_encoder_readings()
        
    except KeyboardInterrupt:
        print("\nTest interrupted")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Ensure motor is stopped
        try:
            GPIO.output(MOTOR_IN1, GPIO.LOW)
            GPIO.output(MOTOR_IN2, GPIO.LOW)
            time.sleep(0.5)
        except:
            pass
        GPIO.cleanup()
        print("\nGPIO cleaned up")

if __name__ == "__main__":
    main()