#!/usr/bin/env python3
"""
DRV8833 Dual Motor Controller with Encoder Tracking for Raspberry Pi 5
Compatible with RPi.GPIO (no pigpio required)
"""

import RPi.GPIO as GPIO
import time
import threading
from collections import deque

class MotorEncoder:
    def __init__(self, pin_a, pin_b, name="Motor"):
        """
        Initialize encoder for a single motor
        
        Args:
            pin_a: GPIO pin for encoder channel A
            pin_b: GPIO pin for encoder channel B
            name: Name for this encoder (for debugging)
        """
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.name = name
        self.position = 0
        self.last_a_state = 0
        self.last_b_state = 0
        self.direction = 0  # 1 for forward, -1 for backward, 0 for stopped
        
        # Speed calculation
        self.last_time = time.time()
        self.speed_buffer = deque(maxlen=10)  # Rolling average for speed
        self.rpm = 0.0
        
        # Setup GPIO pins
        GPIO.setup(self.pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Read initial states
        self.last_a_state = GPIO.input(self.pin_a)
        self.last_b_state = GPIO.input(self.pin_b)
        
        # Setup interrupts for both channels
        GPIO.add_event_detect(self.pin_a, GPIO.BOTH, callback=self._encoder_callback, bouncetime=1)
        GPIO.add_event_detect(self.pin_b, GPIO.BOTH, callback=self._encoder_callback, bouncetime=1)
    
    def _encoder_callback(self, channel):
        """Interrupt callback for encoder changes"""
        current_time = time.time()
        a_state = GPIO.input(self.pin_a)
        b_state = GPIO.input(self.pin_b)
        
        # Quadrature decoding
        if a_state != self.last_a_state:
            if a_state == b_state:
                self.position += 1
                self.direction = 1
            else:
                self.position -= 1
                self.direction = -1
        
        elif b_state != self.last_b_state:
            if a_state != b_state:
                self.position += 1
                self.direction = 1
            else:
                self.position -= 1
                self.direction = -1
        
        # Calculate speed (pulses per second)
        time_diff = current_time - self.last_time
        if time_diff > 0:
            pulse_rate = 1.0 / time_diff  # pulses per second
            self.speed_buffer.append(pulse_rate * self.direction)
            
            # Calculate average speed (smooth out noise)
            if len(self.speed_buffer) > 0:
                avg_speed = sum(self.speed_buffer) / len(self.speed_buffer)
                # Convert to RPM (assuming 200 pulses per revolution - adjust for your encoder)
                self.rpm = (avg_speed * 60) / 200
        
        self.last_a_state = a_state
        self.last_b_state = b_state
        self.last_time = current_time
    
    def get_position(self):
        """Get current encoder position"""
        return self.position
    
    def get_rpm(self):
        """Get current RPM (smoothed)"""
        return self.rpm
    
    def reset_position(self):
        """Reset encoder position to zero"""
        self.position = 0
    
    def get_direction(self):
        """Get current direction: 1=forward, -1=backward, 0=stopped"""
        return self.direction

class DRV8833Controller:
    def __init__(self, ain1_pin, ain2_pin, bin1_pin, bin2_pin, 
                 enc_a_pins=None, enc_b_pins=None, pwm_freq=1000):
        """
        Initialize DRV8833 motor controller with optional encoders
        
        Args:
            ain1_pin: GPIO pin for motor A input 1
            ain2_pin: GPIO pin for motor A input 2  
            bin1_pin: GPIO pin for motor B input 1
            bin2_pin: GPIO pin for motor B input 2
            enc_a_pins: Tuple of (channel_a, channel_b) pins for motor A encoder
            enc_b_pins: Tuple of (channel_a, channel_b) pins for motor B encoder
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
        
        # Setup encoders if provided
        self.encoder_a = None
        self.encoder_b = None
        
        if enc_a_pins:
            self.encoder_a = MotorEncoder(enc_a_pins[0], enc_a_pins[1], "Motor A")
            print(f"Motor A encoder initialized on pins {enc_a_pins}")
        
        if enc_b_pins:
            self.encoder_b = MotorEncoder(enc_b_pins[0], enc_b_pins[1], "Motor B")
            print(f"Motor B encoder initialized on pins {enc_b_pins}")
        
        # Status monitoring thread
        self.monitoring = False
        self.monitor_thread = None
    
    def motor_a_forward(self, speed=100):
        """Move motor A forward at specified speed (0-100%)"""
        speed = max(0, min(100, speed))
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
    
    # Encoder access methods
    def get_encoder_positions(self):
        """Get both encoder positions as tuple (motor_a_pos, motor_b_pos)"""
        pos_a = self.encoder_a.get_position() if self.encoder_a else 0
        pos_b = self.encoder_b.get_position() if self.encoder_b else 0
        return (pos_a, pos_b)
    
    def get_encoder_speeds(self):
        """Get both encoder speeds in RPM as tuple (motor_a_rpm, motor_b_rpm)"""
        rpm_a = self.encoder_a.get_rpm() if self.encoder_a else 0.0
        rpm_b = self.encoder_b.get_rpm() if self.encoder_b else 0.0
        return (rpm_a, rpm_b)
    
    def reset_encoders(self):
        """Reset both encoder positions to zero"""
        if self.encoder_a:
            self.encoder_a.reset_position()
        if self.encoder_b:
            self.encoder_b.reset_position()
    
    def move_distance(self, distance_pulses, speed=50, timeout=10):
        """
        Move forward a specific distance measured in encoder pulses
        
        Args:
            distance_pulses: Target distance in encoder pulses
            speed: Motor speed (0-100%)
            timeout: Maximum time to wait (seconds)
        """
        if not (self.encoder_a and self.encoder_b):
            print("Error: Encoders required for distance-based movement")
            return False
        
        # Reset encoders
        self.reset_encoders()
        start_time = time.time()
        
        print(f"Moving {distance_pulses} pulses at {speed}% speed...")
        
        # Start moving
        self.move_forward(speed)
        
        try:
            while True:
                pos_a, pos_b = self.get_encoder_positions()
                avg_position = (abs(pos_a) + abs(pos_b)) / 2
                
                # Check if we've reached the target
                if avg_position >= distance_pulses:
                    break
                
                # Check for timeout
                if time.time() - start_time > timeout:
                    print("Timeout reached!")
                    break
                
                # Brief pause
                time.sleep(0.01)
        
        finally:
            # Stop motors
            self.stop_all()
            final_pos = self.get_encoder_positions()
            print(f"Final positions: Motor A: {final_pos[0]}, Motor B: {final_pos[1]}")
            return True
    
    def turn_angle(self, angle_pulses, speed=50, timeout=10):
        """
        Turn by a specific angle measured in encoder pulses
        Positive angle = right turn, negative = left turn
        """
        if not (self.encoder_a and self.encoder_b):
            print("Error: Encoders required for angle-based turning")
            return False
        
        self.reset_encoders()
        start_time = time.time()
        
        print(f"Turning {angle_pulses} pulses at {speed}% speed...")
        
        # Determine turn direction
        if angle_pulses > 0:
            self.turn_right(speed)
        else:
            self.turn_left(speed)
            angle_pulses = abs(angle_pulses)
        
        try:
            while True:
                pos_a, pos_b = self.get_encoder_positions()
                # For turning, we want the difference between encoders
                turn_amount = abs(abs(pos_a) - abs(pos_b))
                
                if turn_amount >= angle_pulses:
                    break
                
                if time.time() - start_time > timeout:
                    print("Timeout reached!")
                    break
                
                time.sleep(0.01)
        
        finally:
            self.stop_all()
            final_pos = self.get_encoder_positions()
            print(f"Final positions: Motor A: {final_pos[0]}, Motor B: {final_pos[1]}")
            return True
    
    def start_monitoring(self, interval=0.5):
        """Start continuous monitoring of encoder values"""
        if self.monitoring:
            return
        
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, args=(interval,))
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        print("Started encoder monitoring")
    
    def stop_monitoring(self):
        """Stop continuous monitoring"""
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join()
        print("Stopped encoder monitoring")
    
    def _monitor_loop(self, interval):
        """Internal monitoring loop"""
        while self.monitoring:
            pos_a, pos_b = self.get_encoder_positions()
            rpm_a, rpm_b = self.get_encoder_speeds()
            
            print(f"Positions: A={pos_a:6d}, B={pos_b:6d} | Speeds: A={rpm_a:6.1f} RPM, B={rpm_b:6.1f} RPM")
            time.sleep(interval)
    
    def cleanup(self):
        """Clean up GPIO resources"""
        self.stop_monitoring()
        self.stop_all()
        self.pwm_a1.stop()
        self.pwm_a2.stop()
        self.pwm_b1.stop()
        self.pwm_b2.stop()
        GPIO.cleanup()

def demo_with_encoders(controller):
    """Enhanced demo showing encoder functionality"""
    print("Starting encoder demo sequence...")
    
    try:
        # Start monitoring
        controller.start_monitoring(0.5)
        
        # Test basic movement with encoder feedback
        print("\n=== Basic Movement Test ===")
        controller.reset_encoders()
        print("Moving forward for 3 seconds...")
        controller.move_forward(60)
        time.sleep(3)
        controller.stop_all()
        
        pos_a, pos_b = controller.get_encoder_positions()
        print(f"After 3 seconds: Motor A moved {pos_a} pulses, Motor B moved {pos_b} pulses")
        
        time.sleep(2)
        
        # Test precise distance movement
        print("\n=== Precise Distance Movement ===")
        controller.move_distance(500, speed=50)  # Move exactly 500 encoder pulses
        
        time.sleep(2)
        
        # Test turning
        print("\n=== Precise Turn Test ===")
        controller.turn_angle(200, speed=40)  # Turn using 200 pulses difference
        
        time.sleep(2)
        
        # Speed test
        print("\n=== Speed Ramping Test ===")
        controller.reset_encoders()
        for speed in range(20, 81, 10):
            print(f"Speed: {speed}%")
            controller.move_forward(speed)
            time.sleep(1)
            rpm_a, rpm_b = controller.get_encoder_speeds()
            print(f"  Current speeds: A={rpm_a:.1f} RPM, B={rpm_b:.1f} RPM")
        
        controller.stop_all()
        controller.stop_monitoring()
        print("\nDemo complete!")
        
    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
    finally:
        controller.stop_monitoring()
        controller.stop_all()

if __name__ == "__main__":
    # Define your GPIO pin connections
    AIN1_PIN = 18  # Motor A input 1
    AIN2_PIN = 19  # Motor A input 2
    BIN1_PIN = 20  # Motor B input 1
    BIN2_PIN = 21  # Motor B input 2
    
    # Encoder pins (adjust according to your wiring)
    ENC_A_PINS = (2, 3)   # Motor A encoder: (Channel A, Channel B)
    ENC_B_PINS = (4, 5)   # Motor B encoder: (Channel A, Channel B)
    
    # Create controller instance with encoders
    motors = DRV8833Controller(
        AIN1_PIN, AIN2_PIN, BIN1_PIN, BIN2_PIN,
        enc_a_pins=ENC_A_PINS, enc_b_pins=ENC_B_PINS
    )
    
    try:
        # Run encoder demo
        demo_with_encoders(motors)
        
        # Interactive control with encoder feedback
        print("\nInteractive control with encoder feedback:")
        print("Commands: w=forward, s=backward, a=left, d=right, space=stop")
        print("         r=reset encoders, p=show positions, m=monitor on/off")
        
        monitoring = False
        
        while True:
            cmd = input("Enter command: ").lower().strip()
            
            if cmd == 'w':
                motors.move_forward(70)
                print("Moving forward")
            elif cmd == 's':
                motors.move_backward(70)
                print("Moving backward")
            elif cmd == 'a':
                motors.turn_left(60)
                print("Turning left")
            elif cmd == 'd':
                motors.turn_right(60)
                print("Turning right")
            elif cmd == ' ' or cmd == 'stop':
                motors.stop_all()
                print("Stopped")
            elif cmd == 'r':
                motors.reset_encoders()
                print("Encoders reset")
            elif cmd == 'p':
                pos = motors.get_encoder_positions()
                speeds = motors.get_encoder_speeds()
                print(f"Positions: A={pos[0]}, B={pos[1]}")
                print(f"Speeds: A={speeds[0]:.1f} RPM, B={speeds[1]:.1f} RPM")
            elif cmd == 'm':
                if monitoring:
                    motors.stop_monitoring()
                    monitoring = False
                else:
                    motors.start_monitoring()
                    monitoring = True
            elif cmd == 'dist':
                distance = int(input("Enter distance in pulses: "))
                motors.move_distance(distance, 50)
            elif cmd == 'turn':
                angle = int(input("Enter turn angle in pulses (+right, -left): "))
                motors.turn_angle(angle, 40)
            elif cmd == 'quit' or cmd == 'exit':
                break
            else:
                print("Commands: w/s/a/d/stop/r/p/m/dist/turn/quit")
                
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        motors.cleanup()
        print("GPIO cleaned up. Goodbye!")