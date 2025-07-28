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
        
        # Setup GPIO pins (no pull-up since physical resistors are present)
        GPIO.setup(self.pin_a, GPIO.IN)
        GPIO.setup(self.pin_b, GPIO.IN)
        
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

class MotorController:
    def __init__(self, in1_pin, in2_pin, enc_a_pins=None, pwm_freq=1000):
        """
        Initialize motor controller with IN1, IN2 pins
        
        Args:
            in1_pin: GPIO pin for motor input 1
            in2_pin: GPIO pin for motor input 2
            enc_a_pins: Tuple of (channel_a, channel_b) pins for encoder
            pwm_freq: PWM frequency in Hz (default 1000)
        """
        self.in1 = in1_pin
        self.in2 = in2_pin
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.in1, self.in2], GPIO.OUT)
        
        # Setup PWM for speed control
        self.pwm1 = GPIO.PWM(self.in1, pwm_freq)
        self.pwm2 = GPIO.PWM(self.in2, pwm_freq)
        
        # Start PWM with 0% duty cycle (stopped)
        self.pwm1.start(0)
        self.pwm2.start(0)
        
        # Setup encoder if provided
        self.encoder_a = None
        
        if enc_a_pins:
            self.encoder_a = MotorEncoder(enc_a_pins[0], enc_a_pins[1], "Motor A")
            print(f"Motor encoder initialized on pins {enc_a_pins}")
        
        # Status monitoring thread
        self.monitoring = False
        self.monitor_thread = None
    
    def motor_forward(self, speed=100):
        """Move motor forward at specified speed (0-100%)"""
        speed = max(0, min(100, speed))
        self.pwm1.ChangeDutyCycle(speed)
        self.pwm2.ChangeDutyCycle(0)
    
    def motor_backward(self, speed=100):
        """Move motor backward at specified speed (0-100%)"""
        speed = max(0, min(100, speed))
        self.pwm1.ChangeDutyCycle(0)
        self.pwm2.ChangeDutyCycle(speed)
    
    def motor_stop(self):
        """Stop motor"""
        self.pwm1.ChangeDutyCycle(0)
        self.pwm2.ChangeDutyCycle(0)
    
    def motor_brake(self):
        """Brake motor (both pins high)"""
        self.pwm1.ChangeDutyCycle(100)
        self.pwm2.ChangeDutyCycle(100)
    
    def motor_b_forward(self, speed=100):
        """Motor B removed - this function does nothing"""
        pass
    
    def motor_b_backward(self, speed=100):
        """Motor B removed - this function does nothing"""
        pass
    
    def motor_b_stop(self):
        """Motor B removed - this function does nothing"""
        pass
    
    def motor_b_brake(self):
        """Motor B removed - this function does nothing"""
        pass
    
    def move_forward(self, speed=100):
        """Move motor forward"""
        self.motor_forward(speed)
    
    def move_backward(self, speed=100):
        """Move motor backward"""
        self.motor_backward(speed)
    
    def turn_left(self, speed=100):
        """Turn left (motor backward)"""
        self.motor_backward(speed)
    
    def turn_right(self, speed=100):
        """Turn right (motor forward)"""
        self.motor_forward(speed)
    
    def stop_all(self):
        """Stop motor"""
        self.motor_stop()
    
    def brake_all(self):
        """Brake motor"""
        self.motor_brake()
    
    # Encoder access methods
    def get_encoder_position(self):
        """Get encoder position for motor A"""
        return self.encoder_a.get_position() if self.encoder_a else 0
    
    def get_encoder_speed(self):
        """Get encoder speed in RPM for motor A"""
        return self.encoder_a.get_rpm() if self.encoder_a else 0.0
    
    def reset_encoder(self):
        """Reset encoder position to zero"""
        if self.encoder_a:
            self.encoder_a.reset_position()
    
    def move_distance(self, distance_pulses, speed=50, timeout=10):
        """
        Move forward a specific distance measured in encoder pulses
        
        Args:
            distance_pulses: Target distance in encoder pulses
            speed: Motor speed (0-100%)
            timeout: Maximum time to wait (seconds)
        """
        if not self.encoder_a:
            print("Error: Encoder required for distance-based movement")
            return False
        
        # Reset encoder
        self.reset_encoder()
        start_time = time.time()
        
        print(f"Moving {distance_pulses} pulses at {speed}% speed...")
        
        # Start moving
        self.move_forward(speed)
        
        try:
            while True:
                position = self.get_encoder_position()
                
                # Check if we've reached the target
                if abs(position) >= distance_pulses:
                    break
                
                # Check for timeout
                if time.time() - start_time > timeout:
                    print("Timeout reached!")
                    break
                
                # Brief pause
                time.sleep(0.01)
        
        finally:
            # Stop motor
            self.stop_all()
            final_pos = self.get_encoder_position()
            print(f"Final position: Motor: {final_pos}")
            return True
    
    def turn_angle(self, angle_pulses, speed=50, timeout=10):
        """
        Turn by a specific angle measured in encoder pulses
        Positive angle = right turn, negative = left turn
        Note: With single motor, this just moves forward/backward
        """
        if not self.encoder_a:
            print("Error: Encoder required for angle-based movement")
            return False
        
        self.reset_encoder()
        start_time = time.time()
        
        print(f"Moving {angle_pulses} pulses at {speed}% speed...")
        
        # Determine direction
        if angle_pulses > 0:
            self.turn_right(speed)
        else:
            self.turn_left(speed)
            angle_pulses = abs(angle_pulses)
        
        try:
            while True:
                position = abs(self.get_encoder_position())
                
                if position >= angle_pulses:
                    break
                
                if time.time() - start_time > timeout:
                    print("Timeout reached!")
                    break
                
                time.sleep(0.01)
        
        finally:
            self.stop_all()
            final_pos = self.get_encoder_position()
            print(f"Final position: Motor: {final_pos}")
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
            position = self.get_encoder_position()
            rpm = self.get_encoder_speed()
            
            print(f"Position: A={position:6d} | Speed: A={rpm:6.1f} RPM")
            time.sleep(interval)
    
    def cleanup(self):
        """Clean up GPIO resources"""
        self.stop_monitoring()
        self.stop_all()
        self.pwm1.stop()
        self.pwm2.stop()
        GPIO.cleanup()

def demo_with_encoders(controller):
    """Enhanced demo showing encoder functionality"""
    print("Starting encoder demo sequence...")
    
    try:
        # Start monitoring
        controller.start_monitoring(0.5)
        
        # Test basic movement with encoder feedback
        print("\n=== Basic Movement Test ===")
        controller.reset_encoder()
        print("Moving forward for 3 seconds...")
        controller.move_forward(60)
        time.sleep(3)
        controller.stop_all()
        
        pos = controller.get_encoder_position()
        print(f"After 3 seconds: Motor A moved {pos} pulses")
        
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
        controller.reset_encoder()
        for speed in range(20, 81, 10):
            print(f"Speed: {speed}%")
            controller.move_forward(speed)
            time.sleep(1)
            rpm = controller.get_encoder_speed()
            print(f"  Current speed: A={rpm:.1f} RPM")
        
        controller.stop_all()
        controller.stop_monitoring()
        print("\nDemo complete!")
        
    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
    finally:
        controller.stop_monitoring()
        controller.stop_all()

if __name__ == "__main__":
    # Define your GPIO pin connections for your motor driver
    IN1_PIN = 18  # Motor input 1
    IN2_PIN = 19  # Motor input 2
    
    # Encoder pins (adjust according to your wiring)
    ENC_PINS = (2, 3)   # Motor encoder: (Channel A, Channel B)
    
    # Create controller instance with encoder
    motors = MotorController(
        IN1_PIN, IN2_PIN,
        enc_a_pins=ENC_PINS
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
                motors.reset_encoder()
                print("Encoder reset")
            elif cmd == 'p':
                pos = motors.get_encoder_position()
                speed = motors.get_encoder_speed()
                print(f"Position: A={pos}")
                print(f"Speed: A={speed:.1f} RPM")
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