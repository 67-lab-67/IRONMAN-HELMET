# ironman_helmet_control.py
import RPi.GPIO as GPIO
import time
import threading
from picamera import PiCamera

# ---------------------------
# GPIO SETUP
# ---------------------------
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Pin Definitions (adjust based on your wiring)
SERVO_PINS = [17, 18, 27, 22, 23, 24]
SOLENOID_PIN = 25
TOUCH_SENSOR_PINS = [5, 6]
LED_PIN = 26
VOLTMETER_ADC_PIN = 0  # If using ADC for voltmeter

# Setup outputs
for pin in SERVO_PINS:
    GPIO.setup(pin, GPIO.OUT)
GPIO.setup(SOLENOID_PIN, GPIO.OUT)
GPIO.setup(LED_PIN, GPIO.OUT)

# Setup inputs
for pin in TOUCH_SENSOR_PINS:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Servo setup
servos = [GPIO.PWM(pin, 50) for pin in SERVO_PINS]  # 50Hz for servos
for servo in servos:
    servo.start(0)

# ---------------------------
# SERVO CONTROL
# ---------------------------
def set_servo_angle(servo_index, angle):
    """Move servo to specific angle (0-180)"""
    duty = angle / 18 + 2
    servos[servo_index].ChangeDutyCycle(duty)
    time.sleep(0.5)
    servos[servo_index].ChangeDutyCycle(0)  # Stop sending signal

def helmet_open_sequence():
    """Open helmet servos in sequence"""
    for i in range(6):
        set_servo_angle(i, 90)  # Open position
        time.sleep(0.3)

def helmet_close_sequence():
    """Close helmet servos in sequence"""
    for i in range(6):
        set_servo_angle(i, 0)  # Closed position
        time.sleep(0.3)

# ---------------------------
# SOLENOID LOCK MECHANISM
# ---------------------------
def lock_helmet():
    GPIO.output(SOLENOID_PIN, GPIO.HIGH)
    print("Helmet locked")

def unlock_helmet():
    GPIO.output(SOLENOID_PIN, GPIO.LOW)
    print("Helmet unlocked")

# ---------------------------
# TOUCH SENSOR HANDLING
# ---------------------------
def check_touch_sensors():
    """Read touch sensors for open/close commands"""
    touch1 = GPIO.input(TOUCH_SENSOR_PINS[0])
    touch2 = GPIO.input(TOUCH_SENSOR_PINS[1])
    
    if touch1 == GPIO.LOW:
        print("Touch sensor 1 activated - Opening helmet")
        unlock_helmet()
        helmet_open_sequence()
        return "open"
    elif touch2 == GPIO.LOW:
        print("Touch sensor 2 activated - Closing helmet")
        helmet_close_sequence()
        lock_helmet()
        return "close"
    return None

# ---------------------------
# BATTERY MONITORING
# ---------------------------
def read_battery_voltage():
    """Read battery voltage via ADC (if connected)"""
    # This requires ADC hardware (e.g., MCP3008)
    # Placeholder implementation
    try:
        # Example for MCP3008
        # voltage = read_adc(VOLTMETER_ADC_PIN)
        # return voltage * 3.3  # Adjust based on voltage divider
        return 12.6  # Placeholder
    except:
        return 0.0

# ---------------------------
# CAMERA SYSTEM
# ---------------------------
class CameraSystem:
    def __init__(self):
        self.cameras = []
        # Initialize up to 3 cameras if connected
        for i in range(3):
            try:
                cam = PiCamera(camera_num=i)
                cam.resolution = (640, 480)
                self.cameras.append(cam)
            except:
                break
    
    def capture_images(self):
        """Capture from all available cameras"""
        timestamps = []
        for idx, cam in enumerate(self.cameras):
            filename = f"/home/pi/helmet_cam_{idx}_{time.time()}.jpg"
            cam.capture(filename)
            timestamps.append(filename)
        return timestamps
    
    def night_vision_mode(self, enable=True):
        """Simulate night vision mode (adjust camera settings)"""
        for cam in self.cameras:
            if enable:
                cam.iso = 800  # Higher ISO for low light
                # Additional night vision adjustments
            else:
                cam.iso = 0  # Auto

# ---------------------------
# LED CONTROL
# ---------------------------
def led_breathing_effect():
    """Create a breathing effect for the blue LED"""
    pwm_led = GPIO.PWM(LED_PIN, 100)
    pwm_led.start(0)
    
    try:
        while True:
            for dc in range(0, 101, 5):
                pwm_led.ChangeDutyCycle(dc)
                time.sleep(0.05)
            for dc in range(100, -1, -5):
                pwm_led.ChangeDutyCycle(dc)
                time.sleep(0.05)
    except KeyboardInterrupt:
        pwm_led.stop()

# ---------------------------
# MAIN CONTROL LOOP
# ---------------------------
def main():
    camera_system = CameraSystem()
    
    # Start LED effect in background
    led_thread = threading.Thread(target=led_breathing_effect, daemon=True)
    led_thread.start()
    
    print("Ironman Helmet System Active")
    print("Touch sensors ready...")
    
    try:
        while True:
            # Check touch sensors
            action = check_touch_sensors()
            
            # Monitor battery
            voltage = read_battery_voltage()
            if voltage < 11.0:
                print(f"Warning: Low battery - {voltage}V")
            
            # Display status on 5" screen or smart lenses
            # (Add your display code here)
            
            time.sleep(0.1)  # Small delay to prevent CPU overload
            
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        # Cleanup
        for servo in servos:
            servo.stop()
        GPIO.cleanup()
        for cam in camera_system.cameras:
            cam.close()

if __name__ == "__main__":
    main()