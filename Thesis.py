import torch
import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
from collections import Counter
from adafruit_servokit import ServoKit

# Create the PCA9685 PWM driver object
kit = ServoKit(channels=16)

def set_servo_angle(servo, angle):
    servo.angle = angle

servo1 = kit.servo[0]
servo2 = kit.servo[1]
servo3 = kit.servo[2]
servo4 = kit.servo[3]

set_servo_angle(servo1, 45)
set_servo_angle(servo2, 0)
set_servo_angle(servo3, 0)
set_servo_angle(servo4, 0)

# Load YOLOv5 model
cap = cv2.VideoCapture(0)
ret, frame = cap.read()# Use 0 for the default camera, change accordingly if you have multiple cameras
model_path = '/home/adrianPogi/Downloads/try.pt'
model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)

# GPIO pin configuration for motor
ENA = 18  # PWM pin
IN1 = 17
IN2 = 27

# GPIO pin configuration for ultrasonic sensor
TRIG_PIN = 22  # GPIO pin 22 for Trig
ECHO_PIN = 23  # GPIO pin 23 for Echo

# Setup GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Setup motor pins
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

# Initialize PWM on ENA pin with 1000Hz frequency
pwm = GPIO.PWM(ENA, 1000)
pwm.start(0)

# Setup ultrasonic sensor pins
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

def set_motor_speed(speed):
    if speed > 100:
        speed = 100
    elif speed < 0:
        speed = 0
    pwm.ChangeDutyCycle(speed)

def motor_forward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)

def stop_motor():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)

def measure_distance():
    # Send a 10us pulse to trigger the measurement
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)  # 10us
    GPIO.output(TRIG_PIN, False)

    # Wait for the echo response
    start_time = time.time()
    while GPIO.input(ECHO_PIN) == 0:
        start_time = time.time()
    while GPIO.input(ECHO_PIN) == 1:
        end_time = time.time()

    # Calculate the distance
    duration = end_time - start_time
    distance = (duration * 34300) / 2

    return distance

def get_most_frequent_label(labels):
    if labels:
        return Counter(labels).most_common(1)[0][0]
    return None

def sorter():
    if diameter_cm <= 5:
        set_servo_angle(servo1, 0)
        set_servo_angle(servo2, 80)
        time.sleep(1)
        set_servo_angle(servo1, 45)
        set_servo_angle(servo2, 0)
        time.sleep(1)
        set_servo_angle(servo2, 0)
    elif diameter_cm > 5 or diameter_cm <= 8:
        set_servo_angle(servo1, 0)
        set_servo_angle(servo3, 80)
        time.sleep(2)
        set_servo_angle(servo1, 45)
        set_servo_angle(servo3, 0)
        time.sleep(1)
        set_servo_angle(servo3, 0)
    elif diameter_cm >= 9:
        set_servo_angle(servo1, 0)
        set_servo_angle(servo4, 80)
        time.sleep(1)
        set_servo_angle(servo1, 45)
        set_servo_angle(servo4, 0)
        time.sleep(1)
        set_servo_angle(servo4, 0)
        

# Define the scaling factor
scaling_factor = 21 / 1792  # inches per pixel
inches_to_cm = 2.54  # Conversion factor from inches to centimeters

try:
    running = True
    while True:
        distance = measure_distance()
        print("Distance: {:.2f} cm".format(distance))

        if distance < 12:
            time.sleep(0.4)
            stop_motor()
            print("Obstacle detected! Motor stopped.")

            # Capture labels for 5 seconds and determine the most frequent one
            labels = []
            start_time = time.time()
            while time.time() - start_time < 5:
                ret, frame = cap.read()
                if not ret:
                    break

                # Perform object detection
                results = model(frame)
                detections = results.xyxy[0].cpu().numpy()  # Get x1, y1, x2, y2, confidence, class

                if len(detections) > 0:
                    for det in detections:
                        x1, y1, x2, y2, conf, cls = det
                        label = model.names[int(cls)]
                        labels.append(label)

                        # Measure diameter at the 4th second
                        if time.time() - start_time >= 4:
                            bbox_width = x2 - x1
                            bbox_height = y2 - y1
                            diameter_px = min(bbox_width, bbox_height)
                            diameter_inch = diameter_px * scaling_factor
                            diameter_cm = diameter_inch * inches_to_cm
                            print(f"Diameter: {diameter_cm:.2f} cm")
                            sorter()

            # Get the most frequent label
            most_frequent_label = get_most_frequent_label(labels)
            if most_frequent_label:
                print(f"Detected Object: {most_frequent_label}")

            # Restart the motor after 5 seconds
            running = True

        if running:
            motor_forward()
            set_motor_speed(60)  # Constant speed of 70

        time.sleep(0.5)  # Small delay to avoid rapid looping

except KeyboardInterrupt:
    pass

finally:
    pwm.stop()
    GPIO.cleanup()
    cap.release()
    cv2.destroyAllWindows()
