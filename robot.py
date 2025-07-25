#!/usr/bin/env python3
"""
Obstacle‐avoiding rover for Raspberry Pi 3B + Pi Cam + L298N + 4× DC motors.

Wiring (example GPIO assignments; change as needed):

  Left motor channel (drives 2 motors in parallel):
    IN1 (forward):     GPIO17
    IN2 (backward):    GPIO18
    ENA (PWM speed):   GPIO22

  Right motor channel (drives other 2 motors):
    IN3 (forward):     GPIO23
    IN4 (backward):    GPIO24
    ENB (PWM speed):   GPIO25
"""

import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
import sys

# -----------------------------
# GPIO pin configuration
# -----------------------------
LEFT_IN1   = 17  # Left channel forward
LEFT_IN2   = 18  # Left channel backward
LEFT_EN    = 22  # Left channel PWM

RIGHT_IN1  = 23  # Right channel forward
RIGHT_IN2  = 24  # Right channel backward
RIGHT_EN   = 25  # Right channel PWM

PWM_FREQ   = 100  # Hz

# obstacle detection parameters
FRAME_WIDTH       = 320
FRAME_HEIGHT      = 240
BLUR_KERNEL       = (5, 5)
THRESHOLD_VALUE   = 60
MIN_CONTOUR_AREA  = 500  # pixels

def setup_gpio():
    """Initialise GPIO pins and PWM channels."""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # set motor pins as outputs
    motor_pins = [LEFT_IN1, LEFT_IN2, LEFT_EN, RIGHT_IN1, RIGHT_IN2, RIGHT_EN]
    for pin in motor_pins:
        GPIO.setup(pin, GPIO.OUT)

    # set up PWM on enable pins
    global pwm_left, pwm_right
    pwm_left  = GPIO.PWM(LEFT_EN,  PWM_FREQ)
    pwm_right = GPIO.PWM(RIGHT_EN, PWM_FREQ)
    pwm_left.start(0)    # initially stopped
    pwm_right.start(0)

def cleanup_gpio():
    """Stop motors and clean up GPIO."""
    stop()
    pwm_left.stop()
    pwm_right.stop()
    GPIO.cleanup()

# -----------------------------
# Motor control primitives
# -----------------------------
def forward(speed=70):
    """Drive both channels forward."""
    GPIO.output(LEFT_IN1, GPIO.HIGH)
    GPIO.output(LEFT_IN2, GPIO.LOW)
    GPIO.output(RIGHT_IN1, GPIO.HIGH)
    GPIO.output(RIGHT_IN2, GPIO.LOW)
    pwm_left.ChangeDutyCycle(speed)
    pwm_right.ChangeDutyCycle(speed)

def backward(speed=70):
    """Drive both channels backward."""
    GPIO.output(LEFT_IN1, GPIO.LOW)
    GPIO.output(LEFT_IN2, GPIO.HIGH)
    GPIO.output(RIGHT_IN1, GPIO.LOW)
    GPIO.output(RIGHT_IN2, GPIO.HIGH)
    pwm_left.ChangeDutyCycle(speed)
    pwm_right.ChangeDutyCycle(speed)

def turn_left(speed=70):
    """In‐place left turn: left wheels backward, right wheels forward."""
    GPIO.output(LEFT_IN1, GPIO.LOW)
    GPIO.output(LEFT_IN2, GPIO.HIGH)
    GPIO.output(RIGHT_IN1, GPIO.HIGH)
    GPIO.output(RIGHT_IN2, GPIO.LOW)
    pwm_left.ChangeDutyCycle(speed)
    pwm_right.ChangeDutyCycle(speed)

def turn_right(speed=70):
    """In‐place right turn: right wheels backward, left wheels forward."""
    GPIO.output(LEFT_IN1, GPIO.HIGH)
    GPIO.output(LEFT_IN2, GPIO.LOW)
    GPIO.output(RIGHT_IN1, GPIO.LOW)
    GPIO.output(RIGHT_IN2, GPIO.HIGH)
    pwm_left.ChangeDutyCycle(speed)
    pwm_right.ChangeDutyCycle(speed)

def stop():
    """Halt all motion."""
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)
    GPIO.output(LEFT_IN1, GPIO.LOW)
    GPIO.output(LEFT_IN2, GPIO.LOW)
    GPIO.output(RIGHT_IN1, GPIO.LOW)
    GPIO.output(RIGHT_IN2, GPIO.LOW)

# -----------------------------
# Obstacle detection + main loop
# -----------------------------
def detect_and_avoid():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("ERROR: Could not open camera.")
        sys.exit(1)

    # set resolution small for speed
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    time.sleep(2)  # allow camera to warm up

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("WARNING: empty frame.")
                continue

            # preprocess
            gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur  = cv2.GaussianBlur(gray, BLUR_KERNEL, 0)
            _, thresh = cv2.threshold(blur,
                                      THRESHOLD_VALUE,
                                      255,
                                      cv2.THRESH_BINARY_INV)

            # find obstacles
            contours, _ = cv2.findContours(thresh,
                                           cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                # pick largest contour
                largest = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest)
                if area > MIN_CONTOUR_AREA:
                    x, y, w, h = cv2.boundingRect(largest)
                    cx = x + w / 2

                    # debugging overlay
                    cv2.rectangle(frame, (x, y), (x + w, y + h),
                                  (0, 255, 0), 2)
                    cv2.line(frame,
                             (FRAME_WIDTH // 2, 0),
                             (FRAME_WIDTH // 2, FRAME_HEIGHT),
                             (255, 0, 0), 1)

                    # decide turn direction
                    if cx < FRAME_WIDTH / 2:
                        turn_right()
                    else:
                        turn_left()
                else:
                    # too small → keep going
                    forward()
            else:
                # no obstacles → go straight
                forward()

            # (optional) show for debugging; remove on headless Pi
            cv2.imshow("Frame", frame)
            cv2.imshow("Threshold", thresh)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()

def main():
    setup_gpio()
    try:
        detect_and_avoid()
    finally:
        cleanup_gpio()

if __name__ == "__main__":
    main()
