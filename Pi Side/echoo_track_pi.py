import cv2
from cvzone.FaceDetectionModule import FaceDetector
import pyfirmata
import numpy as np
import RPi.GPIO as GPIO
import time

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)


# GPIO pins - STEPPER
DIR_PIN = 27   # Direction pin
STEP_PIN = 17  # Step pin

GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(STEP_PIN, GPIO.OUT)


# GPIO pin - SERVO
SERVO_PIN = 16
GPIO.setup(SERVO_PIN, GPIO.OUT)

# Create a PWM instance
pwm = GPIO.PWM(SERVO_PIN, 50)  # 50Hz frequency
pwm.start(0)  # Start PWM with 0% duty cycle

cap = cv2.VideoCapture(4)
ws, hs = 1280, 720
cap.set(3, ws)
cap.set(4, hs)

if not cap.isOpened():
    print("Camera couldn't Access!!!")
    exit()

detector = FaceDetector()
servoPos = [90, 90]  # initial servo position

frame_drop_const = 10
frame_drop = 0


def map_value(x, in_min=0, in_max=180, out_min=30, out_max=60):
    return ((x - in_min) * (out_max - out_min)) / (in_max - in_min) + out_min


def set_angle(angle):
    angle = map_value(angle)
    print("Mapped Angle Insidee:", angle)
    duty_cycle = (angle / 18) + 2.5  # Convert angle to duty cycle
    print(duty_cycle)
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.2)  # Wait for the servo to reach the position
    pwm.ChangeDutyCycle(0)  # Stop sending signals


def move_stepper(steps, delay):
    # Set the direction
    if steps > 0:
        GPIO.output(DIR_PIN, GPIO.HIGH)
    else:
        GPIO.output(DIR_PIN, GPIO.LOW)

    # Move the motor
    for _ in range(abs(steps)):
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(delay)


try:
    set_angle(0)
    while True:
        success, img = cap.read()
        if frame_drop % frame_drop_const == 0:
            frame_drop = 0
            img, bboxs = detector.findFaces(img, draw=False)

            if bboxs:
                # get the coordinate
                fx, fy = bboxs[0]["center"][0], bboxs[0]["center"][1]
                pos = [fx, fy]
                # convert coordinat to servo degree
                servoX = np.interp(fx, [0, ws], [0, 180])
                servoY = np.interp(fy, [0, hs], [0, 180])

                # servoY = map_value(servoY, 0, 180, 180, 0)
                print("SERVO Y:", servoY)

                if servoX < 0:
                    servoX = 0
                elif servoX > 180:
                    servoX = 180
                if servoY < 0:
                    servoY = 0
                elif servoY > 180:
                    servoY = 180

                servoPos[0] = servoX
                servoPos[1] = servoY

                if servoX < 80:
                    print("Clockwise")
                    move_stepper(100, 0.0001)  # Rotate forward
                elif servoX > 100:
                    print("Anti - Clockwise")
                    move_stepper(-100, 0.0001)  # Rotate forward

                if servoY < 80 or servoY > 100:
                    print("Angle:", servoY)
                    servoY = map_value(servoY, 0, 180, 180, 0)
                    print("Mapped Angle:", servoY)
                    set_angle(servoY)

                cv2.circle(img, (fx, fy), 80, (0, 0, 255), 2)
                cv2.putText(img, str(pos), (fx+15, fy-15),
                            cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
                cv2.line(img, (0, fy), (ws, fy), (0, 0, 0), 2)  # x line
                cv2.line(img, (fx, hs), (fx, 0), (0, 0, 0), 2)  # y line

                # cv2.line(img, (0, fy-10), (ws, fy-10), (0, 0, 0), 2)  # x line
                # cv2.line(img, (fx-10, hs), (fx-10, 0), (0, 0, 0), 2)  # y line

                # cv2.line(img, (0, fy+10), (ws, fy+10), (0, 0, 0), 2)  # x line
                # cv2.line(img, (fx+10, hs), (fx+10, 0), (0, 0, 0), 2)  # y line

                cv2.circle(img, (fx, fy), 15, (0, 0, 255), cv2.FILLED)
                cv2.putText(img, "TARGET LOCKED", (850, 50),
                            cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)

            else:
                cv2.putText(img, "NO TARGET", (880, 50),
                            cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3)
                cv2.circle(img, (640, 360), 80, (0, 0, 255), 2)
                cv2.circle(img, (640, 360), 15, (0, 0, 255), cv2.FILLED)
                cv2.line(img, (0, 360), (ws, 360), (0, 0, 0), 2)  # x line
                cv2.line(img, (640, hs), (640, 0), (0, 0, 0), 2)  # y line

            cv2.putText(img, f'Servo X: {int(servoPos[0])} deg',
                        (50, 50), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 1)
            cv2.putText(img, f'Servo Y: {int(servoPos[1])} deg',
                        (50, 100), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 1)

            cv2.imshow("Image", img)
            cv2.waitKey(1)
        frame_drop += 1
except KeyboardInterrupt:
    print("!!")
    pass
finally:
    # pwm.stop()  # Stop PWM
    GPIO.cleanup()  # Clean up GPIO settings
