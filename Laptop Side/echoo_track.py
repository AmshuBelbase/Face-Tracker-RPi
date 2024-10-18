import cv2
from cvzone.FaceDetectionModule import FaceDetector
import pyfirmata
import numpy as np
import time
import requests

url = 'http://192.168.232.23:5002/data'  # Replace with Pi's IP

cap = cv2.VideoCapture(2)
ws, hs = 1280, 720
cap.set(3, ws)
cap.set(4, hs)

if not cap.isOpened():
    print("Camera couldn't Access!!!")
    exit()

detector = FaceDetector()
servoPos = [90, 90]  # initial servo position

frame_drop_const = 1
frame_drop = 0


def map_value(x, in_min=0, in_max=180, out_min=30, out_max=60):
    return ((x - in_min) * (out_max - out_min)) / (in_max - in_min) + out_min


try:
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
                # print("SERVO Y:", servoY)

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

                stepper_v = 0
                servo_v = 0
                send_flag = 0

                if servoX < 80:
                    stepper_v = int(abs(90 - servoX)*1)
                    send_flag = 1
                elif servoX > 100:
                    stepper_v = int(abs(90 - int(servoX))*-1)
                    send_flag = 1

                if servoY < 70 or servoY > 110:
                    servoY = map_value(servoY, 0, 180, 90, 0)
                    servo_v = servoY - 45
                    send_flag = 1

                if send_flag:
                    data = {
                        'stepper': stepper_v,
                        'servo': servo_v
                    }
                    print(data)
                    response = requests.post(url, json=data)
                    print(f"Response: {response.text}")

                cv2.circle(img, (fx, fy), 80, (0, 0, 255), 2)
                cv2.putText(img, str(pos), (fx+15, fy-15),
                            cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
                cv2.line(img, (0, fy), (ws, fy), (0, 0, 0), 2)  # x line
                cv2.line(img, (fx, hs), (fx, 0), (0, 0, 0), 2)  # y line

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
    # GPIO.cleanup()  # Clean up GPIO settings
    pass
