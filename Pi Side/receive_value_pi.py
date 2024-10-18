from flask import Flask, request
from piservo import Servo
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
SERVO_PIN = 12
myservo = Servo(SERVO_PIN)


last_angle = 57


def move_stepper(steps, delay):
    # Set the direction
    print("Steps: ", steps)
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


def map_value(x, in_min=0, in_max=180, out_min=0, out_max=180):
    return ((x - in_min) * (out_max - out_min)) / (in_max - in_min) + out_min


def set_angle(deviation):
    global last_angle
    print("Current Deviation:", deviation)
    if abs(deviation) >= 3:
        if deviation > 0:
            last_angle += 2
        elif deviation < 0:
            last_angle -= 2
        if last_angle < 30:
            last_angle = 30
        elif last_angle > 150:
            last_angle = 150

        print("New Angle:", last_angle)
        myservo.write(last_angle)


set_angle(3)

app = Flask(__name__)


@app.route('/data', methods=['POST'])
def receive_data():
    data = request.json
    # print(data["stepper"], data["servo"])
    move_stepper(data["stepper"]//8, 0.003)  # Rotate Stepper
    # move_stepper(data["stepper"]//5, 0.004)  # Rotate Stepper
    set_angle(data["servo"])  # Rotate Servo
    return "Data received", 200


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5002)  # Run on port 5000
