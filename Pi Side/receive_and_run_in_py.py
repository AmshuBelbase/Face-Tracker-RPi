from flask import Flask, request
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


last_angle = 0


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


def map_value(x, in_min=0, in_max=180, out_min=0, out_max=180):
    return ((x - in_min) * (out_max - out_min)) / (in_max - in_min) + out_min

# def set_angle(angle):
#     global last_angle
#     angle = map_value(angle)
#     print("Last Angle:", last_angle)
#     print("Mapped Angle Insidee:", angle)
#     if abs(last_angle-angle) >= 15:
#         last_angle = angle
#         duty_cycle = (angle / 18) + 2.5  # Convert angle to duty cycle
#         print(duty_cycle)
#         pwm.ChangeDutyCycle(duty_cycle)
#         time.sleep(0.1)  # Wait for the servo to reach the position
#         pwm.ChangeDutyCycle(0)  # Stop sending signals


def set_angle(deviation):
    global last_angle
    print("Current Deviation:", deviation)
    if abs(deviation) >= 25:
        if deviation > 0:
            last_angle += 2
        elif deviation < 0:
            last_angle -= 2
        if last_angle < 30:
            last_angle = 30
        elif last_angle > 150:
            last_angle = 150

        duty_cycle = (last_angle / 18) + 2.5  # Convert angle to duty cycle
        print(duty_cycle)
        pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(0.1)  # Wait for the servo to reach the position
        pwm.ChangeDutyCycle(0)  # Stop sending signals


set_angle(30)

app = Flask(__name__)


@app.route('/data', methods=['POST'])
def receive_data():
    data = request.json
    print(data["stepper"], data["servo"])
    move_stepper(data["stepper"], 0.0002)  # Rotate Stepper
    set_angle(data["servo"])  # Rotate Servo
    return "Data received", 200


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)  # Run on port 5000
