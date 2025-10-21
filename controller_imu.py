'''
    Read Gyro and Accelerometer and Control 4 Servos using Raspberry Pi
'''
import smbus
import RPi.GPIO as GPIO
from time import sleep

# MPU6050 Registers and Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

# Initialize MPU6050
def MPU_Init():
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    bus.write_byte_data(Device_Address, CONFIG, 0)
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

# Read raw 16-bit value
def read_raw_data(addr):
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr+1)
    value = ((high << 8) | low)
    if(value > 32768):
        value = value - 65536
    return value

# Servo setup
servo_pins = [17, 18, 22, 23]  # GPIO pins for servos
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Initialize PWM at 50Hz (standard for servo motors)
servos = []
for pin in servo_pins:
    GPIO.setup(pin, GPIO.OUT)
    pwm = GPIO.PWM(pin, 50)
    pwm.start(7.5)  # Neutral position (90 degrees)
    servos.append(pwm)

# MPU6050 setup
bus = smbus.SMBus(1)  # or smbus.SMBus(0) for older boards
Device_Address = 0x68
MPU_Init()

print("Reading Data of Gyroscope and Accelerometer and Controlling Servos")

def map_value(x, in_min, in_max, out_min, out_max):
    # Simple mapping function
    return max(min((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min, out_max), out_min)

try:
    while True:
        # Read Accelerometer raw value
        acc_x = read_raw_data(ACCEL_XOUT_H)
        acc_y = read_raw_data(ACCEL_YOUT_H)
        acc_z = read_raw_data(ACCEL_ZOUT_H)

        # Read Gyroscope raw value
        gyro_x = read_raw_data(GYRO_XOUT_H)
        gyro_y = read_raw_data(GYRO_YOUT_H)
        gyro_z = read_raw_data(GYRO_ZOUT_H)

        # Convert to proper units
        Ax = acc_x / 16384.0
        Ay = acc_y / 16384.0
        Az = acc_z / 16384.0

        Gx = gyro_x / 131.0
        Gy = gyro_y / 131.0
        Gz = gyro_z / 131.0

        # Print sensor data
        print("Gx=%.2f" % Gx, u'\u00b0' + "/s", "\tGy=%.2f" % Gy, u'\u00b0' + "/s", "\tGz=%.2f" % Gz, u'\u00b0' + "/s",
              "\tAx=%.2f g" % Ax, "\tAy=%.2f g" % Ay, "\tAz=%.2f g" % Az)

        # Map accelerometer X and Y to servo duty cycle (between 5% and 10% for 0-180 degrees)
        servo1_duty = map_value(Ax, -2, 2, 5, 10)
        servo2_duty = map_value(Ay, -2, 2, 5, 10)
        servo3_duty = map_value(Gx, -250, 250, 5, 10)
        servo4_duty = map_value(Gy, -250, 250, 5, 10)

        # Update servos
        servos[0].ChangeDutyCycle(servo1_duty)
        servos[1].ChangeDutyCycle(servo2_duty)
        servos[2].ChangeDutyCycle(servo3_duty)
        servos[3].ChangeDutyCycle(servo4_duty)

        sleep(0.1)

except KeyboardInterrupt:
    print("\nExiting...")

finally:
    for servo in servos:
        servo.stop()
    GPIO.cleanup()
