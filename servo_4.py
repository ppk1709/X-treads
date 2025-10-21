import pygame
import serial
import time
import threading
import RPi.GPIO as GPIO
import smbus

# === SERIAL CONFIG ===
SERIAL_PORT = '/dev/ttyACM0'  # Change if needed
BAUDRATE = 9600
try:
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
    print(f"[INFO] Connected to {SERIAL_PORT} at {BAUDRATE} baud.")
except Exception as e:
    print(f"[ERROR] Could not open serial port: {e}")
    exit(1)

# === IMU (MPU6050) CONFIG ===
I2C_ADDR = 0x68
bus = smbus.SMBus(1)
# Wake up MPU6050
bus.write_byte_data(I2C_ADDR, 0x6B, 0)
imu_logs = []
LOG_MAX = 8

# === PYGAME SETUP ===
pygame.init()
# Increased window size for servo states and IMU logs
screen = pygame.display.set_mode((800, 480))
pygame.display.set_caption("X-Tread Controller (Directions Only)")
font = pygame.font.SysFont(None, 28)

# === TOKENS ===
DIRECTIONS = {'N', 'S', 'E', 'W', 'NE', 'NW', 'SE', 'SW'}
STOP_TOKEN = 'STOP'

current_direction = None
serial_logs = []

# === SERVO CONFIG ===
SERVO_ANGLE = 180  # Default actuation angle
NEUTRAL_ANGLE = 90  # Center position for MG995
SERVO_PINS = {'X': 17, 'Y': 13, 'Z': 22, 'E': 23}
# Track angle state for each servo
servo_states = {k: NEUTRAL_ANGLE for k in SERVO_PINS}

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Initialize PWM for each servo
servo_pwms = {}
for dir_key, pin in SERVO_PINS.items():
    GPIO.setup(pin, GPIO.OUT)
    pwm = GPIO.PWM(pin, 50)
    pwm.start(7.5)
    servo_pwms[dir_key] = pwm

# Servo pairs for each diagonal command
DIAGONAL_SERVO_MAP = {
    'NE': ['Y', 'E'],
    'NW': ['X', 'Z'],
    'SE': ['Z', 'E'],
    'SW': ['X', 'Z']
}

# === HELPERS ===
def set_servo_angle(pwm, angle):
    duty = (0.05 * angle) / 9 + 2.5
    pwm.ChangeDutyCycle(duty)

def activate_servos(cmd):
    for d in DIAGONAL_SERVO_MAP.get(cmd, []):
        set_servo_angle(servo_pwms[d], SERVO_ANGLE)
        servo_states[d] = SERVO_ANGLE

def release_servos(cmd):
    for d in DIAGONAL_SERVO_MAP.get(cmd, []):
        set_servo_angle(servo_pwms[d], NEUTRAL_ANGLE)
        servo_states[d] = NEUTRAL_ANGLE

# === SERIAL MONITOR THREAD ===
def serial_monitor():
    global serial_logs
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    serial_logs.append(line)
                    if len(serial_logs) > LOG_MAX:
                        serial_logs.pop(0)
        except Exception as e:
            print(f"[ERROR] Serial read error: {e}")
            break

# === IMU MONITOR THREAD ===
def read_imu():
    data = bus.read_i2c_block_data(I2C_ADDR, 0x3B, 14)
    def conv(msb, lsb):
        val = (msb << 8) | lsb
        return val - 65536 if val > 32767 else val

    ax = conv(data[0], data[1])
    ay = conv(data[2], data[3])
    az = conv(data[4], data[5])
    gx = conv(data[8], data[9])
    gy = conv(data[10], data[11])
    gz = conv(data[12], data[13])
    return ax, ay, az, gx, gy, gz


def imu_monitor():
    global imu_logs
    while True:
        try:
            ax, ay, az, gx, gy, gz = read_imu()
            log = f"A:({ax},{ay},{az}) G:({gx},{gy},{gz})"
            imu_logs.append(log)
            if len(imu_logs) > LOG_MAX:
                imu_logs.pop(0)
            time.sleep(0.1)
        except Exception as e:
            print(f"[ERROR] IMU read error: {e}")
            break

threading.Thread(target=serial_monitor, daemon=True).start()
threading.Thread(target=imu_monitor, daemon=True).start()

# === COMM / INPUT HELPERS ===
def send_command(cmd: str):
    ser.write((cmd + "\n").encode('utf-8'))
    print(f"[TX] {cmd}")

def get_diagonal_key(keys):
    if keys[pygame.K_UP] and keys[pygame.K_RIGHT]: return 'NE'
    if keys[pygame.K_UP] and keys[pygame.K_LEFT]: return 'NW'
    if keys[pygame.K_DOWN] and keys[pygame.K_RIGHT]: return 'SE'
    if keys[pygame.K_DOWN] and keys[pygame.K_LEFT]: return 'SW'
    return None

# === MAIN LOOP ===
clock = pygame.time.Clock()
running = True

while running:
    screen.fill((20, 20, 20))
    keys = pygame.key.get_pressed()
    diag = get_diagonal_key(keys)

    if diag:
        new_direction = diag
    elif keys[pygame.K_UP]: new_direction = 'N'
    elif keys[pygame.K_DOWN]: new_direction = 'S'
    elif keys[pygame.K_LEFT]: new_direction = 'W'
    elif keys[pygame.K_RIGHT]: new_direction = 'E'
    else: new_direction = None

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
            send_command(STOP_TOKEN)
            if current_direction in DIAGONAL_SERVO_MAP:
                release_servos(current_direction)
            current_direction = None

    if new_direction != current_direction:
        if new_direction in DIRECTIONS:
            send_command(new_direction)
            if current_direction in DIAGONAL_SERVO_MAP:
                release_servos(current_direction)
            if new_direction in DIAGONAL_SERVO_MAP:
                activate_servos(new_direction)
        elif current_direction is not None:
            send_command(STOP_TOKEN)
            if current_direction in DIAGONAL_SERVO_MAP:
                release_servos(current_direction)
        current_direction = new_direction

    # --- RENDER ---
    # Direction label
    label = font.render(f"Direction: {current_direction or 'STOPPED'}", True, (200, 200, 200))
    screen.blit(label, (30, 20))

    # Servo states display
    servo_title = font.render("Servo States:", True, (180, 180, 180))
    screen.blit(servo_title, (30, 60))
    for i, (k, angle) in enumerate(servo_states.items()):
        txt = font.render(f"{k}: {angle}°", True, (150, 255, 150))
        screen.blit(txt, (50, 90 + i * 25))

    # Serial logs
    log_title = font.render("Serial Monitor (Arduino):", True, (180, 180, 180))
    screen.blit(log_title, (30, 200))
    for i, log in enumerate(serial_logs):
        text_surface = font.render(log, True, (150, 150, 255))
        screen.blit(text_surface, (30, 230 + i * 25))

    # IMU logs on right
    imu_title = font.render("IMU Monitor (MPU6050):", True, (180, 180, 180))
    screen.blit(imu_title, (430, 60))
    for i, log in enumerate(imu_logs):
        txt = font.render(log, True, (255, 150, 150))
        screen.blit(txt, (430, 90 + i * 25))

    pygame.display.flip()
    clock.tick(30)

# === CLEANUP ===
pygame.quit()
ser.close()
for pwm in servo_pwms.values():
    pwm.stop()
GPIO.cleanup()
