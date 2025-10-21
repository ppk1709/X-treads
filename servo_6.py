import pygame
import serial
import time
import threading
import RPi.GPIO as GPIO
import smbus

# === USER CONFIGURABLE PARAMETERS ===
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 480
FONT_SIZE = 28
LOG_MAX = 8
PWM_FREQ = 50
MIN_ANGLE = 0
MAX_ANGLE = 180
DEFAULT_SERVO_ANGLE = 180
DEFAULT_NEUTRAL_ANGLE = 90
DUTY_OFFSET = 2.5
DUTY_SCALE = 1/18
CALIBRATE_KEY = pygame.K_c
ZERO_KEY = pygame.K_z
EMERGENCY_KEY = pygame.K_SPACE
CAL_SELECT_KEYS = [pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4]
CAL_ADJUST_LEFT = pygame.K_LEFT
CAL_ADJUST_RIGHT = pygame.K_RIGHT
CAL_ADJUST_STEP = 1

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

# === PYGAME SETUP ===
pygame.init()
# right after pygame.init()
pygame.key.set_repeat(200, 50)  # (delay ms, interval ms)

screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("X-Tread Controller (Directions & Calibration)")
font = pygame.font.SysFont(None, FONT_SIZE)

# === TOKENS ===
DIRECTIONS = {'N', 'S', 'E', 'W', 'NE', 'NW', 'SE', 'SW'}
STOP_TOKEN = 'STOP'

# === SERVO CONFIG ===
SERVO_PINS = {'X': 17, 'Y': 13, 'Z': 22, 'E': 23}
servo_states = {k: DEFAULT_NEUTRAL_ANGLE for k in SERVO_PINS}

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Initialize PWM for each servo
servo_pwms = {}
for key, pin in SERVO_PINS.items():
    GPIO.setup(pin, GPIO.OUT)
    pwm = GPIO.PWM(pin, PWM_FREQ)
    pwm.start((DEFAULT_NEUTRAL_ANGLE * DUTY_SCALE) + DUTY_OFFSET)
    servo_pwms[key] = pwm

# Diagonal mappings
DIAGONAL_SERVO_MAP = {
    'NE': ['Y', 'E'],
    'NW': ['X', 'Z'],
    'SE': ['Z', 'E'],
    'SW': ['X', 'E']
}

# === HELPERS ===

def set_servo_angle(pwm, angle):
    # Map angle to duty cycle
    angle = max(MIN_ANGLE, min(MAX_ANGLE, angle))
    duty = (angle * DUTY_SCALE) + DUTY_OFFSET
    pwm.ChangeDutyCycle(duty)


def release_all_servos():
    for key, pwm in servo_pwms.items():
        set_servo_angle(pwm, DEFAULT_NEUTRAL_ANGLE)
        servo_states[key] = DEFAULT_NEUTRAL_ANGLE


def zero_all_servos():
    for key, pwm in servo_pwms.items():
        set_servo_angle(pwm, MIN_ANGLE)
        servo_states[key] = MIN_ANGLE


def activate_diagonal(cmd):
    for key in DIAGONAL_SERVO_MAP.get(cmd, []):
        set_servo_angle(servo_pwms[key], DEFAULT_SERVO_ANGLE)
        servo_states[key] = DEFAULT_SERVO_ANGLE

# === THREADS ===

def serial_monitor():
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    serial_logs.append(line)
                    if len(serial_logs) > LOG_MAX:
                        serial_logs.pop(0)
        except Exception as e:
            print(f"[ERROR] Serial read: {e}")
            break


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
    while True:
        try:
            ax, ay, az, gx, gy, gz = read_imu()
            imu_logs.append(f"A:({ax},{ay},{az}) G:({gx},{gy},{gz})")
            if len(imu_logs) > LOG_MAX:
                imu_logs.pop(0)
            time.sleep(0.1)
        except Exception as e:
            print(f"[ERROR] IMU read: {e}")
            break

# Start monitors
serial_logs = []
threading.Thread(target=serial_monitor, daemon=True).start()
threading.Thread(target=imu_monitor, daemon=True).start()

# === CALIBRATION STATE ===
calibrating = False
servo_keys = list(SERVO_PINS.keys())
selected_idx = 0

# === INPUT HELPERS ===
def send_command(cmd: str):
    ser.write((cmd + "\n").encode())
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
current_direction = None

while running:
    screen.fill((20, 20, 20))
    keys = pygame.key.get_pressed()

    # --- UI Buttons ---
    help_text = f"[{pygame.key.name(CALIBRATE_KEY).upper()}] Calibrate  [{pygame.key.name(ZERO_KEY).upper()}] Zero  [SPACE] Stop"
    help_surf = font.render(help_text, True, (200, 200, 200))
    screen.blit(help_surf, (30, WINDOW_HEIGHT - FONT_SIZE - 10))

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        # Toggle calibration mode
        if event.type == pygame.KEYDOWN and event.key == CALIBRATE_KEY:
            calibrating = not calibrating
            print(f"[INFO] Calibration {'ON' if calibrating else 'OFF'}")

        # Zero servos
        if event.type == pygame.KEYDOWN and event.key == ZERO_KEY:
            zero_all_servos()
            print("[INFO] All servos set to zero position.")

        if calibrating:
            if event.type == pygame.KEYDOWN:
                if event.key in CAL_SELECT_KEYS:
                    selected_idx = CAL_SELECT_KEYS.index(event.key)
                elif event.key == CAL_ADJUST_LEFT:
                    key = servo_keys[selected_idx]
                    servo_states[key] = max(MIN_ANGLE, servo_states[key] - CAL_ADJUST_STEP)
                    set_servo_angle(servo_pwms[key], servo_states[key])
                elif event.key == CAL_ADJUST_RIGHT:
                    key = servo_keys[selected_idx]
                    servo_states[key] = min(MAX_ANGLE, servo_states[key] + CAL_ADJUST_STEP)
                    set_servo_angle(servo_pwms[key], servo_states[key])
            continue

        if event.type == pygame.KEYDOWN and event.key == EMERGENCY_KEY:
            send_command(STOP_TOKEN)
            release_all_servos()
            current_direction = None

    if not calibrating:
        diag = get_diagonal_key(keys)
        if diag:
            new_direction = diag
        elif keys[pygame.K_UP]: new_direction = 'N'
        elif keys[pygame.K_DOWN]: new_direction = 'S'
        elif keys[pygame.K_LEFT]: new_direction = 'W'
        elif keys[pygame.K_RIGHT]: new_direction = 'E'
        else: new_direction = None

        if new_direction != current_direction:
            if new_direction in DIRECTIONS:
                send_command(new_direction)
                if current_direction in DIAGONAL_SERVO_MAP:
                    release_all_servos()
                if new_direction in DIAGONAL_SERVO_MAP:
                    activate_diagonal(new_direction)
            elif current_direction is not None:
                send_command(STOP_TOKEN)
                release_all_servos()
            current_direction = new_direction

    # --- RENDER ---
    label = font.render(f"Direction: {current_direction or 'STOPPED'}", True, (200, 200, 200))
    screen.blit(label, (30, 20))

    if calibrating:
        cal_text = font.render("[CALIBRATION MODE] Press 1-4 to select servo, <-/-> to adjust, C to exit", True, (255, 200, 50))
        screen.blit(cal_text, (30, 60))
        for i, key in enumerate(servo_keys):
            color = (50, 255, 50) if i == selected_idx else (200, 200, 200)
            txt = font.render(f"{i+1}. {key}: {servo_states[key]}°", True, color)
            screen.blit(txt, (50, 100 + i * 30))
    else:
        servo_title = font.render("Servo States:", True, (180, 180, 180))
        screen.blit(servo_title, (30, 60))
        for i, (k, angle) in enumerate(servo_states.items()):
            txt = font.render(f"{k}: {angle}°", True, (150, 255, 150))
            screen.blit(txt, (50, 90 + i * 25))

    log_title = font.render("Serial Monitor (Arduino):", True, (180, 180, 180))
    screen.blit(log_title, (30, 220))
    for i, log in enumerate(serial_logs):
        surf = font.render(log, True, (150, 150, 255))
        screen.blit(surf, (30, 250 + i * 25))

    imu_title = font.render("IMU Monitor (MPU6050):", True, (180, 180, 180))
    screen.blit(imu_title, (430, 20))
    for i, log in enumerate(imu_logs):
        surf = font.render(log, True, (255, 150, 150))
        screen.blit(surf, (430, 50 + i * 25))

    pygame.display.flip()
    clock.tick(30)

# === CLEANUP ===
pygame.quit()
ser.close()
for pwm in servo_pwms.values():
    pwm.stop()
GPIO.cleanup()
