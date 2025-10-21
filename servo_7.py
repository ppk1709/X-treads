import pygame
import serial
import time
import threading
import RPi.GPIO as GPIO
import smbus

# === USER CONFIGURABLE PARAMETERS ===
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 720
FONT_SIZE = 28
ARROW_FONT_SIZE = 48
LOG_MAX = 12
PWM_FREQ = 50
MIN_ANGLE = 0
MAX_ANGLE = 180
DEFAULT_SERVO_ANGLE = 180
DEFAULT_NEUTRAL_ANGLE = 90
DUTY_OFFSET = 2.5
DUTY_SCALE = 1/18
CALIBRATE_KEY = pygame.K_c
ZERO_KEY = pygame.K_z
NEUTRAL_KEY = pygame.K_n
MAP_KEY = pygame.K_m
EMERGENCY_KEY = pygame.K_SPACE
CAL_SELECT_KEYS = [pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4]
CAL_ADJUST_LEFT = pygame.K_LEFT
CAL_ADJUST_RIGHT = pygame.K_RIGHT
CAL_ADJUST_STEP = 1
CAL_CYCLE_DIAG_KEY = pygame.K_TAB
ARROW_ACTIVE_COLOR = (0, 255, 0)
ARROW_INACTIVE_COLOR = (100, 100, 100)

# === SERIAL CONFIG ===
SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 9600
try:
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
    print(f"[INFO] Connected to {SERIAL_PORT} at {BAUDRATE} baud.")
except Exception as e:
    print(f"[ERROR] Could not open serial port: {e}")
    exit(1)

# === IMU CONFIG (MPU6050) ===
I2C_ADDR = 0x68
bus = smbus.SMBus(1)
bus.write_byte_data(I2C_ADDR, 0x6B, 0)
imu_logs = []

# === PYGAME SETUP ===
pygame.init()
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("X-Tread Controller (Directions & Calibration)")
font = pygame.font.SysFont(None, FONT_SIZE)
arrow_font = pygame.font.SysFont(None, ARROW_FONT_SIZE)

# === TOKENS ===
DIRECTIONS = {'N', 'S', 'E', 'W', 'NE', 'NW', 'SE', 'SW'}
STOP_TOKEN = 'STOP'

# === SERVO CONFIG ===
SERVO_PINS = {'X': 17, 'Y': 13, 'Z': 22, 'E': 23}
servo_states = {k: DEFAULT_NEUTRAL_ANGLE for k in SERVO_PINS}
servo_duties = {k: (DEFAULT_NEUTRAL_ANGLE * DUTY_SCALE) + DUTY_OFFSET for k in SERVO_PINS}

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
servo_pwms = {}
for key, pin in SERVO_PINS.items():
    GPIO.setup(pin, GPIO.OUT)
    pwm = GPIO.PWM(pin, PWM_FREQ)
    pwm.start(servo_duties[key])
    servo_pwms[key] = pwm

# Base diagonal mapping, dynamic copy for runtime edits
BASE_DIAGONAL_SERVO_MAP = {
    'NE': ['Y', 'E'],
    'NW': ['X', 'Z'],
    'SE': ['Z', 'E'],
    'SW': ['X', 'E']
}
diag_servo_map = {k: list(v) for k, v in BASE_DIAGONAL_SERVO_MAP.items()}
DIAGONALS = ['NE', 'NW', 'SE', 'SW']

# === HELPERS ===
def set_servo_angle(key, angle):
    angle = max(MIN_ANGLE, min(MAX_ANGLE, angle))
    duty = (angle * DUTY_SCALE) + DUTY_OFFSET
    servo_pwms[key].ChangeDutyCycle(duty)
    servo_states[key] = angle
    servo_duties[key] = duty


def release_all_servos():
    for key in SERVO_PINS:
        set_servo_angle(key, DEFAULT_NEUTRAL_ANGLE)


def zero_all_servos():
    for key in SERVO_PINS:
        set_servo_angle(key, MIN_ANGLE)


def activate_diagonal(cmd):
    for key in diag_servo_map.get(cmd, []):
        set_servo_angle(key, DEFAULT_SERVO_ANGLE)

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
    ax, ay, az = conv(data[0], data[1]), conv(data[2], data[3]), conv(data[4], data[5])
    gx, gy, gz = conv(data[8], data[9]), conv(data[10], data[11]), conv(data[12], data[13])
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

serial_logs = []
threading.Thread(target=serial_monitor, daemon=True).start()
threading.Thread(target=imu_monitor, daemon=True).start()

# === CALIBRATION STATE ===
calibrating = False
mapping_mode = False
servo_keys = list(SERVO_PINS.keys())
selected_servo_idx = 0
diag_selected_idx = 0

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

    # UI buttons
    help_text = (f"[C] Calibrate ({'Map' if mapping_mode else 'Angle'})  "
                 f"[M] Toggle Map Mode  [Z] Zero  [N] Neutral  [SPACE] Stop")
    screen.blit(font.render(help_text, True, (200, 200, 200)),
                (30, WINDOW_HEIGHT - FONT_SIZE - 10))

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == CALIBRATE_KEY:
                calibrating = not calibrating
                mapping_mode = False
            elif event.key == MAP_KEY and calibrating:
                mapping_mode = not mapping_mode
            elif event.key == ZERO_KEY:
                zero_all_servos()
                print("[INFO] All servos set to zero.")
            elif event.key == NEUTRAL_KEY:
                release_all_servos()
                print(f"[INFO] Servos returned to neutral ({DEFAULT_NEUTRAL_ANGLE}°).")
            elif event.key == EMERGENCY_KEY:
                send_command(STOP_TOKEN)
                release_all_servos()
                current_direction = None

            if calibrating:
                if mapping_mode:
                    # Mapping calibration: TAB to cycle diagonals, 1-4 toggle servo
                    if event.key == CAL_CYCLE_DIAG_KEY:
                        diag_selected_idx = (diag_selected_idx + 1) % len(DIAGONALS)
                    elif event.key in CAL_SELECT_KEYS:
                        idx = CAL_SELECT_KEYS.index(event.key)
                        servo = servo_keys[idx]
                        diag = DIAGONALS[diag_selected_idx]
                        if servo in diag_servo_map[diag]:
                            diag_servo_map[diag].remove(servo)
                        else:
                            diag_servo_map[diag].append(servo)
                        print(f"[INFO] Mapping {diag}: {diag_servo_map[diag]}")
                else:
                    # Angle calibration: 1-4 select servo, arrows adjust
                    if event.key in CAL_SELECT_KEYS:
                        selected_servo_idx = CAL_SELECT_KEYS.index(event.key)
                    elif event.key == CAL_ADJUST_LEFT:
                        key = servo_keys[selected_servo_idx]
                        set_servo_angle(key, servo_states[key] - CAL_ADJUST_STEP)
                    elif event.key == CAL_ADJUST_RIGHT:
                        key = servo_keys[selected_servo_idx]
                        set_servo_angle(key, servo_states[key] + CAL_ADJUST_STEP)
                continue

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
                release_all_servos()
                activate_diagonal(new_direction)
            elif current_direction:
                send_command(STOP_TOKEN)
                release_all_servos()
            current_direction = new_direction

    # Render direction label
    screen.blit(font.render(f"Direction: {current_direction or 'STOPPED'}", True, (200,200,200)), (30,20))

    # Render arrows
    cx, cy = WINDOW_WIDTH//2, WINDOW_HEIGHT//3
    ofs = 80
    arrow_map = {
        'N': ('↑', (cx, cy-ofs)),
        'S': ('↓', (cx, cy+ofs)),
        'W': ('←', (cx-ofs, cy)),
        'E': ('→', (cx+ofs, cy)),
        'NE': ('↗', (cx+ofs, cy-ofs)),
        'NW': ('↖', (cx-ofs, cy-ofs)),
        'SE': ('↘', (cx+ofs, cy+ofs)),
        'SW': ('↙', (cx-ofs, cy+ofs))
    }
    for d, (char, pos) in arrow_map.items():
        color = ARROW_ACTIVE_COLOR if d == current_direction else ARROW_INACTIVE_COLOR
        surf = arrow_font.render(char, True, color)
        rect = surf.get_rect(center=pos)
        screen.blit(surf, rect)

    # Servo states & PWM display
    for i, key in enumerate(servo_keys):
        txt = f"{key}: {servo_states[key]}° ({servo_duties[key]:.2f} duty)"
        screen.blit(font.render(txt, True, (150,255,150)), (30, 100 + i*30))

    # Calibration UI
    if calibrating:
        mode = "Mapping" if mapping_mode else "Angle"
        screen.blit(font.render(f"[CALIBRATION MODE - {mode}]", True, (255,200,50)), (30,140))
        if mapping_mode:
            screen.blit(font.render("[TAB] Cycle Dir  [1-4] Toggle Servo", True, (200,200,100)), (30,170))
            for i, diag in enumerate(DIAGONALS):
                color = (50,255,50) if i==diag_selected_idx else (200,200,200)
                txt = f"{diag}: {diag_servo_map[diag]} (Neutral: {DEFAULT_NEUTRAL_ANGLE}°, Active: {DEFAULT_SERVO_ANGLE}°)"
                screen.blit(font.render(txt, True, color), (30,200 + i*30))
        else:
            screen.blit(font.render("[1-4] Select Servo  [←/→] Adjust", True, (200,200,100)), (30,170))
            for i, key in enumerate(servo_keys):
                color = (50,255,50) if i==selected_servo_idx else (200,200,200)
                txt = f"{i+1}. {key}: {servo_states[key]}° (Duty: {servo_duties[key]:.2f})"
                screen.blit(font.render(txt, True, color), (30,200 + i*30))

    # Serial logs
    screen.blit(font.render("Serial Monitor:", True, (180,180,180)), (30,350))
    for i, log in enumerate(serial_logs):
        screen.blit(font.render(log, True, (150,150,255)), (30,380 + i*25))

    # IMU logs
    screen.blit(font.render("IMU Monitor:", True, (180,180,180)), (WINDOW_WIDTH//2+30, 350))
    for i, log in enumerate(imu_logs):
        screen.blit(font.render(log, True, (255,150,150)), (WINDOW_WIDTH//2+30,380 + i*25))

    pygame.display.flip()
    clock.tick(30)

# === CLEANUP ===
pygame.quit()
ser.close()
for pwm in servo_pwms.values(): pwm.stop()
GPIO.cleanup()
