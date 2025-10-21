import pygame
import serial
import time
import threading
import RPi.GPIO as GPIO
import smbus
import math

# === USER CONFIGURABLE PARAMETERS ===
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 720
FONT_SIZE = 24
ARROW_FONT_SIZE = 72
LOG_MAX = 12
PWM_FREQ = 50
MIN_ANGLE = 0
MAX_ANGLE = 180
DEFAULT_SERVO_ANGLE = 180
DEFAULT_NEUTRAL_ANGLE = 90
DUTY_OFFSET = 2.5
DUTY_SCALE = 1/18

# === INPUT KEYS ===
KEY_ANGLE = pygame.K_c        # Angle calibration mode
KEY_MAPPING = pygame.K_m      # Mapping mode
KEY_ZERO = pygame.K_z         # Zero all servos
KEY_NEUTRAL = pygame.K_n      # Neutral all servos
KEY_STOP = pygame.K_SPACE     # Emergency stop
KEY_IMU_INCREASE = pygame.K_o # Increase IMU rate
KEY_IMU_DECREASE = pygame.K_i # Decrease IMU rate
KEY_CYCLE_DIAG = pygame.K_TAB # Cycle diagonal in mapping
KEY_TOGGLE_SERVOS = [pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4]  # Toggle servo inclusion
KEY_ADJ_NEUTRAL_DEC = pygame.K_LEFT  # Adjust neutral angle (mapping mode)
KEY_ADJ_NEUTRAL_INC = pygame.K_RIGHT
KEY_ADJ_ACTIVE_INC = pygame.K_UP      # Adjust active angle (mapping mode)
KEY_ADJ_ACTIVE_DEC = pygame.K_DOWN
ADJUST_STEP = 1

IMU_INTERVAL = 0.1  # seconds between IMU reads

# === COLORS ===
COLOR_BG = (20, 20, 20)
COLOR_TEXT = (200, 200, 200)
COLOR_HIGHLIGHT = (50, 200, 50)
COLOR_SEPARATOR = (80, 80, 80)
COLOR_ARROW_ACTIVE = (0, 255, 0)
COLOR_ARROW_INACTIVE = (100, 100, 100)

# === SERIAL SETUP ===
SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 9600
try:
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
except Exception as e:
    print(f"[ERROR] Serial port open failed: {e}")
    exit(1)

# === IMU SETUP ===
I2C_ADDR = 0x68
bus = smbus.SMBus(1)
# Wake up MPU6050
bus.write_byte_data(I2C_ADDR, 0x6B, 0)
imu_logs = []
serial_logs = []

# === PYGAME INIT ===
pygame.init()
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("X-Tread Controller")
font = pygame.font.SysFont(None, FONT_SIZE)
narrow_font = pygame.font.SysFont(None, ARROW_FONT_SIZE)

arrow_font = pygame.font.SysFont(None, ARROW_FONT_SIZE)
clock = pygame.time.Clock()

# === TOKENS & STATES ===
DIRECTIONS = {'N', 'S', 'E', 'W', 'NE', 'NW', 'SE', 'SW'}
STOP_TOKEN = 'STOP'

# === SERVO SETUP ===
SERVO_PINS = {'X':17, 'Y':18, 'Z':22, 'E':23}
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

# === DIAGONAL MAPPING ===
diagonals = ['NE','NW','SE','SW']
diag_map = {d: list(v) for d, v in {'NE':['X','E'], 'NW':['Y','Z'], 'SE':['Z','Y'], 'SW':['X','E']}.items()}
diag_neutral = {d: DEFAULT_NEUTRAL_ANGLE for d in diagonals}
diag_active = {d: DEFAULT_SERVO_ANGLE for d in diagonals}

# === MODES & STATE ===
angle_mode = False
mapping_mode = False
sel_servo_idx = 0
sel_diag_idx = 0
current_dir = None
heading = 0.0  # integrated heading (degrees)

# === HELPERS ===
def set_servo_angle(key, angle):
    angle = max(MIN_ANGLE, min(MAX_ANGLE, angle))
    duty = (angle * DUTY_SCALE) + DUTY_OFFSET
    servo_pwms[key].ChangeDutyCycle(duty)
    servo_states[key] = angle
    servo_duties[key] = duty


def release_all():
    for k in SERVO_PINS:
        set_servo_angle(k, DEFAULT_NEUTRAL_ANGLE)


def zero_all():
    for k in SERVO_PINS:
        set_servo_angle(k, MIN_ANGLE)


def release_diag(diag):
    if diag in diag_map:
        for k in diag_map[diag]:
            set_servo_angle(k, diag_neutral[diag])


def activate_diag(diag):
    if diag in diag_map:
        for k in diag_map[diag]:
            set_servo_angle(k, diag_active[diag])

# === THREADS ===
def serial_thread():
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                serial_logs.append(line)
                if len(serial_logs) > LOG_MAX:
                    serial_logs.pop(0)


def imu_thread():
    global IMU_INTERVAL, heading
    while True:
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
        # Integrate gyro Z to get heading (deg/s -> deg)
        gz_dps = gz / 131.0
        heading = (heading + gz_dps * IMU_INTERVAL) % 360
        imu_logs.append(f"Heading: {heading:.1f}° A:({ax},{ay},{az}) G:({gx},{gy},{gz})")
        if len(imu_logs) > LOG_MAX:
            imu_logs.pop(0)
        time.sleep(IMU_INTERVAL)

# Start threads
t = threading.Thread(target=serial_thread, daemon=True)
t.start()
t2 = threading.Thread(target=imu_thread, daemon=True)
t2.start()

# === MAIN LOOP ===
running = True
while running:
    screen.fill(COLOR_BG)
    keys = pygame.key.get_pressed()

    # Draw separators
    pygame.draw.line(screen, COLOR_SEPARATOR, (350, 0), (350, WINDOW_HEIGHT), 2)
    pygame.draw.line(screen, COLOR_SEPARATOR, (850, 0), (850, WINDOW_HEIGHT), 2)
    pygame.draw.line(screen, COLOR_SEPARATOR, (0, 650), (WINDOW_WIDTH, 650), 2)

    # Bottom help
    help_str = "[C] AngleCal  [M] Mapping  [Z] Zero  [N] Neutral  [I/O] IMU rate  [Space] Stop"
    screen.blit(font.render(help_str, True, COLOR_TEXT), (10, 655))

    # Event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            # Toggle modes
            if event.key == KEY_ANGLE:
                angle_mode = not angle_mode
                mapping_mode = False
            if event.key == KEY_MAPPING:
                mapping_mode = not mapping_mode
                angle_mode = False
            # Zero and Neutral
            if event.key == KEY_ZERO:
                zero_all()
            if event.key == KEY_NEUTRAL:
                release_all()
            # Emergency stop
            if event.key == KEY_STOP:
                ser.write((STOP_TOKEN + "\n").encode('utf-8'))
                release_all()
                current_dir = None
            # IMU rate adjust
            if event.key == KEY_IMU_INCREASE:
                IMU_INTERVAL = max(0.01, IMU_INTERVAL - 0.01)
            if event.key == KEY_IMU_DECREASE:
                IMU_INTERVAL += 0.01

            # Mapping controls
            if mapping_mode:
                if event.key == KEY_CYCLE_DIAG:
                    sel_diag_idx = (sel_diag_idx + 1) % len(diagonals)
                elif event.key in KEY_TOGGLE_SERVOS:
                    i = KEY_TOGGLE_SERVOS.index(event.key)
                    servo = list(SERVO_PINS.keys())[i]
                    diag = diagonals[sel_diag_idx]
                    if servo in diag_map[diag]:
                        diag_map[diag].remove(servo)
                    else:
                        diag_map[diag].append(servo)
                elif event.key == KEY_ADJ_NEUTRAL_DEC:
                    diag_neutral[diagonals[sel_diag_idx]] = max(
                        MIN_ANGLE,
                        diag_neutral[diagonals[sel_diag_idx]] - ADJUST_STEP
                    )
                elif event.key == KEY_ADJ_NEUTRAL_INC:
                    diag_neutral[diagonals[sel_diag_idx]] = min(
                        MAX_ANGLE,
                        diag_neutral[diagonals[sel_diag_idx]] + ADJUST_STEP
                    )
                elif event.key == KEY_ADJ_ACTIVE_INC:
                    diag_active[diagonals[sel_diag_idx]] = min(
                        MAX_ANGLE,
                        diag_active[diagonals[sel_diag_idx]] + ADJUST_STEP
                    )
                elif event.key == KEY_ADJ_ACTIVE_DEC:
                    diag_active[diagonals[sel_diag_idx]] = max(
                        MIN_ANGLE,
                        diag_active[diagonals[sel_diag_idx]] - ADJUST_STEP
                    )
                continue

            # Angle calibration controls
            if angle_mode:
                if event.key in KEY_TOGGLE_SERVOS:
                    sel_servo_idx = KEY_TOGGLE_SERVOS.index(event.key)
                elif event.key == KEY_ADJ_NEUTRAL_DEC:
                    srv = list(SERVO_PINS.keys())[sel_servo_idx]
                    set_servo_angle(srv, servo_states[srv] - ADJUST_STEP)
                elif event.key == KEY_ADJ_NEUTRAL_INC:
                    srv = list(SERVO_PINS.keys())[sel_servo_idx]
                    set_servo_angle(srv, servo_states[srv] + ADJUST_STEP)
                continue

    # Movement handling
    if not (angle_mode or mapping_mode):
        new_dir = None
        if keys[pygame.K_UP] and keys[pygame.K_RIGHT]:
            new_dir = 'NE'
        elif keys[pygame.K_UP] and keys[pygame.K_LEFT]:
            new_dir = 'NW'
        elif keys[pygame.K_DOWN] and keys[pygame.K_RIGHT]:
            new_dir = 'SE'
        elif keys[pygame.K_DOWN] and keys[pygame.K_LEFT]:
            new_dir = 'SW'
        elif keys[pygame.K_UP]:
            new_dir = 'N'
        elif keys[pygame.K_DOWN]:
            new_dir = 'S'
        elif keys[pygame.K_LEFT]:
            new_dir = 'W'
        elif keys[pygame.K_RIGHT]:
            new_dir = 'E'

        if new_dir != current_dir:
            if new_dir in DIRECTIONS:
                ser.write((new_dir + "\n").encode('utf-8'))
                release_all()
                activate_diag(new_dir)
            else:
                ser.write((STOP_TOKEN + "\n").encode('utf-8'))
                release_all()
            current_dir = new_dir

    # DRAW UI PANELS
    screen.fill(COLOR_BG)
    # separators
    pygame.draw.line(screen, COLOR_SEPARATOR, (350, 0), (350, WINDOW_HEIGHT), 2)
    pygame.draw.line(screen, COLOR_SEPARATOR, (850, 0), (850, WINDOW_HEIGHT), 2)
    pygame.draw.line(screen, COLOR_SEPARATOR, (0, 650), (WINDOW_WIDTH, 650), 2)

    # Left: Servo states & modes
    x0, y0 = 10, 10
    screen.blit(font.render("Servo States:", True, COLOR_HIGHLIGHT), (x0, y0))
    for i, k in enumerate(SERVO_PINS):
        screen.blit(font.render(f"{k}: {servo_states[k]}°", True, COLOR_TEXT), (x0, y0 + 30*(i+1)))

    if angle_mode:
        screen.blit(font.render("-- Angle Calibration --", True, COLOR_HIGHLIGHT), (x0, y0+160))
        srv = list(SERVO_PINS.keys())[sel_servo_idx]
        screen.blit(font.render(f"Selected: {srv}", True, COLOR_TEXT), (x0, y0+190))
        screen.blit(font.render("Use 1-4 to select, <-/-> to adjust", True, COLOR_TEXT), (x0, y0+220))

    if mapping_mode:
        screen.blit(font.render("-- Mapping Mode --", True, COLOR_HIGHLIGHT), (x0, y0+280))
        current_diag = diagonals[sel_diag_idx]
        screen.blit(font.render(f"Dir: {current_diag}", True, COLOR_TEXT), (x0, y0+310))
        screen.blit(font.render("1-4 toggle servos, TAB cycle diag", True, COLOR_TEXT), (x0, y0+340))
        # Show neutral/active for this diagonal
        screen.blit(font.render(f"Neutral: {diag_neutral[current_diag]}°  Active: {diag_active[current_diag]}°", True, COLOR_TEXT), (x0, y0+370))
        # List mapping
        for i, d in enumerate(diagonals):
            mark = '>' if i == sel_diag_idx else ' '
            screen.blit(font.render(f"{mark}{d}: {diag_map[d]}", True, COLOR_TEXT), (x0, y0+400+30*i))

    # Center: Direction & arrows
    cx, cy = WINDOW_WIDTH//2, WINDOW_HEIGHT//2 - 50
    screen.blit(font.render(f"Direction: {current_dir or 'STOPPED'}", True, COLOR_TEXT), (WINDOW_WIDTH//2-100, 10))
    ofs = 100
    # Draw heading indicator above arrows
    hx, hy = cx, cy - ofs - 50
    rad = math.radians(heading)
    dx = math.sin(rad)
    dy = -math.cos(rad)
    # perpendicular for base of triangle
    ux, uy = -dy, dx
    length = 40
    tip = (hx + dx*length, hy + dy*length)
    base1 = (hx - dx*10 + ux*10, hy - dy*10 + uy*10)
    base2 = (hx - dx*10 - ux*10, hy - dy*10 - uy*10)
    pygame.draw.polygon(screen, COLOR_HIGHLIGHT, [tip, base1, base2])
    # Directional arrows
    arrow_symbols = {'N':'↑','S':'↓','W':'←','E':'→','NE':'↗','NW':'↖','SE':'↘','SW':'↙'}
    for d, sym in arrow_symbols.items():
        pos = {
            'N': (cx, cy-ofs), 'S': (cx, cy+ofs),
            'W': (cx-ofs, cy), 'E': (cx+ofs, cy),
            'NE': (cx+ofs, cy-ofs), 'NW': (cx-ofs, cy-ofs),
            'SE': (cx+ofs, cy+ofs), 'SW': (cx-ofs, cy+ofs)
        }[d]
        color = COLOR_ARROW_ACTIVE if d == current_dir else COLOR_ARROW_INACTIVE
        surf = narrow_font.render(sym, True, color)
        rect = surf.get_rect(center=pos)
        screen.blit(surf, rect)

    # Right: Logs & IMU freq
    rx, ry = 860, 10
    screen.blit(font.render(f"IMU Rate: {1/IMU_INTERVAL:.1f} Hz", True, COLOR_HIGHLIGHT), (rx, ry))
    screen.blit(font.render("Serial Log:", True, COLOR_TEXT), (rx, ry+30))
    for i, ln in enumerate(serial_logs):
        screen.blit(font.render(ln, True, COLOR_TEXT), (rx, ry+60+20*i))
    screen.blit(font.render("IMU Log:", True, COLOR_TEXT), (rx, ry+300))
    for i, ln in enumerate(imu_logs):
        screen.blit(font.render(ln, True, COLOR_TEXT), (rx, ry+330+20*i))

    # Bottom help
    screen.blit(font.render(help_str, True, COLOR_TEXT), (10, 655))

    pygame.display.flip()
    clock.tick(30)

# Cleanup
pygame.quit()
ser.close()
for pwm in servo_pwms.values(): pwm.stop()
GPIO.cleanup()
