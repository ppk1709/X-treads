import pygame
import serial
import time
import threading
import RPi.GPIO as GPIO

# === SERIAL CONFIG ===
SERIAL_PORT = '/dev/ttyACM0'  # Change if needed
BAUDRATE = 9600
try:
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
    print(f"[INFO] Connected to {SERIAL_PORT} at {BAUDRATE} baud.")
except Exception as e:
    print(f"[ERROR] Could not open serial port: {e}")
    exit(1)

# === PYGAME SETUP ===
pygame.init()
screen = pygame.display.set_mode((600, 320))
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
SERVO_PINS = {'X': 17, 'Y': 13, 'Z': 22, 'E': 23}  # Updated naming

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Initialize PWM for each servo
servo_pwms = {}
for dir_key, pin in SERVO_PINS.items():
    GPIO.setup(pin, GPIO.OUT)
    pwm = GPIO.PWM(pin, 50)  # 50Hz for standard servos
    pwm.start(7.5)  # Neutral position (90 degrees approx)
    servo_pwms[pin] = pwm

def set_servo_angle(pwm, angle):
    duty = (0.05 * angle) / 9 + 2.5
    pwm.ChangeDutyCycle(duty)

# Servo pairs for each diagonal command
DIAGONAL_SERVO_MAP = {
    'NE': ['Y', 'E'],  # Front-right (Y) and back-right (E)
    'NW': ['X', 'Z'],  # Front-left (X) and back-left (Z)
    'SE': ['Z', 'E'],  # Back-left (Z) and back-right (E)
    'SW': ['X', 'Z']   # Front-left (X) and back-left (Z)
}

def activate_servos(cmd):
    """Move two servos corresponding to the diagonal."""
    servos = DIAGONAL_SERVO_MAP.get(cmd, [])
    for d in servos:
        pin = SERVO_PINS.get(d)
        if pin:
            set_servo_angle(servo_pwms[pin], SERVO_ANGLE)

def release_servos(cmd):
    """Reset two servos back to neutral for the diagonal."""
    servos = DIAGONAL_SERVO_MAP.get(cmd, [])
    for d in servos:
        pin = SERVO_PINS.get(d)
        if pin:
            set_servo_angle(servo_pwms[pin], NEUTRAL_ANGLE)

# === SERIAL READER THREAD ===
def serial_monitor():
    global serial_logs
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"[RX] {line}")
                    serial_logs.append(line)
                    if len(serial_logs) > 8:
                        serial_logs.pop(0)
        except Exception as e:
            print(f"[ERROR] Serial read error: {e}")
            break

threading.Thread(target=serial_monitor, daemon=True).start()

# === COMM / INPUT HELPERS ===
def send_command(cmd: str):
    """Send a single token (e.g. 'N', 'SW', or 'STOP') over serial."""
    ser.write((cmd + "\n").encode('utf-8'))
    print(f"[TX] {cmd}")

def get_diagonal_key(keys):
    if keys[pygame.K_UP] and keys[pygame.K_RIGHT]:
        return 'NE'
    if keys[pygame.K_UP] and keys[pygame.K_LEFT]:
        return 'NW'
    if keys[pygame.K_DOWN] and keys[pygame.K_RIGHT]:
        return 'SE'
    if keys[pygame.K_DOWN] and keys[pygame.K_LEFT]:
        return 'SW'
    return None

# === MAIN LOOP ===
clock = pygame.time.Clock()
running = True

while running:
    screen.fill((20, 20, 20))
    keys = pygame.key.get_pressed()
    diag = get_diagonal_key(keys)

    # Decide new_direction based on diagonals first, then single keys
    if diag:
        new_direction = diag
    elif keys[pygame.K_UP]:
        new_direction = 'N'
    elif keys[pygame.K_DOWN]:
        new_direction = 'S'
    elif keys[pygame.K_LEFT]:
        new_direction = 'W'
    elif keys[pygame.K_RIGHT]:
        new_direction = 'E'
    else:
        new_direction = None

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        # Space bar forces an immediate STOP
        if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
            send_command(STOP_TOKEN)
            if current_direction in DIAGONAL_SERVO_MAP:
                release_servos(current_direction)
            current_direction = None

    # Only send when direction really changes
    if new_direction != current_direction:
        if new_direction in DIRECTIONS:
            send_command(new_direction)
            # Servo logic
            if current_direction in DIAGONAL_SERVO_MAP:
                release_servos(current_direction)
            if new_direction in DIAGONAL_SERVO_MAP:
                activate_servos(new_direction)
        elif current_direction is not None:
            # No key pressed → send STOP
            send_command(STOP_TOKEN)
            if current_direction in DIAGONAL_SERVO_MAP:
                release_servos(current_direction)
        current_direction = new_direction

    # --- RENDER ---
    label = font.render(f"Direction: {current_direction or 'STOPPED'}", True, (200, 200, 200))
    screen.blit(label, (30, 20))

    log_title = font.render("Serial Monitor (Arduino):", True, (180, 180, 180))
    screen.blit(log_title, (30, 70))
    for i, log in enumerate(serial_logs):
        text_surface = font.render(log, True, (150, 255, 150))
        screen.blit(text_surface, (30, 100 + i * 25))

    pygame.display.flip()
    clock.tick(30)

# === CLEANUP ===
pygame.quit()
ser.close()
for pwm in servo_pwms.values():
    pwm.stop()
GPIO.cleanup()

