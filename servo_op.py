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

# === SERVO SETUP ===
# MG995 servos (180 deg range), default angle parameter
SERVO_ANGLE = 180
NEUTRAL_ANGLE = SERVO_ANGLE / 2
# Map cardinal directions to GPIO pins
SERVO_PINS = {'N': 17, 'S': 18, 'E': 22, 'W': 23}
servo_pwms = {}

GPIO.setmode(GPIO.BCM)
for pin in SERVO_PINS.values():
    GPIO.setup(pin, GPIO.OUT)
    pwm = GPIO.PWM(pin, 50)  # 50Hz PWM
    pwm.start(0)
    servo_pwms[pin] = pwm


def set_servo_angle(pwm, angle):
    # Convert angle (0-180) to duty cycle
    duty = 2.5 + (angle / 18.0)
    pwm.ChangeDutyCycle(duty)


def activate_servos(cmd):
    # cmd is a two-letter diagonal like 'NE', 'SW', etc.
    for d in cmd:
        pin = SERVO_PINS.get(d)
        if pin:
            set_servo_angle(servo_pwms[pin], SERVO_ANGLE)


def release_servos(cmd):
    # Return servos for the diagonal back to neutral
    for d in cmd:
        pin = SERVO_PINS.get(d)
        if pin:
            set_servo_angle(servo_pwms[pin], NEUTRAL_ANGLE)

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
            # Release any diagonal servos
            if current_direction in {'NE','NW','SE','SW'}:
                release_servos(current_direction)
            current_direction = None

    # Only send when direction really changes
    if new_direction != current_direction:
        old_direction = current_direction
        if new_direction in DIRECTIONS:
            send_command(new_direction)
        elif current_direction is not None:
            # no key pressed → send STOP
            send_command(STOP_TOKEN)
        # Actuate servos for diagonals
        if new_direction in {'NE','NW','SE','SW'}:
            activate_servos(new_direction)
        # Release servos when leaving a diagonal
        if old_direction in {'NE','NW','SE','SW'} and new_direction not in {'NE','NW','SE','SW'}:
            release_servos(old_direction)
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

pygame.quit()
# Cleanup servos and GPIO
for pwm in servo_pwms.values():
    pwm.stop()
GPIO.cleanup()
ser.close()
