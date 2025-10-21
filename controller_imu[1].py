import pygame
import serial
import time
import threading
import string

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
pygame.display.set_caption("X-Tread Controller with Logs")
font = pygame.font.SysFont(None, 28)

# === COMMAND MAPPING ===
DIRECTION_COMMANDS = {
    'N':  "X+,Y-,Z+,E-",
    'S':  "X-,Y+,Z-,E+",
    'E':  "X+,Y+,Z-,E-",
    'W':  "X-,Y-,Z+,E+",
    'NE': "X+,E+",
    'NW': "Y-,Z+",
    'SE': "Y+,Z-",
    'SW': "X-,E-"
}

current_direction = None
serial_logs = []

# === SERIAL READER THREAD ===
def serial_monitor():
    global serial_logs
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                # Sanitize line: replace null characters with an empty string
                line = line.replace('\x00', '')
                # Avoid empty lines
                if line:
                    print(f"[RX] {line}")
                    serial_logs.append(line)
                    if len(serial_logs) > 8:
                        serial_logs.pop(0)
        except Exception as e:
            print(f"[ERROR] Serial read error: {e}")
            break

threading.Thread(target=serial_monitor, daemon=True).start()

# === INPUT HELPERS ===
def send_command(command):
    ser.write((command + "\n").encode('utf-8'))
    print(f"[TX] {command}")

def get_diagonal_key(keys):
    if keys[pygame.K_UP] and keys[pygame.K_RIGHT]:
        return 'NE'
    elif keys[pygame.K_UP] and keys[pygame.K_LEFT]:
        return 'NW'
    elif keys[pygame.K_DOWN] and keys[pygame.K_RIGHT]:
        return 'SE'
    elif keys[pygame.K_DOWN] and keys[pygame.K_LEFT]:
        return 'SW'
    return None

# === MAIN LOOP ===
clock = pygame.time.Clock()
running = True

while running:
    screen.fill((20, 20, 20))

    keys = pygame.key.get_pressed()
    diag = get_diagonal_key(keys)

    new_direction = None
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

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                send_command("X0,Y0,Z0,E0")
                current_direction = None

    if new_direction != current_direction and new_direction in DIRECTION_COMMANDS:
        send_command(DIRECTION_COMMANDS[new_direction])
        current_direction = new_direction

    if not new_direction and current_direction:
        send_command("X0,Y0,Z0,E0")  # Stop if no key pressed
        current_direction = None

    # Display direction
    label = font.render(f"Direction: {current_direction or 'STOPPED'}", True, (200, 200, 200))
    screen.blit(label, (30, 20))

    # Display logs
    log_title = font.render("Serial Monitor (Arduino):", True, (180, 180, 180))
    screen.blit(log_title, (30, 70))
    for i, log in enumerate(serial_logs):
        # Replace or remove any problematic characters in the log
        sanitized_log = log.replace('\x00', '')  # Remove null characters if present
        text_surface = font.render(sanitized_log, True, (150, 255, 150))
        screen.blit(text_surface, (30, 100 + i * 25))

    pygame.display.flip()
    clock.tick(30)

pygame.quit()
ser.close()

