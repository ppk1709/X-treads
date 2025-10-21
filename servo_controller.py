import pygame
import serial
import time
import threading

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

# === ARROW GUI DRAWING FUNCTION ===
# === ARROW GUI DRAWING FUNCTION (fixed version) ===
def draw_arrow_gui(surface, active_direction):
    center_x, center_y = 450, 150
    arrow_size = 30
    gap = 10

    color_inactive = (80, 80, 80)
    color_active = (50, 220, 250)

    def draw_single_arrow(x, y, dx, dy, active):
        color = color_active if active else color_inactive
        base_size = arrow_size

        if dx == 0:  # Vertical arrow (N or S)
            points = [
                (x, y - dy * base_size),
                (x - base_size//2, y),
                (x + base_size//2, y),
            ]
        elif dy == 0:  # Horizontal arrow (E or W)
            points = [
                (x + dx * base_size, y),
                (x, y - base_size//2),
                (x, y + base_size//2),
            ]
        else:  # Diagonal arrow
            points = [
                (x + dx * base_size, y + dy * (base_size)),
                (x, y),
                (x + dx * (base_size//2), y),
                (x, y + dy * (base_size//2)),
            ]

        pygame.draw.polygon(surface, color, points)

    # Draw each arrow
    draw_single_arrow(center_x, center_y - (arrow_size + gap), 0, 1, active_direction == 'N')
    draw_single_arrow(center_x, center_y + (arrow_size + gap), 0, -1, active_direction == 'S')
    draw_single_arrow(center_x - (arrow_size + gap), center_y, 1, 0, active_direction == 'W')
    draw_single_arrow(center_x + (arrow_size + gap), center_y, -1, 0, active_direction == 'E')

    draw_single_arrow(center_x + (arrow_size + gap), center_y - (arrow_size + gap), -1, 1, active_direction == 'NE')
    draw_single_arrow(center_x - (arrow_size + gap), center_y - (arrow_size + gap), 1, 1, active_direction == 'NW')
    draw_single_arrow(center_x + (arrow_size + gap), center_y + (arrow_size + gap), -1, -1, active_direction == 'SE')
    draw_single_arrow(center_x - (arrow_size + gap), center_y + (arrow_size + gap), 1, -1, active_direction == 'SW')


    # === DRAW ARROW GUI ===
    draw_arrow_gui(screen, current_direction)

    pygame.display.flip()
    clock.tick(30)

pygame.quit()
ser.close()

