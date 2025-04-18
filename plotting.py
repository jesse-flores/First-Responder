import os
import time
from math import cos, sin, pi, floor, sqrt
import pygame
from adafruit_rplidar import RPLidar

# --- Display Setup ---
W, H = 640, 480
pygame.display.init()
lcd = pygame.display.set_mode((W, H))
pygame.mouse.set_visible(False)
lcd.fill((0, 0, 0))
pygame.display.update()

# --- LIDAR Setup ---
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME)

# --- Globals ---
scan_data = [0] * 360
persistent_map = []  # (x, y, quality, last_seen, hit_count)
previous_frame = []  # List of (x, y, quality) from previous scan
moving_points = []   # List of (x, y) for moving objects

MAX_DISTANCE = 5000  # mm
MATCH_RADIUS = 30    # mm
MAX_AGE = 10         # seconds to keep stable points
MIN_HITS = 3         # min sightings to consider stable
FRAME_TIMEOUT = 0.5  # seconds before a stable point is considered "gone"

# --- Helper Functions ---
def distance(p1, p2):
    return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def update_persistent_map(current_points):
    global persistent_map, moving_points, previous_frame
    now = time.time()
    moving_points = []

    for new_x, new_y, quality in current_points:
        matched = False
        for i, (x, y, q, last_seen, hits) in enumerate(persistent_map):
            if distance((new_x, new_y), (x, y)) < MATCH_RADIUS:
                # Update existing stable point
                persistent_map[i] = (
                    (x + new_x) / 2, (y + new_y) / 2, (q + quality) / 2,
                    now, hits + 1
                )
                matched = True
                break
        if not matched:
            # New or moving point
            moving_points.append((new_x, new_y))
            persistent_map.append((new_x, new_y, quality, now, 1))

    # Detect previously stable points that have disappeared
    for x, y, q, t, hits in persistent_map:
        if now - t < FRAME_TIMEOUT and hits >= MIN_HITS:
            still_visible = any(distance((x, y), (px, py)) < MATCH_RADIUS for (px, py, _) in current_points)
            if not still_visible:
                moving_points.append((x, y))

    # Remove old or unstable points
    persistent_map = [
        (x, y, q, t, hits)
        for (x, y, q, t, hits) in persistent_map
        if (now - t < MAX_AGE and hits >= MIN_HITS)
    ]

    previous_frame = current_points

def process_data(data):
    lcd.fill((0, 0, 0))
    center = (W // 2, H // 2)
    pygame.draw.circle(lcd, pygame.Color(255, 255, 255), center, 5)
    pygame.draw.circle(lcd, pygame.Color(50, 50, 50), center, 100, 1)
    pygame.draw.line(lcd, pygame.Color(50, 50, 50), (0, H // 2), (W, H // 2))
    pygame.draw.line(lcd, pygame.Color(50, 50, 50), (W // 2, 0), (W // 2, H))

    current_points = []

    for angle in range(360):
        dist = data[angle]
        if 0 < dist < MAX_DISTANCE:
            radians = angle * pi / 180.0
            x = dist * cos(radians)
            y = dist * sin(radians)
            current_points.append((x, y, 15))  # fake quality

    update_persistent_map(current_points)

    # Draw stable map points in green
    for x, y, q, t, hits in persistent_map:
        draw_x = int(W / 2 + x / MAX_DISTANCE * (W / 2))
        draw_y = int(H / 2 + y / MAX_DISTANCE * (H / 2))
        green_intensity = min(255, hits * 10)
        pygame.draw.circle(lcd, (0, green_intensity, 0), (draw_x, draw_y), 2)

    # Draw moving points in red
    for mx, my in moving_points:
        draw_x = int(W / 2 + mx / MAX_DISTANCE * (W / 2))
        draw_y = int(H / 2 + my / MAX_DISTANCE * (H / 2))
        pygame.draw.circle(lcd, (255, 0, 0), (draw_x, draw_y), 3)

    pygame.display.update()

# --- Main Loop ---
try:
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            scan_data[min([359, floor(angle)])] = distance
        process_data(scan_data)

except KeyboardInterrupt:
    print("Stopping.")

lidar.stop()
lidar.disconnect()
