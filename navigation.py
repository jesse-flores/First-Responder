from math import cos, sin, pi

def analyze(scan_data):
    """Return (throttle, steering) decision based on LIDAR scan."""
    front_angles = list(range(330, 370)) + list(range(0, 20))
    left_angles = range(60, 120)
    right_angles = range(240, 300)

#modify as needed with real rc platform

    front_dist = sum(scan_data[a] for a in front_angles if scan_data[a] > 0) / len(front_angles)
    left_dist = sum(scan_data[a] for a in left_angles if scan_data[a] > 0) / len(left_angles)
    right_dist = sum(scan_data[a] for a in right_angles if scan_data[a] > 0) / len(right_angles)

    # Basic obstacle avoidance logic
    throttle = 0.3 if front_dist > 800 else 0.0
    steering = 0.0
    if front_dist < 800:
        if left_dist > right_dist:
            steering = -0.7  # turn left
        else:
            steering = 0.7   # turn right

    return throttle, steering