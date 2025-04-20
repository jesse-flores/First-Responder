import pigpio

STEERING_PIN = #update when car arrives
THROTTLE_PIN = update when car arrives

pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Cannot connect to pigpio daemon")

def set_controls(throttle: float, steering: float):
    """Set throttle [0–1] and steering [-1 to 1] via PWM (1500 ± range)"""
    throttle_us = int(1500 + 300 * min(max(throttle, 0.0), 1.0))  # limit: 1800
    steering_us = int(1500 + 500 * min(max(steering, -1.0), 1.0))
    pi.set_servo_pulsewidth(THROTTLE_PIN, throttle_us)
    pi.set_servo_pulsewidth(STEERING_PIN, steering_us)

def stop():
    pi.set_servo_pulsewidth(THROTTLE_PIN, 1500)
    pi.set_servo_pulsewidth(STEERING_PIN, 1500)
    pi.stop()