from rtde_control import RTDEControlInterface
import time

robot_ip = "192.168.0.10"
acceleration = 0.5
dt = 0.008
duration = 2.0  # move for 5 seconds

# Cartesian velocity vector (move in X direction)
velocity = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]  # vx, vy, vz, rx, ry, rz

# Connect to robot
rtde_control = RTDEControlInterface(robot_ip)

try:
    start_time = time.time()
    while time.time() - start_time < duration:
        current_time = time.time()
        rtde_control.speedL(velocity, acceleration, dt)
        sleep_duration = dt - (time.time() - current_time)
        if sleep_duration > 0:
            time.sleep(sleep_duration)

    # Stop the motion safely
    rtde_control.stopL()

finally:
    rtde_control.disconnect()


