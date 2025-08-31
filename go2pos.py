import subprocess
import math
import time
import json

# Robot parameters
WHEEL_DISTANCE = 1.2    # meters between left and right wheels

# --- User input ---
TARGET_X = float(input("Enter target X position (m): "))
TARGET_Y = float(input("Enter target Y position (m): "))
MAX_WHEEL_VEL = float(input("Enter max wheel velocity (m/s): "))

def get_current_pose():
    """Get the current robot position (x, y) and yaw from odometry topic."""
    result = subprocess.run(
        ["ign", "topic", "-t", "/model/robot/odometry", "-n", "1", "-e", "--json-output"],
        capture_output=True, text=True
    )
    if not result.stdout.strip():
        raise RuntimeError("No data received from odometry topic")
    
    msg = json.loads(result.stdout)
    x = msg['pose']['position']['x']
    y = msg['pose']['position']['y']
    z = msg['pose']['orientation']['z']
    w = msg['pose']['orientation']['w']
    yaw = math.atan2(2 * w * z, 1 - 2 * z**2)
    return x, y, yaw

def publish_wheel_velocities(v_l, v_r, duration, action_desc="Moving"):
    """Publish wheel velocities for a given duration (open-loop)."""
    omega = (v_r - v_l) / WHEEL_DISTANCE  # robot angular velocity
    cmd = f"linear: {{x: {(v_r + v_l)/2}}}, angular: {{z: {omega}}}"
    
    print(f"{action_desc}: v_l={v_l:.2f}, v_r={v_r:.2f}, duration={duration:.2f}s")
    
    start_time = time.time()
    subprocess.run(["ign", "topic", "-t", "/cmd_vel", "-m", "ignition.msgs.Twist", "-p", cmd])
    
    # Adjust sleep for subprocess delay
    elapsed_subprocess = time.time() - start_time
    sleep_duration = max(0, duration - elapsed_subprocess)
    time.sleep(sleep_duration)
    
    # Stop robot
    subprocess.run(["ign", "topic", "-t", "/cmd_vel", "-m", "ignition.msgs.Twist", "-p",
                    "linear: {x: 0}, angular: {z: 0}"])
    
    total_elapsed = time.time() - start_time
    print(f"Total elapsed time: {total_elapsed:.2f} s\n")

def main():
    # Get current pose
    x_start, y_start, yaw_start = get_current_pose()
    print(f"Initial pose: x={x_start:.2f}, y={y_start:.2f}, yaw={math.degrees(yaw_start):.2f} deg")

    # --- Compute target angle and rotation ---
    dx = TARGET_X - x_start
    dy = TARGET_Y - y_start
    target_angle = math.atan2(dy, dx)
    
    # Relative rotation needed
    rel_angle = (target_angle - yaw_start + math.pi) % (2 * math.pi) - math.pi  # normalize [-pi, pi]

    # Determine wheel velocities for rotation
    v_rot = min(MAX_WHEEL_VEL, 0.5)  # reduce speed to avoid overshoot
    direction = 1 if rel_angle >= 0 else -1
    v_l = -direction * MAX_WHEEL_VEL
    v_r = direction * MAX_WHEEL_VEL

    # Angular velocity and duration
    omega_robot = (v_r - v_l) / WHEEL_DISTANCE
    t_turn = abs(rel_angle / omega_robot)
    
    print(f"Rotating to face target (relative angle: {math.degrees(rel_angle):.2f} deg)")
    publish_wheel_velocities(v_l, v_r, t_turn, action_desc="Rotating")

    # --- Drive straight to target ---
    x_curr, y_curr, yaw_curr = get_current_pose()
    distance = math.hypot(TARGET_X - x_curr, TARGET_Y - y_curr)
    
    v_l = v_r = MAX_WHEEL_VEL
    t_straight = distance / MAX_WHEEL_VEL
    print(f"Driving straight to target (distance: {distance:.2f} m)")
    publish_wheel_velocities(v_l, v_r, t_straight, action_desc="Driving straight")

    # --- Final pose ---
    x_final, y_final, yaw_final = get_current_pose()
    print(f"Reached target pose: x={x_final:.2f}, y={y_final:.2f}, yaw={math.degrees(yaw_final):.2f} deg")

if __name__ == "__main__":
    main()