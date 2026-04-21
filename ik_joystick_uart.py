import math
import time

import numpy as np
import serial


# =========================================================
# 1) USER SETTINGS
# =========================================================
JOYSTICK_PORT = "/dev/ttyUSB0"
STM_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

CONTROL_DT = 0.1
DAMPING = 0.08
MAX_JOINT_VEL = 1.2
MAX_LINEAR_SPEED = 0.12
JOYSTICK_CENTER = 1500.0
JOYSTICK_SCALE = 1000.0
JOYSTICK_DEADZONE = 0.03
EPS = 1e-8

# Measured geometry in meters for the 2-DOF planar reduction.
# This simplified model represents the shoulder-elbow chain seen from the side.
# L1 is the base-near link.
# L2 is the distance from the elbow joint to the wrist/tool mount.
# END_EFFECTOR is the extra tool length after that point.
L1 = 0.38
L2 = 0.40
END_EFFECTOR = 0.05
L2_EFFECTIVE = L2 + END_EFFECTOR
MANUAL_MAX_REACH = L1 + L2_EFFECTIVE

# Internal model must match the real arm's startup pose.
# Assumption from the image-like pose:
# L1 is vertical and L2 is roughly horizontal.
# In this planar model that corresponds to:
# J1 = +90 deg, J2 = -90 deg.
INITIAL_J1_DEG = 90.0
INITIAL_J2_DEG = -90.0
INITIAL_Q = np.radians(np.array([INITIAL_J1_DEG, INITIAL_J2_DEG], dtype=np.float64))

# User-requested safety limits: +/- pi/2 for both planar joints.
# Note: in the full URDF, J1 is base yaw while J2/J3 form the side-view chain.
# This reduced model tracks the side-view pair only.
JOINT_LIMITS = {
    "J1_joint": (-math.pi , math.pi ),
    "J2_joint": (-math.pi , math.pi ),
}


# =========================================================
# 2) IK HELPERS
# =========================================================
def fk_position_xz(q: np.ndarray) -> np.ndarray:
    q1, q2 = float(q[0]), float(q[1])
    q12 = q1 + q2

    x = L1 * math.cos(q1) + L2_EFFECTIVE * math.cos(q12)
    z = L1 * math.sin(q1) + L2_EFFECTIVE * math.sin(q12)
    return np.array([x, z], dtype=np.float64)


def planar_jacobian(q: np.ndarray) -> np.ndarray:
    q1, q2 = float(q[0]), float(q[1])
    q12 = q1 + q2

    return np.array(
        [
            [
                -L1 * math.sin(q1) - L2_EFFECTIVE * math.sin(q12),
                -L2_EFFECTIVE * math.sin(q12),
            ],
            [
                L1 * math.cos(q1) + L2_EFFECTIVE * math.cos(q12),
                L2_EFFECTIVE * math.cos(q12),
            ],
        ],
        dtype=np.float64,
    )


def damped_pseudoinverse(jacobian: np.ndarray, damping: float) -> np.ndarray:
    rows = jacobian.shape[0]
    identity = np.eye(rows, dtype=np.float64)
    return jacobian.T @ np.linalg.inv(jacobian @ jacobian.T + (damping ** 2) * identity)


def clip_joint_velocities(q_dot: np.ndarray, max_vel: float) -> np.ndarray:
    return np.clip(q_dot, -max_vel, max_vel)


def clamp_joint_positions(q: np.ndarray) -> np.ndarray:
    q = q.copy()
    q[0] = np.clip(q[0], JOINT_LIMITS["J1_joint"][0], JOINT_LIMITS["J1_joint"][1])
    q[1] = np.clip(q[1], JOINT_LIMITS["J2_joint"][0], JOINT_LIMITS["J2_joint"][1])
    return q


def apply_deadzone(value: float) -> float:
    if abs(value) < JOYSTICK_DEADZONE:
        return 0.0
    return value


def normalize_joystick(raw_value: int) -> float:
    normalized = (float(raw_value) - JOYSTICK_CENTER) / JOYSTICK_SCALE
    return apply_deadzone(normalized)


def parse_joystick_line(line: str):
    numbers = list(map(int, line.split()))
    if len(numbers) < 2:
        raise ValueError(f"Incomplete joystick data: {line}")
    return numbers[0], numbers[1]


def build_cartesian_velocity_from_joystick(line: str) -> np.ndarray:
    x_raw, z_raw = parse_joystick_line(line)
    vx = normalize_joystick(x_raw) * MAX_LINEAR_SPEED
    vz = normalize_joystick(z_raw) * MAX_LINEAR_SPEED
    return np.array([vx, vz], dtype=np.float64)


def enforce_planar_workspace_velocity_guard(
    position_xz: np.ndarray,
    velocity_xz: np.ndarray,
    max_reach: float,
) -> np.ndarray:
    position = np.array(position_xz, dtype=np.float64)
    velocity = np.array(velocity_xz, dtype=np.float64)

    radius = np.linalg.norm(position)
    if radius < EPS:
        return velocity

    boundary_margin = 0.98 * max_reach
    if radius < boundary_margin:
        return velocity

    normal = position / radius
    radial_speed = np.dot(velocity, normal)
    if radial_speed > 0.0:
        velocity = velocity - radial_speed * normal

    return velocity


def compute_joint_velocity(q: np.ndarray, v_cmd_xz: np.ndarray) -> np.ndarray:
    position_xz = fk_position_xz(q)
    guarded_v_cmd = enforce_planar_workspace_velocity_guard(position_xz, v_cmd_xz, MANUAL_MAX_REACH)

    jacobian_pinv = damped_pseudoinverse(planar_jacobian(q), DAMPING)
    q_dot_active = jacobian_pinv @ guarded_v_cmd
    q_dot_active = clip_joint_velocities(q_dot_active, MAX_JOINT_VEL)
    return q_dot_active


def to_stm_direction(delta_deg: float) -> int:
    return int(delta_deg >= 0.0)


def send_stm_packet(ser, shoulder_delta_deg: float, elbow_delta_deg: float) -> None:
    packet = (
        f"{abs(shoulder_delta_deg):.4f} {to_stm_direction(shoulder_delta_deg)} {abs(elbow_delta_deg):.4f} {to_stm_direction(elbow_delta_deg)}\n"
    )
    ser.write(packet.encode("utf-8"))
    print(f"Sent: {packet.strip()}")


# =========================================================
# 3) MAIN LOOP
# =========================================================
def main():
    q = INITIAL_Q.copy()

    joystick = serial.Serial(JOYSTICK_PORT, BAUD_RATE, timeout=0.1)
    stm = serial.Serial(STM_PORT, BAUD_RATE, timeout=0.1)

    print("2-DOF differential velocity IK started")
    print("Joystick -> task-space X/Z velocity -> joint delta angle -> STM32")
    print(
        f"Model: L1={L1:.2f} m, L2={L2:.2f} m, tool={END_EFFECTOR:.2f} m, "
        f"q0=({INITIAL_J1_DEG:.1f} deg, {INITIAL_J2_DEG:.1f} deg)"
    )

    try:
        while True:
            if joystick.in_waiting <= 0:
                time.sleep(0.01)
                continue

            line = joystick.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            try:
                v_cmd_xz = build_cartesian_velocity_from_joystick(line)
            except ValueError as exc:
                print(exc)
                continue

            if np.linalg.norm(v_cmd_xz) < EPS:
                send_stm_packet(stm, 0.0, 0.0)
                time.sleep(CONTROL_DT)
                continue

            q_dot_active = compute_joint_velocity(q, v_cmd_xz)
            delta_q = q_dot_active * CONTROL_DT

            q += delta_q
            q = clamp_joint_positions(q)

            shoulder_delta_deg = math.degrees(delta_q[0])
            elbow_delta_deg = math.degrees(delta_q[1])

            send_stm_packet(stm, shoulder_delta_deg, elbow_delta_deg)

            end_effector = fk_position_xz(q)
            print(
                f"EE(x,z)=({end_effector[0]:.3f}, {end_effector[1]:.3f}) | "
                f"v=({v_cmd_xz[0]:.3f}, {v_cmd_xz[1]:.3f}) | "
                f"q=({q[0]:.3f}, {q[1]:.3f}) | "
                f"dq_deg=({shoulder_delta_deg:.3f}, {elbow_delta_deg:.3f})"
            )

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Stopped by user")
    finally:
        if joystick.is_open:
            joystick.close()
        if stm.is_open:
            stm.close()
        print("Serial ports closed")


if __name__ == "__main__":
    main()
