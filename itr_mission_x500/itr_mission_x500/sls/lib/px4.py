import numpy as np
from itr_comms_x500 import Comms


# SOME HACKY PARAMETERS
MASS = 2.0 # from x500_base model
GRAVITY = 9.807232
HOVER_THRUST = MASS * GRAVITY
ARM_Y = 0.174
ARM_X = 0.174

MOTOR_CONSTANT   = 8.54858e-6   # [N / (rad/s)²]  F = k * omega²
MOMENT_CONSTANT  = 0.016        # [m]              tau_z = F * cm  (you already had this)
MAX_ROT_VEL      = 10703.0       # [rad/s]          maxRotVelocity from SDF
MIN_ROT_VEL      = 0.0          # idle speed (0 for SITL unless set otherwise)
N_MOTORS         = 4
HOVER_THROTTLE = -0.72 # empiric
MAX_SINGLE_THRUST = MOTOR_CONSTANT * MAX_ROT_VEL**2          # [N]
MAX_TOTAL_THRUST  = N_MOTORS * MAX_SINGLE_THRUST              # [N]
# Max differential torque: two motors on one side at max, two at zero
MAX_TAU_X = ARM_Y * 2 * MAX_SINGLE_THRUST   # roll
MAX_TAU_Y = ARM_X * 2 * MAX_SINGLE_THRUST   # pitch
MAX_TAU_Z = MOMENT_CONSTANT * 2 * MAX_SINGLE_THRUST  # yaw (two CW vs two CCW)

def normalize_thrust(thrust_delta_N: float) -> float:
    """
    Convert delta thrust [N] (output of SLS, linearized around hover)
    to PX4 normalized thrust in [-1, 0] (NED: negative = up).
    
    PX4 expects normalized motor velocity, not normalized force.
    thrust_total = hover_thrust + thrust_delta
    """
    # This calculation is in FLU and does not work lol
    # total_thrust = HOVER_THRUST - thrust_delta_N # minus because in NED
    # thrust_normalized = (np.sqrt(total_thrust/MOTOR_CONSTANT)-MIN_ROT_VEL)/(MAX_ROT_VEL-MIN_ROT_VEL)
    max_thrust = HOVER_THRUST / HOVER_THROTTLE
    thrust = - HOVER_THRUST + thrust_delta_N
    thrust_normalized = thrust / abs(max_thrust)

    return thrust_normalized  # negative = up in NED

def normalize_torques(torques: np.ndarray) -> np.ndarray:
    tau_x, tau_y, tau_z = torques
    return np.array([
        np.clip(tau_x / MAX_TAU_X, -1.0, 1.0),
        np.clip(tau_y / MAX_TAU_Y, -1.0, 1.0),
        np.clip(tau_z / MAX_TAU_Z, -1.0, 1.0),
    ])

def get_state(comms: Comms, debug=False):
    pos = comms.get_position() # From Mocap (Gazebo), pre-converted to NED
    vel = comms.get_velocity() # From PX4, already in NED
    orientation = comms.get_orientation_euler() # From Mocap (Gazebo), pre-converted to NED
    body_rates = comms.get_angular_velocity() # From PX4, already in NED

    obs = np.concatenate([pos, vel, orientation, body_rates])

    if debug:
        print(f"Observation: {obs}")

    return obs


def apply_thrust_and_torque(comms: Comms, input, debug=False):
    thrust, tau_x, tau_y, tau_z = input

    if debug:
        print(f"Normalized Control Input: {input}")

    comms.send_torque_setpoint(np.array([tau_x, tau_y, tau_z]))
    comms.send_thrust_setpoint(thrust)

    return
