"""
State augmentation for SLS control.

Provides two types of augmentation:
1. Tracking augmentation: Unified error/setpoint via configurable styles
2. Delay augmentation: Network communication delays

Key design principle: All functions return inputs unchanged if dim==0 or RTT_max==0
"""

import numpy as np
from dataclasses import dataclass
from typing import Union


@dataclass
class AugmentationStyle:
    """
    Configuration for tracking augmentation behavior.

    Tracking augmentation adds states x_track with dynamics:
        x_track[k+1] = -x_original[k] + u_track[k]

    This creates: x_track = u_track - x_original
    When u_track = reference, x_track = tracking error.

    Different styles (error vs setpoint) are achieved through cost weights
    and trajectory/initial condition fills.

    Attributes:
        q_weight: Cost on augmented states
        r_weight: Cost on augmented inputs
        x0_fill: Initial condition for augmented states ("zeros" or numeric value)
        traj_fill: Trajectory fill ("zeros", "copy", or numeric value)
    """
    q_weight: float
    r_weight: float
    x0_fill: Union[str, float] = "zeros"
    traj_fill: Union[str, float] = "zeros"


# Predefined augmentation styles
ERROR_STYLE = AugmentationStyle(
    q_weight=0.0,      # No cost on error states (driven by tracking term)
    r_weight=0.0,      # No cost on reference inputs
    x0_fill="zeros",   # Errors start at zero
    traj_fill="zeros"  # Error trajectory is zeros
)

SETPOINT_STYLE = AugmentationStyle(
    q_weight=0.1,      # Penalize setpoint deviation
    r_weight=0.5,      # Penalize setpoint changes
    x0_fill="zeros",   # Setpoints start at zero
    traj_fill="zeros"  # Setpoint trajectory is zeros
)


# ============================================================================
# Tracking Augmentation (Unified Error/Setpoint)
# ============================================================================

def tracking_augment_system(A: np.ndarray, B: np.ndarray,
                           dim: int, style: AugmentationStyle = ERROR_STYLE):
    """
    Augment system with tracking states.

    Creates dynamics: x_track[k+1] = -x_original[k] + u_track[k]
    So: x_track = u_track - x_original (tracking error when u_track = reference)

    Args:
        A: State matrix (Nx x Nx)
        B: Input matrix (Nx x Nu)
        dim: Number of tracking states to add
        style: Augmentation configuration (not used for A, B)

    Returns:
        A_aug: Augmented state matrix ((Nx+dim) x (Nx+dim))
        B_aug: Augmented input matrix ((Nx+dim) x (Nu+dim))

    If dim == 0, returns (A, B) unchanged (passthrough).
    """
    if dim == 0:
        return A, B

    Nx = A.shape[0]
    Nu = B.shape[1]

    A_aug = np.block([
        [A,            np.zeros((Nx, dim))],
        [-np.eye(dim), np.zeros((dim, Nx))]  # -I creates error dynamics
    ])

    B_aug = np.block([
        [B,                    np.zeros((Nx, dim))],
        [np.zeros((dim, Nu)), np.eye(dim)]
    ])

    return A_aug, B_aug


def tracking_augment_weights(Q: np.ndarray, R: np.ndarray,
                            dim: int, style: AugmentationStyle = ERROR_STYLE):
    """
    Augment cost matrices for tracking states.

    Args:
        Q: State cost matrix (Nx x Nx)
        R: Input cost matrix (Nu x Nu)
        dim: Number of tracking states
        style: Augmentation configuration

    Returns:
        Q_aug: Augmented state cost ((Nx+dim) x (Nx+dim))
        R_aug: Augmented input cost ((Nu+dim) x (Nu+dim))

    If dim == 0, returns (Q, R) unchanged.
    """
    if dim == 0:
        return Q, R

    Nx = Q.shape[0]
    Nu = R.shape[0]

    Q_aug = np.zeros((Nx + dim, Nx + dim))
    Q_aug[:Nx, :Nx] = Q
    np.fill_diagonal(Q_aug[Nx:, Nx:], style.q_weight)

    R_aug = np.zeros((Nu + dim, Nu + dim))
    R_aug[:Nu, :Nu] = R
    np.fill_diagonal(R_aug[Nu:, Nu:], style.r_weight)

    return Q_aug, R_aug


def tracking_augment_x0(x0: np.ndarray, dim: int,
                       style: AugmentationStyle = ERROR_STYLE):
    """
    Augment initial state with tracking states.

    Args:
        x0: Initial state (Nx,)
        dim: Number of tracking states
        style: Augmentation configuration

    Returns:
        x0_aug: Augmented initial state (Nx+dim,)

    If dim == 0, returns x0 unchanged.
    """
    if dim == 0:
        return x0

    if style.x0_fill == "zeros":
        fill_value = np.zeros(dim)
    else:
        fill_value = np.full(dim, style.x0_fill)

    return np.concatenate([x0, fill_value])


def tracking_augment_trajectory(traj: np.ndarray, dim: int,
                               style: AugmentationStyle = ERROR_STYLE):
    """
    Augment trajectory with tracking reference.

    Args:
        traj: Trajectory (Nx x T)
        dim: Number of tracking states
        style: Augmentation configuration

    Returns:
        traj_aug: Augmented trajectory ((Nx+dim) x T)

    If dim == 0, returns traj unchanged.
    """
    if dim == 0:
        return traj

    Nx, T = traj.shape

    if style.traj_fill == "zeros":
        aug_block = np.zeros((dim, T))
    elif style.traj_fill == "copy":
        # Copy first dim rows of traj
        aug_block = traj[:dim, :]
    else:
        aug_block = np.full((dim, T), style.traj_fill)

    return np.vstack([traj, aug_block])


def tracking_augment_constraints(x_min: list, x_max: list, dim: int,
                                track_min=None, track_max=None):
    """
    Augment state constraints with tracking state limits.

    Args:
        x_min: Minimum state values
        x_max: Maximum state values
        dim: Number of tracking states
        track_min: Minimum tracking state values (None = unconstrained)
        track_max: Maximum tracking state values (None = unconstrained)

    Returns:
        x_min_aug, x_max_aug: Augmented constraint lists

    If dim == 0, returns (x_min, x_max) unchanged.
    """
    if dim == 0:
        return x_min, x_max

    track_min = track_min or [None] * dim
    track_max = track_max or [None] * dim

    return x_min + track_min, x_max + track_max


# ============================================================================
# Delay Augmentation
# ============================================================================

def delay_augment_system(A: np.ndarray, B: np.ndarray, RTT_max: int):
    """
    Augment system with delay buffer states.

    Creates shift register: [x_current; x_delay1; x_delay2; ...; x_delayN]

    Args:
        A: State matrix (Nx x Nx)
        B: Input matrix (Nx x Nu)
        RTT_max: Maximum round-trip time (number of delay steps)

    Returns:
        A_aug: Augmented state matrix ((Nx*(1+RTT_max)) x (Nx*(1+RTT_max)))
        B_aug: Augmented input matrix ((Nx*(1+RTT_max)) x Nu)

    If RTT_max == 0, returns (A, B) unchanged.
    """
    if RTT_max == 0:
        return A, B

    Nx = A.shape[0]
    Nu = B.shape[1]

    # Build augmented A: [A, 0; I, 0] (shift register)
    A_aug = np.vstack([
        np.hstack([A, np.zeros((Nx, RTT_max * Nx))]),
        np.hstack([np.eye(RTT_max * Nx), np.zeros((RTT_max * Nx, Nx))])
    ])

    # Build augmented B: [B; 0]
    B_aug = np.vstack([
        B,
        np.zeros((RTT_max * Nx, Nu))
    ])

    return A_aug, B_aug


def delay_augment_x0(x0: np.ndarray, RTT_max: int, style: str = "repeat"):
    """
    Augment initial condition with delay buffer.

    Args:
        x0: Initial state (Nx,)
        RTT_max: Maximum round-trip time
        style: "zeros" (empty buffer) or "repeat" (buffer filled with x0)

    Returns:
        x0_aug: Augmented initial state (Nx*(1+RTT_max),)

    If RTT_max == 0, returns x0 unchanged.
    """
    if RTT_max == 0:
        return x0

    if style == "zeros":
        return np.concatenate([x0, np.zeros(RTT_max * len(x0))])
    elif style == "repeat":
        return np.tile(x0, RTT_max + 1)
    else:
        raise ValueError(f"Unknown style: {style}")


def delay_augment_trajectory(traj: np.ndarray, RTT_max: int,
                            Nx_base: int, style: str = "shift"):
    """
    Augment trajectory with delayed copies.

    Args:
        traj: Trajectory (Nx_base x T)
        RTT_max: Maximum round-trip time
        Nx_base: Dimension of each delay block (includes tracking if already augmented)
        style: "shift" (proper time-shifted delays) or "repeat" (all blocks identical)

    Returns:
        traj_aug: Augmented trajectory ((Nx_base*(1+RTT_max)) x T)

    If RTT_max == 0, returns traj unchanged.
    """
    if RTT_max == 0:
        return traj

    _, T = traj.shape
    Nx_aug = (RTT_max + 1) * Nx_base
    traj_aug = np.zeros((Nx_aug, T))

    if style == "shift":
        # Current state block
        traj_aug[:Nx_base, :] = traj

        # Delayed blocks (shifted right)
        x0 = traj[:Nx_base, 0]
        for k in range(1, RTT_max + 1):
            start_row = k * Nx_base
            end_row = (k + 1) * Nx_base

            # Shift trajectory right by k steps
            traj_aug[start_row:end_row, k:] = traj[:Nx_base, :-k]
            # Fill initial k steps with x0
            traj_aug[start_row:end_row, :k] = x0[:, None]

    elif style == "repeat":
        # All blocks identical (no actual delay)
        for k in range(RTT_max + 1):
            start_row = k * Nx_base
            end_row = (k + 1) * Nx_base
            traj_aug[start_row:end_row, :] = traj

    return traj_aug


def delay_augment_weights(Q: np.ndarray, RTT_max: int,
                         style: str = "zeros", scale: float = 1.0):
    """
    Augment cost matrix for delay states.

    Args:
        Q: State cost matrix (Nx x Nx)
        RTT_max: Maximum round-trip time
        style: "zeros" (only penalize current),
               "repeat" (same weight all delays),
               "scaled" (scale down delayed states)
        scale: Scaling factor for delayed states (used with style="scaled")

    Returns:
        Q_aug: Augmented state cost ((Nx*(1+RTT_max)) x (Nx*(1+RTT_max)))

    If RTT_max == 0, returns Q unchanged.
    """
    if RTT_max == 0:
        return Q

    if style == "zeros":
        Nx = Q.shape[0]
        Q_aug = np.zeros(((RTT_max + 1) * Nx, (RTT_max + 1) * Nx))
        Q_aug[:Nx, :Nx] = Q  # Only current state weighted
        return Q_aug

    elif style == "repeat":
        return np.kron(np.eye(RTT_max + 1), Q)

    elif style == "scaled":
        blocks = [Q] + [scale * Q for _ in range(RTT_max)]
        return np.block([
            [blocks[i] if i == j else np.zeros_like(Q)
             for j in range(RTT_max + 1)]
            for i in range(RTT_max + 1)
        ])
    else:
        raise ValueError(f"Unknown style: {style}")


def delay_augment_constraints(x_min: list, x_max: list,
                             RTT_max: int, style: str = "repeat"):
    """
    Augment constraints for delay states.

    Args:
        x_min: Minimum state values
        x_max: Maximum state values
        RTT_max: Maximum round-trip time
        style: "repeat" (replicate constraints for all delays)

    Returns:
        x_min_aug, x_max_aug: Augmented constraint lists

    If RTT_max == 0, returns (x_min, x_max) unchanged.
    """
    if RTT_max == 0:
        return x_min, x_max

    if style == "repeat":
        return (x_min * (RTT_max + 1), x_max * (RTT_max + 1))
    else:
        raise ValueError(f"Unknown style: {style}")


# ============================================================================
# Helper Functions
# ============================================================================

def make_delay_matrices_chain(Nx: int, Nu: int, D: np.ndarray):
    """
    Create Delta_s, Delta_a from delay adjacency matrix D.

    Assumes chain topology where delays accumulate along diagonal.

    Args:
        Nx: Number of states
        Nu: Number of inputs
        D: Delay adjacency matrix (Nn x Nn)

    Returns:
        Delta_s: Sensor delays (Nn x Nx)
        Delta_a: Actuator delays (Nn x Nu)
    """
    Nn = D.shape[0]
    delay_x = np.zeros((Nn, 1))
    delay_u = np.zeros((Nn, 1))

    # Cumulative sum along forward diagonal (sensor delays)
    delay_x[1:] = np.cumsum(np.diag(D, k=1)).reshape(-1, 1)
    # Cumulative sum along backward diagonal (actuator delays)
    delay_u[1:] = np.cumsum(np.diag(D, k=-1)).reshape(-1, 1)

    # Replicate across all states/inputs
    Delta_s = np.tile(delay_x, (1, Nx))
    Delta_a = np.tile(delay_u, (1, Nu))

    return Delta_s, Delta_a


def calc_RTT_max(Delta_s: np.ndarray, Delta_a: np.ndarray) -> int:
    """
    Calculate maximum round-trip time from delay matrices.

    Args:
        Delta_s: Sensor delays (Nn x Nx)
        Delta_a: Actuator delays (Nn x Nu)

    Returns:
        RTT_max: Maximum round-trip time (integer)
    """
    RTT_per_row = np.max(Delta_s, axis=1) + np.max(Delta_a, axis=1)
    return int(np.max(RTT_per_row))


def horizon_augment_traj(traj: np.ndarray, horizon: int):
    """
    Extend trajectory by repeating last value for MPC horizon.

    Args:
        traj: Trajectory (Nx x T)
        horizon: Number of steps to extend

    Returns:
        traj_aug: Extended trajectory (Nx x (T+horizon))
    """
    return np.concatenate([traj, np.tile(traj[:, [-1]], (1, horizon))], axis=1)
