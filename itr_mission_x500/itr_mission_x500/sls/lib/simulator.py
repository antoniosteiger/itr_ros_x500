"""
Generic simulation loops for control systems.

Provides simulation functions for trajectory tracking with various controller types.
"""

import numpy as np
from typing import Dict, Optional


def simulate_trajectory_tracking(A: np.ndarray, B: np.ndarray,
                                K: np.ndarray,
                                trajectory: np.ndarray,
                                x0: np.ndarray,
                                Nx_original: int = 12,
                                verbose: bool = False) -> Dict:
    """
    Simulate trajectory tracking with static gain K.

    Control law: u = K @ (x - x_ref)

    Args:
        A: Augmented discrete-time state matrix
        B: Augmented discrete-time input matrix
        K: Static feedback gain
        trajectory: Reference trajectory (augmented, Nx_aug x T)
        x0: Initial state (augmented, Nx_aug)
        Nx_original: Original state dimension before augmentation (for error computation)
        verbose: Print progress

    Returns:
        Dictionary with:
            'x_series': State trajectory (Nx_aug x (T+1))
            'u_series': Control inputs (Nu x T)
            'tracking_errors': Position tracking errors (T,)
            'rms_error': RMS position tracking error
    """
    T = trajectory.shape[1]
    Nx = A.shape[0]
    Nu = B.shape[1]

    xs = np.zeros((Nx, T + 1))
    us = np.zeros((Nu, T))
    tracking_errors = np.zeros(T)

    xs[:, 0] = x0

    for t in range(T):
        # Reference state at time t
        x_ref = trajectory[:, t]

        # Tracking error
        error = xs[:, t] - x_ref

        # Control law: regulate error to zero
        u = K @ error

        # Step system
        xs[:, t+1] = A @ xs[:, t] + B @ u
        us[:, t] = u

        # Position tracking error (Euclidean distance, first 3 states)
        pos_error = xs[:3, t] - x_ref[:3]
        tracking_errors[t] = np.linalg.norm(pos_error)

        if verbose and t % 20 == 0:
            print(f"t={t:3d} | Pos Error: {tracking_errors[t]:6.4f} | "
                  f"Pos: [{xs[0,t]:5.2f}, {xs[1,t]:5.2f}, {xs[2,t]:5.2f}]")

    rms_error = np.sqrt(np.mean(tracking_errors**2))

    return {
        'x_series': xs,
        'u_series': us,
        'tracking_errors': tracking_errors,
        'rms_error': rms_error
    }


def simulate_closed_loop_regulation(A: np.ndarray, B: np.ndarray,
                                   K: np.ndarray,
                                   x0: np.ndarray,
                                   T: int = 100) -> Dict:
    """
    Simulate closed-loop regulation to origin.

    Control law: u = K @ x

    Args:
        A: Discrete-time state matrix
        B: Discrete-time input matrix
        K: Static feedback gain
        x0: Initial state
        T: Number of timesteps

    Returns:
        Dictionary with:
            'x_series': State trajectory (Nx x (T+1))
            'u_series': Control inputs (Nu x T)
    """
    Nx = A.shape[0]
    Nu = B.shape[1]

    xs = np.zeros((Nx, T+1))
    us = np.zeros((Nu, T))
    xs[:, 0] = x0

    for t in range(T):
        u = K @ xs[:, t]
        xs[:, t+1] = A @ xs[:, t] + B @ u
        us[:, t] = u

    return {
        'x_series': xs,
        'u_series': us
    }
