"""
System dynamics definitions.

Provides:
- LTI: Base class for linear time-invariant systems
- Quadcopter: Quadcopter dynamics in NED frame
- QuadcopterParameters: Quadcopter physical parameters
- X500: Holybro X500 quadcopter parameters
"""

from abc import ABC
from dataclasses import dataclass
from typing import Literal

import numpy as np
from numpy import typing as npt
from scipy.signal import cont2discrete
from scipy.sparse import csr_matrix


class LTI(ABC):
    """
    Linear Time-Invariant system base class.
    Generates discrete and sparse versions of all system matrices automatically.
    """

    def __init__(
        self,
        A: npt.NDArray[np.float64],
        B: npt.NDArray[np.float64],
        C: npt.NDArray[np.float64],
        D: npt.NDArray[np.float64],
        d_t: float = 0.01,
        d_method: Literal[
            "zoh", "gbt", "bilinear", "euler", "backward_diff", "foh", "impulse"
        ] = "zoh",
    ):
        """
        Initialize linear time-invariant base class.

        Args:
            A: State matrix
            B: Input matrix
            C: Output matrix
            D: Direct transmission matrix
            d_t: Discretization time-interval in seconds
            d_method: Discretization method
        """
        self.A = A
        self.B = B
        self.C = C
        self.D = D
        self.Ad, self.Bd, self.Cd, self.Dd = self._make_discrete_matrices(
            self.A, self.B, self.C, self.D, d_t, d_method
        )
        self.A_sparse, self.B_sparse, self.C_sparse, self.D_sparse = (
            self._make_sparse_matrices(self.A, self.B, self.C, self.D)
        )
        self.Ad_sparse, self.Bd_sparse, self.Cd_sparse, self.Dd_sparse = (
            self._make_sparse_matrices(self.Ad, self.Bd, self.Cd, self.Dd)
        )

    def _make_sparse_matrices(self, A, B, C, D):
        A_sparse = csr_matrix(A)
        B_sparse = csr_matrix(B)
        C_sparse = csr_matrix(C)
        D_sparse = csr_matrix(D)

        return A_sparse, B_sparse, C_sparse, D_sparse

    def _make_discrete_matrices(self, A, B, C, D, dt, d_method):
        Ad, Bd, Cd, Dd, _ = cont2discrete((A, B, C, D), dt, method=d_method)

        return Ad, Bd, Cd, Dd


@dataclass
class QuadcopterParameters:
    """Physical parameters for quadcopter model."""
    g: float
    mass: float
    I_x: float
    I_y: float
    I_z: float
    c_d: float
    c_t: float
    tau_x_max: float
    tau_y_max: float
    thrust_max: float
    hover_throttle: float


class Quadcopter(LTI):
    """
    Quadcopter dynamics in NED frame.

    State order: [x, y, z, vx, vy, vz, φ, θ, ψ, p, q, r]
    - Position: x (North), y (East), z (Down)
    - Velocity: vx, vy, vz (body-frame velocities)
    - Angles: φ (roll), θ (pitch), ψ (yaw)
    - Rates: p (roll rate), q (pitch rate), r (yaw rate)

    Input order: [thrust, τ_roll, τ_pitch, τ_yaw]
    - All inputs are deviations from hover

    Reference: https://arxiv.org/pdf/1908.07401
    """

    def __init__(self, params: QuadcopterParameters, d_t: float = 0.01, d_method: str = "zoh"):
        A = np.zeros((12, 12))
        B = np.zeros((12, 4))

        # Position derivatives = velocities
        A[0, 3] = 1  # x_dot = vx
        A[1, 4] = 1  # y_dot = vy
        A[2, 5] = 1  # z_dot = vz

        # Velocity dynamics (gravity coupling)
        A[3, 7] = -params.g   # vx affected by pitch θ
        A[4, 6] = params.g    # vy affected by roll φ

        # Angle derivatives = angular rates
        A[6, 9] = 1   # φ_dot = p
        A[7, 10] = 1  # θ_dot = q
        A[8, 11] = 1  # ψ_dot = r

        # Input coupling
        B[5, 0] = 1 / params.mass   # Thrust → vz (to go up, thrust is negative in NED)
        B[9, 1] = 1 / params.I_x    # Roll torque → p
        B[10, 2] = 1 / params.I_y   # Pitch torque → q
        B[11, 3] = 1 / params.I_z   # Yaw torque → r

        C = np.identity(12)
        D = np.zeros((12, 4))

        super().__init__(A, B, C, D, d_t, d_method=d_method)


# Holybro X500 quadcopter parameters
X500 = QuadcopterParameters(
    g=9.807232,  # m/s² (Value for Munich)
    mass=2.0,  # kg
    # Inertia values from PX4 model repository:
    # https://github.com/PX4/PX4-gazebo-models/blob/main/models/x500_base/model.sdf
    I_x=0.02166666,  # Pitch inertia [kgm²]
    I_y=0.02166666,  # Roll inertia [kgm²]
    I_z=0.04000000,  # Yaw inertia [kgm²]
    # Motor coefficients from X500 motor specs:
    # https://de.aliexpress.com/item/1005003708521114.html
    c_d=1.7024e-9,  # Drag coefficient
    c_t=1.1799e-7,  # Lift coefficient
    tau_x_max=2 * 13.4 * 0.2,  # Maximum roll torque [Nm]
    tau_y_max=2 * 13.4 * 0.13,  # Maximum pitch torque [Nm]
    thrust_max=27.062,  # Maximum thrust [N] = (-9.81*2)/-0.725
    hover_throttle=0.725,  # Hover throttle fraction
)
