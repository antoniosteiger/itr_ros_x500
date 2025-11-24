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
            A (npt.NDArray[np.float64]): State matrix
            B (npt.NDArray[np.float64]): Input matrix
            C (npt.NDArray[np.float64]): Output matrix
            D (npt.NDArray[np.float64]): Direct transmission matrix
            d_t (float, optional): Discretization time-interval in seconds. Defaults to 0.01
            d_method (float, optional): Discretization method. Defaults to "zoh"
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
        A_sparse = csr_matrix(self.A)
        B_sparse = csr_matrix(self.B)
        C_sparse = csr_matrix(self.C)
        D_sparse = csr_matrix(self.D)

        return A_sparse, B_sparse, C_sparse, D_sparse

    def _make_discrete_matrices(self, A, B, C, D, dt, d_method):
        Ad, Bd, Cd, Dd, _ = cont2discrete((A, B, C, D), dt, method=d_method)

        return Ad, Bd, Cd, Dd


@dataclass
class QuadcopterParameters:
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
    def __init__(self, params: QuadcopterParameters, d_t: float = 0.01):
        # For Matrix Definitions see: https://arxiv.org/pdf/1908.07401

        A = np.zeros((12, 12))
        A[0, 3] = 1
        A[1, 4] = 1
        A[2, 5] = 1
        A[3, 7] = -params.g
        A[4, 6] = params.g
        A[6, 9] = 1
        A[7, 10] = 1
        A[8, 11] = 1

        B = np.zeros((12, 4))
        B[5, 0] = 1 / params.mass  # To go up, thrust sent to drone needs to be negative
        B[9, 1] = 1 / params.I_x
        B[10, 2] = 1 / params.I_y
        B[11, 3] = 1 / params.I_z

        C = np.identity(12)
        D = np.zeros((12, 4))

        super().__init__(A, B, C, D, d_t)


X500 = QuadcopterParameters(
    g=9.807232,  # m/s² (Value for Munich)
    mass=2.0,  # kg
    # For experimental inertia values see: https://www.mdpi.com/2226-4310/8/11/355
    # Values used here are from px4 model repository: https://github.com/PX4/PX4-gazebo-models/blob/main/models/x500_base/model.sdf
    I_x=0.02166666,  # Pitch kgm^2
    I_y=0.02166666,  # Roll kgm²
    I_z=0.04000000,  # Yaw kgm2
    # c_d and c_t are calculated from a specification table of the x500 motors: https://de.aliexpress.com/item/1005003708521114.html
    c_d=1.7024e-9,  # Drag coefficient
    c_t=1.1799e-7,  # Lift coefficient
    tau_x_max=2 * 13.4 * 0.2,  # Maximum roll torque
    tau_y_max=2 * 13.4 * 0.13,  # Maximum pitch torque
    thrust_max=27.062,  # (-9.81m/s^2*2kg)/-0.725
    hover_throttle=0.725,
)
