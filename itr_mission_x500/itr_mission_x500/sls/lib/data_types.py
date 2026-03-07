"""
Type definitions for SLS control system.

Provides coarse-grained dataclasses for clean interfaces:
- System: Dynamics and cost matrices
- Network: Delay topology
- Constraints: State/input limits
- Trajectory: Reference signal data
"""

from dataclasses import dataclass, field
import numpy as np
from typing import Optional


@dataclass
class System:
    """
    System dynamics and cost matrices.

    Attributes:
        A: Continuous-time state matrix (Nx x Nx)
        B: Continuous-time input matrix (Nx x Nu)
        Q: State cost matrix (Nx x Nx)
        R: Input cost matrix (Nu x Nu)
        dt: Discretization timestep
    """
    A: np.ndarray
    B: np.ndarray
    Q: np.ndarray
    R: np.ndarray
    dt: float

    @property
    def Nx(self) -> int:
        """Number of states"""
        return self.A.shape[0]

    @property
    def Nu(self) -> int:
        """Number of inputs"""
        return self.B.shape[1]


@dataclass
class Network:
    """
    Network delay topology.

    Attributes:
        D: Delay adjacency matrix (Nn x Nn)
        Delta_s: Sensor delays (Nn x Nx), computed via compute_delays()
        Delta_a: Actuator delays (Nn x Nu), computed via compute_delays()
        RTT_max: Maximum round-trip time, computed via compute_delays()
    """
    D: np.ndarray
    Delta_s: Optional[np.ndarray] = None
    Delta_a: Optional[np.ndarray] = None
    RTT_max: Optional[int] = None

    def compute_delays(self, Nx: int, Nu: int):
        """
        Compute Delta_s, Delta_a, RTT_max from D matrix.

        Args:
            Nx: Number of states
            Nu: Number of inputs
        """
        from lib.augmentation import make_delay_matrices_chain, calc_RTT_max

        self.Delta_s, self.Delta_a = make_delay_matrices_chain(Nx, Nu, self.D)
        self.RTT_max = calc_RTT_max(self.Delta_s, self.Delta_a)


@dataclass
class Constraints:
    """
    State and input constraints.

    Use None for unconstrained dimensions.

    Attributes:
        x_min: Minimum state values (list of float | None)
        x_max: Maximum state values (list of float | None)
        u_min: Minimum input values (list of float | None)
        u_max: Maximum input values (list of float | None)
    """
    x_min: list
    x_max: list
    u_min: Optional[list] = None
    u_max: Optional[list] = None


@dataclass
class Trajectory:
    """
    Reference trajectory data.

    Attributes:
        data: Trajectory array (Nx x T)
        dt: Timestep
        Nx_original: Original state dimension before augmentation
    """
    data: np.ndarray
    dt: float
    Nx_original: int

    @property
    def T(self) -> int:
        """Number of timesteps"""
        return self.data.shape[1]

    @property
    def Nx(self) -> int:
        """Current state dimension (may be augmented)"""
        return self.data.shape[0]
