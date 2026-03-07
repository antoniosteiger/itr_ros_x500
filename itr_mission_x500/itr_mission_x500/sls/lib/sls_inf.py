"""
Infinite-horizon SLS via FIR approximation.

Solves infinite-horizon LQR using System Level Synthesis with a Finite Impulse
Response (FIR) constraint.
"""

import numpy as np
import cvxpy as cp
from typing import Tuple, Optional
import warnings


def solve_sls_fir(A: np.ndarray, B: np.ndarray,
                  Q: np.ndarray, R: np.ndarray,
                  horizon: int,
                  Phi_u_sparsity: np.ndarray | None = None,
                  fir_mode: str = "hard",  # "hard" or "soft"
                  solver: str = cp.MOSEK,
                  solver_params: Optional[dict] = None,
                  verbose: bool = False) -> Tuple[np.ndarray, np.ndarray]:
    """
    Solve infinite-horizon LQR using SLS FIR approximation.

    Minimizes: sum_{t=0}^{H-1} trace(Phi_x[t]' Q Phi_x[t]) + trace(Phi_u[t]' R Phi_u[t])

    Subject to:
        Phi_x[0] = I
        Phi_x[t+1] = A Phi_x[t] + B Phi_u[t]  for t = 0, ..., H-2
        A Phi_x[H-1] + B Phi_u[H-1] = 0       (FIR constraint)

    The FIR constraint forces the system to reach the origin in finite time,
    approximating the infinite-horizon solution.

    Args:
        A: Discrete-time state matrix (Nx x Nx)
        B: Discrete-time input matrix (Nx x Nu)
        Q: State cost matrix (Nx x Nx)
        R: Input cost matrix (Nu x Nu)
        horizon: FIR horizon length
        Phi_u_sparsity: Sparsity pattern for Phi_u (horizon x Nu x Nx), optional
        fir_mode: FIR constraint mode
            - "hard": Equality constraint on controllable elements (auto-detects with sparsity)
            - "soft": Penalty on full FIR residual in objective
        solver: cvxpy solver (default: MOSEK)
        solver_params: Optional solver parameters
        verbose: Print solver output

    Returns:
        K: Static feedback gain (Nu x Nx)
        Phi_u: Full system response matrices (horizon x Nu x Nx)
        diagnostics: Dictionary containing:
            - status: Solver status
            - total_cost: Total objective value
            - state_cost: State cost contribution
            - input_cost: Input cost contribution
            - fir_residual_norm: Frobenius norm of FIR residual
            - K_norm: Frobenius norm of gain matrix
            - K_max: Maximum absolute element in K
            - spectral_radius: Closed-loop spectral radius
            - stable: Whether closed-loop is stable
            - (if sparsity) K_current_states_norm, K_delayed_states_norm
    """
    warnings.filterwarnings("ignore", message=".*dimension greater than 2.*")
    warnings.filterwarnings("ignore", message=".*sparse CVXPY expression.*")

    Nx = A.shape[0]
    Nu = B.shape[1]

    # Decision variables
    if Phi_u_sparsity is None:
        Phi_x = cp.Variable((horizon, Nx, Nx))
        Phi_u = cp.Variable((horizon, Nu, Nx))
    else:
        Phi_x_sparsity = make_Phi_x_sparsity(A, B, Phi_u_sparsity)
        Phi_x = cp.Variable((horizon, Nx, Nx), sparsity=np.where(Phi_x_sparsity == 1))
        Phi_u = cp.Variable((horizon, Nu, Nx), sparsity=np.where(Phi_u_sparsity == 1))

    # Cost matrices (use square roots for sum_squares formulation)
    Q_half = np.diag(np.sqrt(np.diag(Q)))
    R_half = np.diag(np.sqrt(np.diag(R)))

    # Objective: minimize trace(Phi_x' Q Phi_x) + trace(Phi_u' R Phi_u)
    objective = 0
    for t in range(horizon):
        objective += cp.sum_squares(Q_half @ Phi_x[t])
        objective += cp.sum_squares(R_half @ Phi_u[t])

    # Affine dynamics constraint
    constraints = [Phi_x[0] == np.eye(Nx)]
    for t in range(horizon - 1):
        constraints += [Phi_x[t+1] == A @ Phi_x[t] + B @ Phi_u[t]]

    # FIR constraint: system must reach origin at horizon
    fir_residual = A @ Phi_x[horizon-1] + B @ Phi_u[horizon-1]
    if fir_mode == "hard":
        if Phi_u_sparsity is None:
            # No sparsity: full hard constraint
            constraints += [fir_residual == np.zeros((Nx, Nx))]
        else:
            # With sparsity: constrain only controllable elements
            feasible, uncont = check_fir_feasibility(A, B, Phi_u_sparsity, horizon)
            if not feasible:
                print(f"WARNING: Sparsity prevents full controllability. "
                            f"States with uncontrollable elements: {uncont}. "
                            f"FIR constraint applied only to controllable elements.")
            else:
                print("✓ FIR feasibility check complete. All states controllable")
            # Compute controllable mask: where B @ Phi_u can be non-zero
            B_Phi_u_pattern = (np.abs(B @ Phi_u_sparsity[horizon-1]) > 0).astype(int)
            controllable_mask = B_Phi_u_pattern
            constraints += [cp.multiply(controllable_mask, fir_residual) == 0]
    elif fir_mode == "soft":
        # Soft constraint: add penalty to objective
        objective += cp.sum_squares(fir_residual)
    elif fir_mode == "none":
        pass
    else:
        raise ValueError(f"Unknown fir_mode: {fir_mode}. Use 'hard' or 'soft'.")

    # Solve
    prob = cp.Problem(cp.Minimize(objective), constraints)
    if solver == cp.MOSEK:
        prob.solve(solver=solver, warm_start=True, verbose=verbose, mosek_params=solver_params or {})
    else:
        prob.solve(solver=solver, warm_start=True, verbose=verbose, **(solver_params or {}))

    if verbose:
        print(f"H={horizon:3d} | Status: {prob.status} | Cost: {prob.value:.4f}")

    if prob.status not in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
        print(f"Warning: SLS FIR solver status: {prob.status}")
        raise Exception

    # Extract static gain: K = Phi_u[0] (since Phi_x[0] = I)
    K = Phi_u[0].value

    # Compute diagnostics
    diagnostics = {}
    diagnostics['status'] = prob.status
    diagnostics['total_cost'] = prob.value

    # Breakdown costs
    Phi_x_val = Phi_x.value
    Phi_u_val = Phi_u.value

    state_cost = 0
    input_cost = 0
    for t in range(horizon):
        state_cost += np.sum((Q_half @ Phi_x_val[t])**2)
        input_cost += np.sum((R_half @ Phi_u_val[t])**2)

    diagnostics['state_cost'] = state_cost
    diagnostics['input_cost'] = input_cost

    # FIR residual
    fir_residual_val = A @ Phi_x_val[horizon-1] + B @ Phi_u_val[horizon-1]
    diagnostics['fir_residual_norm'] = np.linalg.norm(fir_residual_val, 'fro')

    # Controller norms
    diagnostics['K_norm'] = np.linalg.norm(K)
    diagnostics['K_max'] = np.abs(K).max()

    # Sparsity info if applicable
    if Phi_u_sparsity is not None:
        diagnostics['sparsity_enforced'] = True
        diagnostics['K_current_states_norm'] = np.linalg.norm(K[:, :Nx])
        diagnostics['K_delayed_states_norm'] = np.linalg.norm(K[:, Nx:])
        diagnostics['controllable_mask_nonzeros'] = np.sum(controllable_mask) if 'controllable_mask' in locals() else None
    else:
        diagnostics['sparsity_enforced'] = False

    # Closed-loop stability
    A_cl = A + B @ K
    eigs = np.linalg.eigvals(A_cl)
    diagnostics['spectral_radius'] = np.max(np.abs(eigs))
    diagnostics['stable'] = diagnostics['spectral_radius'] < 1.0

    return K, Phi_u_val, diagnostics


def simulate_closed_loop(A: np.ndarray, B: np.ndarray,
                        K: np.ndarray, x0: np.ndarray,
                        T: int = 100) -> np.ndarray:
    """
    Simulate closed-loop system with static gain.

    Control law: u = K @ x

    Args:
        A: Discrete-time state matrix
        B: Discrete-time input matrix
        K: Static feedback gain
        x0: Initial state
        T: Number of timesteps

    Returns:
        xs: State trajectory (Nx x (T+1))
    """
    Nx = A.shape[0]
    xs = np.zeros((Nx, T+1))
    xs[:, 0] = x0

    for t in range(T):
        u = K @ xs[:, t]
        xs[:, t+1] = A @ xs[:, t] + B @ u

    return xs


def make_Phi_x_sparsity(Ad, Bd, Phi_u_sparsity, initial=None):
    """
    Build Phi_x sparsity from Phi_u (controller) sparsity.

    Args:
        Ad: Discrete-time state matrix
        B: Discrete-time input matrix
        Phi_u_sparsity: Controller sparsity over horizon
    """
    Nx = Ad.shape[0]
    horizon = Phi_u_sparsity.shape[0]
    if initial is None:
        initial = np.eye(Nx)
    Phi_x_sparsity_list = [initial]

    # Build sparsity for Phi_x
    for i in range(horizon-1):
        APhix = Ad @ Phi_x_sparsity_list[i]
        BPhiu = Bd @ Phi_u_sparsity[i]

        Phi_x_sparsity_next = (np.abs(APhix + BPhiu) > 0).astype(int)
        Phi_x_sparsity_list.append(Phi_x_sparsity_next)

    Phi_x_sparsity = np.array(Phi_x_sparsity_list)

    return Phi_x_sparsity


def print_diagnostics(diagnostics: dict, detailed: bool = False):
    """
    Pretty-print solver diagnostics.

    Args:
        diagnostics: Dictionary returned by solve_sls_fir
        detailed: Whether to show detailed breakdown
    """
    print("\n" + "="*70)
    print("SLS FIR Solver Diagnostics")
    print("="*70)

    print(f"Status:              {diagnostics['status']}")
    print(f"Total Cost:          {diagnostics['total_cost']:.6e}")

    if detailed:
        print(f"  State Cost:        {diagnostics['state_cost']:.6e}")
        print(f"  Input Cost:        {diagnostics['input_cost']:.6e}")

    print(f"\nController Norms:")
    print(f"  ||K||_F:            {diagnostics['K_norm']:.6e}")
    print(f"  max|K|:             {diagnostics['K_max']:.6e}")

    if diagnostics.get('sparsity_enforced', False):
        print(f"\nSparsity Info:")
        print(f"  ||K[:, current]||:  {diagnostics['K_current_states_norm']:.6e}")
        print(f"  ||K[:, delayed]||:  {diagnostics['K_delayed_states_norm']:.6e}")
        if diagnostics.get('controllable_mask_nonzeros') is not None:
            print(f"  Controllable mask:  {diagnostics['controllable_mask_nonzeros']} non-zeros")

    print(f"\nClosed-Loop Stability:")
    print(f"  Spectral Radius:    {diagnostics['spectral_radius']:.6f}")
    print(f"  Stable:             {'✓ Yes' if diagnostics['stable'] else '✗ No (unstable!)'}")

    print(f"\nFIR Residual:")
    print(f"  ||A Φ_x + B Φ_u||:  {diagnostics['fir_residual_norm']:.6e}")

    print("="*70 + "\n")


def check_fir_feasibility(A: np.ndarray, B: np.ndarray,
                         Phi_u_sparsity: np.ndarray,
                         horizon: int) -> Tuple[bool, list]:
    """
    Check if sparsity pattern allows hard FIR constraint.

    Args:
        A: Discrete-time state matrix
        B: Discrete-time input matrix
        Phi_u_sparsity: Sparsity pattern for Phi_u
        horizon: FIR horizon length

    Returns:
        feasible: Whether full hard FIR is possible
        uncontrollable_states: State indices that can't be driven to zero
    """
    Phi_x_sparsity = make_Phi_x_sparsity(A, B, Phi_u_sparsity)

    # Check: Can we make A @ Phi_x[H-1] + B @ Phi_u[H-1] structurally zero?
    A_Phi_x_pattern = (np.abs(A @ Phi_x_sparsity[horizon-1]) > 0).astype(int)
    B_Phi_u_pattern = (np.abs(B @ Phi_u_sparsity[horizon-1]) > 0).astype(int)

    # Elements that MUST be non-zero in A @ Phi_x but CAN'T be cancelled by B @ Phi_u
    forced_residual = A_Phi_x_pattern & (~B_Phi_u_pattern.astype(bool))

    # Check which states (rows) have uncontrollable elements
    uncontrollable_states = np.where(np.any(forced_residual, axis=1))[0].tolist()

    feasible = len(uncontrollable_states) == 0

    return feasible, uncontrollable_states
