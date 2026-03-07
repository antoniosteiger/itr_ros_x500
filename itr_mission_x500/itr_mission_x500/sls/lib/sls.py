from typing import Callable
import numpy as np
import cvxpy as cp
import warnings
from itr_controller_x500 import Controller
import time
from typing import Optional, Tuple


class SLS():
    def __init__(
        self,
        Ad: np.ndarray,
        Bd: np.ndarray,
        Q: np.ndarray | None = None,
        R: np.ndarray | None = None,
        Phi_u_sparsity: np.ndarray | None = None,
        x_min: list[float | None] | None = None,
        x_max: list[float | None] | None = None,
        u_min: list[float | None] | None = None,
        u_max: list[float | None] | None = None,
        horizon: int = 10,
        solver: str = cp.CLARABEL,
        max_iter: int = 1000,
        solver_params: dict | None = None,
        debug: bool = False,
    ):
        """
        Initialize the SLS controller.

        Args:
            Ad: Discrete state transition matrix.
            Bd: Discrete input matrix.
            Q: State weighting matrix. Defaults to identity if None.
            R: Input weighting matrix. Defaults to identity if None.
            Phi_u_sparsity: Binary matrix defining the sparsity structure of the controller.
            x_min: Lower bounds on states. Each element is a float or None (unconstrained).
            x_max: Upper bounds on states. Each element is a float or None (unconstrained).
            u_min: Lower bounds on inputs. Each element is a float or None (unconstrained).
            u_max: Upper bounds on inputs. Each element is a float or None (unconstrained).
            horizon: Prediction horizon (number of steps).
            solver: CVXPY solver to use (e.g. cp.CLARABEL, cp.MOSEK).
            max_iter: Maximum number of solver iterations.
            solver_params: Dictionary of solver-specific parameters (e.g. tolerances).
            debug: Enable debug output.
        """
        super().__init__()

        self.Ad = Ad
        self.Bd = Bd
        self.Nx = Ad.shape[0]
        self.Nu = Bd.shape[1]
        self.H = horizon
        self.x_min = x_min if x_min is not None else []
        self.x_max = x_max if x_max is not None else []
        self.u_min = u_min if u_min is not None else []
        self.u_max = u_max if u_max is not None else []
        
        self.solver = solver
        self.max_iter = max_iter
        self.solver_params = solver_params or {
            "tol_gap_abs": 1e-4,
            "tol_gap_rel": 1e-4,
            "tol_feas": 1e-4,
        } 
        self.debug = debug
        self.step = 0
        self.horizon = horizon

        if Q is None:
            self.Q = np.eye(self.Nx)
        else:
            self.Q = Q
        if R is None:
            self.R = np.eye(self.Nu)
        else:
            self.R = R
        if Phi_u_sparsity is None:
            self.Phi_u_sparsity = np.ones(self.Nx)
        else:
            self.Phi_u_sparsity = Phi_u_sparsity

        self.Phi_x_sparsity = make_Phi_x_sparsity(self.Ad, self.Bd, self.Phi_u_sparsity)


        # Solver parameters
        self.w = cp.Parameter(self.Nx)
        self.r = [cp.Parameter(self.Nx) for _ in range(self.H)]

        # Solver variables
        self.Phi_x = cp.Variable((self.H, self.Nx, self.Nx), sparsity=np.where(self.Phi_x_sparsity == 1))
        self.Phi_u = cp.Variable((self.H, self.Nu, self.Nx), sparsity=np.where(self.Phi_u_sparsity == 1))

        # Solver objective
        objective = 0
        for t in range(self.H):
            # Don't do tracking error cost if errors are already in state
            # if error_dim == 0:
            #     tracking_error = self.Phi_x[t] @ self.w - self.r[t]
            #     state_cost = cp.sum_squares(self.Q @ tracking_error)
            # else:
                # state_cost = cp.sum_squares(self.Q @ self.Phi_x[t] @ self.w) # With tracking error it performs better
            tracking_error = self.Phi_x[t] @ self.w - self.r[t]
            state_cost = cp.sum_squares(self.Q @ tracking_error)

            control_cost = cp.sum_squares(self.R @ self.Phi_u[t] @ self.w)
            objective += state_cost + control_cost

        # Solver constraints
        ## Initial Condition
        constraints = [
                self.Phi_x[0] == np.eye(self.Nx)
        ]
        ## Dynamics Constraint
        for t in range(self.H - 1):
            constraints += [ self.Phi_x[t+1] == self.Ad @ self.Phi_x[t] + self.Bd @ self.Phi_u[t] ]
        ## Input & State Limit constraints
        for t in range(self.H):
            x_t = self.Phi_x[t] @ self.w
            u_t = self.Phi_u[t] @ self.w
            for j, max in enumerate(self.x_max):
                if max:
                    constraints += [ x_t[j] <= max ]
            for j, min in enumerate(self.x_min):
                if min:
                    constraints += [ x_t[j] >= min ]
            for j, max in enumerate(self.u_max):
                if max:
                    constraints += [ u_t[j] <= max ]
            for j, min in enumerate(self.u_min):
                if min:
                    constraints += [ u_t[j] >= min ]

        # Solver problem definition
        self.prob = cp.Problem(cp.Minimize(objective), constraints)

        # Disable some harmless cvxpy warnings
        warnings.filterwarnings("ignore", message=".*dimension greater than 2.*")
        warnings.filterwarnings("ignore", message=".*sparse CVXPY expression.*")

    def __call__(self, reference, observation):
        
        self.w.value = observation
        for t in range(self.H):
            self.r[t].value = reference[:, t]

        if self.solver == cp.MOSEK:
            self.prob.solve(
                solver=self.solver,
                verbose=self.debug,
                warm_start=True,
                mosek_params=self.solver_params
                # TODO: Max iter
            )
        else:
            self.prob.solve(
                solver=self.solver,
                verbose=self.debug,
                max_iter=self.max_iter,
                warm_start=True,
                **self.solver_params
            )
            

        if self.prob.status == cp.INFEASIBLE:
            return np.zeros(Nu)

        else:
            # Print stats
            n_vars = self.prob.size_metrics.num_scalar_variables
            n_constraints = self.prob.size_metrics.num_scalar_eq_constr + self.prob.size_metrics.num_scalar_leq_constr
            time = self.prob.solver_stats.solve_time
            
            print(
                f"Step {self.step:3d} | Cost: {self.prob.value:8.2f} | Status: {self.prob.status:8s} | "
                f"Time: {time:6.4f}s | Vars: {n_vars:5d} | Cons: {n_constraints:5d}",
            )

            u = self.Phi_u[0].value @ observation            
           
            self.step += 1

            return u, self.Phi_x[0].value, self.Phi_u[0].value


def solve_sls_fir(A: np.ndarray, B: np.ndarray,
                  Q: np.ndarray, R: np.ndarray,
                  horizon: int,
                  Phi_u_sparsity: np.ndarray | None = None,
                  fir_mode: str = "hard",  # "hard" or "soft"
                  solver: str = cp.MOSEK,
                  solver_params: Optional[dict] = None,
                  verbose: bool = False) -> Tuple[np.ndarray, np.ndarray, dict]:
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
    # fir_residual = A @ Phi_x[horizon-1] + B @ Phi_u[horizon-1]
    # if fir_mode == "hard":
    #     if Phi_u_sparsity is None:
    #         # No sparsity: full hard constraint
    #         constraints += [fir_residual == np.zeros((Nx, Nx))]
    #     else:
    #         # With sparsity: constrain only controllable elements
    #         feasible, uncont = check_fir_feasibility(A, B, Phi_u_sparsity, horizon)
    #         if not feasible:
    #             print(f"WARNING: Sparsity prevents full controllability. "
    #                         f"States with uncontrollable elements: {uncont}. "
    #                         f"FIR constraint applied only to controllable elements.")
    #         else:
    #             print("✓ FIR feasibility check complete. All states controllable")
    #         # Compute controllable mask: where B @ Phi_u can be non-zero
    #         B_Phi_u_pattern = (np.abs(B @ Phi_u_sparsity[horizon-1]) > 0).astype(int)
    #         controllable_mask = B_Phi_u_pattern
    #         constraints += [cp.multiply(controllable_mask, fir_residual) == 0]
    # elif fir_mode == "soft":
    #     # Soft constraint: add penalty to objective
    #     objective += cp.sum_squares(fir_residual)
    # elif fir_mode == "none":
    #     pass
    # else:
    #     raise ValueError(f"Unknown fir_mode: {fir_mode}. Use 'hard' or 'soft'.")

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

def make_Phi_x_sparsity(Ad, Bd, Phi_u_sparsity):
    Nx = Ad.shape[0]
    horizon = Phi_u_sparsity.shape[0]
    Phi_x_sparsity_list = [np.eye(Nx)]
    
    # Build sparsity for Phi_x
    for i in range(horizon-1):
        APhix = Ad @ Phi_x_sparsity_list[i]
        BPhiu = Bd @ Phi_u_sparsity[i]

        Phi_x_sparsity_next = (np.abs(APhix + BPhiu) > 0).astype(int)
        Phi_x_sparsity_list.append(Phi_x_sparsity_next)

    Phi_x_sparsity = np.array(Phi_x_sparsity_list)

    return Phi_x_sparsity


def make_delay_matrices_chain(Nx, Nu, D):
    Nn = D.shape[0]
    delay_x = np.zeros((Nn, 1))
    delay_u = np.zeros((Nn, 1))

    delay_x[1:] = np.cumsum(np.diag(D, k=1)).reshape(-1, 1)
    delay_u[1:] = np.cumsum(np.diag(D, k=-1)).reshape(-1, 1)

    Delta_s = np.tile(delay_x, (1, Nx))
    Delta_a = np.tile(delay_u, (1, Nu))

    return Delta_s, Delta_a


