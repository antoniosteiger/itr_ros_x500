import cvxpy as cp
import numpy as np
from numpy import typing as npt

from .controller import Controller


class MPC(Controller):
    def __init__(
        self,
        Ad: npt.NDArray[np.float64],
        Bd: npt.NDArray[np.float64],
        Q: npt.NDArray[np.float64],
        R: npt.NDArray[np.float64],
        horizon: int = 50,
        x_min: list = [],
        x_max: list = [],
        u_min: list = [],
        u_max: list = [],
        solver: str = cp.OSQP,
        max_iter: int = 10000,
        debug=False,
    ):
        super().__init__()
        self.A = Ad
        self.B = Bd
        self.Q = Q
        self.R = R
        self.H = horizon
        self.x_min = x_min
        self.x_max = x_max
        self.u_max = u_max
        self.u_min = u_min
        self.max_iter = max_iter
        self.solver = solver
        self.debug = debug

        self.dim_x = self.A.shape[1]
        self.dim_u = self.B.shape[1]

        self.x = cp.Variable((self.dim_x, self.H + 1), name="x")
        self.u = cp.Variable((self.dim_u, self.H), name="u")
        self.r = cp.Parameter((self.dim_x, self.H), name="r")
        self.y = cp.Parameter((self.dim_x), name="y")

        cost = 0
        constr = [self.x[:, 0] == self.y]

        for i in range(self.H):
            # Dynamics Constraint
            constr += [
                self.x[:, i + 1] == self.A @ self.x[:, i] + self.B @ self.u[:, i]
            ]
            # Input & State Limit Constraints
            for j, max in enumerate(self.x_max):
                if max:
                    constr += [self.x[j, i] <= max]
            for j, min in enumerate(self.x_min):
                if min:
                    constr += [self.x[j, i] >= min]
            for j, max in enumerate(self.u_max):
                if max:
                    constr += [self.u[j, i] <= max]
            for j, min in enumerate(self.u_min):
                if min:
                    constr += [self.u[j, i] >= min]

            error = self.x[:, i] - self.r[:, i]
            cost += cp.quad_form(error, self.Q) + cp.quad_form(self.u[:, i], self.R)

        self.problem = cp.Problem(cp.Minimize(cost), constr)

    def __call__(
        self, reference: npt.NDArray[np.float64], observation: npt.NDArray[np.float64]
    ):
        self.y.value = observation
        self.r.value = reference

        self.problem.solve(solver=self.solver, max_iter=self.max_iter)

        if self.debug:
            self._log(
                f"Status: {self.problem.status} | \
                Min. Cost: {self.problem.value} | \
                Opt. u: {self.u.value} | \
                Opt. x: {self.x.value}"
            )

        return self.u[:, 0].value
