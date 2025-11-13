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
        horizon: int,
        max_iter: int = 10000,
        debug=False,
    ):
        super().__init__(debug=debug)
        self.A = Ad
        self.B = Bd
        self.Q = Q
        self.R = R
        self.H = horizon
        self.max_iter = max_iter

        self.dim_x = self.A.shape[1]
        self.dim_u = self.B.shape[1]

        self.x = cp.Variable((self.dim_x, self.H + 1), name="x")
        self.u = cp.Variable((self.dim_u, self.H), name="u")
        self.r = cp.Parameter((self.dim_x, self.H), name="r")
        self.y = cp.Parameter((self.dim_x), name="y")

        cost = 0
        constr = [self.x[:, 0] == self.y]

        for i in range(self.H):
            constr += [
                self.x[:, i + 1] == self.A @ self.x[:, i] + self.B @ self.u[:, i]
            ]

            error = self.x[:, i] - self.r[:, i]
            cost += cp.quad_form(error, self.Q) + cp.quad_form(self.u[:, i], self.R)

        self.problem = cp.Problem(cp.Minimize(cost), constr)

    def __call__(
        self, reference: npt.NDArray[np.float64], observation: npt.NDArray[np.float64]
    ):
        self.y.value = observation
        self.r.value = reference

        self.problem.solve(max_iter=self.max_iter)

        if self.debug:
            print(
                f"Status: {self.problem.status} | \
                Min. Cost: {self.problem.value} | \
                Opt. u: {self.u.value} | \
                Opt. x: {self.x.value}"
            )

        return self.u.value
