from abc import ABC, abstractmethod

import numpy as np
from numpy import typing as npt


class Controller(ABC):
    def __init__(self):
        return

    @abstractmethod
    def __call__(
        self, reference: npt.NDArray[np.floating], observation: npt.NDArray[np.floating]
    ) -> npt.NDArray[np.floating]:
        """
        Compute the control input for the drone based on a state observation.

        Args:
            observation (npt.NDArray[np.float64]): 13D array: [position, velocity, quaternion, angular velocity]

        Returns:
           (thrust, attitude) : thrust and attitude to send to the drone
        """
        raise NotImplementedError

    # @abstractmethod
    # def is_finished(self) -> bool:
    #     """
    #     Callback function to be called once after one control step.

    #     Returns:
    #         bool: a flag to signal if the controller has finished
    #     """
    #     raise NotImplementedError

    def _log(self, msg: str):
        print(f"\033[32m[ITR_CONTROL]: {msg} \033[0m")


def main(args=None):
    # TODO
    return


if __name__ == "__main__":
    main()
