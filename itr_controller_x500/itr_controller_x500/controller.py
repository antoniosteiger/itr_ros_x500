from abc import ABC, abstractmethod


class Controller(ABC):
    def __init__(self):
        return

    @abstractmethod
    def __call__(self, reference, observation) -> None:
        """
        Compute the control input for the drone based on a state observation and setpoint/reference.

        Args:
            reference (numpy float array)
            observation (numpy float array)

        Returns: None

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
