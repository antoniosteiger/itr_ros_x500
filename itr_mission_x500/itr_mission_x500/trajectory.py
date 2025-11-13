import numpy as np
import numpy.typing as npt


def circle(num_points: int, startpoint: npt.NDArray[np.float64], radius: float):
    # Create an array to store the 12D vectors
    vectors = np.zeros((num_points, 12), dtype=np.float64)

    # Parameters for the circle in 3D space
    center = startpoint[:3]  # Starting position for the center of the circle
    theta = np.linspace(
        0, 2 * np.pi, num_points, endpoint=False
    )  # Angles to trace the circle

    for i in range(num_points):
        # Compute the new position on the circle (in 3D)
        x = center[0] + radius * np.cos(theta[i])
        y = center[1] + radius * np.sin(theta[i])
        z = center[
            2
        ]  # The Z-coordinate remains the same (if you want it to move, modify here)

        # Place the position in the first three entries of the 12D vector
        vectors[i, :3] = [x, y, z]

    return vectors.T
