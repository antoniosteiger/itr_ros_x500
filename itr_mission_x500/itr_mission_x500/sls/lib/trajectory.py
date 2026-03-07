import numpy as np
from scipy.interpolate import CubicSpline


def horizon_augment_traj(traj, horizon):
    Tsim = traj.shape[1]
    if traj is None:
        raise Exception
    else:
        traj_time_aug = np.concatenate(
            [traj, np.tile(traj[:, [-1]], (1, horizon))],
            axis=1
        )
        
    return traj_time_aug

def generate_circle_trajectory(
    radius=5.0,
    period=10.0,
    altitude=2.0,
    Tsim=100,
    dt=0.1,
    nx_total=12,
    pos_indices=[0, 1, 2],
    vel_indices=[3, 4, 5],
    start_pos=(0.0, 0.0, 0.0),  # Desired NED start position (Down is positive-down)
):
    """
    Circle trajectory in the horizontal plane.

    The raw circle at t=0 sits at (radius, 0, -altitude) in NED.
    We subtract that offset and add start_pos so the trajectory
    begins exactly at start_pos.

    Args:
        start_pos: (x, y, z) in NED frame. Default (0,0,0) starts at origin.
                   z is NED Down, so positive z = below origin.
    """
    r_traj = np.zeros((nx_total, Tsim))
    omega = 2 * np.pi / period

    # Value of the raw trajectory at t=0
    raw_start = np.array([radius, 0.0, -altitude])
    # Desired start in NED
    desired_start = np.array(start_pos)
    offset = desired_start - raw_start

    for k in range(Tsim):
        t = k * dt
        theta = omega * t

        r_traj[pos_indices[0], k] = radius * np.cos(theta) + offset[0]
        r_traj[pos_indices[1], k] = radius * np.sin(theta) + offset[1]
        r_traj[pos_indices[2], k] = -altitude              + offset[2]

        r_traj[vel_indices[0], k] = -radius * omega * np.sin(theta)
        r_traj[vel_indices[1], k] =  radius * omega * np.cos(theta)
        r_traj[vel_indices[2], k] = 0.0

    return r_traj


def generate_figure8_trajectory(
    width=6.0,
    height=3.0,
    altitude=2.0,
    period=15.0,
    Tsim=100,
    dt=0.1,
    nx_total=12,
    pos_indices=[0, 1, 2],
    vel_indices=[3, 4, 5],
    start_pos=(0.0, 0.0, 0.0),
):
    """
    Lemniscate (figure-8) trajectory.

    The raw figure-8 at t=0 sits at (0, 0, -altitude).
    Only the z-offset needs shifting for the default origin start,
    but start_pos lets you place it anywhere.
    """
    r_traj = np.zeros((nx_total, Tsim))
    omega = 2 * np.pi / period
    a = width / 2
    b = height / 2

    # Raw position at t=0: sin(0)=0, so x=0, y=0, z=-altitude
    raw_start = np.array([0.0, 0.0, -altitude])
    desired_start = np.array(start_pos)
    offset = desired_start - raw_start

    for k in range(Tsim):
        t = k * dt
        theta = omega * t

        r_traj[pos_indices[0], k] = a * np.sin(theta)            + offset[0]
        r_traj[pos_indices[1], k] = b * np.sin(theta) * np.cos(theta) + offset[1]
        r_traj[pos_indices[2], k] = -altitude                    + offset[2]

        r_traj[vel_indices[0], k] = a * omega * np.cos(theta)
        r_traj[vel_indices[1], k] = b * omega * np.cos(2 * theta)
        r_traj[vel_indices[2], k] = 0.0

    return r_traj


def generate_helix_trajectory(
    radius=4.0,
    climb_rate=0.5,
    period=12.0,
    Tsim=100,
    dt=0.1,
    nx_total=12,
    pos_indices=[0, 1, 2],
    vel_indices=[3, 4, 5],
    start_pos=(0.0, 0.0, 0.0),
):
    """
    Helical trajectory (circle + linear climb in z).

    The raw helix at t=0 sits at (radius, 0, 0) in NED.
    climb_rate is in m/s; positive climb_rate moves in the -Down direction
    (i.e., the quad climbs up in altitude).
    """
    r_traj = np.zeros((nx_total, Tsim))
    omega = 2 * np.pi / period

    # Raw position at t=0: (radius, 0, 0)
    raw_start = np.array([radius, 0.0, 0.0])
    desired_start = np.array(start_pos)
    offset = desired_start - raw_start

    for k in range(Tsim):
        t = k * dt

        r_traj[pos_indices[0], k] = radius * np.cos(omega * t) + offset[0]
        r_traj[pos_indices[1], k] = radius * np.sin(omega * t) + offset[1]
        # climb_rate positive → moving up → Down decreases → negative NED z
        r_traj[pos_indices[2], k] = -climb_rate * t            + offset[2]

        r_traj[vel_indices[0], k] = -radius * omega * np.sin(omega * t)
        r_traj[vel_indices[1], k] =  radius * omega * np.cos(omega * t)
        r_traj[vel_indices[2], k] = -climb_rate

    return r_traj


def generate_waypoint_trajectory(
    waypoints,
    times=None,
    Tsim=100,
    dt=0.1,
    nx_total=12,
    pos_indices=[0, 1, 2],
    vel_indices=[3, 4, 5],
    method='cubic',
):
    """
    Waypoint trajectory interpolated with cubic splines or linear interpolation.
    Waypoints are given in NED frame (z positive down).

    Args:
        waypoints:  (N, 3) array of waypoints in NED frame
        times:      Optional timing for each waypoint. If None, evenly spaced over Tsim.
    """
    waypoints = np.array(waypoints, dtype=float)

    if times is None:
        times = np.linspace(0, (Tsim - 1) * dt, waypoints.shape[0])

    times = np.array(times)
    t_sim = np.arange(Tsim) * dt
    r_traj = np.zeros((nx_total, Tsim))

    if method == 'cubic':
        for dim in range(3):
            spline = CubicSpline(times, waypoints[:, dim], bc_type='clamped')
            r_traj[pos_indices[dim], :] = spline(t_sim)
            r_traj[vel_indices[dim], :] = spline.derivative()(t_sim)

    elif method == 'linear':
        for dim in range(3):
            pos = np.interp(t_sim, times, waypoints[:, dim])
            r_traj[pos_indices[dim], :] = pos
            r_traj[vel_indices[dim], :] = np.gradient(pos, dt)

    return r_traj


def generate_minimum_snap_trajectory(
    waypoints,
    times=None,
    Tsim=100,
    dt=0.1,
    nx_total=12,
    pos_indices=[0, 1, 2],
    vel_indices=[3, 4, 5],
    start_pos=(0.0, 0.0, 0.0),
):
    """
    Minimum-snap trajectory through waypoints using quintic polynomials.

    The first waypoint is shifted to start_pos; all remaining waypoints are
    offset by the same delta so the overall shape is preserved.

    Args:
        waypoints:  (N, 3) in ENU/user frame (z = altitude up).
        start_pos:  Desired NED start (x, y, z). Default (0,0,0).
    """
    waypoints = np.array(waypoints, dtype=float)
    n_waypoints = waypoints.shape[0]

    # Shift waypoints so first one matches start_pos
    start_ned = np.array(start_pos)
    start_enu = np.array([start_ned[0], start_ned[1], -start_ned[2]])
    wp_offset = start_enu - waypoints[0]
    waypoints = waypoints + wp_offset

    # Convert to NED
    waypoints_ned = waypoints.copy()
    waypoints_ned[:, 2] = -waypoints_ned[:, 2]

    if times is None:
        times = np.linspace(0, (Tsim - 1) * dt, n_waypoints)

    times = np.array(times)
    t_sim = np.arange(Tsim) * dt
    r_traj = np.zeros((nx_total, Tsim))

    for dim in range(3):
        pos_vals = waypoints_ned[:, dim]

        vel_vals = np.zeros(n_waypoints)
        if n_waypoints > 2:
            vel_vals[1:-1] = (pos_vals[2:] - pos_vals[:-2]) / (times[2:] - times[:-2])

        acc_vals = np.zeros(n_waypoints)
        traj_dim = np.zeros(Tsim)
        traj_vel = np.zeros(Tsim)

        for i in range(n_waypoints - 1):
            t0, t1 = times[i], times[i + 1]
            dt_seg = t1 - t0

            p0, p1 = pos_vals[i], pos_vals[i + 1]
            v0, v1 = vel_vals[i], vel_vals[i + 1]
            a0, a1 = acc_vals[i], acc_vals[i + 1]

            A = np.array([
                [1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 2, 0, 0, 0],
                [1, dt_seg, dt_seg**2, dt_seg**3, dt_seg**4, dt_seg**5],
                [0, 1, 2*dt_seg, 3*dt_seg**2, 4*dt_seg**3, 5*dt_seg**4],
                [0, 0, 2, 6*dt_seg, 12*dt_seg**2, 20*dt_seg**3],
            ])
            b = np.array([p0, v0, a0, p1, v1, a1])
            coeffs = np.linalg.solve(A, b)

            mask = (t_sim >= t0) & (t_sim < t1)
            tau = t_sim[mask] - t0

            traj_dim[mask] = (
                coeffs[0]
                + coeffs[1] * tau
                + coeffs[2] * tau**2
                + coeffs[3] * tau**3
                + coeffs[4] * tau**4
                + coeffs[5] * tau**5
            )
            traj_vel[mask] = (
                coeffs[1]
                + 2 * coeffs[2] * tau
                + 3 * coeffs[3] * tau**2
                + 4 * coeffs[4] * tau**3
                + 5 * coeffs[5] * tau**4
            )

        traj_dim[t_sim >= times[-1]] = pos_vals[-1]

        r_traj[pos_indices[dim], :] = traj_dim
        r_traj[vel_indices[dim], :] = traj_vel

    return r_traj


# ============ EXAMPLE USAGE ============
if __name__ == "__main__":
    import matplotlib.pyplot as plt

    Tsim = 200
    dt = 0.1
    start = (1.0, 2.0, 0.0)  # Change this to test any starting point

    r_circle = generate_circle_trajectory(
        radius=5, period=15, altitude=2, Tsim=Tsim, dt=dt, start_pos=start)

    r_fig8 = generate_figure8_trajectory(
        width=8, height=4, altitude=3, period=20, Tsim=Tsim, dt=dt, start_pos=start)

    r_helix = generate_helix_trajectory(
        radius=3, climb_rate=0.3, period=12, Tsim=Tsim, dt=dt, start_pos=start)

    waypoints = np.array([
        [0, 0, 1],
        [5, 5, 3],
        [10, 0, 4],
        [5, -5, 2],
        [0, 0, 1],
    ])
    r_waypoint = generate_waypoint_trajectory(
        waypoints, Tsim=Tsim, dt=dt, method='cubic', start_pos=start)

    r_minsnap = generate_minimum_snap_trajectory(
        waypoints, Tsim=Tsim, dt=dt, start_pos=start)

    fig = plt.figure(figsize=(15, 10))
    trajectories = [
        (r_circle,   "Circle"),
        (r_fig8,     "Figure-8"),
        (r_helix,    "Helix"),
        (r_waypoint, "Waypoint (Cubic)"),
        (r_minsnap,  "Minimum Snap"),
    ]

    for idx, (traj, name) in enumerate(trajectories):
        ax = fig.add_subplot(2, 3, idx + 1, projection='3d')
        ax.plot(traj[0, :], traj[1, :], -traj[2, :], 'b-', linewidth=2)
        ax.scatter(traj[0, 0],  traj[1, 0],  -traj[2, 0],  c='g', s=100, marker='o', label='Start')
        ax.scatter(traj[0, -1], traj[1, -1], -traj[2, -1], c='r', s=100, marker='x', label='End')
        ax.set_xlabel('North [m]')
        ax.set_ylabel('East [m]')
        ax.set_zlabel('Altitude [m]')
        ax.set_title(name)
        ax.legend()
        ax.grid(True)
        ax.set_box_aspect([1, 1, 0.5])

    plt.suptitle(f"All trajectories starting at NED {start}", fontsize=13)
    plt.tight_layout()
    plt.show()
