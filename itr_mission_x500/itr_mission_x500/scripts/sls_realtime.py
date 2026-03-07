# class SLS_State(ControllerState):
#     def __init__(
#         self,
#         oc_next_state: str,
#         sls: SLS,
#         comms: Comms,
#         traj: npt.NDArray[np.float64] = None,
#         Tsim: int = 50,
#         Tstep: float = 0.1,
#         horizon: int = 10,
#         debug: bool = False,
#     ):
#         super().__init__(oc_next_state, sls, "thrust_and_torque", comms, 1.0 / Tstep, debug)
#         self.comms = comms
#         self.sls = sls
#         self.Tsim = Tsim
#         self.Tstep = Tstep
#         self.step = 0
#         self.debug = debug
#         if traj is None:
#             self.traj = np.zeros((sls.Ad.shape[0], Tsim + horizon))
#         else:
#             traj_time_ext = np.concatenate(
#                 [traj, np.tile(traj[:, [-1]], (1, Tsim + horizon))],
#                 axis=1
#             )
#             
#             target_rows = self.sls.Ad.shape[0]
#             num_repeats = target_rows // traj.shape[0]
#             remainder = target_rows % traj.shape[0]
#             
#             # Repeat each row num_repeats times, then pad with zeros
#             self.traj = np.pad(
#                 np.repeat(traj_time_ext, num_repeats, axis=0),
#                 ((0, remainder), (0, 0)),
#                 mode='constant'
#             )
#
#
#     def get_observation(self):
#         pos = self.comms.get_position()  # From Mocap, needs sign adjustment for z coord
#         pos[1] = -1 * pos[1]
#         pos[2] = -1 * pos[2]
#         vel = self.comms.get_velocity()  # From PX4, does not need sign adjustment
#         # vel[2] = -1 * vel[2]
#         orientation = self.comms.get_orientation_euler()
#         body_rates = self.comms.get_angular_velocity()
#         obs = np.concatenate([pos, vel, orientation, body_rates])
#
#         # pad observation to fit augmentations:
#         obs = np.pad(obs, (0, self.sls.Ad.shape[0] - len(obs)), constant_values=0)
#
#         if self.debug:
#             print(f"Observation: {obs}")
#
#         return obs
#
#     def get_reference(self):
#         H = self.sls.H
#         traj = self.traj
#         step = self.step
#         section = traj[:, step : step + H]
#
#         if self.debug:
#             print(f"Reference: {section}")
#         return section
#
#     def apply_input(self, input):
#         # The thrust the MPC outputs is a delta from hover thrust.
#         # Furthermore, due to the NED frame convention, all upwards thrust is actually negative
#         # thrust = -X500.g * X500.mass + input[0]
#         # thrust_normalized = thrust / abs(
#         #     X500.thrust_max
#         # )  # Normalize the thrust to between -1.0 and 0.0
#         # thrust_bounded = max(
#         #     -1.0, min(thrust_normalized, 0.0)
#         # )  # Make sure it does not exceed the bounds
#         #
#         # torques_normalized = [input[1] / X500.tau_x_max, input[2] / X500.tau_y_max, 0.0]
#         # # torques_normalized = [0.0, 0.0, 0.0]
#         thrust = input[0] / X500.thrust_max
#         torque = [input[1], input[2], input[3]]
#         self.comms.send_torque_setpoint(torque)
#         self.comms.send_thrust_setpoint(thrust)
#
#         if self.debug:
#             print(f"Input Sent: {thrust}, {torque}")
#
#     def is_finished(self):
#         # TODO: Handle trajectory end better. (e.g. padding)
#         if self.step >= len(self.traj[1]):
#             return True
#         else:
#             return False


