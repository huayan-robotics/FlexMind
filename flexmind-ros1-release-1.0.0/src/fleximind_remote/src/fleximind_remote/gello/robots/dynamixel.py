from typing import Dict, Optional, Sequence, Tuple, List

import numpy as np

from fleximind_remote.gello.robots.robot import Robot


class DynamixelRobot(Robot):
    """A class representing a UR robot."""

    def __init__(
        self,
        joint_ids: Sequence[int],
        joint_offsets: Optional[Sequence[float]] = None,
        joint_signs: Optional[Sequence[int]] = None,
        real: bool = False,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 57600,
        gripper_config: Optional[Tuple[int, float, float]] = None,
        gripper_angle: Optional[int] = None,
        start_joints: Optional[np.ndarray] = None,
        calibrate_joints: Optional[np.ndarray] = None,
    ):
        from fleximind_remote.gello.dynamixel.driver import (
            DynamixelDriver,
            DynamixelDriverProtocol,
            FakeDynamixelDriver,
        )

        print(f"attempting to connect to port: {port}")
        self.gripper_open_close: Optional[Tuple[float, float]]
        if gripper_config is not None:
            assert joint_offsets is not None
            assert joint_signs is not None

            # joint_ids.append(gripper_config[0])
            # joint_offsets.append(0.0)
            # joint_signs.append(1)
            joint_ids = tuple(joint_ids) + (gripper_config[0],)
            joint_offsets = tuple(joint_offsets) + (0.0,)
            joint_signs = tuple(joint_signs) + (1,)
            self.gripper_open_close = (
                gripper_config[1] * np.pi / 180,
                gripper_config[2] * np.pi / 180,
            )
            self.gripper_angle = gripper_angle
        else:
            self.gripper_open_close = None
            self.gripper_angle = 50

        self._joint_ids = joint_ids
        self._driver: DynamixelDriverProtocol

        if joint_offsets is None:
            self._joint_offsets = np.zeros(len(joint_ids))
        else:
            self._joint_offsets = np.array(joint_offsets)

        if joint_signs is None:
            self._joint_signs = np.ones(len(joint_ids))
        else:
            self._joint_signs = np.array(joint_signs)

        assert len(self._joint_ids) == len(self._joint_offsets), (
            f"joint_ids: {len(self._joint_ids)}, "
            f"joint_offsets: {len(self._joint_offsets)}"
        )
        assert len(self._joint_ids) == len(self._joint_signs), (
            f"joint_ids: {len(self._joint_ids)}, "
            f"joint_signs: {len(self._joint_signs)}"
        )
        assert np.all(
            np.abs(self._joint_signs) == 1
        ), f"joint_signs: {self._joint_signs}"

        if real:
            self._driver = DynamixelDriver(joint_ids, port=port, baudrate=baudrate)
            self._driver.set_torque_mode(False)
        else:
            self._driver = FakeDynamixelDriver(joint_ids)
        self._torque_on = False
        self._last_pos = None
        self._alpha = 0.99

        self._setup_offset(start_joints)
        self._calibrate_joints = calibrate_joints

    def _setup_offset(self, start_joints=None):
        self._start_joints = start_joints

        if start_joints is not None:
            # loop through all joints and add +- 2pi to the joint offsets to get the closest to start joints
            new_joint_offsets = []
            current_joints = self.get_joint_state()
            assert current_joints.shape == start_joints.shape
            if gripper_config is not None:
                current_joints = current_joints[:-1]
                start_joints = start_joints[:-1]
            for idx, (c_joint, s_joint, joint_offset) in enumerate(
                zip(current_joints, start_joints, self._joint_offsets)
            ):
                new_joint_offsets.append(
                    np.pi
                    * 2
                    * np.round((-s_joint + c_joint) / (2 * np.pi))
                    * self._joint_signs[idx]
                    + joint_offset
                )
            if gripper_config is not None:
                new_joint_offsets.append(self._joint_offsets[-1])
            self._joint_offsets = np.array(new_joint_offsets)

    def num_dofs(self) -> int:
        return len(self._joint_ids)

    def get_joint_state(self) -> np.ndarray:
        pos = (self._driver.get_joints() - self._joint_offsets) * self._joint_signs
        assert len(pos) == self.num_dofs()

        if self.gripper_open_close is not None:
            # map pos to [0, 1]
            g_pos = (pos[-1] - self.gripper_open_close[0]) / (
                self.gripper_open_close[1] - self.gripper_open_close[0]
            )
            g_pos = min(max(0, g_pos), 1)
            pos[-1] = g_pos

        if self._last_pos is None:
            self._last_pos = pos
        else:
            # exponential smoothing
            pos = self._last_pos * (1 - self._alpha) + pos * self._alpha
            self._last_pos = pos

        return pos

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        self._driver.set_joints((joint_state + self._joint_offsets).tolist())

    def set_torque_mode(self, mode: bool):
        if mode == self._torque_on:
            return
        self._driver.set_torque_mode(mode)
        self._torque_on = mode

    def get_observations(self) -> Dict[str, np.ndarray]:
        return {"joint_state": self.get_joint_state()}

    def _calculate_error(
        self, offset: float, index: int, joint_state: np.ndarray
    ) -> float:
        """Calculate error between current joint state and target state with offset."""
        joint_sign_i = self._joint_signs[index]
        joint_i = joint_sign_i * (joint_state[index] - offset)
        start_i = self._calibrate_joints[index]
        return np.abs(joint_i - start_i)

    def calculate_offsets(
        self, num_iterations: int = 1, reload: bool = False
    ) -> Optional[List[float]]:
        """
        Calculate optimal joint offsets to align with start positions.

        Args:
            num_iterations: Number of iterations to refine offset calculation

        Returns:
            List of optimal offsets or None if calculation failed
        """
        if self._driver is None:
            print("Driver not connected. Call connect() first.")
            return None

        best_offsets = []

        for iteration in range(num_iterations):
            try:
                curr_joints = self._driver.get_joints()
                iteration_offsets = []

                for i in range(len(self._joint_ids) - 1):
                    best_offset = 0
                    best_error = float("inf")

                    # Search for optimal offset in range [-8π, 8π] with π/2 intervals
                    for offset in np.linspace(-8 * np.pi, 8 * np.pi, 8 * 4 + 1):
                        error = self._calculate_error(offset, i, curr_joints)
                        if error < best_error:
                            best_error = error
                            best_offset = offset

                    iteration_offsets.append(best_offset)

                best_offsets = iteration_offsets

            except Exception as e:
                print(f"Error during offset calculation: {e}")
                return None

        if reload:
            self._joint_offsets[:6] = best_offsets

        return best_offsets

    def get_gripper_calibration(self, reload=False) -> Optional[Tuple[float, float]]:
        """Get gripper open and close positions in degrees."""
        if self._driver is None:
            return None

        try:
            gripper_position = self._driver.get_joints()[-1]
            open_position = np.rad2deg(gripper_position) - 0.2
            close_position = np.rad2deg(gripper_position) - self.gripper_angle
            if reload:
                self.gripper_open_close = [open_position  * np.pi / 180 , close_position * np.pi / 180]

            return [open_position, close_position]
        except Exception as e:
            print(f"Error getting gripper calibration: {e}")
            return None
