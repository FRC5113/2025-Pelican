import math

import choreo
import choreo.util
import wpilib
from wpilib import Field2d, RobotBase, SmartDashboard, DataLogManager, DriverStation
from typing import List
from choreo.trajectory import SwerveSample, SwerveTrajectory
from magicbot import AutonomousStateMachine, state, timed_state, will_reset_to
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds
from magicbot import feedback

from components.arm_control import ArmControl
from components.swerve_drive import SwerveDrive
from components.drive_control import DriveControl

# from components.odometry import Odometry
from components.claw import Claw, ClawAngle
from components.elevator import ElevatorHeight, Elevator
from lemonlib.util import AlertManager, is_red


class AutoBase(AutonomousStateMachine):
    swerve_drive: SwerveDrive
    drive_control: DriveControl
    arm_control: ArmControl
    # odometry: Odometry
    claw: Claw
    estimated_field: Field2d
    elevator: Elevator

    DISTANCE_TOLERANCE = 0.1  # metres
    ANGLE_TOLERANCE = math.radians(3)
    TRANSLATIONAL_SPEED_TOLERANCE = 0.2
    ROTATIONAL_SPEED_TOLERANCE = 0.1

    def __init__(self, sequence: List[str]) -> None:
        super().__init__()

        self.sequence = sequence  # List of trajectories and states
        self.current_step = -1
        self.trajectory_index = -1
        self.trajectories: list[SwerveTrajectory] = []
        self.starting_pose = None
        SmartDashboard.putNumber("Distance", 0)
        SmartDashboard.putString("Final Pose", "none")
        # Load trajectories (skip non-trajectory steps)
        for item in self.sequence:
            if not item.startswith("state:"):  # Only load actual trajectories
                try:
                    self.trajectories.append(choreo.load_swerve_trajectory(item))
                    if self.starting_pose is None and RobotBase.isSimulation():
                        self.starting_pose = self.get_starting_pose()
                except ValueError:
                    pass  # Ignore missing trajectories

    def on_enable(self) -> None:
        starting_pose = self.get_starting_pose()
        if starting_pose is not None and RobotBase.isSimulation():
            self.swerve_drive.set_starting_pose(starting_pose)
        self.current_step = -1
        self.trajectory_index = -1

        super().on_enable()

    def _get_full_path_poses(self) -> list[Pose2d]:
        """Get a list of poses for the full path for display."""
        return [
            sample
            for trajectory in self.trajectories
            for sample in trajectory.get_poses()
        ]

    def get_starting_pose(self) -> Pose2d | None:
        if self.trajectories[0].get_initial_pose(is_red()) is not None:
            return self.trajectories[0].get_initial_pose(is_red())
        else:
            return Pose2d()

    def display_trajectory(self) -> None:
        self.estimated_field.getObject("trajectory").setPoses(
            self._get_full_path_poses()
        )

    def on_disable(self) -> None:
        super().on_disable()
        self.estimated_field.getObject("trajectory").setPoses([])

    @state(first=True)
    def next_step(self):
        """Moves to the next step in the sequence, determining if it's a trajectory or a state."""
        self.current_step += 1
        if self.current_step >= len(self.sequence):
            self.done()
            return

        step = self.sequence[self.current_step]
        if step.startswith("state:"):

            self.next_state(step.split("state:")[1])  # Go to the specified state
        else:
            self.trajectory_index += 1
            self.current_trajectory = self.trajectories[self.trajectory_index]
            if self.current_trajectory:
                self.next_state("tracking_trajectory")
            else:
                self.next_step()  # Skip invalid trajectory names

    @state
    def tracking_trajectory(self, state_tm):
        """Follows the current trajectory and transitions when done."""

        current_pose = self.swerve_drive.get_estimated_pose()
        final_pose = self.current_trajectory.get_final_pose(is_red())
        distance = current_pose.translation().distance(final_pose.translation())
        angle_error = (final_pose.rotation() - current_pose.rotation()).radians()
        velocity = self.swerve_drive.get_velocity()
        speed = math.sqrt(math.pow(velocity.vx, 2.0) + math.pow(velocity.vy, 2.0))
        SmartDashboard.putString("Final Pose", f"{final_pose}")

        if (
            distance < self.DISTANCE_TOLERANCE
            and math.isclose(angle_error, 0.0, abs_tol=self.ANGLE_TOLERANCE)
            and math.isclose(speed, 0.0, abs_tol=self.TRANSLATIONAL_SPEED_TOLERANCE)
            and math.isclose(
                velocity.omega, 0.0, abs_tol=self.ROTATIONAL_SPEED_TOLERANCE
            )
            and state_tm > self.current_trajectory.get_total_time() / 2.0
        ):
            self.next_state("next_step")
        sample = self.current_trajectory.sample_at(state_tm, is_red())
        if sample is not None:
            self.drive_control.drive_auto(sample)

            SmartDashboard.putNumber("Distance", distance)

    """
    STATES
    """

    @state
    def intaking_coral(self) -> None:
        self.arm_control.set(ElevatorHeight.L1, ClawAngle.STATION_CLOSE)
        if self.arm_control.at_point(ElevatorHeight.L1, ClawAngle.STATION_CLOSE):
            self.arm_control.set_wheel_voltage(-1)
        if self.claw.get_intake_limit():
            self.next_state("next_step")

    @state
    def level_one(self) -> None:
        self.arm_control.set(ElevatorHeight.L1, ClawAngle.TROUGH)
        if self.arm_control.at_point(ElevatorHeight.L1, ClawAngle.TROUGH):
            self.arm_control.set_wheel_voltage(1)
        if not self.claw.get_intake_limit():
            self.next_state("next_step")

    @state
    def level_two(self) -> None:
        self.arm_control.set(ElevatorHeight.L2, ClawAngle.BRANCH)
        if self.arm_control.at_point(ElevatorHeight.L2, ClawAngle.BRANCH):
            self.arm_control.set_wheel_voltage(1)
        if not self.claw.get_intake_limit():
            self.next_state("next_step")

    @state
    def level_three(self) -> None:
        self.arm_control.set(ElevatorHeight.L3, ClawAngle.BRANCH)
        if self.arm_control.at_point(ElevatorHeight.L3, ClawAngle.BRANCH):
            self.arm_control.set_wheel_voltage(1)
        if not self.claw.get_intake_limit():
            self.next_state("next_step")

    @timed_state(duration=2.8, next_state="next_step")
    def level_four(self) -> None:
        self.arm_control.set(ElevatorHeight.L4, ClawAngle.BRANCH)
        if self.arm_control.at_point(ElevatorHeight.L4, ClawAngle.BRANCH):
            self.arm_control.set_wheel_voltage(-10)
