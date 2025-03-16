import math

import choreo
import wpilib
from wpilib import Field2d, RobotBase, SmartDashboard
from typing import List
from choreo.trajectory import SwerveSample, SwerveTrajectory
from magicbot import AutonomousStateMachine, state, timed_state, will_reset_to
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds
from magicbot import feedback

from wpiutil.log import DataLog
from wpilib import DataLogManager

from components.arm_control import ArmControl
from components.swerve_drive import SwerveDrive
from components.drive_control import DriveControl
from components.odometry import Odometry
from components.claw import Claw, ClawAngle
from components.elevator import ElevatorHeight, Elevator
from lemonlib.util import AlertManager

from wpiutil.log import (
     DataLog,
     BooleanLogEntry,
     DoubleLogEntry,
     StringLogEntry,
     IntegerLogEntry,
 )


class AutoBase(AutonomousStateMachine):
    swerve_drive: SwerveDrive
    drive_control: DriveControl
    arm_control: ArmControl
    odometry: Odometry
    claw: Claw
    estimated_field: Field2d
    elevator: Elevator

    drive_control_state_log: StringLogEntry
    arm_control_state_log: StringLogEntry
    claw_state_log: StringLogEntry
    elevator_state_log: StringLogEntry
    elevator_height_log: DoubleLogEntry
    claw_angle_log: DoubleLogEntry
    claw_intake_limit_log: BooleanLogEntry

    auto_state_log: StringLogEntry
    auto_step_log: IntegerLogEntry
    auto_trajectory_log: StringLogEntry
    auto_pose_log: StringLogEntry
    auto_distance_log: DoubleLogEntry
    auto_target_log: StringLogEntry

    alliance_log: StringLogEntry

    DISTANCE_TOLERANCE = 0.05  # metres
    ANGLE_TOLERANCE = math.radians(3)
    TRANSLATIONAL_SPEED_TOLERANCE = 0.2
    ROTATIONAL_SPEED_TOLERANCE = 0.1

    def __init__(self, sequence: List[str]) -> None:
        super().__init__()

        self.sequence = sequence  # List of trajectories and states
        self.current_step = -1
        self.trajectories: list[SwerveTrajectory] = []
        self.current_trajectory: SwerveTrajectory | None = None
        self.starting_pose = None
        SmartDashboard.putNumber("Distance", 0)

        self.alliance_log.append(wpilib.DriverStation.getAlliance())

        # Load trajectories (skip non-trajectory steps)
        for item in self.sequence:
            if not item.startswith("state:"):  # Only load actual trajectories
                try:
                    self.trajectories.append(choreo.load_swerve_trajectory(item))
                    if self.starting_pose is None:
                        self.starting_pose = self.get_starting_pose()
                except ValueError:
                    pass  # Ignore missing trajectories

    def on_enable(self) -> None:
        self.current_step = -1
        starting_pose = self.get_starting_pose()
        if RobotBase.isSimulation() and starting_pose is not None:
            self.swerve_drive.set_pose(starting_pose)

        super().on_enable()

    def _get_full_path_poses(self) -> list[Pose2d]:
        """Get a list of poses for the full path for display."""
        return [
            sample.get_pose()
            for trajectory in self.trajectories
            for sample in trajectory.get_samples()
        ]

    def display_trajectory(self) -> None:
        self.estimated_field.getObject("Trajectory").setPoses(
            self._get_full_path_poses()
        )

    @feedback
    def is_red(self) -> bool:
        return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed

    def get_starting_pose(self) -> Pose2d | None:
        return self.trajectories[0].get_initial_pose(self.is_red())

    @state(first=True)
    def next_step(self):
        """Moves to the next step in the sequence, determining if it's a trajectory or a state."""

        self.auto_state_log.append("next_step")
        self.auto_step_log.append(self.current_step)

        self.current_step += 1
        if self.current_step >= len(self.sequence):
            self.done()
            return

        step = self.sequence[self.current_step]

        self.auto_state_log(step)
        self.auto_step_log(self.current_step)

        if step.startswith("state:"):

            self.next_state(step.split("state:")[1])  # Go to the specified state
        else:
            self.current_trajectory = self.trajectories[self.current_step]
            if self.current_trajectory:
                self.next_state("tracking_trajectory")
            else:
                self.next_step()  # Skip invalid trajectory names

    @state
    def tracking_trajectory(self, state_tm):
        """Follows the current trajectory and transitions when done."""
        if not self.current_trajectory:
            self.next_state("next_step")
            return

        current_pose = self.swerve_drive.get_estimated_pose()
        final_pose = self.current_trajectory.get_final_pose(self.is_red())
        distance = current_pose.translation().distance(final_pose.translation())

        self.auto_trajectory_log(self.current_trajectory.name)
        self.auto_target_log(f"X: {final_pose.translation().x()}, Y: {final_pose.translation().y()}, Rot: {final_pose.rotation().degrees()}")
        self.auto_pose_log(current_pose)
        self.auto_distance_log(distance)

        if (
            distance < self.DISTANCE_TOLERANCE
            and state_tm > self.current_trajectory.get_total_time() / 2.0
        ):
            self.next_state("next_step")

        sample = self.current_trajectory.sample_at(state_tm, self.is_red())
        if sample:
            self.swerve_drive.follow_trajectory(sample)

            SmartDashboard.putNumber("Distance", distance)
            if distance < self.DISTANCE_TOLERANCE:
                self.next_state("next_step")

    """
    STATES
    """

    @state
    def intaking_coral(self) -> None:
        self.arm_control.engage()
        self.arm_control.set(ElevatorHeight.L1, ClawAngle.STATION)

        self.elevator_height_log(self.elevator.get_height())
        self.claw_angle_log(self.claw.get_angle())

        if self.arm_control.at_setpoint():

            self.arm_control.set_wheel_voltage(-1)
        if self.claw.get_intake_limit():
            self.next_state("next_step")

    @state
    def level_one(self) -> None:
        self.arm_control.engage()
        self.arm_control.set(ElevatorHeight.L1, ClawAngle.TROUGH)

        self.elevator_height_log(self.elevator.get_height())
        self.claw_angle_log(self.claw.get_angle())

        if self.arm_control.at_setpoint():
            self.arm_control.set_wheel_voltage(1)
        if not self.claw.get_intake_limit():
            self.next_state("next_step")

    @state
    def level_two(self) -> None:
        self.arm_control.engage()
        self.arm_control.set(ElevatorHeight.L2, ClawAngle.BRANCH)

        self.elevator_height_log(self.elevator.get_height())
        self.claw_angle_log(self.claw.get_angle())

        if self.arm_control.at_setpoint():
            self.arm_control.set_wheel_voltage(1)
        if not self.claw.get_intake_limit():
            self.next_state("next_step")

    @state
    def level_three(self) -> None:
        self.arm_control.engage()
        self.arm_control.set(ElevatorHeight.L3, ClawAngle.BRANCH)

        self.elevator_height_log(self.elevator.get_height())
        self.claw_angle_log(self.claw.get_angle())

        if self.arm_control.at_setpoint():
            self.arm_control.set_wheel_voltage(1)
        if not self.claw.get_intake_limit():
            self.next_state("next_step")

    @timed_state(duration=2, next_state="next_step")
    def level_four(self) -> None:
        self.arm_control.engage()
        self.arm_control.set(ElevatorHeight.L4, ClawAngle.BRANCH)

        self.elevator_height_log(self.elevator.get_height())
        self.claw_angle_log(self.claw.get_angle())

        if self.arm_control.at_setpoint():
            self.arm_control.engage()
            self.arm_control.set_wheel_voltage(-10)






