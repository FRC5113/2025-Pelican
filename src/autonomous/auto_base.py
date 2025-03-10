import math

import choreo
import wpilib
from wpilib import RobotBase,Field2d,SmartDashboard
from choreo.trajectory import SwerveSample,SwerveTrajectory
from magicbot import AutonomousStateMachine, state, timed_state,will_reset_to
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds

from components.arm_control import ArmControl
from components.swerve_drive import SwerveDrive
from components.drive_control import DriveControl
from components.odometry import Odometry
from components.claw import Claw, ClawAngle
from components.elevator import ElevatorHeight
from lemonlib.util import Alert, AlertType,AlertManager


class AutoBase(AutonomousStateMachine):
    swerve_drive: SwerveDrive
    drive_control: DriveControl
    arm_control: ArmControl
    odometry: Odometry
    claw: Claw
    estimated_field: Field2d

    requestl1 = will_reset_to(False)
    requestl2 = will_reset_to(False)
    requestl3 = will_reset_to(False)
    requestl4 = will_reset_to(False)
    requestintake = will_reset_to(False)


    DISTANCE_TOLERANCE = 0.1  # metres
    ANGLE_TOLERANCE = math.radians(3)
    CORAL_DISTANCE_TOLERANCE = 0.2  # metres
    TRANSLATIONAL_SPEED_TOLERANCE = 0.2
    ROTATIONAL_SPEED_TOLERANCE = 0.1

    def __init__(self, trajectory_names: list[str]) -> None:
        super().__init__()

        self.current_leg = -1
        self.starting_pose = None
        self.trajectories: list[SwerveTrajectory] = []
        for trajectory_name in trajectory_names:
            try:
                self.trajectories.append(choreo.load_swerve_trajectory(trajectory_name))
                if self.starting_pose is None:
                    self.starting_pose = self.get_starting_pose()
            except ValueError:
                # If the trajectory is not found, ChoreoLib already prints to DriverStation
                pass

    def is_red(self) -> bool:
        return wpilib.DriverStation.getAlliance() != wpilib.DriverStation.Alliance.kRed
    
    def request_level_one(self):
        self.requestl1 = True
    def request_level_two(self):
        self.requestl2 = True
    def request_level_three(self):
        self.requestl3 = True
    def request_level_four(self):
        self.requestl4 = True 
    def request_intake(self):
        self.requestintake = True 

    def on_enable(self) -> None:
        starting_pose = self.get_starting_pose()
        if RobotBase.isSimulation() and starting_pose is not None:
            self.swerve_drive.set_pose(starting_pose)
        self.current_leg = -1

        super().on_enable()
        self.engage()

    def get_starting_pose(self) -> Pose2d | None:
        return self.trajectories[0].get_initial_pose(self.is_red)
    
    def _get_full_path_poses(self) -> list[Pose2d]:
        """Get a list of poses for the full path for display."""
        return [
            sample.get_pose()
            for trajectory in self.trajectories
            for sample in trajectory.get_samples()
        ]

    def display_trajectory(self) -> None:
        self.estimated_field.getObject("trajectory").setPoses(self._get_full_path_poses())

    @state(first=True)
    def initialising(self) -> None:
        self.next_state("tracking_trajectory")

    @state
    def tracking_trajectory(self, initial_call, state_tm) -> None:
        SmartDashboard.putNumber("current_leg", self.current_leg)
        if initial_call:
            self.current_leg += 1

        if self.current_leg == len(self.trajectories):
            self.done()
            return

        current_pose = self.swerve_drive.get_estimated_pose()
        final_pose = self.trajectories[self.current_leg].get_final_pose( self.is_red)
        if final_pose is None:
            self.done()
            return
        
        distance = current_pose.translation().distance(final_pose.translation())
        angle_error = (final_pose.rotation() - current_pose.rotation()).radians()
        velocity = self.swerve_drive.get_velocity()
        speed = math.sqrt(math.pow(velocity.vx, 2.0) + math.pow(velocity.vy, 2.0))
        
        if (
            distance < self.DISTANCE_TOLERANCE
            and math.isclose(angle_error, 0.0, abs_tol=self.ANGLE_TOLERANCE)
            and math.isclose(speed, 0.0, abs_tol=self.TRANSLATIONAL_SPEED_TOLERANCE)
            and math.isclose(
                velocity.omega, 0.0, abs_tol=self.ROTATIONAL_SPEED_TOLERANCE
            )
            and state_tm > self.trajectories[self.current_leg].get_total_time() / 2.0
        ):
            if self.request_intake():
                self.next_state("intaking_coral")
            elif self.request_level_four():
                self.next_state("level_four")
            elif self.request_level_three():
                self.next_state("level_three")
            elif self.request_level_two():
                self.next_state("level_two")
            elif self.request_level_one():
                self.next_state("level_one")

        sample = self.trajectories[self.current_leg].sample_at(state_tm,  self.is_red)
        if sample is not None:
            self.follow_trajectory(sample)

    def follow_trajectory(self, sample: SwerveSample):
        speeds = self.swerve_drive.follow_trajectory(sample)
        self.drive_control.drive_auto(speeds.vx, speeds.vy, speeds.omega)
    
    @state
    def temp1(self) -> None:
        self.next_state("tracking_trajectory")
    @state
    def temp2(self) -> None:
        self.next_state("tracking_trajectory")

    @state
    def intaking_coral(self) -> None:
        self.arm_control.set(ElevatorHeight.L1, ClawAngle.STATION)
        if self.arm_control.at_setpoint():
            self.arm_control.set_wheel_voltage(-1)
        if self.claw.get_intake_limit():
            self.next_state("tracking_trajectory")

    @state
    def level_one(self) -> None:
        self.arm_control.set(ElevatorHeight.L1, ClawAngle.TROUGH)
        if self.arm_control.at_setpoint():
            self.arm_control.set_wheel_voltage(1)
        if not self.claw.get_intake_limit():
            self.next_state("tracking_trajectory")

    @state
    def level_two(self) -> None:
        self.arm_control.set(ElevatorHeight.L2, ClawAngle.BRANCH)
        if self.arm_control.at_setpoint():
            self.arm_control.set_wheel_voltage(1)
        if not self.claw.get_intake_limit():
            self.next_state("tracking_trajectory")

    @state
    def level_three(self) -> None:
        self.arm_control.set(ElevatorHeight.L3, ClawAngle.BRANCH)
        if self.arm_control.at_setpoint():
            self.arm_control.set_wheel_voltage(1)
        if not self.claw.get_intake_limit():
            self.next_state("tracking_trajectory")

    @state
    def level_four(self) -> None:
        self.arm_control.set(ElevatorHeight.L4, ClawAngle.BRANCH)
        if self.arm_control.at_setpoint():
            self.arm_control.set_wheel_voltage(1)
        if not self.claw.get_intake_limit():
            self.next_state("tracking_trajectory")
