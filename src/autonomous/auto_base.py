import math

import choreo
import wpilib
from wpilib import RobotBase,Field2d,SmartDashboard
from choreo.trajectory import SwerveSample,SwerveTrajectory
from magicbot import AutonomousStateMachine, state, timed_state
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


x_controller = PIDController(1.0, 0.0, 0.0)
y_controller = PIDController(1.0, 0.0, 0.0)

wpilib.SmartDashboard.putData("Auto X PID", x_controller)
wpilib.SmartDashboard.putData("Auto Y PID", y_controller)


class AutoBase(AutonomousStateMachine):
    swerve_drive: SwerveDrive
    drive_control: DriveControl
    arm_control: ArmControl
    odometry: Odometry
    claw: Claw
    estimated_field: Field2d


    DISTANCE_TOLERANCE = 0.2  # metres
    ANGLE_TOLERANCE = math.radians(3)
    CORAL_DISTANCE_TOLERANCE = 0.2

    def __init__(self, trajectory_names: list[str]) -> None:
        super().__init__()

        self.current_leg = -1
        self.counter = 0
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

    def setup(self) -> None:
        self.drive_control.engage()
    def is_red(self) -> bool:
        return wpilib.DriverStation.getAlliance() != wpilib.DriverStation.Alliance.kRed

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
        SmartDashboard.putNumber("counter", self.counter)
        if initial_call:
            self.current_leg += 1

        if self.current_leg == len(self.trajectories):
            self.done()
            return

        final_pose = self.trajectories[self.current_leg].get_final_pose( self.is_red)
        if final_pose is None:
            self.done()
            return

        sample = self.trajectories[self.current_leg].sample_at(state_tm,  self.is_red)
        if sample is not None:
            self.follow_trajectory(sample)
            self.counter += 1

        
        if self.counter == 0:
            self.next_state("temp1")
        if self.counter == 1:
            self.next_state("temp2")
        

    def follow_trajectory(self, sample: SwerveSample):
        speeds = self.swerve_drive.follow_trajectory(sample)

        self.drive_control.drive_auto(speeds.vx, speeds.vy, speeds.omega)
    
    @state
    def temp1(self) -> None:
        self.next_state("tracking_trajectory")
    @state
    def temp2(self) -> None:
        self.counter == 0
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
        self.counter == 0
        self.arm_control.set(ElevatorHeight.L1, ClawAngle.TROUGH)
        if self.arm_control.at_setpoint():
            self.arm_control.set_wheel_voltage(1)
        if not self.claw.get_intake_limit():
            self.next_state("tracking_trajectory")

    @state
    def level_two(self) -> None:
        self.counter == 0
        self.arm_control.set(ElevatorHeight.L2, ClawAngle.BRANCH)
        if self.arm_control.at_setpoint():
            self.arm_control.set_wheel_voltage(1)
        if not self.claw.get_intake_limit():
            self.next_state("tracking_trajectory")

    @state
    def level_three(self) -> None:
        self.counter == 0
        self.arm_control.set(ElevatorHeight.L3, ClawAngle.BRANCH)
        if self.arm_control.at_setpoint():
            self.arm_control.set_wheel_voltage(1)
        if not self.claw.get_intake_limit():
            self.next_state("tracking_trajectory")

    @state
    def level_four(self) -> None:
        self.counter == 0
        self.arm_control.set(ElevatorHeight.L4, ClawAngle.BRANCH)
        if self.arm_control.at_setpoint():
            self.arm_control.set_wheel_voltage(1)
        if not self.claw.get_intake_limit():
            self.next_state("tracking_trajectory")
