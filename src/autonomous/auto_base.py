import math

import choreo
import wpilib
from choreo.trajectory import SwerveSample
from magicbot import AutonomousStateMachine, state, timed_state
from wpilib import RobotBase
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds

from components.arm_control import ArmControl
from components.swerve_drive import SwerveDrive
from components.drive_control import DriveControl
from components.odometry import Odometry
from components.claw import Claw,ClawAngle
from components.elevator import Elevator,ElevatorHeight


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
    is_red: bool = wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed

    DISTANCE_TOLERANCE = 0.2  # metres
    ANGLE_TOLERANCE = math.radians(3)

    def __init__(self, trajectory_names: list[str]) -> None:
        super().__init__()

        self.current_leg = -1
        self.starting_pose = None
        self.trajectories = []
        for trajectory_name in trajectory_names:
            try:
                self.trajectories.append(choreo.load_swerve_trajectory(trajectory_name))
                if self.starting_pose is None:
                    self.starting_pose = self.get_starting_pose()
            except ValueError:
                pass

    def setup(self) -> None:
        pass

    def on_enable(self) -> None:
        starting_pose = self.get_starting_pose()
        if RobotBase.isSimulation() and starting_pose is not None:
            self.odometry.set_pose(starting_pose)
        self.current_leg = -1

        super().on_enable()

    def get_starting_pose(self) -> Pose2d | None:
        return self.trajectories[0].get_initial_pose(self.is_red)

    @state(first=True)
    def initialising(self) -> None:
        self.next_state("tracking_trajectory")

    @state
    def tracking_trajectory(self, initial_call, state_tm) -> None:
        if initial_call:
            self.current_leg += 1

        if self.current_leg >= len(self.trajectories):
            self.done()
            return

        final_pose = self.trajectories[self.current_trajectory_index].get_final_pose(self.is_red)
        if final_pose is None:
            self.done()
            return

        sample = self.trajectories[self.current_trajectory_index].sample_at(state_tm, self.is_red)
        if sample is not None:
            self.follow_trajectory(sample)

    def follow_trajectory(self, sample: SwerveSample):
        pose = self.odometry.get_pose()

        speeds = ChassisSpeeds(
            sample.vx + x_controller.calculate(pose.X(), sample.x),
            sample.vy + y_controller.calculate(pose.Y(), sample.y),
            sample.omega
        )

        self.drive_control.drive_auto(speeds.vx, speeds.vy, speeds.omega)

    @timed_state(duration=2)
    def intaking_coral(self) -> None:
        if self.current_leg == 0:
            self.arm_control.set(ElevatorHeight.L1,ClawAngle.STATION)
            if self.arm_control.at_setpoint():
                self.arm_control.set_wheel_voltage(-1)
        if self.claw.get_intake_limit():
            self.next_state("tracking_trajectory")

    @timed_state(duration=2)
    def level_one(self) -> None:
        self.arm_control.set(ElevatorHeight.L1,ClawAngle.TROUGH)
        if self.arm_control.at_setpoint():
            self.arm_control.set_wheel_voltage(1)
        if not self.claw.get_intake_limit():
            self.next_state("tracking_trajectory")
    @timed_state(duration=2)
    def level_two(self) -> None:
        self.arm_control.set(ElevatorHeight.L2,ClawAngle.BRANCH)
        if self.arm_control.at_setpoint():
            self.arm_control.set_wheel_voltage(1)
        if not self.claw.get_intake_limit():
            self.next_state("tracking_trajectory")
    @timed_state(duration=2)
    def level_three(self) -> None:
        self.arm_control.set(ElevatorHeight.L3,ClawAngle.BRANCH)
        if self.arm_control.at_setpoint():
            self.arm_control.set_wheel_voltage(1)
        if not self.claw.get_intake_limit():
            self.next_state("tracking_trajectory")
    @timed_state(duration=2)
    def level_four(self) -> None:
        self.arm_control.set(ElevatorHeight.L4,ClawAngle.BRANCH)
        if self.arm_control.at_setpoint():
            self.arm_control.set_wheel_voltage(1)
        if not self.claw.get_intake_limit():
            self.next_state("tracking_trajectory")
    
