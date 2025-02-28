import math

import choreo
import wpilib
from choreo.trajectory import SwerveSample
from magicbot import AutonomousStateMachine, state, timed_state
from wpilib import RobotBase
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds

from components.swerve_drive import SwerveDrive
from components.arm_control import ArmControl
from components.claw import ClawAngle, Claw
from components.elevator import ElevatorHeight

x_controller = PIDController(0.5, 0.0, 0.0)
y_controller = PIDController(0.5, 0.0, 0.0)

wpilib.SmartDashboard.putData("Auto X PID", x_controller)
wpilib.SmartDashboard.putData("Auto Y PID", y_controller)


class AutoBase(AutonomousStateMachine):
    swerve_drive: SwerveDrive
    arm_control: ArmControl
    claw: Claw

    DISTANCE_TOLERANCE = 0.2  # metres
    ANGLE_TOLERANCE = math.radians(3)

    def __init__(self, trajectory_names: list[str]) -> None:
        super().__init__()
        self.current_leg = -1
        self.starting_pose = None
        self.trajectories = []
        for trajectory_name in trajectory_names:
            try:
                traj = choreo.load_swerve_trajectory(trajectory_name)
                self.trajectories.append(traj)
                if self.starting_pose is None:
                    self.starting_pose = traj.get_initial_pose(
                        wpilib.DriverStation.getAlliance()
                        == wpilib.DriverStation.Alliance.kRed
                    )
            except ValueError:
                # If trajectory not found, ChoreoLib already prints to DriverStation
                pass

    def setup(self) -> None:
        # Optionally, set up any defaults for your PID controllers here.
        pass

    def on_enable(self) -> None:
        # (Optional) If your swerve drive supports pose setting in simulation,
        # you can set the starting pose here.
        if RobotBase.isSimulation() and self.starting_pose is not None:
            # Assumes your SwerveDrive provides a set_pose() method.
            self.swerve_drive.set_pose(self.starting_pose)
        self.current_leg = -1
        super().on_enable()

    @state(first=True)
    def stand_by(self):
        self.next_state("tracking_trajectory")

    def tracking_trajectory(self, initial_call, state_tm) -> None:
        if initial_call:
            self.current_leg += 1

        if self.current_leg == len(self.trajectories):
            self.done()
            return

        # Get the current and target poses
        current_pose = self.swerve_drive.get_estimated_pose()
        final_pose = self.trajectories[self.current_leg].get_final_pose(
            wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed
        )
        if final_pose is None:
            self.done()
            return

        distance = current_pose.translation().distance(final_pose.translation())
        angle_error = (final_pose.rotation() - current_pose.rotation()).radians()

        # If within tolerance and halfway through, move to the next segment.
        if (
            distance < self.DISTANCE_TOLERANCE
            and math.isclose(angle_error, 0.0, abs_tol=self.ANGLE_TOLERANCE)
            and state_tm > self.trajectories[self.current_leg].get_total_time() / 2.0
        ):
            self.next_state("tracking_trajectory")
            return

        sample = self.trajectories[self.current_leg].sample_at(
            state_tm,
            wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed,
        )
        if sample is not None:
            self.follow_trajectory(sample)

    def follow_trajectory(self, sample: SwerveSample):
        # Get current pose
        pose = self.swerve_drive.get_estimated_pose()

        # Calculate chassis speeds using the PID controllers.
        speeds = ChassisSpeeds(
            sample.vx + x_controller.calculate(pose.X(), sample.x),
            sample.vy + y_controller.calculate(pose.Y(), sample.y),
            sample.omega,  # Using sample.omega directly
        )

        # Set drive commands
        self.swerve_drive.drive(
            speeds.vx,
            speeds.vy,
            speeds.omega,
            True,  # field_relative drive
            self.swerve_drive.period,  # use the period from your drive component.
        )

        # Update module states using the computed chassis speeds.
        self.swerve_drive.chassis(speeds)

    @timed_state(duration=4)
    def scoring_L1(self):
        self.arm_control.set(ElevatorHeight.L1, ClawAngle.TROUGH)
        self.arm_control.set_wheel_voltage(6.0)
        self.next_state("stand_by")

    @timed_state(duration=4)
    def scoring_L2(self):
        self.arm_control.set(ElevatorHeight.L2, ClawAngle.BRANCH)
        self.arm_control.set_wheel_voltage(6.0)
        self.next_state("stand_by")

    @timed_state(duration=4)
    def scoring_L3(self):
        self.arm_control.set(ElevatorHeight.L3, ClawAngle.BRANCH)
        self.arm_control.set_wheel_voltage(6.0)
        self.next_state("stand_by")

    @timed_state(duration=4)
    def scoring_L4(self):
        self.arm_control.set(ElevatorHeight.L4, ClawAngle.BRANCH)
        self.arm_control.set_wheel_voltage(6.0)
        self.next_state("stand_by")

    @state
    def intake(self):
        self.arm_control.set(ElevatorHeight.L1, ClawAngle.STATION)
        self.arm_control.set_wheel_voltage(-6.0)
        if self.claw.get_intake_limit():
            self.next_state("stand_by")
