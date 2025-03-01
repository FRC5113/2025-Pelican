from components.swerve_drive import SwerveDrive
from components.arm_control import ArmControl
from components.claw import Claw
from components.elevator import Elevator
import math


from phoenix6.hardware import Pigeon2
from wpilib import SmartDashboard, DriverStation, Timer
from wpimath import units
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveModulePosition,
)
from magicbot import StateMachine, will_reset_to
from magicbot.state_machine import state, timed_state


class DriveControl(StateMachine):
    swerve_drive: SwerveDrive
    pigeon: Pigeon2
    arm_control: ArmControl
    remove_algae = will_reset_to(False)

    def setup(self):
        self.engage()
        self.timer = Timer()

    def drive_manual(
        self,
        translationX: units.meters_per_second,
        translationY: units.meters_per_second,
        rotationX: units.radians_per_second,
        field_relative: bool,
        period: units.seconds,
    ):
        if self.current_state == "free":
            self.translationX = translationX
            self.translationY = translationY
            self.rotationX = rotationX
            self.period = period
            self.field_relative = field_relative

    def request_remove_algae(
        self,
        elevator_setpoint: units.meters,
        claw_setpoint: units.degrees,
        period: units.seconds,
    ):
        self.elevator_setpoint = elevator_setpoint
        self.claw_setpoint = claw_setpoint
        self.period = period
        self.remove_algae = True

    def drive_auto():
        return

    @state(first=True)
    def free(self):
        self.swerve_drive.drive(
            self.translationX,
            self.translationY,
            self.rotationX,
            self.field_relative,
            self.period,
        )
        if DriverStation.isAutonomousEnabled():
            self.next_state("auto")

    @state
    def remove_algae_state(self):
        self.arm_control.set(self.elevator_setpoint, self.claw_setpoint)
        if self.arm_control.at_setpoint():
            self.swerve_drive.drive(-1, 0, 0, True, self.period)
            self.next_state("free")

    @state
    def auto(self):
        if DriverStation.isTeleop():
            self.next_state("free")
