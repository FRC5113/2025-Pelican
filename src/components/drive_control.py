from components.swerve_drive import SwerveDrive
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
from magicbot.state_machine import state
from lemonlib.preference import SmartProfile


class DriveControl(StateMachine):
    swerve_drive: SwerveDrive
    pigeon: Pigeon2
    auto_x_profile: SmartProfile
    auto_y_profile: SmartProfile
    auto_heading_profile: SmartProfile

    def setup(self):
        self.engage()

    def on_enable(self):
        self.auto_x_controller = self.auto_x_profile.create_pid_controller("auto_x")
        self.auto_y_controller = self.auto_y_profile.create_pid_controller("auto_y")
        self.auto_heading_controller = self.auto_heading_profile.create_pid_controller(
            "auto_heading"
        )

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

    @state(first=True)
    def free(self):
        print("free")
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
    def auto(self):
        print("auto")
        if DriverStation.isTeleop():
            self.next_state("free")
