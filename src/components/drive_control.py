import math

from phoenix6.hardware import Pigeon2
from wpilib import SmartDashboard, DriverStation, Timer
from wpimath import units
from wpimath.controller import HolonomicDriveController
from wpimath.geometry import Pose2d
from magicbot import StateMachine, will_reset_to
from magicbot.state_machine import state, timed_state

from lemonlib.preference import SmartProfile

from components.odometry import Odometry
from components.swerve_drive import SwerveDrive
from components.arm_control import ArmControl
from components.claw import Claw, ClawAngle
from components.elevator import Elevator, ElevatorHeight


class DriveControl(StateMachine):
    odometry: Odometry
    swerve_drive: SwerveDrive
    pigeon: Pigeon2
    arm_control: ArmControl

    remove_algae = will_reset_to(False)
    go_to_pose = will_reset_to(False)
    desired_pose = Pose2d()
    period: units.seconds = 0.02

    def drive_manual(
        self,
        translationX: units.meters_per_second,
        translationY: units.meters_per_second,
        rotationX: units.radians_per_second,
        field_relative: bool,
    ):
        if self.current_state == "free":
            self.translationX = translationX
            self.translationY = translationY
            self.rotationX = rotationX
            self.field_relative = field_relative

    def request_remove_algae(self, period: units.seconds = 0.02):
        self.period = period
        self.remove_algae = True

    def request_pose(self, pose: Pose2d):
        self.go_to_pose = True
        self.desired_pose = pose

    def drive_auto(
        self,
        translationX: units.meters_per_second,
        translationY: units.meters_per_second,
        rotationX: units.radians_per_second,
    ):
        self.translationX = translationX
        self.translationY = translationY
        self.rotationX = rotationX
        self.field_relative = True

    @state(first=True)
    def free(self):
        self.swerve_drive.drive(
            self.translationX,
            self.translationY,
            self.rotationX,
            self.field_relative,
            self.period,
        )
        if self.remove_algae:
            self.next_state("removing_algae")
        if self.go_to_pose:
            self.next_state("going_to_pose")
        if DriverStation.isAutonomousEnabled():
            self.next_state("auto")

    @timed_state(duration=4, next_state="free")
    def removing_algae(self):
        self.arm_control.set(ElevatorHeight.L1, ClawAngle.STOWED)
        self.swerve_drive.drive(-1, 0, 0, True, self.period)

    @state
    def going_to_pose(self):
        if not self.go_to_pose:
            self.next_state("free")
        self.swerve_drive.set_desired_pose(self.desired_pose)

    @state
    def run_auton_routine(self):
        # used to drive the bot and used here to keep driving in one place
        # main controls are in the auto_base.py like intake eject etc
    
        self.swerve_drive.drive(
            self.translationX,
            self.translationY,
            self.rotationX,
            self.field_relative,
            self.period,
        )
        if DriverStation.isTeleop():
            self.next_state("free")
