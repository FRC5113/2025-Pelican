import math

from phoenix6.hardware import Pigeon2
from wpilib import SmartDashboard, DriverStation, Timer
from wpimath import units
from wpimath.controller import HolonomicDriveController
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from choreo.trajectory import SwerveSample
from magicbot import StateMachine, will_reset_to
from magicbot.state_machine import state, timed_state

from lemonlib.smart import SmartProfile

# from components.odometry import Odometry
from components.swerve_drive import SwerveDrive
from components.arm_control import ArmControl
from components.claw import Claw, ClawAngle
from components.elevator import Elevator, ElevatorHeight


class DriveControl(StateMachine):
    arm_control: ArmControl
    # odometry: Odometry
    swerve_drive: SwerveDrive

    pigeon: Pigeon2

    remove_algae_var = will_reset_to(False)
    go_to_pose = will_reset_to(False)
    desired_pose = Pose2d()
    period: units.seconds = 0.02
    drive_auto_man = will_reset_to(False)
    translationX = will_reset_to(0)
    translationY = will_reset_to(0)
    rotationX = will_reset_to(0)
    raiseclaw = will_reset_to(False)
    sample: SwerveSample = None

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

    def request_remove_algae(
        self, elevatorheight, raiseclaw, period: units.seconds = 0.02
    ):
        self.remove_algae_var = True
        self.elevatorheight = elevatorheight
        self.raiseclaw = raiseclaw

    def request_pose(self, pose: Pose2d):
        self.go_to_pose = True
        self.desired_pose = pose

    def drive_auto(self, sample: SwerveSample = None):
        self.sample = sample

    def drive_auto_manual(
        self,
        translationX: units.meters_per_second,
        translationY: units.meters_per_second,
        rotationX: units.radians_per_second,
        field_relative: bool,
    ):

        self.translationX = translationX
        self.translationY = translationY
        self.rotationX = rotationX
        self.field_relative = field_relative
        self.drive_auto_man = True

    @state(first=True)
    def initialise(self):
        self.translationX = 0
        self.translationY = 0
        self.rotationX = 0
        self.field_relative = False
        if self.remove_algae_var:
            self.next_state("remove_algae_placement")
        if self.go_to_pose:
            self.next_state("going_to_pose")
        if DriverStation.isAutonomousEnabled():
            self.next_state("run_auton_routine")
        self.next_state("free")

    @state
    def free(self):
        self.swerve_drive.drive(
            self.translationX,
            self.translationY,
            self.rotationX,
            self.field_relative,
            self.period,
        )
        if DriverStation.isAutonomousEnabled():
            self.next_state("run_auton_routine")
        if self.remove_algae_var:
            self.next_state("remove_algae_placement")
        if self.go_to_pose:
            self.next_state("going_to_pose")

    @state
    def remove_algae_placement(self, state_tm):
        self.arm_control.set(self.elevatorheight, ClawAngle.SAFE_END)
        if state_tm > 1.0:
            self.swerve_drive.drive(-1, 0, 0, False, self.period)
        if not self.remove_algae_var:
            self.next_state("free")
        if (
            self.arm_control.at_point(self.elevatorheight, ClawAngle.SAFE_END)
            and not self.raiseclaw
        ):
            self.next_state("remove_algae")

    @timed_state(duration=1.5, next_state="free", must_finish=True)
    def remove_algae(self, state_tm):
        self.arm_control.set(self.elevatorheight, ClawAngle.SAFE_START)
        if state_tm > 1.0:
            self.swerve_drive.drive(1, 0, 0, False, self.period)

    @state
    def going_to_pose(self):
        if not self.go_to_pose:
            self.next_state("free")
        self.swerve_drive.set_desired_pose(self.desired_pose)

    @state
    def run_auton_routine(self):
        # used to drive the bot and used here to keep driving in one place
        # main controls are in the auto_base.py like intake eject etc
        if self.sample is not None:
            self.swerve_drive.follow_trajectory(self.sample)
        if DriverStation.isTeleop():
            self.next_state("free")
