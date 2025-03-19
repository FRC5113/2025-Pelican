from autonomous.auto_base import AutoBase
from magicbot import timed_state, state, AutonomousStateMachine
from components.drive_control import DriveControl
from wpimath.geometry import Pose2d, Translation2d, Rotation2d, Transform2d
from wpilib import DriverStation
from lemonlib import LemonCamera
from components.swerve_drive import SwerveDrive
from components.arm_control import ArmControl
from components.elevator import ElevatorHeight, Elevator
from components.claw import ClawAngle, Claw


"""
Trajectories:
- 2L-TopStation
- TopStation-2R
- Reg-Start-2L
- BottomStation-6R
- Center-Start-4R
- 4R-BottomStation
- BottomStation-6L
"""

"""
States:
- level_four
- level_three
- level_two
- level_one
- intaking_coral
- spit
"""


class Center_4R(AutoBase):
    MODE_NAME = "Bottom>L4"

    def __init__(self):
        super().__init__(
            [
                "Center-Start-4R",
                "state:level_four",
            ]
        )


class Center_4R_Station(AutoBase):
    MODE_NAME = "Bottom>L4>Station"

    def __init__(self):
        super().__init__(
            [
                "Center-Start-4R",
                "state:level_four",
                "4R-BottomStation",
            ]
        )


class Center_4R__6L(AutoBase):
    MODE_NAME = "Bottom>L4>Station>6L"

    def __init__(self):
        super().__init__(
            [
                "Center-Start-4R",
                "state:level_four",
                "4R-BottomStation",
                "state:intaking_coral",
                "BottomStation-6L",
                "state:level_four",
            ]
        )


class Top_2L(AutoBase):
    MODE_NAME = "Top>L4"

    def __init__(self):
        super().__init__(
            [
                "Reg-Start-2L",
                "state:level_four",
            ]
        )


class Top_2L_Station(AutoBase):
    MODE_NAME = "Top>L4>Station"

    def __init__(self):
        super().__init__(
            [
                "Reg-Start-2L",
                "state:level_four",
                "2L-TopStation",
            ]
        )


class Top_2L__2R(AutoBase):
    MODE_NAME = "Top>L4>Station>2R"

    def __init__(self):
        super().__init__(
            [
                "Reg-Start-2L",
                "state:level_four",
                "2L-TopStation",
                "state:intaking_coral",
                "TopStation-2R",
                "state:level_four",
            ]
        )


class passline(AutonomousStateMachine):
    MODE_NAME = "passline"

    drive_control: DriveControl

    @timed_state(duration=1, first=True, must_finish=True)
    def drive(self):
        self.drive_control.engage()
        self.drive_control.drive_auto_manual(-1, 0, 0, True)

    @state
    def finish(self):
        self.done()


class blue_l4(AutonomousStateMachine):
    MODE_NAME = "blue l4"

    drive_control: DriveControl
    camera: LemonCamera
    swerve_drive: SwerveDrive
    arm_control: ArmControl
    claw: Claw
    elevator: Elevator

    @timed_state(duration=1, first=True, must_finish=True, next_state="raise_arm")
    def drive(self):
        self.arm_control.engage()
        self.drive_control.engage()
        self.drive_control.drive_auto_manual(1, 0, 0, True)

    @state
    def raise_arm(self, state_tm):
        self.arm_control.engage()
        self.arm_control.set(ElevatorHeight.L4, ClawAngle.BRANCH)
        if self.arm_control.current_state == "standby" and state_tm > 1.0:
            self.next_state("align")

    @state
    def align(self):
        self.arm_control.engage()
        self.arm_control.set(ElevatorHeight.L4, ClawAngle.BRANCH)
        self.drive_control.engage()
        self.drive_control.request_pose(
            self.camera.get_tag_pose(21,True).transformBy(
                Transform2d(0.3, 0.2, Rotation2d())
            )
        )
        self.swerve_drive.has_desired_pose = True
        # print(self.swerve_drive.has_desired_pose)
        if 0.0 < self.swerve_drive.get_distance_from_desired_pose() < 0.03:
            self.next_state("score")

    @timed_state(duration=1, next_state="back_up")
    def score(self):
        self.arm_control.engage()
        self.arm_control.set(ElevatorHeight.L4, ClawAngle.BRANCH)
        self.arm_control.set_wheel_voltage(-8)

    @timed_state(duration=1, next_state="finish")
    def back_up(self):
        self.arm_control.engage()
        self.drive_control.engage()
        self.drive_control.drive_auto_manual(-1, 0, 0, True)

    @state
    def finish(self):
        self.done()
