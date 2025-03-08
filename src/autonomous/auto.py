from autonomous.auto_base import AutoBase
from magicbot import timed_state, state,AutonomousStateMachine
from components.drive_control import DriveControl
from wpimath.geometry import Pose2d, Translation2d, Rotation2d


class passline(AutonomousStateMachine):
    MODE_NAME = "Passline"
    drive_control: DriveControl


    @timed_state(duration=2, next_state="stop",first=True)
    def passline(self):
        self.drive_control.drive_auto(1, 0, 0)
    @state
    def stop(self):
        self.drive_control.drive_auto(0, 0, 0)
        self.done()
class test(AutonomousStateMachine):
    MODE_NAME = "Test"
    drive_control: DriveControl


    @timed_state(duration=2, next_state="stop",first=True)
    def test(self):
        self.drive_control.request_pose(
                    Pose2d(Translation2d(-0.35, 0.19), Rotation2d())
                )
    @state
    def stop(self):
        self.drive_control.drive_auto(0, 0, 0)
        self.done()
class coral(AutoBase):
    MODE_NAME = "1 Coral L4"
    def __init__(self):
        super().__init__(
            [
                "1CoralL4",
                "coral>station",
            ]
        )
    




