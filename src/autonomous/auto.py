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
class Coral(AutoBase):
    MODE_NAME = "1 Coral L4"

    def __init__(self):
        super().__init__([
            "1CoralL4",             # Run trajectory 1
            "state:intaking_coral", # Run intake state
        ])