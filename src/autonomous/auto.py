from autonomous.auto_base import AutoBase
from magicbot import timed_state, state, AutonomousStateMachine
from components.drive_control import DriveControl
from wpimath.geometry import Pose2d, Translation2d, Rotation2d


class passline(AutonomousStateMachine):
    MODE_NAME = "Passline"
    drive_control: DriveControl

    @timed_state(duration=2, next_state="stop", first=True)
    def passline(self):
        self.drive_control.drive_auto(1, 0, 0)

    @state
    def stop(self):
        self.drive_control.drive_auto(0, 0, 0)
        self.done()


class Only_L4(AutoBase):
    MODE_NAME = "Only L4"

    def __init__(self):
        super().__init__(
            [
                "Start-4R",  # Run trajectory 1
                "state:level_four",  # Run intake state
            ]
        )


class l4_station(AutoBase):
    MODE_NAME = "L4 Station"

    def __init__(self):
        super().__init__(
            [
                "Start-4R",  # Run trajectory 1
                "state:level_four",  # Run intake state
                "4R-Station",  # Run intake state
            ]
        )


class l4_station_l4(AutoBase):
    MODE_NAME = "L4 Station L4"

    def __init__(self):
        super().__init__(
            [
                "Start-4R",
                "state:level_four",
                "4R-Station",
                "state:intaking_coral",
                "Station-6L",
                "state:level_four",
            ]
        )

class test_l4(AutoBase):
    MODE_NAME = "test"
    def __init__(self):
        super().__init__(["state:level_four"])
