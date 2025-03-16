from autonomous.auto_base import AutoBase
from magicbot import timed_state, state, AutonomousStateMachine
from components.drive_control import DriveControl
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpilib import DriverStation


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

    @timed_state(duration=1,first=True,must_finish=True)
    def drive(self):
        self.drive_control.engage()
        self.drive_control.drive_auto_manual(-1,0,0,True)
    
    @state
    def finish(self):
        self.done()
    
