from commands2.sysid import SysIdRoutine

from lemonlib.util import MagicSysIdRoutine

from components.elevator import Elevator


class SysIdElevator(MagicSysIdRoutine):
    elevator: Elevator

    def setup(self):
        self.setup_sysid(
            SysIdRoutine.Config(rampRate=0.25, stepVoltage=4.0),
            SysIdRoutine.Mechanism(
                self.elevator.sysid_drive,
                self.elevator.sysid_log,
                self.elevator,
                "Elevator",
            ),
        )
