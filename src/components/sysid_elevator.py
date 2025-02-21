from commands2.sysid import SysIdRoutine

from lemonlib.util import MagicSysIdRoutine

from components.elevator import Elevator


class SysIdElevator(MagicSysIdRoutine):
    elevator: Elevator

    def setup(self):
        self.setup_sysid(
            SysIdRoutine.Config(rampRate=0.2, stepVoltage=7.0),
            SysIdRoutine.Mechanism(
                self.elevator.sysid_drive,
                self.elevator.sysid_log,
                self.elevator,
                "Elevator",
            ),
        )
