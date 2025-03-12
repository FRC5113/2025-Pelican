from commands2.sysid import SysIdRoutine

from lemonlib.util import MagicSysIdRoutine
from components.swerve_drive import SwerveDrive
from components.drive_control import DriveControl
from wpimath import units


class SysIdDrive(MagicSysIdRoutine):
    swerve_drive: SwerveDrive
    drive_control: DriveControl
    period: units.seconds = 0.02

    def setup(self):
        self.setup_sysid(
            SysIdRoutine.Config(rampRate=0.2, stepVoltage=7.0),
            SysIdRoutine.Mechanism(
                self.drive_sysid,
                self.swerve_drive.log,
                self.swerve_drive,
                "Drive",
            ),
        )

    def drive_sysid(self,voltage: units.volts) -> None:
        self.swerve_drive.drive(voltage,0,0,False,self.period)

