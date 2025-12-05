from commands2.sysid import SysIdRoutine

from lemonlib.util import MagicSysIdRoutine
from components.swerve_drive import SwerveDrive
from components.drive_control import DriveControl
from wpimath import units


class SysIdDriveLinear(MagicSysIdRoutine):
    drive_control: DriveControl
    swerve_drive: SwerveDrive
    period: units.seconds = 0.02

    def setup(self):
        self.setup_sysid(
            SysIdRoutine.Config(rampRate=1, stepVoltage=7.0, timeout=5),
            SysIdRoutine.Mechanism(
                self.drive_sysid,
                self.swerve_drive.log,
                self.swerve_drive,
                "Drive Linear",
            ),
        )

    def drive_sysid(self, voltage: units.volts) -> None:
        self.drive_control.drive_sysid_manual(voltage)


class SysIdDriveRotation(MagicSysIdRoutine):
    drive_control: DriveControl
    swerve_drive: SwerveDrive
    period: units.seconds = 0.02

    def setup(self):
        self.setup_sysid(
            SysIdRoutine.Config(rampRate=0.2, stepVoltage=7.0),
            SysIdRoutine.Mechanism(
                self.drive_sysid,
                self.swerve_drive.log,
                self.swerve_drive,
                "Drive Rotatinal",
            ),
        )

    def drive_sysid(self, voltage: units.volts) -> None:
        self.drive_control.drive_sysid_manual(voltage, 1)
