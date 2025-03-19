from wpilib import Color
from magicbot import will_reset_to, feedback
from wpilib import DriverStation
from components.arm_control import ArmControl
from components.swerve_drive import SwerveDrive
from components.climber import Climber
from components.claw import Claw

from lemonlib.util import LEDController, AlertManager, AlertType


class LEDStrip:
    swerve_drive: SwerveDrive
    climber: Climber
    claw: Claw
    leds: LEDController

    justin_bool = will_reset_to(False)

    """
    INITIALIZATION METHODS
    """

    def setup(self):
        self.leds.set_solid_color((10, 10, 10))
        self.coral_detected = (0, 0, 255)
        self.aligned_branch = (0, 255, 0)
        self.fully_climbed = (255, 0, 255)
        self.error_color = (255, 0, 0)
        self.warning_color = (255, 255, 0)
        self.auton_color = (255, 187, 0)

    def on_disable(self):
        self.leds.set_solid_color((10, 10, 10))

    """
    INFORMATIONAL METHODS
    """

    @feedback
    def get_colors(self) -> list[str]:
        """Returns LED colors in list of hex strings"""
        return [Color(led.r, led.g, led.b).hexString() for led in self.leds.buffer]

    @feedback
    def has_warnings_present(self) -> bool:
        return len(AlertManager.get_strings(AlertType.WARNING)) > 0

    @feedback
    def has_errors_present(self) -> bool:
        return len(AlertManager.get_strings(AlertType.ERROR)) > 0

    """
    CONTROL METHODS
    """

    def justin_fun(self):
        self.justin_bool = True

    """
    EXECUTE
    """

    def execute(self):
        if self.has_errors_present():
            self.leds.set_solid_color(self.error_color)
        elif self.has_warnings_present():
            self.leds.set_solid_color(self.warning_color)
        elif 0 < self.swerve_drive.get_distance_from_desired_pose() <= 0.03:
            self.leds.set_solid_color(self.aligned_branch)
        elif self.climber.is_deployed():
            self.leds.set_solid_color(self.fully_climbed)
        elif self.claw.get_intake_limit():
            self.leds.set_solid_color(self.coral_detected)
        elif self.justin_bool:
            self.leds.scolling_rainbow(6)
        elif DriverStation.isAutonomousEnabled():
            self.leds.set_solid_color(self.auton_color)

        else:
            self.leds.move_across((255, 255, 0), 15, 12)
