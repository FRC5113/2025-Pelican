from wpilib import Color
from magicbot import will_reset_to, feedback
from wpilib import DriverStation, LEDPattern
from components.arm_control import ArmControl
from components.swerve_drive import SwerveDrive
from components.climber import Climber
from components.claw import Claw

from lemonlib.util import LEDController, AlertManager, AlertType
from commands2 import Command
from lemonlib import LemonComponent


class LEDStrip:
    swerve_drive: SwerveDrive
    climber: Climber
    claw: Claw
    leds: LEDController

    justin_bool = will_reset_to(False)
    is_aligned = will_reset_to(False)

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
        self.auton_color = (255, 50, 0)
        self.disabled = (10, 10, 10)
        self.idle = (50, 50, 50)
        # Halloween temp

        self.halloween_color_a = (255, 0, 228)  # purple
        self.halloween_color_b = (255, 30, 0)  # orange
        self.halloween_color_c = (133, 255, 0)  # green

        self.fully_climbed = self.halloween_color_a
        self.coral_detected = self.halloween_color_b
        self.auton_color = self.halloween_color_c

    def on_disable(self):
        self.leds.set_gradient(self.halloween_color_b, (255, 165, 0))

    """
    INFORMATIONAL METHODS
    """

    def get_colors(self) -> list[str]:
        """Returns LED colors in list of hex strings"""
        return [Color(led.r, led.g, led.b).hexString() for led in self.leds.buffer]

    def has_warnings_present(self) -> bool:
        return len(AlertManager.get_strings(AlertType.WARNING)) > 0

    def has_errors_present(self) -> bool:
        return len(AlertManager.get_strings(AlertType.ERROR)) > 0

    """
    CONTROL METHODS
    """

    def set_is_aligned(self):
        self.is_aligned = True

    def justin_fun(self):
        self.justin_bool = True

    def commandtest(self) -> Command:
        return self.leds.set_solid_color((0, 255, 0))

    def hollows_eve_disabled(self):
        self.leds.move_across_multi(
            [self.halloween_color_a, self.halloween_color_b, self.halloween_color_c],
            20,
            20,
        )

    def hollows_eve(self):
        self.leds.move_across_multi(
            [self.halloween_color_a, self.halloween_color_b, self.halloween_color_c],
            20,
            40,
        )

    """
    EXECUTE
    """

    def execute(self):
        if self.has_errors_present():
            self.leds.set_solid_color(self.error_color)
        elif self.has_warnings_present():
            self.leds.set_solid_color(self.warning_color)
        elif (
            0 < self.swerve_drive.get_distance_from_desired_pose() <= 0.03
            or self.is_aligned
        ):
            self.leds.set_solid_color(self.aligned_branch)
            # Changed for halloween
        elif self.climber.is_deployed():
            self.leds.set_solid_color(self.fully_climbed)
        elif self.claw.get_intake_limit():
            self.leds.set_solid_color(self.coral_detected)
        elif self.justin_bool:
            self.leds.scolling_rainbow(6)
        elif DriverStation.isAutonomousEnabled():
            self.leds.move_across(self.auton_color, 20, 50)
        else:
            # self.leds.set_solid_color((50,50,50))
            self.hollows_eve()
