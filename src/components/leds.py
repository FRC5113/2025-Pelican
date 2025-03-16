from wpilib import LEDPattern, Color, Timer, _wpilib, SmartDashboard, RobotController
from lemonlib.util import LEDController
from components.arm_control import ArmControl
from components.climber import Climber
from components.claw import Claw
from components.error import Errors
from wpimath import units
from magicbot import will_reset_to, feedback

from lemonlib import LemonInput
from components import error


class LEDStrip:
    leds: LEDController
    climber: Climber
    claw: Claw
    arm_control: ArmControl
    error: Errors

    period: units.seconds = 0.02

    justin_bool = will_reset_to(False)

    def setup(self):
        self.leds.set_solid_color((10, 10, 10))
        self.coral_detected = (0, 0, 255)
        self.aligned_branch = (0, 255, 0)
        self.fully_climbed = (0, 255, 255)
        self.error_color = (255, 0, 0)
        self.warning_color = (255, 255, 0)
        self.error_bool = self.error.get_alert_error()
        self.warn_bool = self.error.get_alert_warn()

    def justin_fun(self):
        self.justin_bool = True

    @feedback
    def get_color(self):
        return Color(self.leds.buffer[0].r,self.leds.buffer[0].g,self.leds.buffer[0].b).hexString()

    def execute(self):
        if self.error_bool:
            self.leds.set_solid_color(self.error_color)
        elif self.warn_bool:
            self.leds.set_solid_color(self.warning_color)
        elif self.climber.deployed():
            self.leds.set_solid_color(self.fully_climbed)
        elif self.claw.get_intake_limit():
            self.leds.set_solid_color(self.coral_detected)
        else:
            if self.justin_bool:
                self.leds.scolling_rainbow(6)
            else:
                self.leds.move_across((255,255,0),15,12)
