from wpilib import LEDPattern, Color, Timer
from lemonlib.util import LEDController
from components.arm_control import ArmControl
from components.climber import Climber
from components.claw import Claw
from components.error import Errors


class LEDStrip:
    leds: LEDController
    climber: Climber
    claw: Claw
    arm_control: ArmControl
    error: Errors

    def setup(self):
        self.leds.set_rainbow()
        self.coral_detected = (0, 0, 255)
        self.aligned_branch = (0, 255, 0)
        self.fully_climbed = (0, 255, 0)
        self.error1 = (255, 0, 0)
        self.error2 = (255, 255, 0)
        self.timer = Timer()
        self.error_bool = self.error.get_all_errors()

    def execute(self):
        if self.error:
            self.timer.start()

            if int(self.timer.get()) % 2 == 0:
                self.leds.set_solid_color(self.error1)
            else:
                self.leds.set_solid_color(self.error2)
        elif self.climber.fully_climbed():
            self.leds.set_solid_color(self.fully_climbed)
        elif self.claw.get_intake_limit():
            self.leds.set_solid_color(self.coral_detected)
        else:
            self.leds.set_rainbow()
