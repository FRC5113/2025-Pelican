from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration
from phoenix6.signals import NeutralModeValue
from phoenix6 import controls
from wpilib import Encoder, DigitalInput, DriverStation
from magicbot import feedback, will_reset_to
from lemonlib.util import Alert, AlertManager, AlertType


class Climber:

    debug = True

    motor: TalonFX
    encoder: Encoder
    winch_limit_switch: DigitalInput
    left_hook_limit_switch: DigitalInput
    right_hook_limit_switch: DigitalInput

    motor_speed = will_reset_to(0)
    manual = will_reset_to(False)
    moving_down = False
    position_known = False

    """
    INITIALIZATION METHODS
    """

    def setup(self):
        self.motor_configs = TalonFXConfiguration()
        self.motor_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.motor.configurator.apply(self.motor_configs)
        if self.debug:
            AlertManager.instant_alert("Climber is in debug mode", AlertType.WARNING)
            self.manual_climber = Alert(
                "Climber is moving Manually and can break", AlertType.WARNING, 0.5
            )

    """
    INFORMATIONAL METHODS
    """

    @feedback
    def get_position(self):
        return self.encoder.getDistance()

    """
    CONTROL METHODS
    """

    def move(self, speed: float):
        self.motor_speed = speed
        self.moving_down = True

    def move_manual(self, speed: float):
        """Move the climber Manually. DO NOT USE IN COMPETITION OR LET JUSTIN TOUCH THIS"""

        self.motor_speed = speed
        self.moving_down = True
        self.manual_climber.enable()

    """
    EXECUTE
    """

    def execute(self):
        if self.manual and self.debug:
            self.encoder.reset()
            self.motor.set(self.motor_speed)
            return
        if self.winch_limit_switch.get():
            self.motor_speed = 0
            self.encoder.reset()
            self.position_known = True
        if not self.position_known:
            self.motor_speed = -0.5
            return

        # make sure the climber doesn't go past the limit
        if (self.winch_limit_switch.get() and self.motor_speed < 0) or (
            self.encoder.getDistance() > 1000
            and self.motor_speed > 0  # MUST CHANGE 1000 THIS IS A PLACEHOLDER
        ):
            self.motor_speed = 0  # Stop the motor
            return

        # allows you to take bot off chain once climbed and disabled
        if (
            self.right_hook_limit_switch.get() or self.left_hook_limit_switch.get()
        ) and DriverStation.isDisabled():
            self.motor_speed = 0
            self.motor.set_control(controls.coast_out.CoastOut())
            return

        self.motor.set(self.motor_speed)
