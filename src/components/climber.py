from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration
from phoenix6.signals import NeutralModeValue
from wpilib import DutyCycleEncoder
from magicbot import feedback, will_reset_to
from lemonlib.util import Alert, AlertType


class Climber:

    motor: TalonFX
    encoder: DutyCycleEncoder
    min_position: float
    max_position: float

    motor_speed = will_reset_to(0)

    """
    INITIALIZATION METHODS
    """

    def setup(self):
        self.motor_configs = TalonFXConfiguration()
        self.motor_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.motor.configurator.apply(self.motor_configs)

    """
    INFORMATIONAL METHODS
    """

    @feedback
    def get_position(self):
        return self.encoder.get()

    """
    CONTROL METHODS
    """

    def set_speed(self, speed: float):
        self.motor_speed = speed

    """
    EXECUTE
    """

    def execute(self):
        # assumes a positive voltage creates an increase in angle
        # if self.get_position() < self.min_position and self.motor_speed < 0:
        #     return
        # if self.get_position() > self.max_position and self.motor_speed > 0:
        #     return
        self.motor.set(self.motor_speed)
