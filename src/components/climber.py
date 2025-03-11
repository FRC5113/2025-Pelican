from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration
from phoenix6.signals import NeutralModeValue
from wpilib import DutyCycleEncoder
from wpimath import units
from magicbot import feedback, will_reset_to
from enum import Enum

from lemonlib.util import Alert, AlertType


class ClimberAngle(Enum):
    MIN = 0.0
    MAX = 50.0  # for the love of god change this


class Climber:

    motor: TalonFX
    encoder: DutyCycleEncoder

    motor_speed = will_reset_to(0)
    error = will_reset_to(False)

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

    def get_position(self) -> units.degrees:
        return self.encoder.get()

    @feedback
    def get_angle(self) -> units.degrees:
        angle = self.get_position() * 360
        if angle > 180:
            angle -= 360
        return angle

    @feedback
    def fully_climbed(self) -> bool:
        return self.get_angle() >= ClimberAngle.MAX.value

    def error_detected(self) -> bool:
        return self.error

    """
    CONTROL METHODS
    """

    def set_speed(self, speed: float):
        self.motor_speed = speed

    """
    EXECUTE
    """

    def execute(self):
        if (self.get_angle() < ClimberAngle.MIN.value) or (
            self.get_angle() > ClimberAngle.MAX.value
        ):
            self.motor_speed = 0
            self.error = True
        self.motor.set(self.motor_speed)
