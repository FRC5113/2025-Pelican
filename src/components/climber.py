from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration
from phoenix6.signals import NeutralModeValue
from wpilib import DutyCycleEncoder
from wpimath import units
from magicbot import feedback, will_reset_to
from enum import Enum

from lemonlib.util import Alert, AlertType


class ClimberAngle(Enum):
    MIN = -95.0
    MAX = 0.0
    STOWED = -85.0
    DEPLOYED = -25.0


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
    def get_falcon_encoder(self) -> units.turns:
        return self.motor.get_position().value

    @feedback
    def fully_climbed(self) -> bool:
        return self.get_angle() >= ClimberAngle.MAX.value

    def fully_out(self) -> bool:
        return self.get_angle() <= ClimberAngle.MIN.value

    def deployed(self) -> bool:
        return self.get_falcon_encoder() <= -425

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
        # if (self.get_angle() < ClimberAngle.MIN.value) or (
        #     self.get_angle() > ClimberAngle.MAX.value
        # ):
        #     self.motor_speed = 0
        #     self.error = True
        if self.get_angle() <= ClimberAngle.STOWED.value:
            self.motor.set_position(0)
        self.motor.set(self.motor_speed)
