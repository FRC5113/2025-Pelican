from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration
from phoenix6.signals import NeutralModeValue
from wpilib import DutyCycleEncoder
from wpimath import units
from magicbot import feedback, will_reset_to
from enum import Enum

from lemonlib.util import Alert, AlertType


class ClimberAngle(Enum):
    MIN = -45
    MAX = 55.0
    STOWED = -40.0
    DEPLOYED = -25.0


class Climber:

    motor: TalonFX
    encoder: DutyCycleEncoder

    motor_speed = will_reset_to(0)

    """
    INITIALIZATION METHODS
    """

    def setup(self):
        self.motor_configs = TalonFXConfiguration()
        self.motor_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.motor.configurator.apply(self.motor_configs)
        self.limit_alert = Alert("Climber has exceeded limits!", type=AlertType.ERROR)
        self.offset = 0

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
        return self.motor.get_position().value - self.offset

    def is_deployed(self) -> bool:
        return self.get_falcon_encoder() <= -425

    """
    CONTROL METHODS
    """

    def set_speed(self, speed: float):
        self.motor_speed = speed

    """
    EXECUTE
    """

    def execute(self):
        if self.get_angle() < ClimberAngle.MIN.value - 20:
            self.limit_alert.enable()
            self.limit_alert.set_text(
                f"Climber has exceeded limits! Current angle: {self.get_angle()}"
            )
        else:
            self.limit_alert.disable()

        if self.get_angle() < ClimberAngle.MIN.value and self.motor_speed > 0:
            self.motor_speed = 0
        if self.get_falcon_encoder() < -550.0 and self.motor_speed < 0:
            self.motor_speed = 0
        if self.get_angle() <= ClimberAngle.STOWED.value:
            self.offset = self.motor.get_position().value
        self.motor.set(self.motor_speed)
