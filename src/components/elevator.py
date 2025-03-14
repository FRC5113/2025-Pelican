import math
from enum import Enum

from wpimath import units
from magicbot import feedback, will_reset_to
from wpilib import DigitalInput
from lemonlib.smart import SmartProfile
from rev import SparkMax, SparkBaseConfig, SparkRelativeEncoder

from lemonlib.util import Alert, AlertType


class ElevatorHeight(float, Enum):
    # values likely inaccurate
    STATION = 0.045
    L1 = 0.0
    L2 = 0.16
    L3 = 0.36
    L4 = 0.69


class Elevator:

    # Motors and encoders
    right_motor: SparkMax
    left_motor: SparkMax
    right_encoder: SparkRelativeEncoder
    left_encoder: SparkRelativeEncoder
    upper_switch: DigitalInput
    lower_switch: DigitalInput
    gearing: float
    spool_radius: units.meters
    elevator_profile: SmartProfile
    tolerance: units.meters

    target_height = will_reset_to(ElevatorHeight.L1)
    motor_voltage = will_reset_to(0)
    manual_control = False

    """
    INITIALIZATION METHODS
    """

    def setup(self):
        """Initialize motors and encoder."""
        self.left_motor.configure(
            SparkBaseConfig().setIdleMode(SparkBaseConfig.IdleMode.kBrake),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.right_motor.configure(
            SparkBaseConfig().setIdleMode(SparkBaseConfig.IdleMode.kBrake),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.limit_error_alert = Alert(
            "At least one elevator limit switch is unplugged! Halting elevator.",
            AlertType.ERROR,
        )

    def on_enable(self):
        self.controller = self.elevator_profile.create_elevator_controller("elevator")

    """
    INFORMATIONAL METHODS
    """

    def get_encoder_rotations(self) -> units.turns:
        """Return the average position of the encoders in motor
        rotations. 0 should correspond to the lowest position.
        (Assumes right motor must be inverted)
        """
        return (self.left_encoder.getPosition() - self.right_encoder.getPosition()) / 2

    @feedback
    def get_height(self) -> units.meters:
        """Get the current height of the elevator."""
        return (
            self.get_encoder_rotations() / self.gearing * math.tau * self.spool_radius
        )

    def get_setpoint(self) -> units.meters:
        return self.target_height

    def get_lower_switch(self) -> bool:
        return self.lower_switch.get()

    def at_setpoint(self) -> bool:
        return abs(self.target_height - self.get_height()) <= self.tolerance

    def error_detected(self) -> bool:
        self.lower_switch.get() and self.upper_switch.get()

    """
    CONTROL METHODS
    """

    def set_target_height(self, height: units.meters):
        """Set the target height for the elevator."""
        self.target_height = height
        self.manual_control = False

    def reset_encoders(self):
        """Set the position of the encoders to zero."""
        self.left_encoder.setPosition(0)
        self.right_encoder.setPosition(0)

    def set_voltage(self, voltage: units.volts):
        """Move the elevator at a specified voltage. (Testing only)"""
        self.motor_voltage = voltage
        self.manual_control = True

    """
    EXECUTE
    """

    def execute(self):

        if self.lower_switch.get():
            self.reset_encoders()

        # calculate voltage from feedforward (only if voltage has not already been set)
        if not self.manual_control:
            self.motor_voltage = self.controller.calculate(
                self.get_height(), self.target_height
            )

        if self.error_detected():
            self.limit_error_alert.enable()
            return
        else:
            self.limit_error_alert.disable()

        # prevent motors from moving the elevator past the limits
        if self.lower_switch.get() and self.motor_voltage < 0:
            self.motor_voltage = 0
        if self.upper_switch.get() and self.motor_voltage > 0:
            self.motor_voltage = 0
        # assumes right motor must be inverted
        self.left_motor.setVoltage(self.motor_voltage)
        self.right_motor.setVoltage(-self.motor_voltage)
