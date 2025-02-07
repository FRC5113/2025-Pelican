import math
from enum import Enum

from magicbot import feedback, will_reset_to
from wpilib import DigitalInput
from lemonlib.preference import SmartProfile
from rev import SparkMax, SparkBaseConfig, SparkRelativeEncoder

from lemonlib.util import Alert, AlertType


class ElevatorHeight(float, Enum):
    # values likely inaccurate
    BOTTOM = 0.0
    TOP = 2.0


class Elevator:

    # Motors and encoders
    right_motor: SparkMax
    left_motor: SparkMax
    right_encoder: SparkRelativeEncoder
    left_encoder: SparkRelativeEncoder
    upper_switch: DigitalInput
    lower_switch: DigitalInput
    gearing: float  # ratio (10:1)
    spool_radius: float  # meters (1.5 inches)
    elevator_profile: SmartProfile

    target_height = will_reset_to(ElevatorHeight.BOTTOM)
    motor_voltage = will_reset_to(0)
    position_known = False

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
        # assumes that elevator starts at lowest position
        # eventually use limit switches to confirm
        self.reset_encoders()

        self.uncalibrated_alert = Alert(
            "Elevator encoders not calibrated! Moving elevator down.", AlertType.WARNING
        )

    def on_enable(self):
        self.controller = self.elevator_profile.create_elevator_controller("elevator")
        self.position_known = False
        self.uncalibrated_alert.enable()

    def get_encoder_rotations(self) -> float:
        """Return the average position of the encoders in motor
        rotations. 0 should correspond to the lowest position.
        (Assumes right motor must be inverted)
        """
        return (self.left_encoder.getPosition() - self.right_encoder.getPosition()) / 2

    @feedback
    def get_height(self) -> float:
        """Get the current height of the elevator."""
        return (
            self.get_encoder_rotations() / self.gearing * math.tau * self.spool_radius
        )

    def set_target_height(self, height: float):
        """Set the target height for the elevator."""
        self.target_height = height

    def reset_encoders(self):
        """Set the position of the encoders to zero."""
        self.left_encoder.setPosition(0)
        self.right_encoder.setPosition(0)

    def move_manual(self, voltage: float):
        """Move the elevator at a specified voltage. (Testing only)"""
        self.motor_voltage = voltage

    def execute(self):
        """Update the motor control to reach the target height."""
        if not self.lower_switch.get():
            self.position_known = True
            self.uncalibrated_alert.disable()
            self.reset_encoders()

        if not self.position_known:
            # move elevator down slowly until limit switch is reached and position is known
            self.motor_voltage = -0.5
            return

        if self.motor_voltage == 0:
            # calculate voltage from feedforward (only if voltage has not already been set)
            self.motor_voltage = self.controller.calculate(
                self.get_height(), self.target_height
            )

        # prevent motors from moving the elevator past the limits
        if not self.lower_switch.get() and self.motor_voltage < 0:
            return
        if not self.upper_switch.get() and self.motor_voltage > 0:
            return

        # assumes right motor must be inverted
        self.left_motor.setVoltage(self.motor_voltage)
        self.right_motor.setVoltage(-self.motor_voltage)
