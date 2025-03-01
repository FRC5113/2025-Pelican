from enum import Enum
from wpimath import applyDeadband
from wpimath import units
from rev import SparkMax, SparkBaseConfig, SparkAbsoluteEncoder, SparkLimitSwitch
from lemonlib.preference import SmartProfile
from magicbot import feedback, will_reset_to

from lemonlib.util import Alert, AlertType


class ClawAngle(float, Enum):
    STOWED = 29.0 #for 
    STATION = 30.0
    TROUGH = 100.0
    BRANCH = 115.0
    SAFE_START = 45.0  # has to be adusted just an estimate
    SAFE_END = 117.0  # has to be adusted just an estimate


class Claw:

    hinge_motor: SparkMax
    left_motor: SparkMax
    right_motor: SparkMax
    claw_profile: SmartProfile
    gearing: float
    max_angle: units.degrees  # maximum angle claw can rotate downwards
    hinge_tolerance: units.degrees
    hinge_encoder: SparkAbsoluteEncoder
    intake_limit: SparkLimitSwitch

    target_angle = will_reset_to(ClawAngle.STOWED)
    left_wheel_voltage = will_reset_to(0)
    right_wheel_voltage = will_reset_to(0)
    hinge_voltage = will_reset_to(0)
    hinge_manual_control = False

    """
    INITIALIZATION METHODS
    """

    def setup(self):
        self.right_motor.configure(
            SparkBaseConfig().setIdleMode(SparkBaseConfig.IdleMode.kBrake),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.left_motor.configure(
            SparkBaseConfig().setIdleMode(SparkBaseConfig.IdleMode.kBrake),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.hinge_alert = Alert(
            "Claw hinge has rotated too far!", type=AlertType.ERROR
        )

    def on_enable(self):
        self.hinge_motor.configure(
            SparkBaseConfig().setIdleMode(SparkBaseConfig.IdleMode.kBrake),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.controller = self.claw_profile.create_arm_controller("claw")

    def on_disable(self):
        self.hinge_motor.configure(
            SparkBaseConfig().setIdleMode(SparkBaseConfig.IdleMode.kCoast),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

    """
    INFORMATIONAL METHODS
    """

    def get_position(self) -> units.turns:
        """Return hinge rotations from encoder. Should be calibrated so
        that 0 rotations corresponds to up."""
        return self.hinge_encoder.getPosition()

    @feedback
    def get_angle(self) -> units.degrees:
        """Return the angle of the hinge normalized to [-180,180].
        An angle of 0 refers to the claw in the up/stowed position.
        """
        angle = self.get_position() * 360
        if angle > 180:
            angle -= 360
        return angle

    def get_setpoint(self) -> units.degrees:
        return self.target_angle

    @feedback
    def get_intake_limit(self) -> bool:
        return self.intake_limit.get()

    def is_safe(self) -> bool:
        return ClawAngle.SAFE_START <= self.get_angle() <= ClawAngle.SAFE_END

    def at_setpoint(self) -> bool:
        return abs(self.target_angle - self.get_angle()) <= self.hinge_tolerance 
    """
    CONTROL METHODS
    """

    def set_wheel_voltage(self, voltage: units.volts, eject_differential: float):
        self.left_wheel_voltage = voltage
        self.right_wheel_voltage = voltage
        if voltage < 0:
            self.right_wheel_voltage *= eject_differential

    def set_target_angle(self, angle: units.degrees):
        self.target_angle = angle
        self.hinge_manual_control = False

    def set_hinge_voltage(self, voltage: units.volts):
        self.hinge_voltage = voltage
        self.hinge_manual_control = True

    """
    EXECUTE
    """

    def execute(self):
        if self.intake_limit.get() and (
            self.left_wheel_voltage > 0 and self.right_wheel_voltage > 0
        ):
            self.left_wheel_voltage = 0
            self.right_wheel_voltage = 0
        # positive voltage (left) = intake
        self.left_motor.setVoltage(self.left_wheel_voltage)
        self.right_motor.setVoltage(-self.right_wheel_voltage)

        if not self.hinge_manual_control:
            # calculate voltage from feedforward (only if voltage has not already been set)
            self.hinge_voltage = -self.controller.calculate(
                self.get_angle(), self.target_angle
            )
        if self.get_angle() - self.max_angle > 10 or self.get_angle() < -10:
            self.hinge_alert.enable()
            self.hinge_alert.set_text(
                f"Claw hinge has rotated too far! Current angle: {self.get_angle()}"
            )
        else:
            self.hinge_alert.disable()
        if self.get_angle() > self.max_angle and self.hinge_voltage < 0:
            self.hinge_voltage = 0
        if self.get_angle() < 0 and self.hinge_voltage > 0:
            self.hinge_voltage = 0
        # positive voltage = decrease angle = raise claw
        self.hinge_motor.setVoltage(self.hinge_voltage)
