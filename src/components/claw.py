from rev import SparkMax, SparkBaseConfig, SparkAbsoluteEncoder, SparkLimitSwitch
from lemonlib.preference import SmartProfile
from magicbot import feedback, will_reset_to
from enum import Enum
from lemonlib.util import Alert, AlertType


class ClawAngle(float, Enum):
    # values likely inaccurate
    UP = -111.6
    DOWN = 7.6


class Claw:

    hinge_motor: SparkMax
    left_motor: SparkMax
    right_motor: SparkMax
    claw_profile: SmartProfile
    gearing: float
    min_angle: float
    max_angle: float
    hinge_encoder: SparkAbsoluteEncoder
    intake_limit: SparkLimitSwitch

    target_angle = will_reset_to(ClawAngle.UP)
    intake_left_motor_voltage = will_reset_to(0)
    intake_right_motor_voltage = will_reset_to(0)
    hinge_voltage = will_reset_to(0)
    hinge_manual_control = False

    """
    INITIALIZATION METHODS
    """

    def setup(self):
        self.right_motor.configure(
            SparkBaseConfig().setIdleMode(SparkBaseConfig.IdleMode.kCoast),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.left_motor.configure(
            SparkBaseConfig().setIdleMode(SparkBaseConfig.IdleMode.kCoast),
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
        self.controller = self.claw_profile.create_arm_controller(
            "claw"
        )  # using turret controller for claw until arm is made

    def on_disable(self):
        self.hinge_motor.configure(
            SparkBaseConfig().setIdleMode(SparkBaseConfig.IdleMode.kCoast),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

    """
    INFORMATIONAL METHODS
    """

    def get_position(self) -> float:
        return self.hinge_encoder.getPosition()

    @feedback
    def get_angle(self) -> float:
        """Return the angle of the hinge normalized to [-180,180]"""
        angle = self.hinge_encoder.getPosition() * 360
        if angle > 180:
            angle -= 360
        return angle

    @feedback
    def get_intake_limit(self) -> bool:
        return self.intake_limit.get()

    """
    CONTROL METHODS
    """

    def set_intake(self, left_voltage: float, right_voltage: float):
        self.intake_left_motor_voltage = left_voltage
        self.intake_right_motor_voltage = right_voltage

    def set_target_angle(self, angle: float):
        self.target_angle = angle
        self.hinge_manual_control = False

    def set_hinge_voltage(self, voltage: float):
        self.hinge_voltage = voltage
        self.hinge_manual_control = True

    """
    EXECUTE
    """

    def execute(self):
        if self.intake_limit.get() and (
            self.intake_left_motor_voltage < 0 and self.intake_right_motor_voltage < 0
        ):
            self.intake_left_motor_voltage = 0
            self.intake_right_motor_voltage = 0
        # positive voltage (left) = intake
        self.left_motor.set(self.intake_left_motor_voltage)
        self.right_motor.set(-self.intake_right_motor_voltage)

        if not self.hinge_manual_control:
            # calculate voltage from feedforward (only if voltage has not already been set)
            self.hinge_voltage = self.controller.calculate(
                self.get_angle(), self.target_angle.value
            )
        if (
            self.get_angle() - self.max_angle > 10
            or self.min_angle - self.get_angle() > 10
        ):
            self.hinge_alert.enable()
            self.hinge_alert.set_text(
                f"Claw hinge has rotated too far! Current angle: {self.get_angle()}"
            )
        else:
            self.hinge_alert.disable()
        if self.get_angle() > self.max_angle and self.hinge_voltage < 0:
            self.hinge_voltage = 0
        if self.get_angle() < self.min_angle and self.hinge_voltage > 0:
            self.hinge_voltage = 0
        # positive voltage = decrease angle = raise claw
        self.hinge_motor.setVoltage(self.hinge_voltage)
