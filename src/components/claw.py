from rev import SparkMax, SparkBaseConfig, SparkAbsoluteEncoder
from lemonlib.preference import SmartProfile
from magicbot import feedback, will_reset_to
from enum import Enum
from lemonlib.util import Alert, AlertType, AlertManager


class ClawAngle(float, Enum):
    # values likely inaccurate
    UP = 5.0
    DOWN = 111.2


class Claw:

    hinge_motor: SparkMax
    left_motor: SparkMax
    right_motor: SparkMax
    claw_profile: SmartProfile
    gearing: float
    min_angle: float
    max_angle: float
    hinge_encoder: SparkAbsoluteEncoder

    target_angle = will_reset_to(ClawAngle.UP)
    intake_left_motor_voltage = will_reset_to(0)
    intake_right_motor_voltage = will_reset_to(0)
    hinge_motor_voltage = will_reset_to(0)
    hinge_manual_control = False

    """
    INITIALIZATION METHODS
    """

    def setup(self):
        self.hinge_motor.configure(
            SparkBaseConfig().setIdleMode(SparkBaseConfig.IdleMode.kBrake),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
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

    def on_enable(self):
        self.controller = self.claw_profile.create_turret_controller(
            "claw"
        )  # using turret controller for claw until arm is made

    """
    INFORMATIONAL METHODS
    """

    def get_position(self) -> float:
        return self.hinge_encoder.getPosition()

    @feedback
    def get_angle(self) -> float:
        angle = self.hinge_encoder.getPosition() * 360
        if angle > 180:
            angle -= 360
        return angle

    """
    CONTROL METHODS
    """

    def set_intake(self, left_voltage: float, right_voltage: float):
        self.intake_left_motor_voltage = left_voltage
        self.intake_right_motor_voltage = right_voltage

    def set_target_angle(self, angle: float):
        self.target_angle = angle
        self.hinge_manual_control = False

    def hinge_move_manual(self, voltage: float):
        self.hinge_motor_voltage = voltage
        self.hinge_manual_control = True

    """
    EXECUTE
    """

    def execute(self):
        self.left_motor.set(self.intake_left_motor_voltage)
        self.right_motor.set(-self.intake_right_motor_voltage)
        if not self.hinge_manual_control:
            # calculate voltage from feedforward (only if voltage has not already been set)
            self.hinge_motor_voltage = self.controller.calculate(
                self.get_angle(), self.target_angle.value
            )
        # will eventually need to be tweaked!!!
        # if self.get_angle() < self.max_angle and self.get_angle() > self.min_angle:
        #     self.hinge_motor.set(self.hinge_motor_voltage)
        # else:
        #     self.hinge_motor.stopMotor()
        if (
            self.get_angle() - self.max_angle > 18
            or self.min_angle - self.get_angle() > 18
        ):
            AlertManager.instant_alert(
                f"The motor has exceded max/min bounds, the angle is {self.get_angle()} degrees",
                AlertType.ERROR,
            )
        # negative voltage = increase angle
        if self.get_angle() > self.max_angle and self.hinge_motor_voltage < 0:
            return
        if self.get_angle() < self.min_angle and self.hinge_motor_voltage > 0:
            return
        self.hinge_motor.setVoltage(self.hinge_motor_voltage)
