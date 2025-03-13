import math

from phoenix6 import controls
from phoenix6.units import ampere
from phoenix6.configs import TalonFXConfiguration, CurrentLimitsConfigs
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.signals import NeutralModeValue
from wpimath import units
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState


from magicbot import will_reset_to
from lemonlib.smart import SmartPreference, SmartProfile


class SwerveWheel:
    drive_gear_ratio: float
    wheel_radius: units.meters
    speed_motor: TalonFX
    speed_profile: SmartProfile
    direction_motor: TalonFX
    direction_profile: SmartProfile
    cancoder: CANcoder

    direction_amps: units.amperes
    speed_amps: units.amperes

    """Module must be explicitly told to move (via setDesiredState) each
    loop, otherwise it defaults to stopped for safety.
    """
    stopped = will_reset_to(True)
    angle_deadband = SmartPreference(0.0349)

    """
    INITIALIZATION METHODS
    """

    def setup(self) -> None:
        """
        This function is automatically called after the motors and encoders have been injected.
        """

        # apply configs
        self.motor_configs = TalonFXConfiguration()
        self.speed_current_limit_configs = CurrentLimitsConfigs()
        self.direction_current_limit_configs = CurrentLimitsConfigs()
        self.direction_current_limit_configs.stator_current_limit = self.direction_amps
        self.speed_current_limit_configs.stator_current_limit = self.speed_amps
        self.motor_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.direction_motor.configurator.apply(self.motor_configs)
        self.direction_motor.configurator.apply(self.direction_current_limit_configs)
        self.speed_motor.configurator.apply(self.motor_configs)
        self.speed_motor.configurator.apply(self.speed_current_limit_configs)

        self.desired_state = None

    def on_enable(self):
        self.speed_controller = self.speed_profile.create_flywheel_controller(
            f"{self.speed_motor.device_id}_speed"
        )
        self.direction_controller = self.direction_profile.create_turret_controller(
            f"{self.direction_motor.device_id}_direction"
        )

    """
    INFORMATIONAL METHODS
    """

    def getMeasuredState(self):
        """Retrieve list of measured angle and velocity
        (used for AdvantageScope)
        """

        return [
            self.cancoder.get_absolute_position().value * 360,
            self.speed_motor.get_velocity().value
            * (self.wheel_radius * 2 * math.pi)
            / self.drive_gear_ratio,
        ]

    def getPosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(
            self.speed_motor.get_position().value
            / self.drive_gear_ratio
            * (self.wheel_radius * 2 * math.pi),
            Rotation2d(self.cancoder.get_absolute_position().value * math.tau),
        )

    def getVoltage(self) -> units.volts:
        return self.speed_motor.get_motor_voltage().value/ self.drive_gear_ratio * (self.wheel_radius * 2 * math.pi)

    def getVelocity(self):
        return self.speed_motor.get_velocity().value / self.drive_gear_ratio * (self.wheel_radius * 2 * math.pi)

    """
    CONTROL METHODS
    """

    def setDesiredState(self, state: SwerveModuleState):
        self.stopped = False
        self.desired_state = state

    """
    EXECUTE
    """

    def execute(self) -> None:
        encoder_rotation = Rotation2d(
            self.cancoder.get_absolute_position().value * 2 * math.pi
        )

        if self.stopped:
            self.speed_motor.set_control(controls.static_brake.StaticBrake())
            self.direction_motor.set_control(controls.coast_out.CoastOut())
            self.speed_controller.calculate(0.0, 0.0)
            self.direction_controller.calculate(
                encoder_rotation.radians(), encoder_rotation.radians()
            )
            return

        state = self.desired_state
        state.optimize(encoder_rotation)
        # scale speed while turning
        state.speed *= (state.angle - encoder_rotation).cos()
        # convert speed from m/s to r/s
        state.speed *= self.drive_gear_ratio / (self.wheel_radius * 2 * math.pi)
        speed_output = self.speed_controller.calculate(
            self.speed_motor.get_velocity().value, state.speed
        )
        self.speed_motor.set_control(controls.VoltageOut(speed_output))

        direction_output = self.direction_controller.calculate(
            encoder_rotation.radians(),
            state.angle.radians(),
        )
        if abs(self.direction_controller.error) < 0.03:
            self.direction_motor.set_control(controls.static_brake.StaticBrake())
            return
        self.direction_motor.set_control(controls.VoltageOut(-direction_output))
