import math
from pathlib import Path

import wpilib
from wpilib import (
    Field2d,
    SmartDashboard,
    DataLogManager,
    Mechanism2d,
    Color8Bit,
    RobotController,
    DigitalInput,
    DutyCycleEncoder,
    DriverStation,
    PowerDistribution,
    PWM
)
from wpilib import RobotController

from wpimath import units, applyDeadband
from wpimath.geometry import (
    Transform3d,
    Transform2d,
    Rotation2d,
    Rotation3d,
)


from phoenix6.hardware import CANcoder, TalonFX, Pigeon2
from phoenix6 import CANBus
from rev import SparkMax, SparkLowLevel
from robotpy_apriltag import AprilTagFieldLayout

from magicbot import feedback

from lemonlib import LemonInput
from lemonlib.util import (
    curve,
    AlertManager,
    AlertType,
    LEDController,
    SnapX,
    SnapY,
)
from lemonlib.smart import SmartPreference, SmartProfile
from lemonlib import LemonRobot
from lemonlib.util import AsymmetricSlewLimiter

# from autonomous.auto_base import AutoBase
from components.swerve_drive import SwerveDrive
from components.swerve_wheel import SwerveWheel
from components.elevator import Elevator, ElevatorHeight
from components.claw import Claw, ClawAngle
from components.climber import Climber
from components.arm_control import ArmControl
from components.drive_control import DriveControl
from components.leds import LEDStrip
from components.sysid_drive import SysIdDriveLinear

class MyRobot(LemonRobot):
    sysid_drive: SysIdDriveLinear
    drive_control: DriveControl
    arm_control: ArmControl
    led_strip: LEDStrip

    swerve_drive: SwerveDrive
    front_left: SwerveWheel
    front_right: SwerveWheel
    rear_left: SwerveWheel
    rear_right: SwerveWheel
    elevator: Elevator
    claw: Claw
    climber: Climber

    # greatest speed that chassis should move (not greatest possible speed)
    top_speed = SmartPreference(3.0)
    top_omega = SmartPreference(6.0)

    rasing_slew_rate = SmartPreference(5.0)
    falling_slew_rate = SmartPreference(5.0)

    keaton_mode = SmartPreference(False)

    def createObjects(self):
        """This method is where all attributes to be injected are
        initialized. This is done here rather that inside the components
        themselves so that all constants and initialization parameters
        can be found in one place. Also, attributes shared by multiple
        components, such as the NavX, need only be created once.
        """

        self.canicore_canbus = CANBus("can0")
        self.ctre_canbus = CANBus.system_core(0)
        self.rev_canbus = 0

        """
        SWERVE
        """

        # hardware
        self.front_left_speed_motor = TalonFX(21, self.canicore_canbus)
        self.front_left_direction_motor = TalonFX(22, self.canicore_canbus)
        self.front_left_cancoder = CANcoder(23, self.canicore_canbus)

        self.front_right_speed_motor = TalonFX(31, self.canicore_canbus)
        self.front_right_direction_motor = TalonFX(32, self.canicore_canbus)
        self.front_right_cancoder = CANcoder(33, self.canicore_canbus)

        self.rear_left_speed_motor = TalonFX(11, self.canicore_canbus)
        self.rear_left_direction_motor = TalonFX(12, self.canicore_canbus)
        self.rear_left_cancoder = CANcoder(13, self.canicore_canbus)

        self.rear_right_speed_motor = TalonFX(41, self.canicore_canbus)
        self.rear_right_direction_motor = TalonFX(42, self.canicore_canbus)
        self.rear_right_cancoder = CANcoder(43, self.canicore_canbus)

        # physical constants
        self.offset_x: units.meters = 0.381
        self.offset_y: units.meters = 0.381
        self.drive_gear_ratio = 6.75
        self.wheel_radius: units.meters = 0.0508
        self.max_speed: units.meters_per_second = 4.7
        self.direction_amps: units.amperes = 40.0
        self.speed_amps: units.amperes = 60.0

        # profiles
        self.speed_profile = SmartProfile(
            "speed",
            {
                "kP": 0.0,
                "kI": 0.0,
                "kD": 0.0,
                "kS": 0.17,
                "kV": 0.104,
                "kA": 0.01,
            },
            not self.low_bandwidth,
        )
        self.direction_profile = SmartProfile(
            "direction",
            {
                "kP": 3.0,
                "kI": 0.0,
                "kD": 0.0,
                "kS": 0.14,
                "kV": 0.375,
                "kA": 0.0,
                "kMaxV": 400.0,
                "kMaxA": 4000.0,
                "kMinInput": -math.pi,
                "kMaxInput": math.pi,
            },
            not self.low_bandwidth,
        )
        self.translation_profile = SmartProfile(
            "translation",
            {
                "kP": 1.0,
                "kI": 0.0,
                "kD": 0.0,
            },
            not self.low_bandwidth,
        )
        self.rotation_profile = SmartProfile(
            "rotation",
            {
                "kP": 0.0,
                "kI": 0.0,
                "kD": 0.0,
                "kMaxV": 10.0,
                "kMaxA": 100.0,
                "kMinInput": -math.pi,
                "kMaxInput": math.pi,
            },
            not self.low_bandwidth,
        )

        """
        ELEVATOR
        """

        # hardware
        BRUSHLESS = SparkLowLevel.MotorType.kBrushless
        self.elevator_right_motor = SparkMax(self.rev_canbus, 59, BRUSHLESS)
        self.elevator_left_motor = SparkMax(self.rev_canbus, 57, BRUSHLESS)
        self.elevator_right_encoder = self.elevator_right_motor.getEncoder()
        self.elevator_left_encoder = self.elevator_left_motor.getEncoder()
        self.elevator_upper_switch = DigitalInput(0)
        self.elevator_lower_switch = DigitalInput(1)

        # physical constants
        self.elevator_carriage_mass = 15.0
        self.elevator_min_height = 0.0
        self.elevator_max_height = 0.7
        self.elevator_gearing = 10.0
        self.elevator_spool_radius: units.meters = 0.0223

        # profile (estimated)
        self.elevator_profile = SmartProfile(
            "elevator",
            {
                "kP": 50.0,
                "kI": 0.0,
                "kD": 0.0,
                "kS": 0.0,
                "kV": 0.0,
                "kA": 0.0,
                "kG": 0.0,
                "kMaxV": 2.0,
                "kMaxA": 20.0,
            },
            not self.low_bandwidth,
        )
        self.elevator_tolerance = 0.02

        """
        CLAW
        """

        # hardware
        self.claw_hinge_motor = SparkMax(self.rev_canbus, 56, BRUSHLESS)
        self.claw_left_motor = SparkMax(self.rev_canbus, 55, SparkLowLevel.MotorType.kBrushed)
        self.claw_right_motor = SparkMax(self.rev_canbus, 58, SparkLowLevel.MotorType.kBrushed)
        self.claw_hinge_encoder = self.claw_hinge_motor.getAbsoluteEncoder()
        self.claw_intake_limit = self.claw_left_motor.getReverseLimitSwitch()

        # physical constants
        self.claw_gearing = 82.5

        # profile (estimated)
        self.claw_profile = SmartProfile(
            "claw",
            {
                "kP": 0.15,
                "kI": 0.0,
                "kD": 0.0,
                "kS": 0.0,
                "kV": 0.0,
                "kA": 0.0,
                "kG": 0.0,
                "kMaxV": 150.0,
                "kMaxA": 500.0,
            },
            not self.low_bandwidth,
        )
        self.claw_hinge_tolerance = 3.0

        """
        CLIMBER
        """

        # hardware
        self.climber_motor = TalonFX(51, self.ctre_canbus)
        self.climber_encoder = DutyCycleEncoder(2)


        """
        MISCELLANEOUS
        """

        # self.period: units.seconds = 0.02
        self.led_length = 112
        self.leds = LEDController(3, self.led_length)  # broken amount is 46

        self.pigeon = Pigeon2(30, self.canicore_canbus)

        self.fms = DriverStation.isFMSAttached()

        # driving curve
        self.sammi_curve = curve(
            lambda x: 1.89 * x**3 + 0.61 * x, 0.0, deadband=0.1, max_mag=1.0
        )

        # alerts
        AlertManager(self.logger)
        if self.low_bandwidth:
            AlertManager.instant_alert(
                "Low Bandwidth Mode is active! Tuning is disabled.", AlertType.INFO
            )

        self.pdh = PowerDistribution(self.rev_canbus)

        self.estimated_field = Field2d()

        self.arm_visuize = Mechanism2d(20, 50)
        self.arm_root = self.arm_visuize.getRoot("Arm Root", 10, 0)
        self.elevator_ligament = self.arm_root.appendLigament("Elevator", 5, 90)
        self.claw_ligament = self.elevator_ligament.appendLigament(
            "Claw", 5, 0, color=Color8Bit(0, 150, 0)
        )
        SmartDashboard.putData("Arm", self.arm_visuize)
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self.alliance = True
        else:
            self.alliance = False

    def disabledPeriodic(self):
        # self.odometry.execute()
        self.swerve_drive.execute()
        # self.leds.move_across((5, 5, 0), 20, 20)
        self.led_strip.hollows_eve_disabled()

    def enabledperiodic(self):
        self.drive_control.engage()
        self.arm_control.engage()

    def autonomousInit(self):
        # if DriverStation.isFMSAttached():
        DataLogManager.start()

    def autonomousPeriodic(self):
        pass
        # self._display_auto_trajectory()

    def teleopInit(self):
        # initialize HIDs here in case they are changed after robot initializes
        self.primary = LemonInput(0, "PS5")
        self.secondary = LemonInput(1, "Xbox")
        # self.sysid_con = LemonInput(2)

        self.x_filter = AsymmetricSlewLimiter(
            self.rasing_slew_rate, self.falling_slew_rate
        )
        self.y_filter = AsymmetricSlewLimiter(
            self.rasing_slew_rate, self.falling_slew_rate
        )
        self.theta_filter = AsymmetricSlewLimiter(
            self.rasing_slew_rate, self.falling_slew_rate
        )

        self.upper_algae_button_released = True
        self.lower_algae_button_released = True

    def teleopPeriodic(self):
        print(f"Lower: {self.elevator_lower_switch.get()}, Upper: {self.elevator_upper_switch.get()}")
        with self.consumeExceptions():

            """
            SWERVE
            """

            if self.primary.getR1Button():
                # SAMMI: Replace -54.0 with 126.0 to flip orientiation
                self.swerve_drive.set_pigeon_offset(126.0)
                self.getLefty = SnapY(self.primary.getLeftX(), self.primary.getLeftY())
                self.getLeftx = SnapX(self.primary.getLeftX(), self.primary.getLeftY())
            elif self.primary.getL1Button():
                self.swerve_drive.set_pigeon_offset(-126.0)
                self.getLefty = SnapY(self.primary.getLeftX(), self.primary.getLeftY())
                self.getLeftx = SnapX(self.primary.getLeftX(), self.primary.getLeftY())
            else:
                self.swerve_drive.set_pigeon_offset(0.0)
                self.getLefty = self.primary.getLeftY()
                self.getLeftx = self.primary.getLeftX()

            rotate_mult = 0.75
            mult = 1
            if self.primary.getR2Axis() >= 0.8:
                mult *= 0.5
            if self.primary.getL2Axis() >= 0.8:
                mult *= 0.5
            mult *= self.arm_control.get_drive_scalar()
            self.omega = self.theta_filter.calculate(
                -self.sammi_curve(self.primary.getRightX())
                * rotate_mult
                * self.top_omega
            )
            self.drive_control.drive_manual(
                self.x_filter.calculate(
                    self.sammi_curve(self.getLefty) * mult * self.top_speed
                ),
                self.y_filter.calculate(
                    self.sammi_curve(self.getLeftx) * mult * self.top_speed
                ),
                self.omega,
                not self.primary.getCreateButton(),  # temporary
            )

            # # algae removal
            # if self.primary.getPOV() == 180:
            #     self.lower_algae_button_released = False
            #     self.drive_control.request_remove_algae(ElevatorHeight.L1, True)
            # elif not self.lower_algae_button_released:
            #     self.lower_algae_button_released = True
            #     self.drive_control.request_remove_algae(ElevatorHeight.L1, False)
            # if self.primary.getPOV() == 0:
            #     self.upper_algae_button_released = False
            #     self.drive_control.request_remove_algae(ElevatorHeight.L2, True)
            # elif not self.upper_algae_button_released:
            #     self.upper_algae_button_released = True
            #     self.drive_control.request_remove_algae(ElevatorHeight.L2, False)

            if self.primary.getSquareButton():
                self.swerve_drive.reset_gyro()

        with self.consumeExceptions():
            """
            ARM
            """
            if self.elevator.error_detected():
                self.arm_control.next_state("elevator_failsafe")
            if self.secondary.getAButton():
                self.arm_control.set(ElevatorHeight.L1, ClawAngle.TROUGH)
            if self.secondary.getBButton():
                self.arm_control.set(ElevatorHeight.L2, ClawAngle.BRANCH)
            if self.secondary.getXButton():
                self.arm_control.set(ElevatorHeight.L3, ClawAngle.BRANCH)
            if self.secondary.getYButton():
                self.arm_control.set(ElevatorHeight.L4, ClawAngle.BRANCH)

            if self.secondary.getStartButton():
                self.arm_control.set(
                    ElevatorHeight.STATION_CLOSE, ClawAngle.STATION_CLOSE
                )
            if self.secondary.getBackButton():
                self.arm_control.set(ElevatorHeight.STATION_FAR, ClawAngle.STATION_FAR)

            if self.secondary.getPOV() == 0:
                self.arm_control.set(ElevatorHeight.L3, ClawAngle.TROUGH)

            if self.secondary.getLeftTriggerAxis() > 0.5:
                self.arm_control.set_wheel_voltage(
                    8.0 * applyDeadband(self.secondary.getLeftTriggerAxis(), 0.1)
                )
            if self.secondary.getRightTriggerAxis() > 0.5:
                self.arm_control.set_wheel_voltage(
                    6.0 * applyDeadband(-self.secondary.getRightTriggerAxis(), 0.1)
                )

            if self.secondary.getRightBumper():
                self.arm_control.set_wheel_voltage(-1)

            self.elevator_ligament.setLength((self.elevator.get_height() * 39.37) + 5)
            self.claw_ligament.setAngle(self.claw.get_angle() - 90)

        with self.consumeExceptions():
            """
            CLIMBER
            """

            if self.primary.getTriangleButton():
                self.climber.set_speed(-1)
            if self.primary.getCrossButton():
                self.climber.set_speed(1)

        with self.consumeExceptions():
            """
            MISC
            """
            if self.secondary.getPOV() == 90:
                self.arm_control.next_state_now("positioning_claw")
                self.led_strip.justin_fun()

            if self.primary.getYButton():
                self.led_strip.commandtest()

        # """
        # SYS-ID
        # """
        # if self.sysid_con.getAButton():
        #     self.sysid_drive.quasistatic_forward()
        # if self.sysid_con.getBButton():
        #     self.sysid_drive.quasistatic_reverse()
        # if self.sysid_con.getXButton():
        #     self.sysid_drive.dynamic_forward()
        # if self.sysid_con.getYButton():
        #     self.sysid_drive.dynamic_reverse()

    @feedback
    def get_voltage(self) -> units.volts:
        return RobotController.getBatteryVoltage()

    # def _display_auto_trajectory(self) -> None:
    #     selected_auto = self._automodes.chooser.getSelected()
    #     if isinstance(selected_auto, AutoBase):
    #         selected_auto.display_trajectory()

    # @feedback
    # def display_auto_state(self) -> None:
    #     selected_auto = self._automodes.chooser.getSelected()
    #     if isinstance(selected_auto, AutoBase):
    #         return selected_auto.current_state
    #     return "No Auto Selected"


if __name__ == "__main__":
    wpilib.run(MyRobot)
