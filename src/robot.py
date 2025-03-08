import math
from pathlib import Path

import wpilib
from wpilib import Field2d
from phoenix6.hardware import CANcoder, TalonFX, Pigeon2
from rev import SparkMax, SparkLowLevel
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from wpilib import (
    RobotController,
    DigitalInput,
    DutyCycleEncoder,
    DriverStation,
    RobotBase,
)
from wpimath import units, applyDeadband
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Transform3d, Pose2d, Translation2d, Rotation2d, Rotation3d
from photonlibpy.photonCamera import PhotonCamera
import magicbot
from magicbot import feedback

from lemonlib.control import LemonInput
from lemonlib.util import curve, AlertManager, AlertType, LEDController,SnapX,SnapY
from lemonlib.preference import SmartPreference, SmartProfile

from components.odometry import Odometry
from components.swerve_drive import SwerveDrive
from components.swerve_wheel import SwerveWheel
from components.elevator import Elevator, ElevatorHeight
from components.claw import Claw, ClawAngle
from components.climber import Climber
from components.arm_control import ArmControl
from components.drive_control import DriveControl


class MyRobot(magicbot.MagicRobot):
    arm_control: ArmControl
    drive_control: DriveControl
    odometry: Odometry

    swerve_drive: SwerveDrive
    front_left: SwerveWheel
    front_right: SwerveWheel
    rear_left: SwerveWheel
    rear_right: SwerveWheel
    elevator: Elevator
    claw: Claw
    climber: Climber

    low_bandwidth = False
    # greatest speed that chassis should move (not greatest possible speed)
    top_speed = SmartPreference(3.0)
    top_omega = SmartPreference(6.0)
    slew_rate = SmartPreference(5.0)

    def createObjects(self):
        """This method is where all attributes to be injected are
        initialized. This is done here rather that inside the components
        themselves so that all constants and initialization parameters
        can be found in one place. Also, attributes shared by multiple
        components, such as the NavX, need only be created once.
        """

        """
        SWERVE
        """

        # hardware
        self.front_left_speed_motor = TalonFX(21)
        self.front_left_direction_motor = TalonFX(22)
        self.front_left_cancoder = CANcoder(23)

        self.front_right_speed_motor = TalonFX(31)
        self.front_right_direction_motor = TalonFX(32)
        self.front_right_cancoder = CANcoder(33)

        self.rear_left_speed_motor = TalonFX(11)
        self.rear_left_direction_motor = TalonFX(12)
        self.rear_left_cancoder = CANcoder(13)

        self.rear_right_speed_motor = TalonFX(41)
        self.rear_right_direction_motor = TalonFX(42)
        self.rear_right_cancoder = CANcoder(43)

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
        self.elevator_right_motor = SparkMax(59, BRUSHLESS)
        self.elevator_left_motor = SparkMax(57, BRUSHLESS)
        self.elevator_right_encoder = self.elevator_right_motor.getEncoder()
        self.elevator_left_encoder = self.elevator_left_motor.getEncoder()
        self.elevator_upper_switch = DigitalInput(0)
        self.elevator_lower_switch = DigitalInput(1)

        # physical constants
        self.elevator_carriage_mass = 15.0
        self.elevator_min_height = 0.0254
        self.elevator_max_height = 2.032
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
        self.claw_hinge_motor = SparkMax(56, BRUSHLESS)
        self.claw_left_motor = SparkMax(55, SparkLowLevel.MotorType.kBrushed)
        self.claw_right_motor = SparkMax(58, SparkLowLevel.MotorType.kBrushed)
        self.claw_hinge_encoder = self.claw_hinge_motor.getAbsoluteEncoder()
        self.claw_intake_limit = self.claw_left_motor.getReverseLimitSwitch()

        # physical constants
        self.claw_gearing = 82.5
        self.claw_max_angle: units.degrees = 119.2

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
        self.climber_motor = TalonFX(51)
        self.climber_encoder = DutyCycleEncoder(2)

        """
        ODOMETRY
        """

        self.camera = PhotonCamera("USB_Camera")
        self.robot_to_camera = Transform3d(0.0, 0.0, 0.0, Rotation3d(0.0, 0.0, math.pi))
        self.field_layout = AprilTagFieldLayout(
            str(Path(__file__).parent.resolve() / "test_reef.json")
        )
        # self.field_layout = AprilTagFieldLayout.loadField(
        #     AprilTagField.k2025ReefscapeAndyMark
        # )

        """
        MISCELLANEOUS
        """

        self.period: units.seconds = 0.02

        # self.primary = PS5Controller(0)
        # self.secondary = XboxController(1)

        self.leds = LEDController(0, 150)  # have to check amount of leds

        self.pigeon = Pigeon2(30)

        self.fms = DriverStation.isFMSAttached()

        # driving curve
        self.sammi_curve = curve(
            lambda x: 1.89 * x**3 + 0.61 * x, 0.0, deadband=0.1, max_mag=1.0
        )

        # alerts
        AlertManager(self.logger)
        if self.low_bandwidth:
            AlertManager.instant_alert(
                "Low Bandwidth Mode is active! Tuning is disabled.", AlertType.WARNING
            )

        self.estimated_field = Field2d()

    def teleopInit(self):
        # initialize HIDs here in case they are changed after robot initializes
        if RobotBase.isSimulation():
            self.primary = LemonInput(0)
            self.secondary = LemonInput(1)
        else:
            self.primary = LemonInput(type="PS5")
            self.secondary = LemonInput(type="Xbox")

        self.x_filter = SlewRateLimiter(self.slew_rate)
        self.y_filter = SlewRateLimiter(self.slew_rate)
        self.theta_filter = SlewRateLimiter(self.slew_rate)


    def teleopPeriodic(self):
        with self.consumeExceptions():

            """
            SWERVE
            """

            if self.primary.getR1Button():
                # SAMMI: Replace -54.0 with 126.0 to flip orientiation
                self.swerve_drive.set_pigeon_offset(-54.0)
                self.getLefty = SnapY(
                    self.primary.getLeftX(), self.primary.getLeftY()
                )
                self.getLeftX = SnapX(
                    self.primary.getLeftX(), self.primary.getLeftY()
                )
            else:
                self.swerve_drive.set_pigeon_offset(180.0)
                self.getLefty = self.primary.getLeftY()
                self.getLeftx = self.primary.getLeftX()

            mult = 1
            if self.primary.getR2Axis() >= 0.8:
                mult *= 0.5
            if self.primary.getL2Axis() >= 0.8:
                mult *= 0.5
            mult *= self.arm_control.get_drive_scalar()

            self.drive_control.engage()
            self.drive_control.drive_manual(
                self.x_filter.calculate(
                    -self.sammi_curve(self.getLefty) * mult * self.top_speed
                ),
                self.y_filter.calculate(
                    -self.sammi_curve(self.getLeftx) * mult * self.top_speed
                ),
                self.theta_filter.calculate(
                    -self.sammi_curve(self.primary.getRightX()) * self.top_omega
                ),
                not self.primary.getCreateButton(),  # temporary
            )
            if self.primary.getPOV() == 90:
                self.drive_control.request_pose(
                    Pose2d(Translation2d(-0.35, -0.18), Rotation2d())
                )
            if self.primary.getPOV() == 270:
                self.drive_control.request_pose(
                    Pose2d(Translation2d(-0.35, 0.19), Rotation2d())
                )

            if self.primary.getPOV() == 0:
                self.drive_control.request_remove_algae(
                    ElevatorHeight.L1, ClawAngle.TROUGH, self.period
                )
            if self.primary.getPOV() == 180:
                self.drive_control.request_remove_algae(
                    ElevatorHeight.L2, ClawAngle.TROUGH, self.period
                )

            if self.primary.getSquareButton():
                self.swerve_drive.reset_gyro()

            """
            ARM
            """

            self.arm_control.engage()
            if self.elevator.error_detected():
                self.arm_control.next_state("elevator_failsafe")
            # self.elevator.set_voltage(
            #     -1.5 * applyDeadband(self.secondary.getRightY(), 0.1)
            # )
            if self.secondary.getAButton():
                self.arm_control.set(ElevatorHeight.L1, ClawAngle.TROUGH)
            if self.secondary.getBButton():
                self.arm_control.set(ElevatorHeight.L2, ClawAngle.BRANCH)
            if self.secondary.getXButton():
                self.arm_control.set(ElevatorHeight.L3, ClawAngle.BRANCH)
            if self.secondary.getYButton():
                self.arm_control.set(ElevatorHeight.L4, ClawAngle.BRANCH)
            if self.secondary.getStartButton():
                self.arm_control.set(ElevatorHeight.STATION, ClawAngle.STATION)

            if self.secondary.getLeftTriggerAxis() > 0.9:
                self.arm_control.set_wheel_voltage(
                    6.0 * applyDeadband(self.secondary.getLeftTriggerAxis(), 0.1)
                )
            if self.secondary.getRightTriggerAxis() > 0.9:
                self.arm_control.set_wheel_voltage(
                    6.0 * applyDeadband(-self.secondary.getRightTriggerAxis(), 0.1)
                )

            # if self.secondary.getLeftBumper():
            #     self.claw.set_hinge_voltage(-1)
            # if self.secondary.getRightBumper():
            #     self.claw.set_hinge_voltage(1)

            """
            CLIMBER
            """

            if self.primary.getTriangleButton():
                self.climber.set_speed(1)
            if self.primary.getCrossButton():
                self.climber.set_speed(-1)

    @feedback
    def get_voltage(self) -> units.volts:
        return RobotController.getBatteryVoltage()

    # override _do_periodics() to access watchdog
    # DON'T DO ANYTHING ELSE HERE UNLESS YOU KNOW WHAT YOU'RE DOING
    def _do_periodics(self):
        super()._do_periodics()
        self.period = max(0.02, self.watchdog.getTime())


if __name__ == "__main__":
    wpilib.run(MyRobot)
