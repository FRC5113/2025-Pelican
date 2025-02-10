import math
from pathlib import Path

import wpilib
from phoenix6.hardware import CANcoder, TalonFX, Pigeon2
from rev import SparkMax, SparkLowLevel
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from wpilib import RobotController, DigitalInput, Encoder
from wpimath import units
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Transform3d, Rotation3d, Transform2d, Rotation2d
import magicbot
from magicbot import feedback
from photonlibpy.photonCamera import PhotonCamera

from lemonlib.control import LemonInput
from lemonlib.util import curve, Alert, AlertManager, AlertType
from lemonlib.preference import SmartPreference, SmartProfile
from lemonlib.vision import LemonCamera, LemonCameraSim

from components.odometry import Odometry
from components.swerve_drive import SwerveDrive
from components.swerve_wheel import SwerveWheel
from components.elevator import Elevator
from components.claw import Claw
from components.climber import Climber


class MyRobot(magicbot.MagicRobot):
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
        self.offset_x = 0.381
        self.offset_y = 0.381
        self.drive_gear_ratio = 6.75
        self.wheel_radius = 0.0508
        self.max_speed = 4.7

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
                "kP": 18.0,
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

        """
        ELEVATOR
        """

        # hardware
        BRUSHLESS = SparkLowLevel.MotorType.kBrushless
        self.elevator_right_motor = SparkMax(61, BRUSHLESS)
        self.elevator_left_motor = SparkMax(62, BRUSHLESS)
        self.elevator_right_encoder = self.elevator_right_motor.getEncoder()
        self.elevator_left_encoder = self.elevator_left_motor.getEncoder()
        self.elevator_upper_switch = DigitalInput(0)
        self.elevator_lower_switch = DigitalInput(1)

        # physical constants
        self.elevator_carriage_mass = 15.0
        self.elevator_min_height = 0.0254
        self.elevator_max_height = 2.032
        self.elevator_gearing = 10.0
        self.elevator_spool_radius = 0.0381

        # profile (estimated)
        self.elevator_profile = SmartProfile(
            "elevator",
            {
                "kP": 0.0,
                "kI": 0.0,
                "kD": 0.0,
                "kS": 0.0,
                "kV": 10.23,
                "kA": 0.02,
                "kG": 0.23,
                "kMaxV": 10.0,
                "kMaxA": 100.0,
            },
            not self.low_bandwidth,
        )

        """
        CLAW
        """

        # hardware
        self.claw_hinge_motor = SparkMax(63, BRUSHLESS)
        self.claw_left_motor = SparkMax(64, BRUSHLESS)
        self.claw_right_motor = SparkMax(65, BRUSHLESS)
        self.claw_hinge_encoder = self.claw_hinge_motor.getAbsoluteEncoder()

        # physical constants
        self.claw_gearing = 82.5
        self.claw_min_angle = 0.0
        self.claw_max_angle = 90.0

        # profile (estimated)
        self.claw_profile = SmartProfile(
            "claw",
            {
                "kP": 0.0,
                "kI": 0.0,
                "kD": 0.0,
                "kS": 0.0,
                "kV": 1.61,
                "kA": 0.0,
                "kG": 0.8,
                "kMaxV": 0.0,
                "kMaxA": 0.0,
            },
            not self.low_bandwidth,
        )

        """
        CLIMBER
        """

        # hardware
        self.climber_motor = TalonFX(51)
        self.climber_encoder = Encoder(3, 4)
        self.climber_winch_limit_switch = DigitalInput(5)
        self.climber_right_hook_limit_switch = DigitalInput(6)
        self.climber_left_hook_limit_switch = DigitalInput(7)

        self.pigeon = Pigeon2(30)

        """
        MISCELLANEOUS
        """

        # driving curve
        self.sammi_curve = curve(
            lambda x: 1.89 * x**3 + 0.61 * x, 0.0, deadband=0.1, max_mag=1.0
        )
        self.x_filter = SlewRateLimiter(self.slew_rate)
        self.y_filter = SlewRateLimiter(self.slew_rate)
        self.theta_filter = SlewRateLimiter(self.slew_rate)

        # odometry
        # self.field_layout = loadAprilTagLayoutField(AprilTagField.k2024Crescendo)
        # AprilTagFieldLayout(
        #     str(Path(__file__).parent.resolve() / "test_field.json")
        # )
        # if self.isSimulation():
        #     self.camera = LemonCameraSim(
        #         # , 120
        #         self.field_layout,
        #         120,
        #         Transform2d(0.2921, 0.384175, Rotation2d(0)),
        #     )
        # else:
        #     self.camera = LemonCamera(
        #         "Global_Shutter_Camera",
        #         Transform3d(
        #             0.0, 0.0, 0.0, Rotation3d(0, 0.523599, 0.0)
        #         ),  # Transform3d(0.2921, 0.384175, 0.26035, Rotation3d(0, -0.523599, 0)),
        #     )
        self.camera = PhotonCamera("Global_Shutter_Camera")
        self.robot_to_camera = Transform3d()
        self.field_layout = AprilTagFieldLayout.loadField(AprilTagField.k2025Reefscape)
        # self.theta_profile = SmartProfile(
        #     "theta",
        #     {"kP": 18.0, "kI": 0.0, "kD": 0.0, "kMinInput": -180, "kMaxInput": 180},
        #     not self.low_bandwidth,
        # )

        # alerts
        AlertManager(self.logger)
        if self.low_bandwidth:
            AlertManager.instant_alert(
                "Low Bandwidth Mode is active! Tuning is disabled.", AlertType.WARNING
            )

    def teleopPeriodic(self):
        controller = LemonInput(0)

        mult = 1
        if controller.lefttrigger() >= 0.8:
            mult *= 0.5
        if controller.righttrigger() >= 0.8:
            mult *= 0.5

        if controller.pov() >= 0:
            # use pov inputs to steer if present
            self.swerve_drive.drive(
                controller.pov_y() * mult * self.top_speed,
                -controller.pov_x() * mult * self.top_speed,
                -self.sammi_curve(self.theta_filter.calculate(controller.rightx()))
                * mult
                * self.top_omega,
                not controller.leftbumper(),
                self.period,
            )
        else:
            # otherwise steer with joysticks
            self.swerve_drive.drive(
                -self.sammi_curve(self.x_filter.calculate(controller.lefty()))
                * mult
                * self.top_speed,
                -self.sammi_curve(self.y_filter.calculate(controller.leftx()))
                * mult
                * self.top_speed,
                -self.sammi_curve(self.theta_filter.calculate(controller.rightx()))
                * mult
                * self.top_omega,
                not controller.leftbumper(),
                self.period,
            )

        if controller.xbutton():
            self.swerve_drive.reset_gyro()

        if controller.ybutton():
            self.elevator.move_manual(1.5)
        if controller.bbutton():
            self.elevator.move_manual(-1.5)

        if controller.rightbumper() and controller.ybutton():
            self.claw.set_intake(0.5)
        if controller.rightbumper() and controller.bbutton():
            self.claw.set_intake(-0.5)
        if controller.rightbumper() and controller.abutton():
            self.claw.hinge_manual_control(-0.5)
        if controller.rightbumper() and controller.xbutton():
            self.claw.hinge_manual_control(0.5)

        if controller.startbutton() and controller.ybutton():
            self.climber.move(0.5)
        if controller.startbutton() and controller.bbutton():
            self.climber.move(-0.5)
        if controller.startbutton() and controller.abutton():
            self.climber.move_manual(0.1)

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
