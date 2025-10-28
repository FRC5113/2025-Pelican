import math
from pathlib import Path

import wpilib
from wpilib import (
    Field2d,
    SmartDashboard,
    DataLogManager,
    CameraServer,
    Mechanism2d,
    MechanismLigament2d,
    Color8Bit,
    RobotController,
    DigitalInput,
    DutyCycleEncoder,
    DriverStation,
    RobotBase,
    PowerDistribution,
)

from wpimath import units, applyDeadband
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import (
    Transform3d,
    Pose2d,
    Transform2d,
    Rotation2d,
    Rotation3d,
    Translation2d,
)

from wpinet import WebServer

from phoenix6.hardware import CANcoder, TalonFX, Pigeon2
from rev import SparkMax, SparkLowLevel
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout

import magicbot
from magicbot import feedback

from lemonlib import LemonInput, LemonCamera
from lemonlib.util import (
    curve,
    AlertManager,
    AlertType,
    LEDController,
    SnapX,
    SnapY,
    is_red,
)
from lemonlib.smart import SmartPreference, SmartProfile

from autonomous.auto_base import AutoBase
from components.odometry import Odometry
from components.swerve_drive import SwerveDrive
from components.swerve_wheel import SwerveWheel
from components.elevator import Elevator, ElevatorHeight
from components.claw import Claw, ClawAngle
from components.climber import Climber
from components.arm_control import ArmControl
from components.drive_control import DriveControl
from components.leds import LEDStrip
from components.sysid_drive import SysIdDriveLinear

from lemonlib import LemonRobot, fms_feedback
from lemonlib.util import get_file

from lemonlib.funnies.funnycontroller import Funnycontrollers


class MyRobot(LemonRobot):
    sysid_drive: SysIdDriveLinear
    drive_control: DriveControl
    arm_control: ArmControl
    odometry: Odometry
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
    slew_rate = SmartPreference(5.0)

    keaton_mode = SmartPreference(False)

    funny_mode = SmartPreference(False)

    reset_gyro = SmartPreference(False)

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
        self.front_left_speed_motor = TalonFX(21,canbus="can0")
        self.front_left_direction_motor = TalonFX(22,canbus="can0")
        self.front_left_cancoder = CANcoder(23,canbus="can0")

        self.front_right_speed_motor = TalonFX(31,canbus="can0")
        self.front_right_direction_motor = TalonFX(32,canbus="can0")
        self.front_right_cancoder = CANcoder(33,canbus="can0")

        self.rear_left_speed_motor = TalonFX(11,canbus="can0")
        self.rear_left_direction_motor = TalonFX(12,canbus="can0")
        self.rear_left_cancoder = CANcoder(13,canbus="can0")

        self.rear_right_speed_motor = TalonFX(41,canbus="can0")
        self.rear_right_direction_motor = TalonFX(42,canbus="can0")
        self.rear_right_cancoder = CANcoder(43,canbus="can0")

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
        self.claw_hinge_motor = SparkMax(56, BRUSHLESS)
        self.claw_left_motor = SparkMax(55, SparkLowLevel.MotorType.kBrushed)
        self.claw_right_motor = SparkMax(58, SparkLowLevel.MotorType.kBrushed)
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
        self.climber_motor = TalonFX(51)
        self.climber_encoder = DutyCycleEncoder(2)

        """
        ODOMETRY
        """
        self.robot_to_camera_front = Transform3d(
            -0.2286,
            0.0,
            0.2667,
            Rotation3d(0.0, 0.0, math.pi),
        )
        self.robot_to_camera_back = Transform3d(
            -0.0381,
            0.0,
            0.762,
            Rotation3d(0.0, math.pi / 6, 0.0),
        )

        self.field_layout = AprilTagFieldLayout(
            str(Path(__file__).parent.resolve() / "2025_test_field.json")
        )
        # self.field_layout = AprilTagFieldLayout.loadField(
        #     AprilTagField.k2025ReefscapeWelded
        # )

        self.camera_front = LemonCamera(
            "Global_Shutter_Camera", self.robot_to_camera_front, self.field_layout
        )
        self.camera_back = LemonCamera(
            "USB_Camera", self.robot_to_camera_back, self.field_layout
        )

        """
        MISCELLANEOUS
        """

        # self.period: units.seconds = 0.02

        self.leds = LEDController(0, 112)  # broken amount is 46

        self.pigeon = Pigeon2(30,canbus="can0")

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

        self.pdh = PowerDistribution()

        self.estimated_field = Field2d()
        # CameraServer().launch()

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
        # self.webserver = WebServer.getInstance()
        # if not DriverStation.isFMSAttached():
        #     self.webserver.start(
        #         port=5800,
        #         path=str(
        #             Path(__file__).parent.resolve() / "deploy/elastic-layout.json"
        #         ),
        #     )

    def disabledPeriodic(self):
        self.odometry.execute()
        self.swerve_drive.execute()
        self.leds.move_across((5, 5, 0), 20, 20)

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
        self.primary = LemonInput(0)
        self.secondary = LemonInput(1)
        self.test = LemonInput(2)

        # self.commandprimary = CommandLemonInput(0)

        self.x_filter = SlewRateLimiter(self.slew_rate)
        self.y_filter = SlewRateLimiter(self.slew_rate)
        self.theta_filter = SlewRateLimiter(self.slew_rate)

        self.upper_algae_button_released = True
        self.lower_algae_button_released = True

        self.funny_controllers = Funnycontrollers(0,1)
        

    def teleopPeriodic(self):
        if self.test.getAButton():
            self.swerve_drive.break_wheel()
        with self.consumeExceptions():

            """
            SWERVE
            """
            self.swerve_drive.set_pigeon_offset(0.0)
            self.getLefty = self.funny_controllers.get_leftY_avg()
            self.getLeftx = self.funny_controllers.get_leftX_avg()



            rotate_mult = 0.75
            mult = 1
            mult *= self.arm_control.get_drive_scalar()
            keaton_mode = self.keaton_mode
            if keaton_mode:
                self.omega = self.swerve_drive.point_towards(
                    applyDeadband(self.primary.getRightX(), 0.3), applyDeadband(self.primary.getRightY(), 0.3)
                )
            else:
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

            if self.reset_gyro:
                self.swerve_drive.reset_gyro()

        with self.consumeExceptions():
            """
            ARM
            """
            if self.elevator.error_detected():
                self.arm_control.next_state("elevator_failsafe")
            if self.funny_controllers.get_switched_controller(RobotController.getTime() / 1000000).getAButton():
                self.arm_control.set(ElevatorHeight.L1, ClawAngle.TROUGH)

            if self.funny_controllers.get_switched_controller(RobotController.getTime() / 1000000).getBButton():
                self.arm_control.set(ElevatorHeight.L2, ClawAngle.BRANCH)

            if self.funny_controllers.get_switched_controller(RobotController.getTime() / 1000000).getXButton():
                self.arm_control.set(ElevatorHeight.L3, ClawAngle.BRANCH)

            if self.funny_controllers.get_switched_controller(RobotController.getTime() / 1000000).getYButton():
                self.arm_control.set(ElevatorHeight.L4, ClawAngle.BRANCH)

            if self.funny_controllers.get_switched_controller(RobotController.getTime() / 1000000).getStartButton():
                self.arm_control.set(
                    ElevatorHeight.STATION_CLOSE, ClawAngle.STATION_CLOSE
                )
            if self.funny_controllers.get_switched_controller(RobotController.getTime() / 1000000).getBackButton():
                self.arm_control.set(ElevatorHeight.STATION_FAR, ClawAngle.STATION_FAR)

            if self.funny_controllers.get_switched_controller(RobotController.getTime() / 1000000).getLeftTriggerAxis() > 0.5:
                self.arm_control.set_wheel_voltage(
                    8.0 * applyDeadband(self.funny_controllers.get_switched_controller(RobotController.getTime() / 1000000).getLeftTriggerAxis(), 0.1)
                )
            if self.funny_controllers.get_switched_controller(RobotController.getTime() / 1000000).getRightTriggerAxis() > 0.5:
                self.arm_control.set_wheel_voltage(
                    6.0 * applyDeadband(-self.funny_controllers.get_switched_controller(RobotController.getTime() / 1000000).getRightTriggerAxis(), 0.1)
                )

            if self.funny_controllers.get_switched_controller(RobotController.getTime() / 1000000).getRightBumper():
                self.arm_control.set_wheel_voltage(-1)

            # if self.secondary.getPOV() == 270:
            #     self.arm_control.next_state_now("elevator_failsafe")

            self.elevator_ligament.setLength((self.elevator.get_height() * 39.37) + 5)
            self.claw_ligament.setAngle(self.claw.get_angle() - 90)

        with self.consumeExceptions():
            """
            CLIMBER
            """

            if self.funny_controllers.get_switched_controller(RobotController.getTime() / 1000000).getPOV() == 0:
                self.climber.set_speed(-1)
            if self.funny_controllers.get_switched_controller(RobotController.getTime() / 1000000).getPOV() == 180:
                self.climber.set_speed(1)

        with self.consumeExceptions():
            """
            MISC
            """
            if self.funny_controllers.get_switched_controller(RobotController.getTime() / 1000000).getPOV() == 90:
                self.arm_control.next_state_now("positioning_claw")
                self.led_strip.justin_fun()

        # with self.consumeExceptions():
        #     """
        #     SYS-ID
        #     """
        #     if self.sysid_con.getAButton():
        #         self.sysid_drive.quasistatic_forward()
        #     if self.sysid_con.getBButton():
        #         self.sysid_drive.quasistatic_reverse()
        #     if self.sysid_con.getXButton():
        #         self.sysid_drive.dynamic_forward()
        #     if self.sysid_con.getYButton():
        #         self.sysid_drive.dynamic_reverse()

    @feedback
    def get_voltage(self) -> units.volts:
        return RobotController.getBatteryVoltage()

    def _display_auto_trajectory(self) -> None:
        selected_auto = self._automodes.chooser.getSelected()
        if isinstance(selected_auto, AutoBase):
            selected_auto.display_trajectory()

    @feedback
    def display_auto_state(self) -> None:
        selected_auto = self._automodes.chooser.getSelected()
        if isinstance(selected_auto, AutoBase):
            return selected_auto.current_state
        return "No Auto Selected"


if __name__ == "__main__":
    wpilib.run(MyRobot)
