import math


from wpilib import SmartDashboard,DriverStation
from wpimath import units
from wpimath.controller import HolonomicDriveController
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveModulePosition,
    SwerveModuleState,
)
from wpiutil import Sendable, SendableBuilder
from phoenix6.hardware import Pigeon2

from components.swerve_wheel import SwerveWheel
from magicbot import will_reset_to, feedback
from lemonlib.util import Alert, AlertType
from lemonlib.ctre import LemonPigeon
from lemonlib.smart import SmartProfile
from choreo.trajectory import SwerveSample
from wpilib.sysid import SysIdRoutineLog
from lemonlib import LemonComponent

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants

from commands2 import Command


class SwerveDrive(LemonComponent,Sendable):
    offset_x: units.meters
    offset_y: units.meters
    drive_gear_ratio: float
    wheel_radius: units.meters
    max_speed: units.meters_per_second
    front_left: SwerveWheel
    front_right: SwerveWheel
    rear_left: SwerveWheel
    rear_right: SwerveWheel
    pigeon: Pigeon2
    translation_profile: SmartProfile
    rotation_profile: SmartProfile

    translationX = will_reset_to(0)
    translationY = will_reset_to(0)
    rotationX = will_reset_to(0)
    field_relative = will_reset_to(True)
    has_desired_pose = will_reset_to(False)

    def __init__(self) -> None:
        Sendable.__init__(self)
        

    def shouldFlipPath():
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    """
    INITIALIZATION METHODS
    """

    def setup(self) -> None:
        """
        This function is automatically called after the components have
        been injected.
        """
        # Kinematics
        self.front_left_pose = Translation2d(self.offset_x, self.offset_y)
        self.front_right_pose = Translation2d(self.offset_x, -self.offset_y)
        self.rear_left_pose = Translation2d(-self.offset_x, self.offset_y)
        self.rear_right_pose = Translation2d(-self.offset_x, -self.offset_y)
        self.kinematics = SwerveDrive4Kinematics(
            self.front_left_pose,
            self.front_right_pose,
            self.rear_left_pose,
            self.rear_right_pose,
        )
        self.chassis_speeds = ChassisSpeeds()
        self.still_states = self.kinematics.toSwerveModuleStates(self.chassis_speeds)
        self.swerve_module_states = self.still_states
        SmartDashboard.putData("Swerve Drive", self)

        self.pose_estimator = SwerveDrive4PoseEstimator(
            self.kinematics,
            Rotation2d(),
            (
                SwerveModulePosition(),
                SwerveModulePosition(),
                SwerveModulePosition(),
                SwerveModulePosition(),
            ),
            Pose2d(),
        )
        self.period = 0.02

        self.desired_pose = Pose2d()
        self.starting_pose = None  # only used in sim

        self.pigeon_offset = Rotation2d()
        self.pigeon_alert = Alert(
            "Pigeon heading has been reset.", AlertType.INFO, timeout=3.0
        )
        self.pigeon.set_yaw(180)
        # self.pigeon.reset()

        config = RobotConfig.fromGUISettings()

        # Configure the AutoBuilder last
        AutoBuilder.configure(
            self.get_estimated_pose(), # Robot pose supplier
            self.resetPose, # Method to reset odometry (will be called if your auto has a starting pose)
            self.chassis_speeds, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            lambda speeds, feedforwards: self.driveRobotRelative(speeds), # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also outputs individual module feedforwards
            PPHolonomicDriveController( # PPHolonomicController is the built in path following controller for holonomic drive trains
                PIDConstants(5.0, 0.0, 0.0), # Translation PID constants
                PIDConstants(5.0, 0.0, 0.0) # Rotation PID constants
            ),
            config, # The robot configuration
            self.shouldFlipPath, # Supplier to control path flipping based on alliance color
            self # Reference to this subsystem to set requirements
        )

    def initSendable(self, builder: SendableBuilder) -> None:
        builder.setSmartDashboardType("SwerveDrive")
        builder.addDoubleProperty(
            "Robot Angle",
            # Rotate to match field widget
            lambda: self.pigeon.getRotation2d().degrees(),
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Front Left Velocity",
            lambda: self.swerve_module_states[0].speed / self.max_speed,
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Front Left Angle",
            lambda: self.swerve_module_states[0].angle.degrees(),
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Front Right Velocity",
            lambda: self.swerve_module_states[1].speed / self.max_speed,
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Front Right Angle",
            lambda: self.swerve_module_states[1].angle.degrees(),
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Back Left Velocity",
            lambda: self.swerve_module_states[2].speed / self.max_speed,
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Back Left Angle",
            lambda: self.swerve_module_states[2].angle.degrees(),
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Back Right Velocity",
            lambda: self.swerve_module_states[3].speed / self.max_speed,
            lambda _: None,
        )
        builder.addDoubleProperty(
            "Back Right Angle",
            lambda: self.swerve_module_states[3].angle.degrees(),
            lambda _: None,
        )

    def on_enable(self):
        self.x_controller = self.translation_profile.create_wpi_pid_controller()
        self.y_controller = self.translation_profile.create_wpi_pid_controller()
        self.theta_controller = (
            self.rotation_profile.create_wpi_profiled_pid_controller_radians()
        )
        self.holonomic_controller = HolonomicDriveController(
            self.x_controller, self.y_controller, self.theta_controller
        )
        self.theta_controller.enableContinuousInput(-math.pi, math.pi)

    """
    INFORMATIONAL METHODS
    """

    def get_estimated_pose(self) -> Pose2d:
        return self.pose_estimator.getEstimatedPosition()

    def get_velocity(self) -> ChassisSpeeds:
        return self.chassis_speeds
        
    def get_module_states(
        self,
    ) -> tuple[
        SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
    ]:
        return (
            self.front_left.getMeasuredState(),
            self.front_right.getMeasuredState(),
            self.rear_left.getMeasuredState(),
            self.rear_right.getMeasuredState(),
        )

    @feedback
    def get_distance_from_desired_pose(self) -> units.meters:
        if not self.has_desired_pose:
            return 0
        return self.desired_pose.translation().distance(
            self.get_estimated_pose().translation()
        )

    def get_distance_from_pose(self, pose: Pose2d) -> units.meters:
        return pose.translation().distance(self.get_estimated_pose().translation())

    """
    CONTROL METHODS
    """

    def drive(
        self,
        translationX: units.meters_per_second,
        translationY: units.meters_per_second,
        rotationX: units.radians_per_second,
        field_relative: bool,
        period: units.seconds,
    ):
        self.translationX = translationX
        self.translationY = translationY
        self.rotationX = rotationX
        self.period = period
        self.field_relative = field_relative

    def set_desired_pose(self, pose: Pose2d):
        self.desired_pose = pose
        self.has_desired_pose = True

    def reset_gyro(self) -> None:
        self.pigeon.set_yaw(180)
        # self.pigeon.reset()
        self.pigeon_alert.enable()

    def add_vision_measurement(self, pose: Pose2d, timestamp: units.seconds):
        self.pose_estimator.addVisionMeasurement(pose, timestamp)

    def set_pigeon_offset(self, offset: units.degrees):
        """set angle added to reading from pigeon"""
        self.pigeon_offset = Rotation2d.fromDegrees(offset)

    def follow_trajectory(self, sample: SwerveSample):
        holospeeds = self.holonomic_controller.calculate(
            self.get_estimated_pose(),
            sample.get_pose(),
            0.0,
            sample.get_pose().rotation(),
        )
        speeds = ChassisSpeeds(
            sample.vx + holospeeds.vx,
            sample.vy + holospeeds.vy,
            sample.omega + holospeeds.omega,


        )
        self.drive(speeds.vx, speeds.vy, speeds.omega, False, self.period)


    def driveRobotRelative(self, speeds: ChassisSpeeds) -> Command:
        """Drives the robot using ROBOT RELATIVE speeds.
        This is used for path following."""
        return self.drive(
            speeds.vx,
            speeds.vy,
            speeds.omega,
            False,
            self.period,
        )

    def set_starting_pose(self, pose: Pose2d):
        """ONLY USE IN SIM!"""
        self.starting_pose = pose
        if pose is not None:
            self.pose_estimator.resetPose(pose)

    def resetPose(self):
        self.pose_estimator.resetPose(Pose2d())

    """
    TELEMETRY METHODS
    """

    def sendAdvantageScopeData(self):
        """Put swerve module setpoints and measurements to NT.
        This is used mainly for AdvantageScope's swerve tab"""
        swerve_setpoints = []
        for state in self.swerve_module_states:
            swerve_setpoints += [state.angle.degrees(), state.speed]
        SmartDashboard.putNumberArray("Swerve Setpoints", swerve_setpoints)
        swerve_measurements = []
        swerve_measurements += self.front_left.getMeasuredState()
        swerve_measurements += self.front_right.getMeasuredState()
        swerve_measurements += self.rear_left.getMeasuredState()
        swerve_measurements += self.rear_right.getMeasuredState()
        SmartDashboard.putNumberArray("Swerve Measurements", swerve_measurements)

    """
    sys-id
    """

    # Tell SysId how to record a frame of data for each motor on the mechanism being
    # characterized.
    def log(self, sys_id_routine: SysIdRoutineLog) -> None:
        sys_id_routine.motor("drive-front-left").voltage(
            self.front_left.getVoltage()
        ).position(self.front_left.getPosition().distance).velocity(
            self.front_left.getVelocity()
        )

        sys_id_routine.motor("drive-front-right").voltage(
            self.front_right.getVoltage()
        ).position(self.front_right.getPosition().distance).velocity(
            self.front_right.getVelocity()
        )

        sys_id_routine.motor("drive-rear-left").voltage(
            self.rear_left.getVoltage()
        ).position(self.rear_left.getPosition().distance).velocity(
            self.rear_left.getVelocity()
        )

        sys_id_routine.motor("drive-rear-right").voltage(
            self.rear_right.getVoltage()
        ).position(self.rear_right.getPosition().distance).velocity(
            self.rear_right.getVelocity()
        )

    """
    EXECUTE
    """

    def execute(self) -> None:
        self.sendAdvantageScopeData()
        self.pose_estimator.update(
            self.pigeon.getRotation2d() + self.pigeon_offset,
            (
                self.front_left.getPosition(),
                self.front_right.getPosition(),
                self.rear_left.getPosition(),
                self.rear_right.getPosition(),
            ),
        )

        if self.has_desired_pose and self.get_distance_from_desired_pose() > 0.02:
            self.chassis_speeds = self.holonomic_controller.calculate(
                self.get_estimated_pose(),
                self.desired_pose,
                0.0,
                self.desired_pose.rotation(),
            )
        else:
            if self.translationX == self.translationY == self.rotationX == 0:
                # below line is only to keep NT updated
                self.swerve_module_states = self.still_states
                self.chassis_speeds = ChassisSpeeds()
                return
            self.chassis_speeds = ChassisSpeeds.discretize(
                (
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        self.translationX,
                        self.translationY,
                        self.rotationX,
                        self.pigeon.getRotation2d() + self.pigeon_offset,
                    )
                    if self.field_relative
                    else ChassisSpeeds.fromFieldRelativeSpeeds(
                        self.translationX,
                        self.translationY,
                        self.rotationX,
                        Rotation2d(),
                    )
                ),
                self.period,
            )

        self.swerve_module_states = self.kinematics.toSwerveModuleStates(
            self.chassis_speeds
        )
        self.swerve_module_states = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            self.swerve_module_states,
            self.max_speed,
        )
        self.front_left.setDesiredState(self.swerve_module_states[0])
        self.front_right.setDesiredState(self.swerve_module_states[1])
        self.rear_left.setDesiredState(self.swerve_module_states[2])
        self.rear_right.setDesiredState(self.swerve_module_states[3])
