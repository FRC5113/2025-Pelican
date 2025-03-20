import math
from phoenix6 import unmanaged
from phoenix6.hardware.talon_fx import TalonFX
from rev import SparkMaxSim, SparkRelativeEncoderSim, SparkMax, SparkAbsoluteEncoderSim
from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics.drivetrains import four_motor_swerve_drivetrain
from photonlibpy.simulation.photonCameraSim import PhotonCameraSim
from photonlibpy.simulation.simCameraProperties import SimCameraProperties
from photonlibpy.simulation.visionSystemSim import VisionSystemSim
from wpilib import DriverStation, Mechanism2d, SmartDashboard, Color8Bit
from wpilib.simulation import SingleJointedArmSim, ElevatorSim, DIOSim
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.geometry import Pose2d, Transform2d, Rotation3d
from robot import MyRobot
from lemonlib.simulation import LemonCameraSim
from lemonlib.simulation import FalconSim


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
        # Swerve Drive Setup
        self.physics_controller = physics_controller
        self.robot = robot
        self.pose = Pose2d()
        self.speed_sims = (
            FalconSim(robot.front_left_speed_motor, 0.01, 6.75),
            FalconSim(robot.front_right_speed_motor, 0.01, 6.75),
            FalconSim(robot.rear_left_speed_motor, 0.01, 6.75),
            FalconSim(robot.rear_right_speed_motor, 0.01, 6.75),
        )
        self.direction_sims = (
            FalconSim(robot.front_left_direction_motor, 0.01, 150 / 7),
            FalconSim(robot.front_right_direction_motor, 0.01, 150 / 7),
            FalconSim(robot.rear_left_direction_motor, 0.01, 150 / 7),
            FalconSim(robot.rear_right_direction_motor, 0.01, 150 / 7),
        )

        self.encoders = (
            robot.front_left_cancoder,
            robot.front_right_cancoder,
            robot.rear_left_cancoder,
            robot.rear_right_cancoder,
        )
        for encoder in self.encoders:
            encoder.sim_state.add_position(0.25)

        self.robot.pigeon.sim_state.set_supply_voltage(5.0)

        # Elevator Simulation
        self.elevator_gearbox = DCMotor.NEO(2)
        self.elevator_sim = ElevatorSim(
            self.elevator_gearbox,
            robot.elevator_gearing,
            robot.elevator_carriage_mass,
            robot.elevator_spool_radius,
            robot.elevator_min_height,
            robot.elevator_max_height,
            True,
            0,
            [0.0, 0.0],
        )
        self.elevator_left_encoder_sim = SparkRelativeEncoderSim(
            robot.elevator_left_motor
        )
        self.elevator_right_encoder_sim = SparkRelativeEncoderSim(
            robot.elevator_right_motor
        )
        self.elevator_left_motor_sim = SparkMaxSim(
            robot.elevator_left_motor, DCMotor.NEO(1)
        )
        self.elevator_right_motor_sim = SparkMaxSim(
            robot.elevator_right_motor, DCMotor.NEO(1)
        )
        self.elevator_lower_switch_sim = DIOSim(robot.elevator_lower_switch)
        self.elevator_upper_switch_sim = DIOSim(robot.elevator_upper_switch)

        # Claw Simulation
        self.claw_gearbox = DCMotor.NEO(1)
        self.claw_sim = SingleJointedArmSim(
            self.claw_gearbox,
            robot.claw_gearing,
            0.1,  # gross estimate
            0.2,  # estimate
            -0.52,
            1.57,
            True,
            1.57,
            [0, 0],
        )
        self.claw_encoder_sim = SparkAbsoluteEncoderSim(robot.claw_hinge_motor)
        self.claw_motor_sim = SparkMaxSim(robot.claw_hinge_motor, self.claw_gearbox)

        # Mechanism2d Visualization for Arm
        self.arm_sim = Mechanism2d(20, 50)
        self.arm_root = self.arm_sim.getRoot("Arm Root", 10, 0)
        self.elevator_ligament = self.arm_root.appendLigament("Elevator", 5, 90)
        self.claw_ligament = self.elevator_ligament.appendLigament(
            "Claw", 5, 0, color=Color8Bit(0, 150, 0)
        )

        # Put Mechanism to SmartDashboard
        SmartDashboard.putData("Arm Sim", self.arm_sim)

        # Vision Simulation
        self.vision_sim = LemonCameraSim(
            robot.camera, robot.field_layout, fov=100.0, fps=20.0
        )
        # self.vision_sim.addAprilTags(robot.field_layout)

        # self.camera_props = SimCameraProperties()
        # self.camera_props.setCalibrationFromFOV(640, 480, Rotation2d.fromDegrees(100))
        # self.camera_props.setFPS(20)
        # self.camera_props.setAvgLatency(0.035)
        # self.camera_props.setLatencyStdDev(0.005)

        # self.camera_sim = PhotonCameraSim(
        #     robot.camera, self.camera_props, robot.field_layout
        # )
        # self.vision_sim.addCamera(self.camera_sim, robot.robot_to_camera)

        # self.camera_sim = LemonCameraSim(
        #     robot.camera, robot.field_layout, fov=100.0, fps=20.0
        # )

        # Simulated components

    def update_sim(self, now, tm_diff):
        if DriverStation.isEnabled():
            unmanaged.feed_enable(100)

            if self.robot.swerve_drive.starting_pose is not None:
                self.physics_controller.move_robot(
                    Transform2d(self.pose.translation(), self.pose.rotation()).inverse()
                )
                start = self.robot.swerve_drive.starting_pose
                self.physics_controller.move_robot(
                    Transform2d(start.translation(), start.rotation())
                )
                self.robot.swerve_drive.set_starting_pose(None)

            for i in range(4):
                self.speed_sims[i].update(tm_diff)
                self.direction_sims[i].update(tm_diff)
                self.encoders[i].sim_state.add_position(
                    -self.direction_sims[i].motor_sim.getAngularVelocity()
                    / (2 * math.pi)
                    * tm_diff
                )

            sim_speeds = four_motor_swerve_drivetrain(
                self.speed_sims[2].sim_state.motor_voltage / 12.0,
                self.speed_sims[3].sim_state.motor_voltage / 12.0,
                self.speed_sims[0].sim_state.motor_voltage / 12.0,
                self.speed_sims[1].sim_state.motor_voltage / 12.0,
                (self.encoders[2].get_absolute_position().value * -360) % 360,
                (self.encoders[3].get_absolute_position().value * -360) % 360,
                (self.encoders[0].get_absolute_position().value * -360) % 360,
                (self.encoders[1].get_absolute_position().value * -360) % 360,
                2.5,
                2.5,
                15.52,
            )
            # Artificially soften simulated omega
            sim_speeds.omega_dps *= 0.4
            # Correct chassis speeds to match initial robot orientation
            sim_speeds.vx, sim_speeds.vy = sim_speeds.vy, -sim_speeds.vx
            self.pose = self.physics_controller.drive(sim_speeds, tm_diff)
            # self.robot.camera.set_robot_pose(pose)
            self.robot.pigeon.sim_state.set_raw_yaw(
                self.pose.rotation().degrees() + 180
            )

            # Elevator Simulation Update
            # First, we set our "inputs" (voltages)
            # Use getSetpoint(), NOT getAppliedOutput() (god knows why)
            self.elevator_sim.setInput(0, self.elevator_left_motor_sim.getSetpoint())
            # Next, we update the elevator simulation
            self.elevator_sim.update(tm_diff)

            # Set our simulated encoder's readings and simulated battery voltage
            self.elevator_left_encoder_sim.setPosition(
                self.elevator_sim.getPosition()
                / self.robot.elevator_spool_radius
                / math.tau
                * self.robot.elevator_gearing
            )
            self.elevator_right_encoder_sim.setPosition(
                -self.elevator_sim.getPosition()
                / self.robot.elevator_spool_radius
                / math.tau
                * self.robot.elevator_gearing
            )
            if self.elevator_sim.getPosition() <= 0.0:
                self.elevator_lower_switch_sim.setValue(True)
            else:
                self.elevator_lower_switch_sim.setValue(False)
            if self.elevator_sim.getPosition() >= 0.7:
                self.elevator_upper_switch_sim.setValue(True)
            else:
                self.elevator_upper_switch_sim.setValue(False)

            self.claw_sim.setInput(0, self.claw_motor_sim.getSetpoint())
            self.claw_sim.update(tm_diff)
            self.claw_encoder_sim.setPosition(
                0.25 - self.claw_sim.getAngleDegrees() / 360
            )

            # Update the Elevator length based on the simulated elevator height
            self.elevator_ligament.setLength(self.elevator_sim.getPositionInches() + 5)
            self.claw_ligament.setAngle(self.claw_sim.getAngleDegrees() - 90)

            # Simulate Vision
            self.vision_sim.update(self.pose)
