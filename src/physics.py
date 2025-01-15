import math

from phoenix6 import unmanaged
from phoenix6.hardware.talon_fx import TalonFX
from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics.drivetrains import four_motor_swerve_drivetrain
from wpilib import DriverStation
from wpilib.simulation import DCMotorSim
from wpimath.system.plant import DCMotor, LinearSystemId

from robot import MyRobot


class FalconSim:
    def __init__(self, motor: TalonFX, moi: float, gearing: float):
        self.gearbox = DCMotor.falcon500(1)
        self.plant = LinearSystemId.DCMotorSystem(self.gearbox, moi, gearing)
        self.gearing = gearing
        self.sim_state = motor.sim_state
        self.sim_state.set_supply_voltage(12.0)
        self.motor_sim = DCMotorSim(self.plant, self.gearbox)

    def update(self, dt: float):
        voltage = self.sim_state.motor_voltage
        self.motor_sim.setInputVoltage(voltage)
        self.motor_sim.update(dt)
        self.sim_state.set_raw_rotor_position(
            self.motor_sim.getAngularPositionRotations() * self.gearing
        )
        self.sim_state.set_rotor_velocity(
            self.motor_sim.getAngularVelocityRPM() / 60 * self.gearing
        )


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
        self.physics_controller = physics_controller
        self.robot = robot
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

        self.robot.pigeon.sim_states_voltage(5.0)

    def update_sim(self, now, tm_diff):
        if DriverStation.isEnabled():
            unmanaged.feed_enable(100)
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
            # artificially soften simulated omega
            sim_speeds.omega_dps *= 0.4
            # correct chassis speeds to match initial robot orientation
            sim_speeds.vx, sim_speeds.vy = sim_speeds.vy, -sim_speeds.vx
            pose = self.physics_controller.drive(sim_speeds, tm_diff)
            self.robot.camera.set_robot_pose(pose)
            self.robot.pigeon.sim_states_add_yaw(pose.rotation().degrees())
