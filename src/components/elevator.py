from magicbot import feedback
from wpilib import DigitalInput, Encoder
from lemonlib.preference import SmartProfile
import rev
import math

from rev import SparkMax, SparkMaxConfig


class Elevator:

    # Motors and encoders
    right_elevator_motor: SparkMax
    left_elevator_motor: SparkMax
    elevator_encoder: Encoder
    upper_elevator_switch: DigitalInput
    lower_elevator_switch: DigitalInput
    kCarriageMass: float  # 15 kg
    kMinElevatorHeight: float  # meters (2 inches)
    kMaxElevatorHeight: float  # meters (50 inches)
    kElevatorGearing: float  # ratio (10:1)
    kSpoolRadius: float  # meters (1.5 inches)
    elevator_profile: SmartProfile

    def setup(self):
        """Initialize motors and encoder."""
        BRAKE = rev.SparkBaseConfig.IdleMode.kBrake
        FOLLEWER = rev.SparkBaseConfig.follow(self.left_elevator_motor)

        

        self.left_elevator_motor.IdleMode(BRAKE)
        self.right_elevator_motor.IdleMode(BRAKE)
        self.right_elevator_motor.configure(FOLLEWER)

        # Initialize target height
        self.target_height = self.kMinElevatorHeight

    def on_enable(self):
        self.elevator_controller = self.elevator_profile.create_pid_controller(
            "elevator"
        )

    def set_target_height(self, height: float):
        """Set the target height for the elevator."""
        self.target_height = height

    def stop(self):
        """Stop the elevator."""
        self.left_elevator_motor.stopMotor()

    def move_up(self, speed):
        """Move the elevator up."""
        self.left_elevator_motor.set(speed)

    def move_down(self, speed):
        """Move the elevator down."""
        self.left_elevator_motor(-speed)

    def execute(self):
        """Update the motor control to reach the target height."""
        current_height = (
            self.elevator_encoder.get()
            / self.kElevatorGearing
            * math.tau
            * self.kSpoolRadius
        )  # esimate plz change
        if self.upper_elevator_switch.get():
            self.elevator_encoder.reset()
        if self.upper_elevator_switch.get() and self.target_height > current_height:
            self.stop()
        if self.lower_elevator_switch.get() and self.target_height < current_height:
            self.stop()

        output = self.elevator_controller.calculate(current_height, self.target_height)
        self.left_elevator_motor.set(output)

    @feedback
    def get_current_height(self) -> float:
        """Get the current height of the elevator."""
        return (
            self.elevator_encoder.get()
            / self.kElevatorGearing
            * math.tau
            * self.kSpoolRadius
        )  # esimate plz change
