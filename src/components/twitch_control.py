from wpilib import DigitalInput
from wpimath import units, applyDeadband
from magicbot import StateMachine, will_reset_to
from magicbot.state_machine import state,timed_state

from components.claw import Claw, ClawAngle
from components.elevator import Elevator, ElevatorHeight
from components.arm_control import ArmControl
from lemonlib.util import Alert, AlertType
from lemonlib.smart import SmartPreference


class TwitchControl(StateMachine):
    """State machine that controls the intake on the robot and makes its
    operation safer and easier
    """

    arm_control: ArmControl
    claw: Claw
    elevator: Elevator

    claw_setpoint = ClawAngle.STOWED
    elevator_setpoint = ElevatorHeight.L1


    """
    INITIALIZATION METHODS
    """

    def setup(self):
        self.engage()


    """
    CONTROL METHODS
    """

    def set(self, elevator_setpoint: units.meters, claw_setpoint: units.degrees, wheel_voltage: float = 0):
        self.elevator_setpoint = elevator_setpoint
        self.claw_setpoint = claw_setpoint
        self.wheel_voltage = wheel_voltage

    """
    STATES
    """

    @state(first=True)
    def homing(self):
        self.next_state("standby")

    @state
    def moving(self):
        self.arm_control.set(self.elevator_setpoint, self.claw_setpoint)

        if self.arm_control.at_point(self.elevator_setpoint, self.claw_setpoint):
            self.next_state("shoot")

    @timed_state(duration=1.0, next_state="reset_vars",must_finish=True)
    def shoot(self):
        self.arm_control.set(self.elevator_setpoint, self.claw_setpoint)
        self.arm_control.set_wheel_voltage(self.wheel_voltage)

    @state
    def reset_vars(self):
        self.claw_setpoint = ClawAngle.STOWED
        self.elevator_setpoint = ElevatorHeight.L1
        self.wheel_voltage = 0
        self.next_state("standby")

    @state
    def standby(self):
        if not self.arm_control.at_point(self.elevator_setpoint, self.claw_setpoint):
            self.next_state("moving")
    
    @state
    def disabled(self):
        self.done()


