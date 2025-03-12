from wpilib import DigitalInput
from wpimath import units, applyDeadband
from magicbot import StateMachine, will_reset_to
from magicbot.state_machine import state

from components.claw import Claw, ClawAngle
from components.elevator import Elevator, ElevatorHeight
from lemonlib.util import Alert, AlertType
from lemonlib.smart.preference import SmartPreference


class ArmControl(StateMachine):
    """State machine that controls the intake on the robot and makes its
    operation safer and easier
    """

    # other components
    claw: Claw
    elevator: Elevator

    claw_setpoint = will_reset_to(ClawAngle.STOWED)
    elevator_setpoint = will_reset_to(ElevatorHeight.L1)
    wheel_voltage = will_reset_to(0)
    wheel_twist = SmartPreference(0.2)

    """
    INITIALIZATION METHODS
    """

    def setup(self):
        self.engage()
        self.unhomed_alert = Alert(
            "Elevator encoders not calibrated! Moving elevator down.", AlertType.WARNING
        )
        self.disabled_alert = Alert(
            "Arm StateMachine has been disabled!", AlertType.ERROR
        )
        self.drive_scalar = 1.0

    def on_enable(self):
        self.unhomed_alert.enable()

    """
    INFORMATIONAL METHODS
    """

    def get_drive_scalar(self) -> float:
        return self.drive_scalar

    def at_setpoint(self) -> bool:
        return self.elevator.at_setpoint() and self.claw.at_setpoint()

    """
    CONTROL METHODS
    """

    def set(self, elevator_setpoint: units.meters, claw_setpoint: units.degrees):
        self.elevator_setpoint = elevator_setpoint
        self.claw_setpoint = claw_setpoint

    def set_wheel_voltage(self, voltage: units.volts):
        self.wheel_voltage = voltage

    """
    STATES
    """

    @state(first=True)
    def homing(self):
        self.drive_scalar = 0.5
        if self.elevator.get_lower_switch():
            self.unhomed_alert.disable()
            self.elevator.reset_encoders()
            self.next_state("positioning_claw")
        self.elevator.set_voltage(-1.0)

    @state
    def positioning_claw(self):
        if self.claw.get_setpoint() == ClawAngle.STOWED:
            self.drive_scalar = 1.0
        else:
            self.drive_scalar = 0.5
        if self.elevator_setpoint < 0.1 and self.elevator.get_height() < 0.1:
            self.elevator.set_target_height(self.elevator_setpoint)
        else:
            self.elevator.set_voltage(0.0)
        self.claw.set_target_angle(self.claw_setpoint)
        if self.elevator.at_setpoint() and self.claw.at_setpoint():
            self.next_state("standby")
        if self.claw.is_safe() and (
            not self.elevator.at_setpoint()
            or self.elevator_setpoint != self.elevator.get_setpoint()
        ):
            self.next_state("positioning_arm")

    @state
    def positioning_arm(self):
        # consider scaling proportional to elevator height
        self.drive_scalar = 0.25
        self.elevator.set_target_height(self.elevator_setpoint)
        self.claw.set_target_angle(max(ClawAngle.SAFE_START, self.claw_setpoint))
        if self.elevator.at_setpoint():
            if self.claw.at_setpoint() and self.claw_setpoint >= ClawAngle.SAFE_START:
                self.next_state("standby")

            else:
                self.next_state("positioning_claw")

    @state
    def standby(self):
        self.elevator.set_target_height(self.elevator_setpoint)
        self.claw.set_target_angle(self.claw_setpoint)
        if self.elevator_setpoint == ElevatorHeight.L1 and self.elevator.at_setpoint():
            self.drive_scalar = 1.0
        else:
            self.drive_scalar = 0.25
        if self.claw_setpoint != ClawAngle.STOWED or ClawAngle.STOWED != 0.0:
            self.claw.set_wheel_voltage(
                self.wheel_voltage,
                self.wheel_twist if self.claw_setpoint == ClawAngle.TROUGH else 1.0,
            )
        if self.claw.is_safe():
            if not self.elevator.at_setpoint() or not self.claw.at_setpoint():
                self.next_state("positioning_arm")
        else:
            if not self.claw.at_setpoint():
                self.next_state("positioning_claw")

    @state
    def elevator_failsafe(self):
        if self.claw.get_setpoint() == ClawAngle.STOWED:
            self.drive_scalar = 1.0
        else:
            self.drive_scalar = 0.5
        self.elevator.set_voltage(0.0)
        self.claw.set_wheel_voltage(
            self.wheel_voltage,
            self.wheel_twist if self.claw.get_setpoint() == ClawAngle.TROUGH else 1.0,
        )
        self.claw.set_target_angle(self.claw_setpoint)

    @state
    def disabled(self):
        self.drive_scalar = 1.0
        self.disabled_alert.enable()
