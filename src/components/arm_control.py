from magicbot import StateMachine, will_reset_to
from magicbot.state_machine import state

from components.claw import Claw, ClawAngle
from components.elevator import Elevator, ElevatorHeight
from lemonlib.util import Alert, AlertType

from wpilib import DigitalInput


class ArmControl(StateMachine):
    """State machine that controls the intake on the robot and makes its
    operation safer and easier
    """

    # other components
    claw: Claw
    elevator: Elevator
    elevator_upper_switch: DigitalInput
    elevator_lower_switch: DigitalInput

    # eject_trigger will override intake_trigger
    pclaw_trigger = will_reset_to(False)
    parm_trigger = will_reset_to(False)
    disabled_trigger = will_reset_to(False)

    hinge_setpoint = will_reset_to(ClawAngle.STOWED)
    elevator_setpoint = will_reset_to(ElevatorHeight.L1)

    homed = False
    _efail = False  # should prob change just cound not think of name

    def setup(self):
        self.engage()
        self.unhomed_alert = Alert(
            "Elevator encoders not calibrated! Moving elevator down.", AlertType.WARNING
        )

    def request_stowed(self):
        self.hinge_setpoint = ClawAngle.STOWED
        self.pclaw_trigger = True

    def request_station(self):
        self.hinge_setpoint = ClawAngle.STATION
        self.pclaw_trigger = True

    def request_trough(self):
        self.hinge_setpoint = ClawAngle.TROUGH
        self.pclaw_trigger = True

    def request_branch(self):
        self.hinge_setpoint = ClawAngle.BRANCH
        self.pclaw_trigger = True

    def request_level1(self):
        self.elevator_setpoint = ElevatorHeight.L1
        self.hinge_setpoint = ClawAngle.TROUGH
        self.pclaw_trigger = True
        self.parm_trigger = True

    def request_level2(self):
        self.elevator_setpoint = ElevatorHeight.L2
        self.hinge_setpoint = ClawAngle.BRANCH
        self.pclaw_trigger = True
        self.parm_trigger = True

    def request_level3(self):
        self.elevator_setpoint = ElevatorHeight.L3
        self.hinge_setpoint = ClawAngle.BRANCH
        self.pclaw_trigger = True
        self.parm_trigger = True

    def request_level4(self):
        self.elevator_setpoint = ElevatorHeight.L4
        self.hinge_setpoint = ClawAngle.BRANCH
        self.pclaw_trigger = True
        self.parm_trigger = True

    def request_elevator_fail(self):
        self.efail = True

    def on_enable(self):
        self.unhomed_alert.enable()
        self.homed = False

    # states
    @state(first=True)
    def homing(self):
        if self.elevator_lower_switch.get():
            self.homed = True
            self.unhomed_alert.disable()
            self.elevator.reset_encoders()
            self.next_state("pclaw")

        if not self.homed:
            # move elevator down slowly until limit switch is reached and position is known
            self.elevator.set_voltage(-1.0)

    @state
    def pclaw(self):
        self.claw.set_target_angle(self.hinge_setpoint)

        if (
            self.claw.get_angle() == self.hinge_setpoint
            and self.elevator.get_height() == self.elevator_setpoint
        ):
            self.next_state("standby")
        if self._efail:
            self.next_state("efail")
        if (
            self.claw.get_angle() >= ClawAngle.SAFE_START
            and self.claw.get_angle() <= ClawAngle.SAFE_END
            and self.elevator.get_height() != self.elevator_setpoint
        ):
            self.next_state("parm")

    @state
    def parm(self):
        self.elevator.set_target_height(self.elevator_setpoint)

        if (
            self.elevator.get_height() == self.elevator_setpoint
            and self.claw.get_angle() == self.hinge_setpoint
        ):
            self.next_state("standby")
        if (
            self.elevator.get_height() == self.elevator_setpoint
            and self.claw.get_angle() != self.hinge_setpoint
        ):
            self.next_state("pclaw")

    @state
    def standby(self):
        if self.claw.get_angle() != self.hinge_setpoint:
            self.next_state("pclaw")
        if (
            self.elevator.get_height() != self.elevator_setpoint
            and self.claw.get_angle() >= ClawAngle.SAFE_START
            and self.claw.get_angle() <= ClawAngle.SAFE_END
        ):
            self.next_state("parm")
        self.next_state("standby")

    @state
    def efail(self):
        self.elevator.set_voltage(0)
        self.next_state("parm")

    @state
    def disabled(self):
        self.claw.set_hinge_voltage(0)
        self.elevator.set_voltage(0)
        self.next_state("disabled")
