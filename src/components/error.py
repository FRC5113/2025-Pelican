from components.elevator import Elevator
from components.claw import Claw
from components.climber import Climber
from magicbot import feedback


class Errors:
    elevator: Elevator
    claw: Claw
    climber: Climber

    @feedback
    def get_all_errors(self) -> bool:
        return (
            self.elevator.error_detected()
            and self.claw.error_detected()
            and self.climber.error_detected()
        )

    @feedback
    def get_claw_error(self) -> bool:
        return self.claw.error_detected()

    @feedback
    def get_elevator_error(self) -> bool:
        return self.elevator.error_detected()

    @feedback
    def get_climber_error(self) -> bool:
        return self.climber.error_detected()

    def execute(self):
        pass
