from dataclasses import dataclass
from typing import Callable, List, Optional
@dataclass
class State:
  conditional: Callable[[], bool]
  runner: Callable[[], None]
  test_runner: Callable[[], None]
  _id: int
  name: str


class FiniteStateMachine:
  states: List[State]
  current_state: Optional[State]
  isInTest: bool
  _counter: int
  _state_creation_lock: bool
  _state_is_active: bool

  def __init__(self, isInTest: bool):
    self.isInTest = isInTest
    self.states = []
    self.current_state = None
    self._counter = 0

  def construct_state(self, name: str, conditional: Callable[[], bool], runner: Callable[[], None], test_runner: Callable[[], None]):
    if not self._state_creation_lock:
      state = State(conditional, runner, test_runner, self._counter, name)
      self._counter += 1
      self.states.append(state)
    else:
      raise Exception("[ERROR] Cannot add states after the FSM has been executed")

  def execute(self):
    self._state_is_active = False
    self._state_creation_lock = True
    for state in self.states:
      if state.conditional():
        self.current_state = state
        if self.isInTest:
          state.test_runner()
        else:
          state.runner()
        return
    raise Exception("[ERROR] No state was activated")
