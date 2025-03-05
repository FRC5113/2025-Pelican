from autonomous.auto_base import AutoBase


class passline(AutoBase):
    MODE_NAME = "Passline"

    def __init__(self):
        super().__init__(["passline"])
