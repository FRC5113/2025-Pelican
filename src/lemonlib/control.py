from wpilib import DriverStation, RobotBase
from wpilib.interfaces import GenericHID
from wpiutil import Sendable
from lemonlib.util import Alert,AlertType


class LemonInput(Sendable):
    """
    LemonInput is a wrapper class for Xbox and PS5 controllers allowing automatic or manual detection and use in code.
    """

    xbox_buttons = {
        "kLeftTrigger": 2,
        "kLeftX": 0,
        "kLeftY": 1,
        "kRightTrigger": 3,
        "kRightX": 4,
        "kRightY": 5,
        "kA": 1,
        "kB": 2,
        "kBack": 7,
        "kLeftBumper": 5,
        "kLeftStick": 9,
        "kRightBumper": 6,
        "kRightStick": 10,
        "kStart": 8,
        "kX": 3,
        "kY": 4,
    }

    ps5_buttons = {
        "kLeftTrigger": 3,
        "kLeftX": 0,
        "kLeftY": 1,
        "kRightTrigger": 4,
        "kRightX": 2,
        "kRightY": 5,
        "kA": 2,
        "kB": 3,
        "kBack": 9,
        "kLeftBumper": 5,
        "kLeftStick": 11,
        "kStart": 10,
        "kPS": 13,
        "kRightBumper": 6,
        "kRightStick": 12,
        "kX": 1,
        "kTouchpad": 14,
        "kY": 4,
    }

    def __init__(self, port_number: int, type: str = "auto", primary: int = 0, secondary: int = 1):
        """
        Initializes the control object with the specified port number and type.
        Args:
            port_number (int): The port number of the controller.
            type (str, optional): The type of the controller. Defaults to "auto".
                - "auto": Automatically detects the controller type.
                - "Xbox": Forces the controller type to Xbox.
                - "PS5": Forces the controller type to PS5.
            primary (int, optional): The port number of the primary controller.
            secondary (int, optional): The port number of the secondary controller.
        """
        Sendable.__init__(self)
        self.con = GenericHID(port_number)
        self.primary = primary
        self.secondary = secondary

        def switch_ports():
            self.con = GenericHID(self.secondary if port_number == self.primary else self.primary)

        if type == "auto":
            if RobotBase.isSimulation():
                self.button_map = self.xbox_buttons
                self.contype = "Sim/Xbox"
            elif DriverStation.getJoystickIsXbox(port_number):
                self.button_map = self.xbox_buttons
                self.contype = "Xbox"
            else:
                self.button_map = self.ps5_buttons
                self.contype = "PS5"
        elif type == "Xbox":
            if RobotBase.isSimulation():
                self.button_map = self.xbox_buttons
                self.contype = "Sim/Xbox"
            else:
                if DriverStation.getJoystickIsXbox(port_number):
                    self.button_map = self.xbox_buttons
                    self.contype = "Xbox"
                else:
                    switch_ports()
                    self.button_map = self.xbox_buttons
                    self.contype = "Xbox"
        elif type == "PS5":
            if RobotBase.isSimulation():
                self.button_map = self.ps5_buttons
                self.contype = "Sim/PS5"
            else:
                if not DriverStation.getJoystickIsXbox(port_number):
                    self.button_map = self.ps5_buttons
                    self.contype = "PS5"
                else:
                    switch_ports()
                    self.button_map = self.ps5_buttons
                    self.contype = "PS5"
            
        

    def getType(self):
        """Returns the type of controller (Xbox or PS5)."""
        return self.contype

    def getLeftBumper(self):
        """Returns the state of the left bumper button."""
        return self.con.getRawButton(self.button_map["kLeftBumper"])

    def getRightBumper(self):
        """
        Returns the state of the right bumper button.

        Returns:
            bool: The state of the right bumper button (pressed or not).
        """
        return self.con.getRawButton(self.button_map["kRightBumper"])

    def getStartButton(self):
        """
        Returns the state of the start button.

        Returns:
            bool: The state of the start button (pressed or not).
        """
        return self.con.getRawButton(self.button_map["kStart"])

    def getBackButton(self):
        """
        Returns the state of the back button.

        Returns:
            bool: The state of the back button (pressed or not).
        """
        return self.con.getRawButton(self.button_map["kBack"])

    def getAButton(self):
        """
        Returns the state of the 'A' button.

        Returns:
            bool: The state of the 'A' button (pressed or not).
        """
        return self.con.getRawButton(self.button_map["kA"])

    def getBButton(self):
        """
        Returns the state of the 'B' button.

        Returns:
            bool: The state of the 'B' button (pressed or not).
        """
        return self.con.getRawButton(self.button_map["kB"])

    def getXButton(self):
        """
        Returns the state of the 'X' button.

        Returns:
            bool: The state of the 'X' button (pressed or not).
        """
        return self.con.getRawButton(self.button_map["kX"])

    def getYButton(self):
        """
        Returns the state of the 'Y' button.

        Returns:
            bool: The state of the 'Y' button (pressed or not).
        """
        return self.con.getRawButton(self.button_map["kY"])

    def getLeftStickButton(self):
        """
        Returns the state of the left stick button.

        Returns:
            bool: The state of the left stick button (pressed or not).
        """
        return self.con.getRawButton(self.button_map["kLeftStick"])

    def getRightStickButton(self):
        """
        Returns the state of the right stick button.

        Returns:
            bool: The state of the right stick button (pressed or not).
        """
        return self.con.getRawButton(self.button_map["kRightStick"])

    def getLeftX(self) -> float:
        """
        Returns the X-axis value of the left joystick.

        Returns:
            float: The X-axis value of the left joystick, ranging from -1.0 to 1.0.
        """
        return self.con.getRawAxis(self.button_map["kLeftX"])

    def getLeftY(self) -> float:
        """
        Returns the Y-axis value of the left joystick.

        Returns:
            float: The Y-axis value of the left joystick, ranging from -1.0 to 1.0.
        """
        return self.con.getRawAxis(self.button_map["kLeftY"])

    def getRightX(self) -> float:
        """
        Returns the X-axis value of the right joystick.

        Returns:
            float: The X-axis value of the right joystick, ranging from -1.0 to 1.0.
        """
        return self.con.getRawAxis(self.button_map["kRightX"])

    def getRightY(self) -> float:
        """
        Returns the Y-axis value of the right joystick.

        Returns:
            float: The Y-axis value of the right joystick, ranging from -1.0 to 1.0.
        """
        return self.con.getRawAxis(self.button_map["kRightY"])

    def getPOV(self) -> int:
        """
        Returns the Point of View (POV) value as an integer.

        Returns:
            int: The current POV value. Returns -1 if no POV is pressed.
        """
        return self.con.getPOV()

    def getRightTrigger(self) -> float:
        """
        Returns the state of the right trigger button.

        Returns:
            float: The state of the right trigger button ranging from 0.0 to 1.0.
        """
        return self.con.getRawAxis(self.button_map["kRightTrigger"])

    def getLeftTrigger(self) -> float:
        """
        Returns the state of the left trigger button.

        Returns:
            float: The state of the left trigger button ranging from 0.0 to 1.0.
        """
        return self.con.getRawAxis(self.button_map["kLeftTrigger"])

    def __pov_xy(self):
        """
        Returns the X and Y values of the POV as a tuple.

        Returns:
            tuple: The X and Y values of the POV as a tuple.
        """
        pov_value = self.pov()
        pov_mapping = {
            0: (1, 0),
            45: (0.707, -0.707),
            90: (0, -1),
            135: (-0.707, -0.707),
            180: (-1, 0),
            225: (-0.707, 0.707),
            270: (0, 1),
            315: (0.707, 0.707),
        }
        return pov_mapping.get(
            pov_value, (0, 0)
        )  # Return (0, 0) for unmapped POV values

    def getPovX(self) -> float:
        """
        Returns the X-axis value of the POV (Point of View) of a joystick.

        Example:
        ```
        controller = SmartController(0)

        if controller.pov() >= 0:
            left_joy_x = controller.pov_x()
            left_joy_y = controller.pov_y()
        ```

        Returns:
            float: The X-axis value of the POV.
        """
        return self.__pov_xy()[0]

    def getPovY(self) -> float:
        """
        Returns the Y-axis value of the POV (Point of View) of a joystick.

        Example:
        ```
        controller = SmartController(0)

        if controller.pov() >= 0:
            left_joy_x = controller.pov_x()
            left_joy_y = controller.pov_y()
        ```

        Returns:
            float: The Y-axis value of the POV.
        """
        return self.__pov_xy()[1]

    def initSendable(self, builder):
        """
        Initializes the sendable for the LemonInput class.

        Args:
            builder: The sendable builder.
        """
        builder.setSmartDashboardType("LemonInput")
        builder.addStringProperty("Type", lambda: self.contype, lambda: None)
        builder.addBooleanProperty(
            "LeftBumper", lambda: self.getLeftBumper(), lambda: None
        )
        builder.addBooleanProperty(
            "RightBumper", lambda: self.getRightBumper(), lambda: None
        )
        builder.addBooleanProperty(
            "StartButton", lambda: self.getStartButton(), lambda: None
        )
        builder.addBooleanProperty(
            "BackButton", lambda: self.getBackButton(), lambda: None
        )
        builder.addBooleanProperty("AButton", lambda: self.getAButton(), lambda: None)
        builder.addBooleanProperty("BButton", lambda: self.getBButton(), lambda: None)
        builder.addBooleanProperty("XButton", lambda: self.getXButton(), lambda: None)
        builder.addBooleanProperty("YButton", lambda: self.getYButton(), lambda: None)
        builder.addBooleanProperty(
            "LStickButton", lambda: self.getLeftStickButton(), lambda: None
        )
        builder.addBooleanProperty(
            "RStickButton", lambda: self.getRightStickButton(), lambda: None
        )
        builder.addDoubleProperty("LeftX", lambda: self.getLeftX(), lambda: None)
        builder.addDoubleProperty("LeftY", lambda: self.getLeftY(), lambda: None)
        builder.addDoubleProperty("RightX", lambda: self.getRightX(), lambda: None)
        builder.addDoubleProperty("RightY", lambda: self.getRightY(), lambda: None)
        builder.addDoubleProperty(
            "RightTrigger", lambda: self.getRightTrigger(), lambda: None
        )
        builder.addDoubleProperty(
            "LeftTrigger", lambda: self.getLeftTrigger(), lambda: None
        )
        builder.addDoubleProperty("POV_X", lambda: self.getPovX(), lambda: None)
        builder.addDoubleProperty("POV_Y", lambda: self.getPovY(), lambda: None)
