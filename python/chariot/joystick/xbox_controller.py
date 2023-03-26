from dataclasses import dataclass
from inputs import get_gamepad
import math
import threading


@dataclass
class ControllerResult:
    LeftJoystickY: float = 0
    LeftJoystickX: float = 0
    RightJoystickY: float = 0
    RightJoystickX: float = 0
    LeftTrigger: float = 0
    RightTrigger: float = 0
    LeftBumper: float = 0
    RightBumper: float = 0
    A: float = 0
    X: float = 0
    Y: float = 0
    B: float = 0
    LeftThumb: float = 0
    RightThumb: float = 0
    Back: float = 0
    Start: float = 0
    LeftDPad: float = 0
    RightDPad: float = 0
    UpDPad: float = 0
    DownDPad: float = 0


class XboxController(object):
    MAX_TRIG_VAL: float = math.pow(2, 8)
    MAX_JOY_VAL: float = math.pow(2, 15)

    def __init__(self):
        self._controller_result = ControllerResult()
        self._monitor_thread = threading.Thread(
            target=self._monitor_controller, args=()
        )
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def read(self):  # return the buttons/triggers that you care about in this methode
        # x = self.LeftJoystickX
        # y = self.LeftJoystickY
        # a = self.A
        # b = self.X # b=1, x=2
        # rb = self.RightBumper
        return self._controller_result

    def _monitor_controller(self):
        while True:
            events = get_gamepad()
            for event in events:
                if event.code == "ABS_Y":
                    self._controller_resultLeftJoystickY = (
                        event.state / XboxController.MAX_JOY_VAL
                    )  # normalize between -1 and 1
                elif event.code == "ABS_X":
                    self._controller_resultLeftJoystickX = (
                        event.state / XboxController.MAX_JOY_VAL
                    )  # normalize between -1 and 1
                elif event.code == "ABS_RY":
                    self._controller_resultRightJoystickY = (
                        event.state / XboxController.MAX_JOY_VAL
                    )  # normalize between -1 and 1
                elif event.code == "ABS_RX":
                    self._controller_resultRightJoystickX = (
                        event.state / XboxController.MAX_JOY_VAL
                    )  # normalize between -1 and 1
                elif event.code == "ABS_Z":
                    self._controller_resultLeftTrigger = (
                        event.state / XboxController.MAX_TRIG_VAL
                    )  # normalize between 0 and 1
                elif event.code == "ABS_RZ":
                    self._controller_resultRightTrigger = (
                        event.state / XboxController.MAX_TRIG_VAL
                    )  # normalize between 0 and 1
                elif event.code == "BTN_TL":
                    self._controller_resultLeftBumper = event.state
                elif event.code == "BTN_TR":
                    self._controller_resultRightBumper = event.state
                elif event.code == "BTN_SOUTH":
                    self._controller_resultA = event.state
                elif event.code == "BTN_NORTH":
                    self._controller_resultX = event.state
                elif event.code == "BTN_WEST":
                    self._controller_resultY = event.state
                elif event.code == "BTN_EAST":
                    self._controller_resultB = event.state
                elif event.code == "BTN_THUMBL":
                    self._controller_resultLeftThumb = event.state
                elif event.code == "BTN_THUMBR":
                    self._controller_resultRightThumb = event.state
                elif event.code == "BTN_SELECT":
                    self._controller_resultBack = event.state
                elif event.code == "BTN_START":
                    self._controller_resultStart = event.state
                elif event.code == "BTN_TRIGGER_HAPPY1":
                    self._controller_resultLeftDPad = event.state
                elif event.code == "BTN_TRIGGER_HAPPY2":
                    self._controller_resultRightDPad = event.state
                elif event.code == "BTN_TRIGGER_HAPPY3":
                    self._controller_resultUpDPad = event.state
                elif event.code == "BTN_TRIGGER_HAPPY4":
                    self._controller_resultDownDPad = event.state


if __name__ == "__main__":
    joy = XboxController()
    while True:
        print(joy.read())
