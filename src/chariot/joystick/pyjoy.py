"""
This file is a python-only interpreter for gamepads. It focuses on an XboxOne controller, but could easily be generalized.
"""
import datetime
from dataclasses import dataclass
import pyjoystick
from pyjoystick.sdl2 import (
    Joystick,
    run_event_loop,
    get_mapping_name,
)
import structlog

LOG = structlog.get_logger()
DEADBAND = 0.1  # Values less than this will register as 0


@dataclass
class XboxControllerResult:
    LeftJoystickX: float = 0
    LeftJoystickY: float = 0
    RightJoystickX: float = 0
    RightJoystickY: float = 0
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


@dataclass
class ControllerResult:
    controller: XboxControllerResult
    born: datetime.datetime


class JoystickController:
    def __init__(self):
        self.set_empty_result()

    def run(self):
        # Start the event loop. It's likely that you'll want to run this as a separate thread.
        self.repeater = pyjoystick.Repeater(
            first_repeat_timeout=0.05, repeat_timeout=0.03, check_timeout=0.01
        )
        mngr = pyjoystick.ThreadEventManager(
            event_loop=run_event_loop,
            handle_key_event=self.key_received,
            button_repeater=self.repeater,
        )
        mngr.start()

    def set_empty_result(self) -> None:
        self.res = ControllerResult(XboxControllerResult(), datetime.datetime.now())

    def print_add(self, joy):
        self.set_empty_result()
        LOG.info("Added a new gamepad.", joy)

    def print_remove(self, joy):
        self.set_empty_result()
        LOG.info("Remove a gamepad.", joy)

    def init_game_pad(self):
        self._devices = Joystick.get_joysticks()
        self._device = self._devices[0]  # Assumes a single controller connected
        self._monitor = self._device

    def key_received(self, key) -> None:
        self.init_game_pad()
        self._monitor.update_key(key)
        key_name = get_mapping_name(key.joystick, key)

        # see https://wiki.libsdl.org/SDL_GameControllerAddMapping
        # Joysticks are inverted so that "forward" relative to the human is a joystick movement away from them
        if key_name == "leftx":
            if abs(key.value) > DEADBAND:
                self.res.controller.LeftJoystickX = -key.value
            else:
                self.res.controller.LeftJoystickX = 0
        if key_name == "lefty":
            if abs(key.value) > DEADBAND:
                self.res.controller.LeftJoystickY = -key.value
            else:
                self.res.controller.LeftJoystickY = 0
        if key_name == "rightx":
            if abs(key.value) > DEADBAND:
                self.res.controller.RightJoystickX = -key.value
            else:
                self.res.controller.RightJoystickX = 0
        if key_name == "righty":
            if abs(key.value) > DEADBAND:
                self.res.controller.RightJoystickY = -key.value
            else:
                self.res.controller.RightJoystickY = 0
        if key_name == "righttrigger":
            if abs(key.value) > DEADBAND:
                self.res.controller.RightTrigger = 0
            else:
                self.res.controller.RightTrigger = -key.value
        if key_name == "a":
            self.res.controller.A = key.value
        if key_name == "b":
            self.res.controller.B = key.value
        if key_name == "x":
            self.res.controller.X = key.value
        if key_name == "y":
            self.res.controller.Y = key.value

        # Update the message birth date for lifecycle tracking
        self.res.born = datetime.datetime.now()

    def print_output(self):
        LOG.info(controller_result=self.res)

    def get_input(self):
        return self.res


if __name__ == "__main__":
    jc = JoystickController()
    jc.run()
