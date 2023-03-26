# The Chariot Python Package

This is a control library that was developed for testing during the development of the Chariot robot platform.
Specifically, it targets control of an abitrary number of ODrive controllers over the CAN protocol the ODrive firmware supports.

It is not designed to be used in production, so use at your own risk.

## Building this package
This package uses Poetry to manage dependencies. To get started, I first recommend:

- installing [pyenv]()
- using `pyenv` to install `python3.9` and create a virtual environment
- installing poetry while in the venv:
```
pip install poetry
```
- finally, install dependencies in this package by changing directories to the package root and running
```
$ poetry install
```

While this package doesn't exactly require Python `3.9` it is strongly recommended in the case that you wish to use ODrive Robotics'
`odrivetool` front end. This requires a USB connection to the controller, but allows for simple configuration and debugging of the controller.

## Style
This package requires linting and strongly emphasizes type checking. There are currently some issues with third party libraries and their
lack of typing support, but it must be used when possible.

## Package breakdown

### CanSimplerInterface
This package is responsible for the "low level" interfacing with CAN. At the moment this package uses SocketCan as the arbiter between
CAN and the PC. This limits the use to Linux, which has Kernel support for SocketCan, but other Operating Systems do not.

### OdriveMotorManager
This is an intermediary library that is intended to be client facing. It is slightly geared towards the Chariot platform, but
can easily be generalized by making kinematic dataclasses implement a protocol.

This class should expand over time to fufill the requirements of running the motors on the platform.

### Joystick Support
Additionally this package includes a python-only interpreter for commanding motors using an Xbox gamepad.
This bootstraps off of Pyjoystick, and can easily be generalized to other gamepad devices.
