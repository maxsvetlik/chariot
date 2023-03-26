"""
This is an experimental script for executing anti-clogging calibration for the ODrive Pro.
The anticlogging proceedure is not strongly supported by ODrive.. it exists, but works on
some motors and not others without a clear reason why.

As-is, this script does not overwrite any settings to the ODrive's NVM, though it should
only be run when there is no frictional load on the motor.

More can be read about this on the [ODrive anticlogging documentation](https://github.com/odriverobotics/ODrive/blob/ebd237673a483dec87b0d372dd09dfbf0bc7dd64/docs/anticogging.md)
"""
import odrive
from odrive.enums import *

import time

odrv0 = odrive.find_any()
P = odrv0.axis0.controller.config.pos_gain
I = odrv0.axis0.controller.config.vel_integrator_gain
odrv0.axis0.requested_state = AXIS_STATE_IDLE
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
odrv0.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
odrv0.axis0.controller.config.vel_integrator_gain = 0.5 * 10 * 0.08
odrv0.axis0.controller.config.pos_gain = 100
odrv0.axis0.controller.config.vel_gain = 0.08
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.start_anticogging_calibration()

print("Starting calibration...")
cont = odrv0.axis0.controller.config.anticogging
while cont.calib_anticogging == True:
    time.sleep(1.0)
    cont = odrv0.axis0.controller.config.anticogging
    print("...", end="")

odrv0.axis0.requested_state = AXIS_STATE_IDLE
odrv0.axis0.controller.config.pos_gain = P
odrv0.axis0.controller.config.vel_integrator_gain = I
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

print("Anti clogging calibration done.")
print("Evaluate the calib by sending the following commands.")
print("odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL")
print("odrv0.axis0.controller.input_vel = 0.1")
print("The configuration can be saved with odrive0.save_configuration()")
