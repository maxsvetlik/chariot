
# Configuration settings for a new ODrive Pro

This file references the commands required to set up a new Odrive Pro with a AMT21-series encoder and
a OdriveRobotics D5065 motor. In practice, it is more reasonable to load one of the configs already present in this repo
to an ODrive Pro rather than configure it from scratch. 

Still, this serves as a guide. The code snippets are written as python commands using the `odrive` Python library. This
is generally available only via USB connection with the odrive.
The USB interface has full support to configure the ODrive, while things like CAN has only a small subset. For configuration
of the ODrive, you simply must interface over USB. An alternative to using the Python library would be to use `odrivetool`,
a commandline-esque wrapper to the Python library.

No code snippet should be run as a script, as some commands must complete before continuing forward to the next command.

## USB Connection
It is a requirement that you use a USB isolator between the ODrive and your PC. Note that `odrivetool` has a dependency on
Python `3.9` and higher.

## Setting up motor and encoder
```
    # Set IDLE so we can modify settings
    odrv0.axis0.requested_state = AxisState.IDLE

    # Set motor parameters for the D5065
    odrv0.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
    odrv0.axis0.config.motor.pole_pairs = 7
    odrv0.axis0.config.motor.torque_constant = 8.27 / 270

    # Start calibration
    odrv0.axis0.requested_state = AxisState.MOTOR_CALIBRATION
    # [wait for end of motor beep]
    odrv0.save_configuration()

    # Setup the encoder
    odrv0.amt21_encoder_group0.config.event_driven_mode = False
    odrv0.amt21_encoder_group0.config.enable = True
    odrv0.axis0.config.load_encoder = EncoderId.AMT21_ENCODER0
    odrv0.axis0.config.commutation_encoder = EncoderId.AMT21_ENCODER0
    odrv0.save_configuration()
    # [wait for ODrive to reboot]
    odrv0.axis0.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
    # [wait for motor to stop]
    odrv0.save_configuration()
```

## Thermistor
Only if the motor is using one. The D5065 has one installed by default.
```
    set_motor_thermistor_coeffs(odrv0.axis0, Rload=1000, R_25=10000, Beta=3435, Tmin=-10, Tmax=150, thermistor_bottom=True)
    # thermistor must be connected
    odrv0.axis0.motor_thermistor.config.enabled = True
```
## Setting control limits limits
This changes with your motor. For the D5065, 65A is its hard maximum.

```
    odrv0.axis0.config.motor.current_hard_max = 65
    odrv0.axis0.config.motor.current_soft_max = 50
    odrv0.axis0.controller.config.vel_limit = 70
```
## Setting power source limits

### Benchtop PSU
    """BENCHTOP PSU LIMIT SETTINGS
    # These must be used if powering motors from anything other than a battery
    odrv0.config.dc_max_positive_current = 10
    odrv0.config.dc_bus_overvoltage_trip_level = 27 # DC benchtop PSU
    odrv0.config.dc_max_negative_current = -1.0 # Amps. The rec. is current limit + current margin for PSUs
    """

### Battery
```
    # Voltage Limits
    odrv0.config.dc_bus_undervoltage_trip_level = 23.1
    odrv0.config.dc_bus_overvoltage_trip_level = 29.95

    # Current Limits
    odrv0.config.dc_max_positive_current = 25 # max discharge current
    odrv0.config.dc_max_negative_current = -25 # max charging current
```
## Set velocity ramp acceleration profile
Set the velocity ramp limits so sudden accelerations don't trip the motor's `bus_overvoltage_trip_level`
```
    odrv0.axis0.controller.config.vel_ramp_rate = 80
    odrv0.save_configuration()
```

## Setup CAN
If using CAN for communication. Note that the `node_id` must be unique for all nodes on your bus.
```
    odrv0.axis0.requested_state = AxisState.IDLE
    odrv0.config.enable_can_a = True
    odrv0.axis0.config.can.node_id = <number>
    odrv0.can.config.baud_rate = 500000 #500 kbps
    odrv0.save_configuration()
```

# Settings on every boot
This sets up your motor to be in the desired control mode. These calls should be a part of your main software
control loop as part of starting the controllers before they are engaged.
```
    odrv0.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
    odrv0.axis0.controller.config.input_mode = InputMode.VEL_RAMP
    odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
```
