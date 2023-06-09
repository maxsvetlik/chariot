"""
author: Corrie Van Sice, 2023
"""

from chariot.control.odrive_manager import OdriveMotorManager
import time
import structlog
import matplotlib.pyplot as plt
import matplotlib.animation as animation

"""This is an example script that prints all errors to the log."""
LOG = structlog.get_logger()

# Motor IDs
BACK_LEFT = 0
BACK_RIGHT = 1
FRONT_LEFT = 3
FRONT_RIGHT = 2

motor_name = ["BL", "BR", "FR", "FL"]

axis_ids = [BACK_LEFT, BACK_RIGHT, FRONT_RIGHT, FRONT_LEFT]
pos_estimates = list()
vel_estimates = list()  # Velocity estimates using the encoder
voltages = list()  # The controller's VBus voltage
axis_states = list()  # The state the axis is in
errors = list()
last_errors = list()
controller_error = False
encoder_error = False
motor_error = False
odrive_error = False

def clear_lists():
    pos_estimates.clear()
    vel_estimates.clear()
    voltages.clear()
    axis_states.clear()
    errors.clear()
    controller_error = False
    encoder_error = False
    motor_error = False
    odrive_error = False

# def plot_data():


if __name__ == "__main__":
    mgr = OdriveMotorManager()
    
    try:
        with open('../logs/' + time.strftime("%Y-%m-%d_%H:%M:%S") + ".log", 'w') as f:
            
            try:
                while True: 
                    # msgs = mgr.get_motor_info()
                    #t = time.strftime("%H:%M:%S")
                    t = time.time()
                    heartbeats = mgr.get_heartbeat()
                    estimates = mgr.get_encoder_estimate()
                    voltages = mgr.get_vbus_voltage()

                    for id in axis_ids:
                        pos_estimates.append(estimates[id].pos_estiamte)
                        vel_estimates.append(estimates[id].vel_estimate)
                        axis_states.append(heartbeats[id].axis_state)

                        if (heartbeats[id].controller_flags != '0x0'):
                            controller_error = True
                            errors.append("Motor " + str(id) + motor_name[id] + ", Controller error")
                            errors.append(heartbeats[id].controller_flags)
                        if (heartbeats[id].encoder_flags != '0x0'):
                            encoder_error = True
                            errors.append("Motor " + str(id) + motor_name[id] + ", Encoder error")
                            errors.append(heartbeats[id].encoder_flags)
                        if (heartbeats[id].motor_flags != '0x0'):
                            motor_error = True
                            errors.append("Motor " + str(id) + motor_name[id] + ", Motor error")
                            errors.append(heartbeats[id].motor_flags)
                        if (heartbeats[id].axis_error != '0x0'):
                            odrive_error = True
                            errors.append("Motor " + str(id) + motor_name[id] + ", Axis error")
                            errors.append(heartbeats[id].axis_error)

                    f.write(str(t) + ", positions: " + str(pos_estimates) + "\n")
                    f.write(str(t) + ", velocities: " + str(vel_estimates) + "\n")
                    f.write(str(t) + ", voltages: " + str(voltages) + "\n")

                    if (controller_error | encoder_error | motor_error | odrive_error):
                        if (errors != last_errors):
                            f.write(str(t) + ", errors: " + str(errors) + "\n")
                            last_errors = errors

                    clear_lists()

            except KeyboardInterrupt:
                print("Shutdown signal received. Ending execution.")

    except FileNotFoundError:
        print("The 'logs' directory does not exist")


