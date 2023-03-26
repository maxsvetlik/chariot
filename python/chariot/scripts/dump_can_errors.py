import time
from chariot.control.odrive_manager import OdriveMotorManager

if __name__ == "__main__":
    mgr = OdriveMotorManager(bring_up_interface=False)

    try:
        while True:
            mgr.dump_heartbeat()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Shutdown signal received. Ending execution.")
