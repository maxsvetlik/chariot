from chariot.control.odrive_manager import OdriveMotorManager
import time
import structlog

"""This is an example script that prints all errors to the log."""

LOG = structlog.get_logger()

if __name__ == "__main__":
    mgr = OdriveMotorManager()

    try:
        msgs = mgr.get_motor_info()
        LOG.info("Received messages", heartbeats=msgs)
    except KeyboardInterrupt:
        print("Shutdown signal received. Ending execution.")
