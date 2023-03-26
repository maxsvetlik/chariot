from chariot.control.odrive_manager import OdriveMotorManager
import time
import structlog

"""This is an example script that clears all ODrive error flags."""

LOG = structlog.get_logger()

if __name__ == "__main__":
    mgr = OdriveMotorManager()
    mgr.clear_motor_errors()
    LOG.info("Request to clear motor flags sent.")
