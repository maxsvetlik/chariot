from chariot.control.odrive_manager import OdriveMotorManager
import time
import structlog

LOG = structlog.get_logger()

if __name__ == "__main__":
    mgr = OdriveMotorManager()

    try:
        msgs = mgr.dump_heartbeat()
        LOG.info(heartbeats=msgs)
    except KeyboardInterrupt:
        print("Shutdown signal received. Ending execution.")
