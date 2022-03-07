import os
import sys
import time
import subprocess

from lib import arguments
arguments.init()  # initialize ConfigManager and LoggerManager

from lib.config import ConfigManager
from lib.logger_manager import LoggerManager
from lib.exceptions import *
from lib.session import Session


logger = LoggerManager.get_logger()
general_config = ConfigManager.get_general_config()


def shutdown(session):
    logger.warn("Shutdown function called. Shutting down everything.")
    session.stop()
    subprocess.call("sudo shutdown -h now", shell=True)

def reboot(session):
    logger.warn("Reboot function called. Shutting down everything.")
    session.stop()
    subprocess.call("sudo reboot now", shell=True)

def relaunch(session):
    logger.warn("Relaunch function called.")
    session.stop()

    # os.execv(sys.argv[0], sys.argv)
    try:
        p = psutil.Process(os.getpid())
        for handler in p.get_open_files() + p.connections():
            os.close(handler.fd)
    except BaseException as e:
        logger.error(e, exc_info=True)

    python = sys.executable
    os.execl(python, python, *sys.argv)

def close(session):
    logger.info("Close function called. Exiting\n\n")
    session.stop()


def main():
    logger.info("Starting robot")

    session = Session()
    update_delay = 1.0 / general_config.update_rate_hz
    try:
        session.start()
        while True:
            session.update()
            time.sleep(update_delay)
    except (LowBatteryException, ShutdownException) as e:
        logger.error(str(e), exc_info=True)
        shutdown(session)
    except RebootException as e:
        logger.error(str(e), exc_info=True)
        reboot(session)
    except (RelaunchException, DeviceRestartException) as e:
        logger.error(str(e), exc_info=True)
        relaunch(session)

    except BaseException as e:
        logger.error(str(e), exc_info=True)
        close(session)


if __name__ == "__main__":
    try:
        main()
    except BaseException as e:
        logger.error(str(e), exc_info=True)
