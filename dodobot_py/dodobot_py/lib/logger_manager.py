import logging
import datetime
from logging import handlers

from .config import ConfigManager

log_config = ConfigManager.get_log_config()


class MyFormatter(logging.Formatter):
    converter = datetime.datetime.fromtimestamp

    def formatTime(self, record, datefmt=None):
        ct = self.converter(record.created)
        if datefmt:
            s = ct.strftime(datefmt)
        else:
            s = ct.strftime("%Y-%m-%dT%H:%M:%S,%f")
            # s = "%s,%03d" % (t, record.msecs)
        return s


class LoggerManager:
    logger = None

    def __init__(self):
        raise Exception("{} is class only".format(self.__class__.__name__))

    @classmethod
    def get_logger(cls, log_only=False):
        if cls.logger is not None:
            return cls.logger
        cls.logger = cls._create_logger(**log_config.to_dict(), log_only=log_only)
        return cls.logger

    @staticmethod
    def _create_logger(**kwargs):
        name = kwargs["name"]
        path = kwargs["path"]
        level = kwargs["level"]
        format = kwargs["format"]
        suffix = kwargs["suffix"]
        log_only = kwargs["log_only"]

        logger = logging.getLogger(name)
        logger.setLevel(level)

        formatter = MyFormatter(format)

        rotate_handle = handlers.TimedRotatingFileHandler(
            path,
            when="midnight", interval=1
        )
        rotate_handle.setLevel(level)
        rotate_handle.setFormatter(formatter)
        rotate_handle.suffix = suffix
        logger.addHandler(rotate_handle)

        if not log_only:  # add printing to stdout if this is not true
            print_handle = logging.StreamHandler()
            print_handle.setLevel(level)
            print_handle.setFormatter(formatter)
            logger.addHandler(print_handle)

        return logger
