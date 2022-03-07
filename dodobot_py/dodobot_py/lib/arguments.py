import argparse


def init():
    parser = argparse.ArgumentParser(description="dodobot_py")
    parser.add_argument("base_dir",
                        help="Base directory for writing logs and reading config files")
    parser.add_argument("--log_only", action='store_true',
                        help="If set, this session will only write to logs instead of printing to stdout and logs")

    args = parser.parse_args()

    from .config import ConfigManager
    ConfigManager.init_configs(args.base_dir)

    from .logger_manager import LoggerManager
    LoggerManager.get_logger(args.log_only)
