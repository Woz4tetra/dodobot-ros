import threading

from lib.logger_manager import LoggerManager

logger = LoggerManager.get_logger()


class Task:
    def __init__(self, task_fn):
        self.should_stop = False
        self.task_fn = task_fn
        self.should_stop_fn = (lambda: self.should_stop,)
        self.thread = threading.Thread(target=self.run, args=self.should_stop_fn)
        self.thread_exception = None
        self.is_finished = False

    def start(self):
        self.thread.start()

    def stop(self):
        self.should_stop = True

    def is_errored(self):
        return self.thread_exception is None

    def run(self, should_stop):
        try:
            result = self.task_fn(should_stop)
            if result is not None:
                self.thread_exception = result
        except BaseException as e:
            logger.error(str(e), exc_info=True)
            self.thread_exception = e
        self.is_finished = True
