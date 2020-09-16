
from db_planning.pid import PID

class BasicGuidance(object):
    def __init__(self):
        self.angle_pid = PID()
        self.dist_pid = PID()
