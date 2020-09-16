import rospy

class PID(object):
    def __init__(self):
        self.K_ff = 0.0
        self.deadzone = 0.0
        self.upper_bound = None
        self.lower_bound = None

        self.target = 0.0
        self.error_sum = 0.0
        self.prev_error = 0.0
        self.out = 0.0

        self.feedforward = 0.0
        self.prev_setpoint_time = 0.0

        self.prev_update_time = rospy.Time.now()
        self.current_time = rospy.Time.now()
        self.dt = 0.0

        self.Kp = 1.0
        self.Kd = 0.0
        self.Ki = 0.0

    def set(self, target):
        self.feedforward = self.K_ff * target
        self.target = target
        self.prev_setpoint_time = rospy.Time.now()
        self.prev_update_time = rospy.Time.now()

    def limit(self, value):
        if self.upper_bound is not None and value > self.upper_bound:
            return self.upper_bound
        elif self.lower_bound is not None and value < self.lower_bound:
            return self.lower_bound
        return value

    def update(self, measurement):
        error = self.target - measurement
        self.current_time = rospy.Time.now()
        self.dt = (self.current_time - self.prev_update_time).to_sec()
        self.prev_update_time = self.current_time

        self.out = 0.0
        if abs(target) < self.deadzone:
            return self.out

        if (self.Kp != 0.0):
            self.out += self.Kp * error
        elif (self.Kd != 0.0):
            self.out += self.Kd * (error - self.prev_error) / self.dt
        elif (self.Ki != 0.0):
            self.out += self.Ki * error_sum * self.dt
            error_sum += error
        self.out += feedforward

        return self.limit(self.out)
