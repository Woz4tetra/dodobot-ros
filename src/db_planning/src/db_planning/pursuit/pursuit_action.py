import rospy
from db_planning.robot_state import Pose2d


class PursuitAction:
    def __init__(self, parameters, set_velocity, get_state, should_stop):
        self.parameters = parameters
        self.set_velocity = set_velocity
        self.get_state = get_state
        self.should_stop = should_stop
        self.clock_rate = rospy.Rate(30.0)
        self._timeout = 0.0
        self.goal_pose = Pose2d()

    def set_goal(self, pose: Pose2d):
        self.goal_pose = pose

    def get_error(self, state: Pose2d) -> Pose2d:
        raise NotImplementedError

    def get_timeout(self, error: Pose2d):
        raise NotImplementedError

    def set_timeout(self, timeout: float):
        self._timeout = timeout

    def stop_motors(self):
        self.set_velocity(0.0, 0.0)

    def update(self, state: Pose2d, error: Pose2d):
        raise NotImplementedError

    def is_goal_reached(self, error: Pose2d):
        raise NotImplementedError

    def get_command(self, error: Pose2d):
        return 0.0, 0.0

    def now(self) -> float:
        return rospy.Time.now().to_sec()

    def run(self):
        state = self.get_state()
        error = self.get_error(state)

        timeout = self.get_timeout(error)
        rospy.loginfo("Pursuing goal: %s. Pursuing for %0.2fs at most" % (error, timeout))
        self.set_timeout(timeout)
        start_time = self.now()
        success_time = None

        while True:
            self.clock_rate.sleep()
            now = self.now()
            if self.should_stop():
                self.stop_motors()
                return "preempted"

            state = self.get_state()
            error = self.get_error(state)

            update_result = self.update(state, error)
            if update_result is not None:
                return update_result

            if self.is_goal_reached(error):
                self.stop_motors()
                if success_time is None:
                    rospy.loginfo("%s Tolerance reached. Waiting for stablization" % (self.__class__.__name__))
                    success_time = now
            else:
                success_time = None

            if now - start_time > self._timeout:
                rospy.logwarn("%s Timeout reaching. Giving up on pursuing object" % (self.__class__.__name__))
                self.stop_motors()
                return "failure"

            if success_time is not None:
                if now - success_time > self.parameters.stabilization_timeout:
                    self.stop_motors()
                    rospy.loginfo("%s Pursuit error: %s" % (self.__class__.__name__, error))
                    return "success"
                continue

            linear_v, ang_v = self.get_command(error)
            self.set_velocity(linear_v, ang_v)
