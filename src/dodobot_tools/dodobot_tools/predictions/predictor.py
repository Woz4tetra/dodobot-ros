import numpy as np
from tj2_tools.particle_filter.state import FilterState


def iter_pairs(l, reverse=False):
    if reverse:
        for index in range(len(l) - 1, 0, -1):
            yield l[index], l[index - 1]
    else:
        for index in range(0, len(l) - 1):
            yield l[index], l[index + 1]
    


class BouncePredictor:
    def __init__(self, v_max_robot, past_window_size, vx_std_dev_threshold, vy_std_dev_threshold):
        self.v_max_robot = v_max_robot
        self.past_window_size = past_window_size
        self.vx_std_dev_threshold = vx_std_dev_threshold
        self.vy_std_dev_threshold = vy_std_dev_threshold

        self.odom_buffer = []
        self.obj_buffer = []

    def update_buffers(self, obj_state: FilterState, odom_state: FilterState):
        self.odom_buffer.append(odom_state)
        while len(self.odom_buffer) > self.past_window_size:
            self.odom_buffer.pop(0)

        self.obj_buffer.append(obj_state)
        while len(self.obj_buffer) > self.past_window_size:
            self.obj_buffer.pop(0)

    def get_object_velocity(self, obj_buffer: list):
        sumx = sumy = sumxx = sumyy = sumxy = 0
        for state in obj_buffer:
            sumx += state.x
            sumy += state.y
            sumxx += state.x * state.x
            sumyy += state.y * state.y
            sumxy += state.x * state.y

        denox = self.past_window_size * sumxx - sumx * sumx
        denoy = self.past_window_size * sumyy - sumy * sumy

        first_state = obj_buffer[0]
        last_state = obj_buffer[-1]

        first_x = first_state.x
        first_y = first_state.y
        last_x = last_state.x
        last_y = last_state.y
        first_time = first_state.stamp
        last_time = last_state.stamp

        if denox > denoy:
            k = (self.past_window_size * sumxy - sumx * sumy) / denox if denox != 0.0 else 1000.0
            vx = (last_x - first_x) / (last_time - first_time)
            vy = vx * k
            vy = vy if vy * (last_y - first_y) > 0 else -vy
        else:
            k = denoy / (self.past_window_size * sumxy - sumx * sumy) if (self.past_window_size * sumxy - sumx * sumy) != 0.0 else 1000.0
            k = 1.0 / k
            vy = (last_y - first_y) / (last_time - first_time)
            vx = vy * k
            vx = vx if vx*(last_x - first_x) > 0 else -vx

        return vx, vy

    def get_robot_intersection(self, obj_state: FilterState, odom_state: FilterState):
        # plot a course to where the object will head
        # given robot parameters, find where the robot and object intersect

        self.update_buffers(obj_state, odom_state)

        if len(self.obj_buffer) < self.past_window_size:
            return obj_state
        if len(self.odom_buffer) < self.past_window_size:
            return obj_state

        vxs, vys = [], []
        for next_state, prev_state in iter_pairs(self.obj_buffer, reverse=True):
            delta_state = next_state - prev_state
            dt = next_state.stamp - prev_state.stamp
            if dt <= 0.0:
                continue
            obj_vx = delta_state.x / dt
            obj_vy = delta_state.y / dt
            vxs.append(obj_vx)
            vys.append(obj_vy)

        if np.std(vxs) > self.vx_std_dev_threshold:
            return obj_state
        if np.std(vys) > self.vy_std_dev_threshold:
            return obj_state

        proj_vx, proj_vy = self.get_object_velocity(self.obj_buffer)
        
        obj_dist = obj_state.distance()
        future_time_window = obj_dist / self.v_max_robot

        future_obj_state = FilterState.from_state(self.obj_buffer[-1])
        future_obj_state.stamp += future_time_window
        future_obj_state.x += proj_vx * future_time_window
        future_obj_state.y += proj_vy * future_time_window
        future_obj_state.z = 0.0
        future_obj_state.theta = future_obj_state.heading()
        future_obj_state.vx = proj_vx
        future_obj_state.vy = proj_vy
        future_obj_state.vz = 0.0

        return future_obj_state
