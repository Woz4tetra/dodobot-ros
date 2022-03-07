import math
import time
import numpy as np
from plotter import ParticleFilterPlotter3D, ParticleFilterPlotter2D

from state_loader import read_pkl
from particle_filter import ParticleFilter


class InputVector:
    def __init__(self):
        self.odom_vx = 0.0
        self.odom_vy = 0.0
        self.odom_vt = 0.0
        self.odom_t = 0.0
        self.dist = 0.0
        self.friction = 0.0
        self.u = [0.0, 0.0, 0.0]  # input state vector: dx, dy, dz

        self.prev_stamp = None

        # self.u[4] = self.friction
        # self.u[5] = self.friction

    def update(self, state):
        dt = self.dt(state.stamp)
        angle = -state.t
        rot_mat = np.array([
            [np.cos(angle), -np.sin(angle)],
            [np.sin(angle), np.cos(angle)]
        ])
        velocity = np.array([state.vx, state.vy])
        rot_vel = np.dot(rot_mat, velocity)
        self.odom_vx = -rot_vel[0]
        self.odom_vy = -rot_vel[1]  # ~0.0
        self.odom_vt = -state.vt

        self.u[0] = self.odom_vx
        self.u[1] = self.odom_vt

        return dt

    def dt(self, timestamp):
        if self.prev_stamp is None:
            self.prev_stamp = timestamp
        dt = timestamp - self.prev_stamp
        self.prev_stamp = timestamp
        return dt


def main():
    # simple, single object demo (blue_cut_sphere)
    # path = "data/objects_2021-01-06-23-36-19.json"
    # path = "data/objects_2021-01-06-23-37-06.json"

    # multi object
    # path = "data/objects_2021-01-29-22-33-40.json"
    # path = "data/objects_2021-01-29-22-59-23.json"

    # pan-tilt demo
    # path = "data/objects_2021-01-29-22-35-02.json"

    # moving objects behind the robot's back
    path = "data/objects_2021-01-29-23-06-04.json"

    # moving objects in front of the robot

    # repickle = True
    repickle = False
    states = read_pkl(path, repickle)
    run_pf(states)


def init_pf(num_particles, meas_std_val, initial_range, initial_state):
    pf = ParticleFilter(num_particles, meas_std_val)
    pf.create_uniform_particles(initial_state, initial_range)
    return pf


def run_pf(states):
    initial_range = [1.0, 1.0, 1.0]
    meas_std_val = 0.007
    num_particles = 250
    u_std = [0.007, 0.007, 0.007]

    labels = [
        "BACKGROUND",
        "cozmo_cube",
        "blue_cut_sphere",
        "red_cut_sphere",
        "blue_low_bin",
        "red_low_bin",
        "blue_cube",
        "red_cube",
    ]

    pfs = {}

    input_vector = InputVector()

    sim_start_t = states[0].stamp
    real_start_t = time.time()

    x_width = 10.0
    y_width = 10.0
    z_width = 3.0

    # plotter = ParticleFilterPlotter3D(x_width, y_width, z_width)
    plotter = ParticleFilterPlotter2D(x_width, y_width)

    for state in states:
        sim_time = state.stamp
        real_time = time.time()
        sim_duration = sim_time - sim_start_t
        real_duration = real_time - real_start_t

        if state.type == "odom":
            dt = input_vector.update(state)
            for pf in pfs.values():
                pf.predict(input_vector.u, u_std, dt)
            plotter.update_odom(state)
        elif state.type in labels:
        # elif state.type == "blue_cut_sphere":
            name = state.type
            print(state)
            z = [state.x, state.y, state.z]
            if name not in pfs:
                pf = init_pf(num_particles, meas_std_val, initial_range, z)
                pfs[name] = pf
            pfs[name].update(z)
            plotter.update_measure(name, state)

        for pf in pfs.values():
            pf.check_resample()

        if sim_duration >= real_duration:  # if simulation time has caught up to real time, spend some time drawing
            plotter.clear()
            for name, pf in pfs.items():
                plotter.draw(pf, name)
            plotter.pause()
    plotter.stop()


if __name__ == '__main__':
    main()
