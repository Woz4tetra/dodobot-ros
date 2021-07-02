import time
import numpy as np
from numpy.random import randn, random, uniform
import scipy.stats
import collections


FilterSerial = collections.namedtuple("FilterSerial", "label index")



class ParticleFilter(object):
    def __init__(self, serial, num_particles, measure_std_error, input_std_error, stale_filter_time):
        self.serial = serial
        self.num_states = 3  # x, y, z
        self.particles = np.zeros((num_particles, self.num_states))
        self.num_particles = num_particles
        self.measure_std_error = measure_std_error
        self.input_std_error = np.array(input_std_error)
        self.last_measurement_time = 0.0
        self.stale_filter_time = stale_filter_time
        self.is_stale = False

        self.measure_distribution = scipy.stats.norm(0.0, self.measure_std_error)

        self.initialize_weights()
    
    def is_initialized(self):
        return not np.all(self.particles == 0.0)

    def initialize_weights(self):
        # initialize with uniform weight
        self.weights = np.ones(self.num_particles)
        self.weights /= np.sum(self.weights)

    def create_uniform_particles(self, initial_state, state_range):
        assert len(initial_state) == self.num_states
        assert len(state_range) == self.num_states

        self.initialize_weights()
        for state_num in range(self.num_states):
            min_val = initial_state[state_num] - state_range[state_num]
            max_val = initial_state[state_num] + state_range[state_num]
            self.particles[:, state_num] = uniform(min_val, max_val, size=self.num_particles)

    def create_gaussian_particles(self, mean, var):
        self.initialize_weights()
        for state_num in range(self.num_states):
            self.particles[:, state_num] = mean[state_num] + randn(self.num_particles) * var[state_num]

    def predict(self, u):
        """
        move according to control input u (velocity of robot and velocity of object)
        with noise std
        u[0, 1, 2, 3] = linear_vx * dt, 0.0 (no Y component), 0.0 (no Z component), angular_z * dt
        """
        # if self.is_filter_stale():
        #     return
        # angular update
        th_dot = u[3] + randn(self.num_particles) * self.input_std_error[3]
        self.particles[:, 0] = self.particles[:, 0] * np.cos(th_dot) - self.particles[:, 1] * np.sin(th_dot)
        self.particles[:, 1] = self.particles[:, 0] * np.sin(th_dot) + self.particles[:, 1] * np.cos(th_dot)
        
        # linear update
        self.particles[:, 0] += u[0] + randn(self.num_particles) * self.input_std_error[0]
        self.particles[:, 1] += u[1] + randn(self.num_particles) * self.input_std_error[1]
        self.particles[:, 2] += u[2] + randn(self.num_particles) * self.input_std_error[2]

        return

    def update(self, z):
        """Update particle filter according to measurement z (object position: [x, y, z])"""
        # weight according to how far away the particle is from the measurement in x, y, z
        self.weights.fill(1.0)
        distances = np.linalg.norm(self.particles - z, axis=1)
        self.weights *= self.measure_distribution.pdf(distances)

        self.weights += 1.e-300  # avoid divide by zero error
        self.weights /= np.sum(self.weights)  # normalize
        self.last_measurement_time = time.time()

    def is_filter_stale(self):
        last_measurement_dt = time.time() - self.last_measurement_time
        self.is_stale = self.stale_filter_time is not None and last_measurement_dt > self.stale_filter_time
        return self.is_stale

    def neff(self):
        return 1.0 / np.sum(np.square(self.weights))

    def resample(self):
        # indices = self.simple_resample()
        indices = self.systematic_resample()

        # resample according to indices
        self.particles = self.particles[indices]
        self.weights = self.weights[indices]
        self.weights /= np.sum(self.weights)  # normalize

    def resample_from_index(self, indices):
        assert len(indices) == self.num_particles

        self.particles = self.particles[indices]
        self.weights = self.weights[indices]
        self.weights /= np.sum(self.weights)

    def estimate(self):
        """ returns mean and variance """
        mu = self.mean()
        var = np.average((self.particles - mu) ** 2, weights=self.weights, axis=0)

        return mu, var

    def mean(self):
        """ returns weighted mean position"""
        return np.average(self.particles, weights=self.weights, axis=0)

    def check_resample(self):
        # if self.is_stale:
        #     return False
        neff = self.neff()
        if neff < self.num_particles / 2.0:
            self.resample()
            return True
        else:
            return False

    def simple_resample(self):
        cumulative_sum = np.cumsum(self.weights)
        cumulative_sum[-1] = 1.0  # avoid round-off error
        indices = np.searchsorted(cumulative_sum, random(self.num_particles))
        return indices

    def systematic_resample(self):
        cumulative_sum = np.cumsum(self.weights)
        indices = np.zeros(self.num_particles, 'int')
        t = np.linspace(0, 1.0 - 1.0 / self.num_particles, self.num_particles) + random() / self.num_particles

        i, j = 0, 0
        while i < self.num_particles and j < self.num_particles:
            while cumulative_sum[j] < t[i]:
                j += 1
            indices[i] = j
            i += 1

        return indices
