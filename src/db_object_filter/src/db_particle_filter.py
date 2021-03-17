import numpy as np
from numpy.random import randn, random, uniform
import scipy.stats
import matplotlib.pyplot as plt


class ParticleFilter(object):
    def __init__(self, num_particles, measure_std_error, input_std_error):
        self.num_states = 3  # x, y, z
        self.particles = np.zeros((num_particles, self.num_states))
        self.num_particles = num_particles
        self.measure_std_error = measure_std_error
        self.input_std_error = input_std_error

        # distribute particles randomly with uniform weight
        self.weights = np.ones(num_particles)
        self.weights /= sum(self.weights)
        
    def create_uniform_particles(self, initial_state, state_range):
        assert len(initial_state) == self.num_states
        assert len(state_range) == self.num_states

        for state_num in range(self.num_states):
            min_val = initial_state[state_num] - state_range[state_num]
            max_val = initial_state[state_num] + state_range[state_num]
            self.particles[:, state_num] = uniform(min_val, max_val, size=self.num_particles)

    def create_gaussian_particles(self, mean, var):
        for state_num in range(self.num_states):
            self.particles[:, state_num] = mean[state_num] + randn(self.num_particles) * var[state_num]

    def predict(self, u, dt):
        """
        move according to control input u (velocity of robot and velocity of object)
        with noise std
        u[0, 1, 2, 3] = linear_vx * dt, 0.0 (no Y component), 0.0 (no Z component), angular_z * dt
        """
        # angular update
        th_dot = u[3] * dt + randn(self.num_particles) * self.input_std_error[3]
        self.particles[:, 0] = self.particles[:, 0] * np.cos(th_dot) - self.particles[:, 1] * np.sin(th_dot)
        self.particles[:, 1] = self.particles[:, 0] * np.sin(th_dot) + self.particles[:, 1] * np.cos(th_dot)
        
        # linear update
        self.particles[:, 0] += u[0] * dt + randn(self.num_particles) * self.input_std_error[0]
        self.particles[:, 1] += u[1] * dt + randn(self.num_particles) * self.input_std_error[1]
        self.particles[:, 2] += u[2] * dt + randn(self.num_particles) * self.input_std_error[2]

    def update(self, z):
        """Update particle filter according to measurement z (object position)"""
        # self.weights.fill(1.0)

        # weight according to how far away the particle is from the measurement in x, y, z
        for axis in range(self.num_states):
            axis_val = self.particles[:, axis]
            self.weights *= scipy.stats.norm(axis_val, self.measure_std_error).pdf(z[axis])

        self.weights += 1.e-300  # avoid divide by zero error
        self.weights /= sum(self.weights)  # normalize
        # print "weights:", self.weights

    def neff(self):
        return 1.0 / np.sum(np.square(self.weights))

    def resample(self):
        cumulative_sum = np.cumsum(self.weights)
        cumulative_sum[-1] = 1.0  # avoid round-off error
        indexes = np.searchsorted(cumulative_sum, random(self.num_particles))
        # indexes = self.systemic_resample(self.weights)

        # resample according to indexes
        self.particles = self.particles[indexes]
        self.weights = self.weights[indexes]
        self.weights /= np.sum(self.weights)  # normalize

    def resample_from_index(self, indexes):
        assert len(indexes) == self.num_particles

        self.particles = self.particles[indexes]
        self.weights = self.weights[indexes]
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
        if self.neff() < self.num_particles / 2:
            self.resample()
            return True
        else:
            return False

    def systemic_resample(self, w):
        N = len(w)
        Q = np.cumsum(w)
        indexes = np.zeros(N, 'int')
        t = np.linspace(0, 1 - 1 / N, N) + random() / N

        i, j = 0, 0
        while i < N and j < N:
            while Q[j] < t[i]:
                j += 1
            indexes[i] = j
            i += 1

        return indexes
