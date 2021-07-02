import time
import numpy as np
from numpy.random import randn, random, uniform
import scipy.stats

from numba import njit

from .db_particle_filter import ParticleFilter


@njit
def jit_predict(particles, num_particles, input_std_error, u, dt):
    # angular update
    th_dot = u[3] * dt + randn(num_particles) * input_std_error[3]
    particles[:, 0] = particles[:, 0] * np.cos(th_dot) - particles[:, 1] * np.sin(th_dot)
    particles[:, 1] = particles[:, 0] * np.sin(th_dot) + particles[:, 1] * np.cos(th_dot)
    
    # linear update
    particles[:, 0] += u[0] * dt + randn(num_particles) * input_std_error[0]
    particles[:, 1] += u[1] * dt + randn(num_particles) * input_std_error[1]
    particles[:, 2] += u[2] * dt + randn(num_particles) * input_std_error[2]

@njit
def jit_update(particles, z, num_particles):
    # weight according to how far away the particle is from the measurement in x, y, z
    diff = particles - z
    distances = np.zeros(num_particles)

    for index in range(num_particles):
        distances[index] = np.linalg.norm(diff[index])
    return distances

@njit
def jit_normalize_weights(weights):
    weights += 1.e-300  # avoid divide by zero error
    weights /= np.sum(weights)  # normalize
    return weights


@njit
def jit_systematic_resample(weights, num_particles):
    cumulative_sum = np.cumsum(weights)
    indices = np.zeros(num_particles, np.int32)
    t = np.linspace(0, 1.0 - 1.0 / num_particles, num_particles) + random() / num_particles

    i, j = 0, 0
    while i < num_particles and j < num_particles:
        while cumulative_sum[j] < t[i]:
            j += 1
        indices[i] = j
        i += 1

    return indices

@njit
def jit_resample(particles, weights, num_particles):
    indices = jit_systematic_resample(weights, num_particles)

    # resample according to indices
    particles = particles[indices]
    weights = weights[indices]
    weights /= np.sum(weights)  # normalize

class JitParticleFilter(ParticleFilter):
    def __init__(self, serial, num_particles, measure_std_error, input_std_error, stale_filter_time):
        super(JitParticleFilter, self).__init__(serial, num_particles, measure_std_error, input_std_error, stale_filter_time)

    def predict(self, u, dt):
        # if self.is_filter_stale():
        #     return False
        jit_predict(self.particles, self.num_particles, self.input_std_error, u, dt)
        return True

    def update(self, z):
        self.weights.fill(1.0)
        distances = jit_update(self.particles, z, self.num_particles)
        self.weights *= self.measure_distribution.pdf(distances)
        self.weights = jit_normalize_weights(self.weights)

    # TODO: figure out why numba-fied version of resample causes filter to become more unstable
    # def resample(self):
    #     jit_resample(self.particles, self.weights, self.num_particles)
