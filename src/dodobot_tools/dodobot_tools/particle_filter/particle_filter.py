import time
import numpy as np
from numpy.random import randn, random, uniform, normal
import scipy.stats
import collections
from threading import Lock
from .state import FilterState


FilterSerial = collections.namedtuple("FilterSerial", "label index")

G_CONST = 9.81


def predict(particles, input_std_error, num_particles, bounds, u, dt):
    """
    This is a static function so that it can be fed through numba's "jit" function (see jit_particle_filter.py)

    move according to control input u (velocity of robot and velocity of object)
    with noise std
    u[0, 1, 2, 3] = linear_vx, linear_vy, linear_vz, angular_z
    """
    x_0 = particles[:, 0]
    y_0 = particles[:, 1]
    z_0 = particles[:, 2]
    vx_0 = particles[:, 3]
    vy_0 = particles[:, 4]
    vz_0 = particles[:, 5]

    vx_u = -u[0]
    vy_u = -u[1]
    vz_u = -u[2]
    vt_u = -u[3]

    vx_sd_u = input_std_error[0]
    vy_sd_u = input_std_error[1]
    vz_sd_u = input_std_error[2]
    vt_sd_u = input_std_error[3]

    # angular predict
    theta_delta = vt_u * dt + normal(0.0, 1.0, num_particles) * vt_sd_u
    x_a = x_0 * np.cos(theta_delta) - y_0 * np.sin(theta_delta)
    y_a = x_0 * np.sin(theta_delta) + y_0 * np.cos(theta_delta)

    # x, y linear predict
    x_1 = x_a + vx_u * dt + normal(0.0, 1.0, num_particles) * vx_sd_u - vx_0 * dt
    y_1 = y_a + vy_u * dt + normal(0.0, 1.0, num_particles) * vy_sd_u - vy_0 * dt
    z_1 = z_0 + vz_u * dt + normal(0.0, 1.0, num_particles) * vz_sd_u - vz_0 * dt

    # apply downward accel if not on the ground
    lower_z = bounds[2][0]
    off_ground_indices = z_1 > lower_z
    z_1[off_ground_indices] -= G_CONST * dt * dt

    # linear velocity predict
    vx_1 = vx_u + normal(0.0, 1.0, num_particles) * vx_sd_u
    vy_1 = vy_u + normal(0.0, 1.0, num_particles) * vy_sd_u
    vz_1 = vz_u + normal(0.0, 1.0, num_particles) * vz_sd_u

    # apply downward accel if not on the ground
    vz_1[off_ground_indices] -= G_CONST * dt

    particles[:, 0] = x_1
    particles[:, 1] = y_1
    particles[:, 2] = z_1
    particles[:, 3] = vx_1
    particles[:, 4] = vy_1
    particles[:, 5] = vz_1


class ParticleFilter:
    def __init__(self, serial, num_particles, measure_std_error, input_std_error, stale_filter_time, bounds):
        self.serial = serial
        self.num_states = 6  # x, y, z, vx, vy, vz
        self.particles = np.zeros((num_particles, self.num_states))
        self.num_particles = num_particles
        self.measure_std_error = measure_std_error
        self.input_std_error = np.array(input_std_error)
        self.stale_filter_time = stale_filter_time
        self.last_measurement_time = 0.0
        self.bounds = np.array(bounds)

        self.measure_distribution = scipy.stats.norm(0.0, self.measure_std_error)
        self.weights = np.array([])

        self.initialize_weights()

        self.lock = Lock()

    def reset(self):
        with self.lock:
            self.particles = np.zeros((self.num_particles, self.num_states))
            self.initialize_weights()

    def set_parameters(self, num_particles, measure_std_error, input_std_error, stale_filter_time):
        with self.lock:
            self.num_particles = num_particles
            self.measure_std_error = measure_std_error
            self.input_std_error = np.array(input_std_error)
            self.stale_filter_time = stale_filter_time
        self.reset()

    def get_name(self):
        return "%s_%s" % (self.serial.label, self.serial.index)
    
    def is_initialized(self):
        with self.lock:
            return not np.all(self.particles == 0.0)

    def initialize_weights(self):
        # initialize with uniform weight
        self.weights = np.ones(self.num_particles)
        self.weights /= np.sum(self.weights)
        self.update_meas_timer()

    def create_uniform_particles(self, initial_state, state_range):
        assert len(initial_state) == self.num_states
        assert len(state_range) == self.num_states

        with self.lock:
            self.initialize_weights()
            for state_num in range(self.num_states):
                min_val = initial_state[state_num] - state_range[state_num]
                max_val = initial_state[state_num] + state_range[state_num]
                self.particles[:, state_num] = uniform(min_val, max_val, size=self.num_particles)

    def create_gaussian_particles(self, mean, var):
        with self.lock:
            self.initialize_weights()
            for state_num in range(self.num_states):
                self.particles[:, state_num] = mean[state_num] + randn(self.num_particles) * var[state_num]

    def predict(self, u, dt):
        """
        move according to control input u (velocity of robot and velocity of object)
        with noise std
        u[0, 1, 2, 3] = linear_vx, linear_vy, linear_vz, angular_z
        """
        with self.lock:
            predict(self.particles, self.input_std_error, self.num_particles, self.bounds, u, dt)
        # self.clip()

    def clip(self):
        with self.lock:
            for index, (lower, upper) in enumerate(self.bounds):
                axis_particles = self.particles[:, index]
                axis_particles[axis_particles < lower] = lower
                axis_particles[axis_particles > upper] = upper

    def update(self, z):
        """Update particle filter according to measurement z (object position: [x, y, z, vx, vy, vx])"""
        # self.weights.fill(1.0)  # debateable as to whether this is detrimental or not (shouldn't weights be preserved between measurements?)
        with self.lock:
            distances = np.linalg.norm(self.particles - z, axis=1)
            
            self.weights *= self.measure_distribution.pdf(distances)

            self.weights += 1.e-300  # avoid divide by zero error
            self.weights /= np.sum(self.weights)  # normalize
            self.update_meas_timer()

    def update_meas_timer(self):
        self.last_measurement_time = time.time()

    def is_stale(self):
        last_measurement_dt = time.time() - self.last_measurement_time
        with self.lock:
            return self.stale_filter_time is not None and last_measurement_dt > self.stale_filter_time

    def neff(self):
        with self.lock:
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

        with self.lock:
            self.particles = self.particles[indices]
            self.weights = self.weights[indices]
            self.weights /= np.sum(self.weights)

    def estimate(self):
        """ returns mean and variance """
        mu = self.mean()
        with self.lock:
            var = np.average((self.particles - mu) ** 2, weights=self.weights, axis=0)

        return mu, var

    def mean(self):
        """ returns weighted mean position"""
        with self.lock:
            return np.average(self.particles, weights=self.weights, axis=0)

    def get_state(self) -> FilterState:
        mu = self.mean()
        return FilterState(mu[0], mu[1], mu[2], 0.0, mu[3], mu[4], mu[5], 0.0)

    def check_resample(self):
        neff = self.neff()
        with self.lock:
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
