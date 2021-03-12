import rospy
import threading
import numpy as np
import scipy.stats
from db_particle_filter import ParticleFilter


class ObjectFilter(object):
    def __init__(self, name, label, num_particles, meas_std_val, input_std, initial_state, initial_range):
        self.name = name
        self.label = label
        self.pf = ParticleFilter(num_particles, meas_std_val, input_std)
        self.pf.create_uniform_particles(initial_state, initial_range)
        self.lock = threading.Lock()
    
    def update(self, measurement):
        with self.lock:
            self.pf.update(measurement)

    def predict(self, input_vector, dt):
        with self.lock:
            self.pf.predict(input_vector, dt)
        
    def mean(self):
        with self.lock:
            return self.pf.mean()
    
    def estimate(self):
        with self.lock:
            return self.pf.estimate()
    
    def particles(self):
        with self.lock:
            return self.pf.particles
    
    def check_resample(self):
        with self.lock:
            self.pf.check_resample()


class FilterFactory(object):
    def __init__(self, num_particles, meas_std_val, input_std, initial_range, 
            match_cov, match_threshold, new_filter_threshold, max_num_filters):
        self.num_particles = num_particles
        self.meas_std_val = meas_std_val
        self.input_std = input_std
        self.initial_range = initial_range

        self.match_cov = match_cov
        self.match_threshold = match_threshold
        self.new_filter_threshold = new_filter_threshold
        self.max_num_filters = max_num_filters

        self.filters = {}

    def get_filter_name(self, label, index):
        return "%s_%s" % (label, index)

    def init_filter(self, label, initial_state):
        filter_index = 0
        for obj_filter in self.filters:
            if obj_filter.label == label:
                filter_index += 1
        filter_name = self.get_filter_name(label, filter_index)

        if label not in self.filters:
            self.filters[label] = []
        
        while len(self.filters[label]) >= self.max_num_filters:
            self.remove_least_confidence_filter(label)

        obj_filter = ObjectFilter(
            filter_name, label,
            self.num_particles,
            self.meas_std_val, self.input_std,
            initial_state, self.initial_range
        )
        self.filters[label].append(obj_filter)
    
    def remove_least_confidence_filter(self, label):
        index_to_remove = None
        smallest_variance = None
        for index, obj_filter in enumerate(self.filters[label]):
            mean, variance = obj_filter.estimate()
            if smallest_variance is None:
                smallest_variance = variance
                index_to_remove = index
            elif variance < smallest_variance:
                smallest_variance = variance
                index_to_remove = index
        self.filters.pop(index_to_remove)
    
    def get_particles(self, label, filter_index):
        if label in self.filters and 0 <= filter_index < len(self.filters[label]):
            return self.filters[label][filter_index].particles()
        else:
            return None
    
    def get_all_particles(self):
        particles = []
        for label in self.filters:
            for obj_filter in self.filters[label]:
                particles.extend(obj_filter.particles())
        return particles
        
    def get_means(self):
        means = {}
        for label in self.filters:
            for obj_filter in self.filters[label]:
                means[obj_filter.name] = obj_filter.mean()
        return means

    def get_label_means(self, label):
        means = []
        for obj_filter in self.filters[label]:
            means.append(obj_filter.mean())
        return means

    def update(self, label, measurement):
        if len(self.filters) == 0 or len(self.filters[label]) == 0:
            self.init_filter(label, measurement)
            return

        means = self.get_label_means(label)
        for filter_index, mean in enumerate(means):
            confidence = scipy.stats.multivariate_normal.pdf(measurement, mean=mean, cov=self.match_cov)
            rospy.loginfo("%s confidence: %s" % (label, confidence))
            # confidence = scipy.stats.multivariate_normal.cdf(measurement, mean=mean, cov=self.match_cov)
            if confidence > self.match_threshold:
                obj_filter = self.filters[label][filter_index]
                rospy.loginfo("Measurement matches %s" % obj_filter.name)
                obj_filter.update(measurement)
                break
            elif confidence < self.new_filter_threshold:
                rospy.loginfo("Measurement doesn't match filter. Creating a new one")
                self.init_filter(label, measurement)
                break

    def predict(self, input_vector, dt):
        for label in self.filters:
            for obj_filter in self.filters[label]:
                obj_filter.predict(input_vector, dt)

    def check_resample(self):
        for label in self.filters:
            for obj_filter in self.filters[label]:
                obj_filter.check_resample()

