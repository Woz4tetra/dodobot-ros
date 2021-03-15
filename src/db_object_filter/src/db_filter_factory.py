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
        # self.pf.create_uniform_particles(initial_state, initial_range)
        self.pf.create_gaussian_particles(initial_state, initial_range)
        # self.lock = threading.Lock()
    
    def update(self, measurement):
        # with self.lock:
        self.pf.update(measurement)

    def predict(self, input_vector, dt):
        # with self.lock:
        self.pf.predict(input_vector, dt)
        
    def mean(self):
        # with self.lock:
        return self.pf.mean()
    
    def estimate(self):
        # with self.lock:
        return self.pf.estimate()
    
    def particles(self):
        # with self.lock:
        return self.pf.particles
    
    def check_resample(self):
        # with self.lock:
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
        self.lock = threading.Lock()

        self._compute_max_confidence(len(self.initial_range))

    def get_filter_name(self, label, index):
        return "%s_%s" % (label, index)

    def _init_filter(self, label, initial_state):
        if label not in self.filters:
            self.filters[label] = []
        
        filter_index = 0
        for obj_filter in self.filters[label]:
            if obj_filter.label == label:
                filter_index += 1
        filter_name = self.get_filter_name(label, filter_index)

        while len(self.filters[label]) >= self.max_num_filters:
            self._remove_least_confidence_filter(label)

        obj_filter = ObjectFilter(
            filter_name, label,
            self.num_particles,
            self.meas_std_val, self.input_std,
            initial_state, self.initial_range
        )
        self.filters[label].append(obj_filter)
    
    def _remove_least_confidence_filter(self, label):
        index_to_remove = None
        smallest_variance = None
        for index, obj_filter in enumerate(self.filters[label]):
            mean, variance = obj_filter.estimate()
            variance = np.linalg.norm(variance)
            if smallest_variance is None:
                smallest_variance = variance
                index_to_remove = index
            elif variance < smallest_variance:
                smallest_variance = variance
                index_to_remove = index
        self.filters[label].pop(index_to_remove)
    
    def get_particles(self, label, filter_index):
        with self.lock:
            if label in self.filters and 0 <= filter_index < len(self.filters[label]):
                return self.filters[label][filter_index].particles()
            else:
                return None
    
    def get_all_particles(self):
        with self.lock:
            particles = []
            for label in self.filters:
                for obj_filter in self.filters[label]:
                    particles.extend(obj_filter.particles())
            return particles
        
    def get_means(self):
        with self.lock:
            means = {}
            for label in self.filters:
                for obj_filter in self.filters[label]:
                    means[obj_filter.name] = obj_filter.mean()
            return means

    def _get_label_means(self, label):
        means = []
        for obj_filter in self.filters[label]:
            means.append(obj_filter.mean())
        return means
    
    def is_label_initialized(self, label):
        return len(self.filters) != 0 and label in self.filters and len(self.filters[label]) != 0

    def update(self, label, measurement):
        with self.lock:
            rospy.loginfo("%s Measurement: %s" % (label, measurement))
            if not self.is_label_initialized(label):
                self._init_filter(label, measurement)
                rospy.loginfo("%s mean: %s" % (self.filters[label][0].name, self.filters[label][0].mean()))
                rospy.loginfo("%s weights: %s" % (self.filters[label][0].name, self.filters[label][0].pf.weights))
                return

            measurement_matched = False
            means = self._get_label_means(label)
            for filter_index, mean in enumerate(means):
                obj_filter = self.filters[label][filter_index]
                confidence = self._get_confidences(measurement, mean)
                rospy.loginfo("%s confidence: %s. %s Mean: %s" % (label, confidence, obj_filter.name, mean))
                if confidence > self.match_threshold:
                    rospy.loginfo("Measurement matches %s" % obj_filter.name)
                    obj_filter.update(measurement)
                    measurement_matched = True
                    break
            if not measurement_matched and confidence < self.new_filter_threshold:
                rospy.loginfo("Measurement doesn't match filter. Creating a new one")
                self._init_filter(label, measurement)
    
    def _get_confidences(self, measurement, mean):
        confidence = scipy.stats.multivariate_normal.pdf(measurement, mean=mean, cov=self.match_cov)
        confidence /= self.max_confidence
        return confidence
    
    def _compute_max_confidence(self, dims):
        # for normalizing confidence
        zeros = np.zeros(dims)
        self.max_confidence = scipy.stats.multivariate_normal.pdf(zeros, mean=zeros, cov=self.match_cov)

    def predict(self, input_vector, dt):
        with self.lock:
            for label in self.filters:
                for obj_filter in self.filters[label]:
                    rospy.loginfo("%s predict: %s, %s" % (obj_filter.name, input_vector, dt))
                    obj_filter.predict(input_vector, dt)

    def check_resample(self):
        with self.lock:
            for label in self.filters:
                for obj_filter in self.filters[label]:
                    obj_filter.check_resample()

