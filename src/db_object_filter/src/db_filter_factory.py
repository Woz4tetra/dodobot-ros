import rospy
import threading
import numpy as np
import scipy.stats
from db_particle_filter import ParticleFilter, JitParticleFilter, FilterSerial



class FilterFactory(object):
    def __init__(self, class_labels, num_particles, meas_std_val, input_std, initial_range, 
            match_cov, match_threshold, new_filter_threshold, max_item_count, confident_filter_threshold,
            stale_filter_time, use_numba=True):
        self.class_labels = class_labels
        self.num_particles = num_particles
        self.meas_std_val = meas_std_val
        self.input_std = input_std
        self.initial_range = initial_range
        self.stale_filter_time = stale_filter_time

        self.match_cov = match_cov
        self.match_threshold = match_threshold
        self.new_filter_threshold = new_filter_threshold
        self.max_item_count = max_item_count
        self.default_max_num_filters = 1
        self.confident_filter_threshold = confident_filter_threshold

        self.lock = threading.Lock()

        if use_numba:
            self.ParticleFilterClass = JitParticleFilter
        else:
            self.ParticleFilterClass = ParticleFilter

        self._init_filter_container()
        self._compute_max_confidence(len(self.initial_range))

    def _init_filter_container(self):
        self.filters = {}
        for label in self.class_labels:
            self.filters[label] = []
            max_num_filters = self.max_item_count.get(label, self.default_max_num_filters)
            if max_num_filters < 0:
                rospy.logwarn("Encountered negative number for max item count for label '%s'. Setting to 0" % label)
                max_num_filters = 0
            for filter_index in range(max_num_filters):
                obj_filter = self.ParticleFilterClass(
                    FilterSerial(label=label, index=filter_index),
                    self.num_particles,
                    self.meas_std_val, self.input_std,
                    self.stale_filter_time
                )
                self.filters[label].append(obj_filter)

    def _init_filter(self, label, initial_state):
        if len(self.filters[label]) == 0:
            rospy.logdebug("Ignoring filter initialization. Max item count for label is zero.")
            return
        filter_index = self._get_least_confidence_filter(label)
        if filter_index is None:
            rospy.logdebug("All existing filters are highly confident. Ignoring measurement: %s, %s" % (label, initial_state))
            return

        obj_filter = self.filters[label][filter_index]
        rospy.logdebug("Reinitializing %s_%s" % (obj_filter.serial.label, obj_filter.serial.index))
        obj_filter.create_gaussian_particles(initial_state, self.initial_range)
    
    def _get_least_confidence_filter(self, label):
        index_to_remove = None
        smallest_variance = None
        for obj_filter in self.filters[label]:
            if not obj_filter.is_initialized():
                index_to_remove = obj_filter.serial.index
                break

            mean, variance = obj_filter.estimate()
            variance = np.linalg.norm(variance)

            # don't consider filter for removal if its variance is small
            if variance < self.confident_filter_threshold:
                continue
            if smallest_variance is None:
                smallest_variance = variance
                index_to_remove = obj_filter.serial.index
            elif variance < smallest_variance:
                smallest_variance = variance
                index_to_remove = obj_filter.serial.index
        rospy.logdebug("%s_%s's variance was %s" % (label, index_to_remove, smallest_variance))
        return index_to_remove
    
    def iter_filters(self):
        with self.lock:
            for label in self.filters:
                for obj_filter in self.filters[label]:
                    if not obj_filter.is_initialized():
                        continue
                    yield obj_filter
        
    def update(self, all_measurements):
        with self.lock:
            for label, measurements in all_measurements.items():
                for measurement in measurements:
                    self._analyze_measurement(label, measurement)
    
    def _analyze_measurement(self, label, measurement):
        # if none of the filters for this label are initialized, initialize automatically
        if all([not obj_filter.is_initialized() for obj_filter in self.filters[label]]):
            self._init_filter(label, measurement)
            return
        
        largest_confidence = 0.0
        largest_index = None
        for obj_filter in self.filters[label]:
            if not obj_filter.is_initialized():
                continue
            mean = obj_filter.mean()
            confidence = self._get_confidences(measurement, mean)
            if confidence > largest_confidence:
                largest_confidence = confidence
                largest_index = obj_filter.serial.index

        rospy.logdebug("%s[%s]: %s, %s" % (label, measurement, largest_index, largest_confidence))
        if largest_index is not None and largest_confidence > self.match_threshold:
            self.filters[label][largest_index].update(measurement)
        elif largest_confidence < self.new_filter_threshold:
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
        for obj_filter in self.iter_filters():
            is_active = obj_filter.predict(input_vector, dt)
            # if not is_active:
            # TODO: set filter position to be where the map is

    def check_resample(self):
        for obj_filter in self.iter_filters():
            if obj_filter.check_resample():
                rospy.logdebug("Resampling %s_%s" % (obj_filter.serial.label, obj_filter.serial.index))

