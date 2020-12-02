import numpy as np

import rospy

import tf2_ros
import tf_conversions
import tf2_geometry_msgs

from geometry_msgs.msg import PoseStamped



class StoredObjectContainer(object):
    def __init__(self, camera_model, tf_buffer, main_workspace_tf, close_obj_threshold_m, object_far_away_buffer):
        self.stored_objects = {}
        self.assigned_indices = {}
        self.close_obj_threshold_m = close_obj_threshold_m  # type: float
        self.camera_model = camera_model  # type: image_geometry.PinholeCameraModel
        self.tf_buffer = tf_buffer  # tf2_ros.Buffer
        self.main_workspace_tf = main_workspace_tf
        self.object_far_away_buffer = object_far_away_buffer

    def reset(self):
        self.stored_objects = {}
        self.assigned_indices = {}
    
    def update(self, in_view_objects):
        self.check_objects_in_view(in_view_objects)
        self.check_in_view_points(in_view_objects)

    def check_objects_in_view(self, in_view_objects):
        add_objects = []
        remove_objects = []
        for in_view_obj in in_view_objects:
            assert type(in_view_obj) == StoredObject, str(type(in_view_obj))

            should_add_obj = True
            for label_serial, obj in self.stored_objects.items():
                if self.are_objects_close(in_view_obj, obj):
                    if in_view_obj.label_name != obj.label_name:
                        rospy.loginfo("Removing '%s'. New object has different label" % obj.label_serial)
                        # label changed. probably a different object
                        self.unassign_index(obj)
                        remove_objects.append(obj)
                    else:
                        # labels match and positions are close. It's the same object. Update pose
                        should_add_obj = False
                        obj.map_pose = in_view_obj.map_pose

            if should_add_obj:
                self.assign_index(in_view_obj)
                rospy.loginfo("Adding new object '%s'" % in_view_obj.label_serial)
                add_objects.append(in_view_obj)

        for obj in add_objects:
            self.stored_objects[obj.label_serial] = obj

        for obj in remove_objects:
            del self.stored_objects[obj.label_serial]

    def check_in_view_points(self, in_view_objects):
        remove_objects = []
        width = self.camera_model.width
        height = self.camera_model.height
        for label_serial, obj in self.stored_objects.items():
            camera_pose = self.get_camera_pose(obj)
            if camera_pose is None:
                continue
            object_distance = camera_pose.pose.position.z
            object_far_away_threshold = max(0.0, obj.detect_pose.pose.position.z + self.object_far_away_buffer)
            if object_distance < 0.0 or object_distance > object_far_away_threshold:
                rospy.logdebug("%s is too far away" % label_serial)
                # object is behind the camera or object is too far away. Skip
                continue
            point = self.pose_to_array(camera_pose.pose)
            u, v = self.camera_model.project3dToPixel(point)

            # TODO: check if this needs to be changed to ROI
            if 0 <= u < width and 0 <= v < height:  # object should be in view
                rospy.loginfo("%s should be in frame. In view objects: %s" % (label_serial, in_view_objects))
                rospy.loginfo("%s: %s -> %s, %s" % (label_serial, point, u, v))
                object_is_in_view = False

                for in_view_obj in in_view_objects:
                    if in_view_obj.label_name == obj.label_name and self.are_objects_close(in_view_obj, obj):
                        object_is_in_view = True

                if not object_is_in_view:
                    rospy.loginfo("Removing '%s'. Object is not in original location" % obj.label_serial)
                    remove_objects.append(obj)

        for obj in remove_objects:
            self.unassign_index(obj)
            del self.stored_objects[obj.label_serial]

    def assign_index(self, obj):
        index = 0
        if obj.label_name not in self.assigned_indices:
            self.assigned_indices[obj.label_name] = [0]
        else:
            while index in self.assigned_indices[obj.label_name]:
                index += 1
            self.assigned_indices[obj.label_name].append(index)
        obj.label_index = index

    def unassign_index(self, obj):
        if obj.label_name in self.assigned_indices:
            self.assigned_indices[obj.label_name].remove(obj.label_index)

    def pose_to_array(self, pose):
        position = pose.position
        return np.array([position.x, position.y, position.z])

    def are_objects_close(self, obj1, obj2):
        point1 = self.pose_to_array(obj1.map_pose.pose)
        point2 = self.pose_to_array(obj2.map_pose.pose)
        dist = np.linalg.norm(point2 - point1)
        assert dist >= 0.0, dist
        return dist < self.close_obj_threshold_m

    def iter_tfs(self):
        for label_serial, obj in self.stored_objects.items():
            yield label_serial, obj

    def get_camera_pose(self, obj):
        # transform object's map registered pose back into a pose relative to the camera's frame
        try:
            camera_tf = self.tf_buffer.lookup_transform(
                self.camera_model.tfFrame(),
                self.main_workspace_tf,
                # obj.detection_time,
                rospy.Time(0),
                rospy.Duration(0.1)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr_throttle(15.0, e)
            return None
        tfd_pose = tf2_geometry_msgs.do_transform_pose(obj.map_pose, camera_tf)
        rospy.logdebug("obj pose: %s, %s" % (tfd_pose.header.frame_id, self.pose_to_array(tfd_pose.pose)))
        return tfd_pose


class StoredObject(object):
    def __init__(self, label_name, map_pose, detect_pose, detection_time):
        self.label_name = label_name
        self.label_index = -1
        self.map_pose = map_pose
        self.detect_pose = detect_pose
        self.detection_time = detection_time
        assert type(self.map_pose) == PoseStamped, str(type(self.map_pose))
        assert type(self.detect_pose) == PoseStamped, str(type(self.detect_pose))

    @property
    def label_serial(self):
        return str(self.label_name) + "_" + str(self.label_index)

    def __repr__(self):
        if self.label_index != -1:
            return self.label_serial
        else:
            return self.label_name
