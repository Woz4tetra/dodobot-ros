import numpy as np
from geometry_msgs.msg import PoseStamped


class StoredObjectContainer(object):
    def __init__(self, camera_model, close_obj_threshold_m):
        self.stored_objects = {}
        self.assigned_indices = {}
        self.close_obj_threshold_m = close_obj_threshold_m  # type: float
        self.camera_model = camera_model  # type: image_geometry.PinholeCameraModel

    def check_objects_in_view(self, in_view_objects):
        add_objects = []
        remove_objects = []
        for in_view_obj in in_view_objects:
            assert type(in_view_obj) == StoredObject, str(type(in_view_obj))

            should_add_obj = True
            for label_serial, obj in self.stored_objects.items():
                if self.are_objects_close(in_view_obj, obj):
                    if in_view_obj.label_name != obj.label_name:
                        # label changed. probably a different object
                        self.unassign_index(obj)
                        remove_objects.append(obj)
                    else:
                        # labels match and positions are close. It's the same object
                        should_add_obj = False

            if should_add_obj:
                self.assign_index(new_obj)
                obj_list.append(new_obj)

        for obj in add_objects:
            self.stored_objects[obj.label_serial] = obj

        for obj in remove_objects:
            del self.stored_objects[obj.label_serial]

    def check_in_view_points(self, in_view_objects):
        remove_objects = []
        width = self.camera_model.cameraInfo().width
        height = self.camera_model.cameraInfo().height
        for label_serial, obj in self.stored_objects.items():
            point = self.pose_to_array(obj.pose_stamped)
            u, v = self.camera_model.project3dToPixel(point)

            # TODO: check if this needs to be changed to ROI
            if 0 <= u < width and 0 <= v < height:  # object should be in view
                object_is_in_view = False

                for in_view_obj in in_view_objects:
                    if in_view_obj.label_name == obj.label_name and self.are_objects_close(in_view_obj, obj):
                        object_is_in_view = True

                if not object_is_in_view:
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
        self.assigned_indices[obj.label_name].remove(obj.label_index)

    def pose_to_array(self, pose_stamped):
        position = pose_stamped.pose.position
        return np.array([position.x, position.y, position.z])

    def are_objects_close(self, obj1, obj2):
        point1 = self.pose_to_array(obj1.pose_stamped)
        point2 = self.pose_to_array(obj2.pose_stamped)
        dist = np.linalg.norm(point2 - point1)
        assert dist >= 0.0, dist
        return dist < close_obj_threshold_m

    def iter_tfs(self):
        for label_serial, obj in self.stored_objects.items():
            yield label_serial, obj.pose_stamped

class StoredObject(object):
    def __init__(self, label_name, pose_stamped):
        self.label_name = label_name
        self.label_index = -1
        self.pose_stamped = pose_stamped
        assert type(self.pose_stamped) == PoseStamped, str(type(self.pose_stamped))

    @property
    def label_serial(self):
        return str(self.label_name) + "_" + str(self.label_index)
