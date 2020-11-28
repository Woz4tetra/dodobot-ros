import yaml


class Sequence(object):
    def __init__(self):
        self.header_info = {
            "goal_x": float,
            "goal_y": float,
            "goal_angle": float,
            "base_speed": float,
            "base_ang_v": float,
            "pos_tolerance": float,
            "angle_tolerance": float,
            "drive_forwards": int,
            "goal_z": float,
            "z_speed": float,
            "z_accel": float,
            "grip_distance": float,
            "force_threshold": float,
            "comment": str,
        }

        self.path = ""

        self.sequence = []

    @classmethod
    def from_path(cls, path):
        self = cls()
        self._init_from_path(path)
        return self

    def _init_from_path(self, path):
        self.path = path
        self.sequence = []
        with open(path) as file:
            config = yaml.safe_load(file.read())

            for row in config:
                action = {}
                for column_name in self.header_info.keys():
                    if column_name not in row:
                        if column_name == "drive_forwards":  # special case for this variable
                            value = 1
                        elif self.header_info[column_name] == float:
                            value = float("nan")
                        else:
                            value = self.header_info[column_name]()
                    else:
                        value = self.header_info[column_name](row[column_name])
                    action[column_name] = value
                self.sequence.append(action)

    def reload(self):
        if len(self.path) == 0:
            raise ValueError("Sequence wasn't loaded from a path!")
        self._init_from_path(self.path)

    def insert(self, index, action):
        self.sequence.insert(index, action)

    def append(self, action):
        self.sequence.append(action)
