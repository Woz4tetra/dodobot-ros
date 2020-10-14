import csv


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
            "grip_dist": float,
            "grip_threshold": float,
            "comment": str,
        }

        self.header = []
        self.path = ""

        self.sequence_names = {}  # column indices mapped to column names
        self.sequence_indices = {}  # column names mapped to column index
        self.sequence_types = {}  # column indices mapped to expected data type
        self.sequence = []

    @classmethod
    def from_path(cls, path):
        self = cls()
        self._init_from_path(path)
        return self

    def _init_from_path(self, path):
        with open(path) as file:
            self.path = path
            self.reader = csv.reader(file)
            self.header = next(self.reader)
            self._parse_header()
            self.update_sequence_mapping()

            for row in self.reader:
                if row[0].startswith("#"):
                    continue
                action = {}
                for column_name, index in self.sequence_indices.items():
                    value = self.sequence_types[index](row[index])
                    action[column_name] = value
                self.sequence.append(action)

    def reload(self):
        if len(self.path) == 0:
            raise ValueError("Sequence wasn't loaded from a path!")
        self._init_from_path(self.path)

    def _parse_header(self):
        for column_name in self.header_set:
            self.sequence_indices[column_name] = self.header.index(column_name)

    @classmethod
    def from_sequence(cls, sequence):
        assert isinstance(sequence, Sequence), type(sequence)
        self = cls()
        self.header = sequence.header
        self._parse_header()
        self.update_sequence_mapping()
        return self

    def update_sequence_mapping(self):
        self.sequence_names = {v: k for k, v in self.sequence_indices.items()}
        self.sequence_types = {v: self.header_info[k] for k, v in self.sequence_indices.items()}
        self.sequence = []

    def insert(self, index, action):
        self.sequence.insert(index, action)

    def append(self, action):
        self.sequence.append(action)

    def action_as_list(self, action):
        action_list = [None for _ in xrange(len(action))]
        for name, value in action.items():
            index = self.sequence_indices[name]
            action_list[index] = action[name]
        return action_list

    def to_csv(self, path):
        with open(path, 'w') as file:
            writer = csv.writer(file)
            writer.writerow(self.header)
            for action in self.sequence:
                writer.writerow(self.action_as_list(action))
