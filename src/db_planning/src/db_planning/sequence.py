import csv


class Sequence(object):
    def __init__(self):
        self.goal_x = 0
        self.goal_y = 0
        self.goal_angle = 0
        self.base_speed = 0
        self.base_ang_v = 0
        self.pos_tolerance = 0
        self.drive_forwards = 0
        self.goal_z = 0
        self.z_speed = 0
        self.z_accel = 0
        self.comment = ""

        self.header = []

        self.sequence_names = {}
        self.sequence_indices = {}
        self.sequence_types = {}
        self.sequence = []

    @classmethod
    def from_path(cls, path):
        self = cls()
        with open(path) as file:
            self.reader = csv.reader(file)
            self.header = next(self.reader)
            self._parse_header()
            self.update_sequence_mapping()

            for row in self.reader:
                action = {}
                for index in self.sequence_order:
                    value = self.sequence_types[index](row[index])
                    action[self.sequence_names[index]] = value
                self.sequence.append(action)
        return self

    def _parse_header(self):
        self.goal_x = self.header.index("goal_x")
        self.goal_y = self.header.index("goal_y")
        self.goal_angle = self.header.index("goal_angle")
        self.base_speed = self.header.index("base_speed")
        self.base_ang_v = self.header.index("base_ang_v")
        self.pos_tolerance = self.header.index("pos_tolerance")
        self.drive_forwards = self.header.index("drive_forwards")
        self.goal_z = self.header.index("goal_z")
        self.z_speed = self.header.index("z_speed")
        self.z_accel = self.header.index("z_accel")
        self.comment = self.header.index("comment")

    @classmethod
    def from_sequence(cls, sequence):
        assert isinstance(sequence, Sequence), type(sequence)
        self = cls()
        self.header = sequence.header
        self._parse_header()
        self.update_sequence_mapping()
        return self

    def update_sequence_mapping(self):
        self.sequence_names = {
            self.goal_x        : "goal_x",
            self.goal_y        : "goal_y",
            self.goal_angle    : "goal_angle",
            self.base_speed    : "base_speed",
            self.base_ang_v    : "base_ang_v",
            self.pos_tolerance : "pos_tolerance",
            self.drive_forwards: "drive_forwards",
            self.goal_z        : "goal_z",
            self.z_speed       : "z_speed",
            self.z_accel       : "z_accel",
            self.comment       : "comment",
        }

        self.sequence_indices = {v: k for k, v in self.sequence_names.items()}

        self.sequence_types = {
            self.goal_x        : float,
            self.goal_y        : float,
            self.goal_angle    : float,
            self.base_speed    : float,
            self.base_ang_v    : float,
            self.pos_tolerance : float,
            self.drive_forwards: int,
            self.goal_z        : float,
            self.z_speed       : float,
            self.z_accel       : float,
            self.comment       : str,
        }

        self.sequence_order = [
            self.goal_x,
            self.goal_y,
            self.goal_angle,
            self.base_speed,
            self.base_ang_v,
            self.pos_tolerance,
            self.drive_forwards,
            self.goal_z,
            self.z_speed,
            self.z_accel,
            self.comment
        ]
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
