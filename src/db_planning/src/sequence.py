
import csv

class Sequence(object):
    def __init__(self, path):
        with open(path) as file:
            self.reader = csv.reader(file)

            header = next(self.reader)
            self.goal_x = header.index("goal_x")
            self.goal_y = header.index("goal_y")
            self.goal_angle = header.index("goal_angle")
            self.base_speed = header.index("base_speed")
            self.base_ang_v = header.index("base_ang_v")
            self.pos_tolerance = header.index("pos_tolerance")
            self.drive_forwards = header.index("drive_forwards")
            self.goal_z = header.index("goal_z")
            self.z_speed = header.index("z_speed")
            self.z_accel = header.index("z_accel")

            self.sequence_names = {
                self.goal_x: "goal_x",
                self.goal_y: "goal_y",
                self.goal_angle: "goal_angle",
                self.base_speed: "base_speed",
                self.base_ang_v: "base_ang_v",
                self.pos_tolerance: "pos_tolerance",
                self.drive_forwards: "drive_forwards",
                self.goal_z: "goal_z",
                self.z_speed: "z_speed",
                self.z_accel: "z_accel",
            }

            self.sequence_types = {
                self.goal_x: float,
                self.goal_y: float,
                self.goal_angle: float,
                self.base_speed: float,
                self.base_ang_v: float,
                self.pos_tolerance: float,
                self.drive_forwards: bool,
                self.goal_z: float,
                self.z_speed: float,
                self.z_accel: float,
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
                self.z_accel
            ]

            self.sequence = []

    def parse_sequence(self):
        for row in self.reader:
            action = {}
            for index in self.sequence_order:
                value = self.sequence_types[index](row[index])
                action[self.sequence_names[index]] = value
            self.sequence.append(action)

    def __iter__(self):
        return self

    def __next__(self):
        self.next()

    def next(self):
        for action in self.sequence:
            return action
        raise StopIteration

    def __del__(self):
        if self.file is not None:
            self.file.close()
