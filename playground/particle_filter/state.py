
OBJECT_NAMES = [
    "BACKGROUND",
    "cozmo_cube",
    "blue_cut_sphere",
    "red_cut_sphere",
    "blue_low_bin",
    "red_low_bin",
    "blue_cube",
    "red_cube",
]


class State:
    def __init__(self):
        self.type = ""
        self.stamp = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.t = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.vt = 0.0

    def __str__(self):
        return f"<{self.type}>(" \
               f"x={self.x:0.4f}, y={self.y:0.4f}, z={self.z:0.4f}, t={self.t:0.4f}, " \
               f"vx={self.vx:0.4f}, vy={self.vy:0.4f}, vz={self.vz:0.4f}, vt={self.vt:0.4f}) @ {self.stamp}"

