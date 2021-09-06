import math
import numpy as np
from state import Pose2d
from matplotlib import pyplot as plt
from trajectory.spline_generator import SplineGenerator
from trajectory.time_parameterizer import TimeParameterizer

waypoints = [
    Pose2d(0.0, 0.0, math.radians(0.0)),
    Pose2d(1.0, 0.0, math.radians(90.0)),
    Pose2d(1.0, 1.0, math.radians(180.0)),
    Pose2d(0.0, 1.0, math.radians(270.0)),
    Pose2d(0.0, 0.0, math.radians(0.0)),
]

spline = SplineGenerator(waypoints)

print(spline.spline(0.0, 1))
print(spline.spline(0.0, 0))

# points = np.array([point for point in spline.iter(500)])
# goals = np.array([waypoint.to_list() for waypoint in waypoints])

# plt.plot(points[:, 0], points[:, 1])
# plt.plot(goals[:, 0], goals[:, 1])
# plt.show()