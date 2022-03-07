import math
import numpy as np
from scipy.interpolate import interp1d

armature_length = 0.06
hinge_pin_to_armature_end = 0.004
armature_width = 0.01
hinge_pin_diameter = 0.0028575

rotation_offset_x = armature_length + hinge_pin_to_armature_end
rotation_offset_y = (armature_width + hinge_pin_diameter)/2
hinge_pin_to_pad_plane = 0.0055
pad_extension_offset = 0.01998285
central_axis_dist = 0.015

def angle_to_parallel_dist(angle_rad):
    angle_adj = angle_rad - math.pi*2 + math.pi/2
    hinge_pin_x = rotation_offset_x * math.cos(angle_adj) - rotation_offset_y * math.sin(angle_adj)
    parallel_dist = 2.0 * (hinge_pin_x - hinge_pin_to_pad_plane - pad_extension_offset + central_axis_dist)
    return parallel_dist

open_angle = math.radians(344.89282)
closed_angle = math.radians(302.560457)
angle_samples = np.linspace(open_angle, closed_angle, 180)
dist_samples = []
for angle in angle_samples:
    parallel_dist = angle_to_parallel_dist(angle)
    dist_samples.append(parallel_dist)

parallel_dist_interp_fn = interp1d(dist_samples, angle_samples, kind="linear")

for d in np.arange(0, 0.075, 0.01):
    print(math.degrees(parallel_dist_interp_fn(d)))
