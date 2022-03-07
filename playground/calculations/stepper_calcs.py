import math

gearbox_ratio = 26 + 103/121
encoder_ticks_per_R = 160
stepper_ticks_per_R = 200
# pulley_radius_mm = 13.58874
# pulley_radius_mm = 11.88694
pulley_radius_mm = 12.1
microsteps = 8

cad_full_travel_mm = 153.60

mm_per_R = 2 * math.pi * pulley_radius_mm

def enc_to_step_ticks(ticks):
    return ticks * stepper_ticks_per_R * microsteps / encoder_ticks_per_R

def step_ticks_to_linear_mm(ticks):
    rotations = ticks / (stepper_ticks_per_R * microsteps)
    output_rotations = rotations / gearbox_ratio

    return output_rotations * mm_per_R
    # return ticks * mm_per_R / (stepper_ticks_per_R * microsteps * gearbox_ratio)


def linear_mm_to_step_ticks(mm):
    output_rotations = mm / mm_per_R
    rotations = output_rotations * gearbox_ratio
    return rotations * stepper_ticks_per_R * microsteps

print(step_ticks_to_linear_mm(85000), cad_full_travel_mm)
print(linear_mm_to_step_ticks(cad_full_travel_mm), cad_full_travel_mm)
print(enc_to_step_ticks(8399))
