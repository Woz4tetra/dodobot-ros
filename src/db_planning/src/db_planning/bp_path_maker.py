from __future__ import print_function

import math
from sequence import Sequence

# relative to tag
# bp_inserted_x = 0.29963988
# bp_inserted_y = 0.005
# bp_inserted_z = 0.21798832

# relative to start position of sequence
# bp_inserted_index = 6  # for extract
# bp_inserted_index = 2  # for insert

# relative to tag
start_pos_x = 0.54963988
start_pos_y = 0.005
start_pos_z = 0.21798832

# stepper conversions
STEPPER_GEARBOX_RATIO = 26.0 + 103.0 / 121.0
ENCODER_TICKS_PER_R_NO_GEARBOX = 160.0
STEPPER_TICKS_PER_R_NO_GEARBOX = 200.0
MICROSTEPS = 8
ENCODER_TICKS_PER_R = ENCODER_TICKS_PER_R_NO_GEARBOX * STEPPER_GEARBOX_RATIO
ENCODER_R_PER_TICK = 1.0 / ENCODER_TICKS_PER_R
BELT_PULLEY_RADIUS_MM = 12.1

MAX_POSITION_FULL_STEP = 10625
MAX_SPEED_FULL_STEP = 31250000
STEPPER_ACCEL_FULL_STEP = 1250000

STEPPER_TICKS_PER_R = STEPPER_TICKS_PER_R_NO_GEARBOX * MICROSTEPS * STEPPER_GEARBOX_RATIO
STEPPER_R_PER_TICK = 1.0 / STEPPER_TICKS_PER_R
ENC_TO_STEP_TICKS = STEPPER_TICKS_PER_R_NO_GEARBOX * MICROSTEPS / ENCODER_TICKS_PER_R_NO_GEARBOX
STEP_TICKS_TO_LINEAR_MM = STEPPER_R_PER_TICK * BELT_PULLEY_RADIUS_MM * 2 * math.pi

MAX_SPEED = MAX_SPEED_FULL_STEP * MICROSTEPS
MAX_POSITION = MAX_POSITION_FULL_STEP * MICROSTEPS
STEPPER_ACCEL = STEPPER_ACCEL_FULL_STEP * MICROSTEPS


def find_not_nan(bp_inserted_index, sequence, key_name):
    bp_inserted = float("nan")
    while math.isnan(bp_inserted):
        action = sequence.sequence[bp_inserted_index]
        bp_inserted = action[key_name]
        bp_inserted_index -= 1
        if bp_inserted_index < 0:
            raise ValueError("Can't find value for bp insertion")
    return bp_inserted


def parse_sequence(filename, new_filename, bp_inserted_index):
    sequence = Sequence.from_path(filename)

    bp_inserted_x = find_not_nan(bp_inserted_index, sequence, "goal_x")
    bp_inserted_y = find_not_nan(bp_inserted_index, sequence, "goal_y")
    # bp_inserted_z = find_not_nan(bp_inserted_index, sequence, "goal_z")

    new_sequence = Sequence.from_sequence(sequence)

    for action in sequence.sequence:
        if not math.isnan(action["goal_x"]):
            goal_x = action["goal_x"] - bp_inserted_x + start_pos_x
        else:
            goal_x = action["goal_x"]

        if not math.isnan(action["goal_y"]):
            goal_y = action["goal_y"] - bp_inserted_y + start_pos_y
        else:
            goal_y = action["goal_y"]

        goal_z_ticks = action["goal_z"]
        if not math.isnan(action["goal_z"]):
            goal_z = goal_z_ticks * STEP_TICKS_TO_LINEAR_MM / 1000.0
        else:
            goal_z = goal_z_ticks

        if not math.isnan(action["z_speed"]):
            z_speed = action["z_speed"] * STEP_TICKS_TO_LINEAR_MM / 1000.0
        else:
            z_speed = action["z_speed"]

        if not math.isnan(action["z_accel"]):
            z_accel = action["z_accel"] * STEP_TICKS_TO_LINEAR_MM / 1000.0
        else:
            z_accel = action["z_accel"]

        new_action = {}
        new_action.update(action)
        new_action["goal_x"] = goal_x
        new_action["goal_y"] = goal_y
        new_action["goal_z"] = goal_z
        new_action["z_speed"] = z_speed
        new_action["z_accel"] = z_accel
        new_sequence.append(new_action)

    new_sequence.to_csv(new_filename)


def main():
    parse_sequence("../../sequences/extract-raw.csv", "../../sequences/extract.csv", 6)
    parse_sequence("../../sequences/insert-raw.csv", "../../sequences/insert.csv", 2)

if __name__ == '__main__':
    main()
