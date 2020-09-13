import csv
import math
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt

odom_path = "~/ros_ws/src/dodobot/src/db_config/bags/motors_2020-09-13-15-42-20-dodobot-odom.csv"
linear_path = "~/ros_ws/src/dodobot/src/db_config/bags/motors_2020-09-13-15-42-20-dodobot-linear.csv"

bag_time_format = "%Y/%m/%d/%H:%M:%S.%f"


def bag_time_to_date(time_str):
    return datetime.strptime(time_str, bag_time_format)

def time_to_bag_time(timestamp):
    return datetime.strftime(datetime.fromtimestamp(timestamp), bag_time_format)

def load_odom(path):
    with open(path) as file:
        reader = csv.reader(file)
        header = next(reader)
        data = []
        time_index = header.index("time")
        x_index = header.index(".pose.pose.position.x")
        y_index = header.index(".pose.pose.position.y")

        for index, line in enumerate(reader):
            bag_time = bag_time_to_date(line[time_index]).timestamp()
            x = float(line[x_index])
            y = float(line[y_index])
            data.append([
                bag_time,
                x,
                y,
            ])

    data = np.array(data)
    return data

def load_linear(path):
    with open(path) as file:
        reader = csv.reader(file)
        header = next(reader)
        data = []
        time_index = header.index("time")
        pos_index = header.index(".position")

        for index, line in enumerate(reader):
            bag_time = bag_time_to_date(line[time_index]).timestamp()
            position = float(line[pos_index])
            y = float(line[y_index])
            data.append([
                bag_time,
                position,
            ])

    data = np.array(data)
    return data


def main():
    odom_data = load_odom(odom_path)
    linear_data = load_linear(linear_path)


if __name__ == '__main__':
    main()
