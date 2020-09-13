import csv
import math
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider
from scipy.spatial.transform import Rotation

odom_path = "../../src/db_config/bags/motors_2020-09-13-15-42-20-dodobot-odom.csv"
linear_path = "../../src/db_config/bags/motors_2020-09-13-15-42-20-dodobot-linear.csv"
# odom_path = "../../src/db_config/bags/motors_2020-09-13-16-44-01-dodobot-odom.csv"
# linear_path = "../../src/db_config/bags/motors_2020-09-13-16-44-01-dodobot-linear.csv"

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

        quat_z_index = header.index(".pose.pose.orientation.z")
        quat_w_index = header.index(".pose.pose.orientation.w")


        for index, line in enumerate(reader):
            bag_time = bag_time_to_date(line[time_index]).timestamp()
            x = float(line[x_index])
            y = float(line[y_index])
            quat_z = float(line[quat_z_index])
            quat_w = float(line[quat_w_index])
            rotation = Rotation.from_quat([0.0, 0.0, quat_z, quat_w])
            angles = rotation.as_euler('zyx')
            angle = angles[0]

            data.append([
                bag_time,
                x,
                y,
                angle
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
            data.append([
                bag_time,
                position,
            ])

    data = np.array(data)
    return data


class Plotter:
    def __init__(self):
        self.dot_angle_len = 0.1

        self.fig = plt.figure()
        self.ax1 = self.fig.add_subplot(211)
        self.ax2 = self.fig.add_subplot(212)
        plt.subplots_adjust(bottom=0.25)

        self.odom_data = load_odom(odom_path)
        self.linear_data = load_linear(linear_path)

        self.odom_t = self.odom_data[:, 0]
        self.odom_x = self.odom_data[:, 1]
        self.odom_y = self.odom_data[:, 2]
        self.odom_angles = self.odom_data[:, 3]
        self.linear_interp = np.interp(self.odom_t, self.linear_data[:, 0], self.linear_data[:, 1])

        self.angle_vectors = self.dot_angle_vectors()

        self.odom_line_data = self.ax1.plot(self.odom_x, self.odom_y)[0]
        self.odom_dot_data = self.ax1.plot(self.odom_x[-1], self.odom_y[-1], 'o')[0]
        self.init_angle_vector(-1)

        self.ax1.set_xlim(-0.5, 0.5)
        self.ax1.set_ylim(-0.5, 0.5)

        self.linear_line_data = self.ax2.plot(self.odom_t, self.linear_interp)[0]
        self.linear_dot_data = self.ax2.plot(self.odom_t[-1], self.linear_interp[-1], 'o')[0]

        axcolor = 'lightgoldenrodyellow'
        axtime = plt.axes([0.25, 0.1, 0.65, 0.03], facecolor=axcolor)
        self.time_slider = Slider(axtime, "Time", self.odom_t[0], self.odom_t[-1], valinit=self.odom_t[-1], valstep=0.1)
        self.time_slider.on_changed(self.update_slider)

    def dot_angle_vectors(self):
        return np.array((self.dot_angle_len * np.cos(self.odom_angles) + self.odom_x, self.dot_angle_len * np.sin(self.odom_angles) + self.odom_y))

    def init_angle_vector(self, index):
        x0 = self.odom_x[index]
        y0 = self.odom_y[index]
        x1 = self.angle_vectors[0, index]
        y1 = self.angle_vectors[1, index]
        self.odom_angle_data = self.ax1.plot([x0, x1], [y0, y1])[0]

    def draw_angle_vector(self, index):
        x0 = self.odom_x[index]
        y0 = self.odom_y[index]
        x1 = self.angle_vectors[0, index]
        y1 = self.angle_vectors[1, index]
        self.odom_angle_data.set_xdata([x0, x1])
        self.odom_angle_data.set_ydata([y0, y1])

    @staticmethod
    def find_nearest(array, value):
        array = np.asarray(array)
        idx = (np.abs(array - value)).argmin()
        return idx

    def update_slider(self, val):
        index = self.find_nearest(self.odom_t, val)
        self.odom_line_data.set_xdata(self.odom_x[0: index])
        self.odom_line_data.set_ydata(self.odom_y[0: index])
        self.linear_line_data.set_xdata(self.odom_t[0: index])
        self.linear_line_data.set_ydata(self.linear_interp[0: index])
        self.odom_dot_data.set_xdata(self.odom_x[index])
        self.odom_dot_data.set_ydata(self.odom_y[index])
        self.linear_dot_data.set_xdata(self.odom_t[index])
        self.linear_dot_data.set_ydata(self.linear_interp[index])
        self.draw_angle_vector(index)

        print("%s\t%s\t%s\t%s" % (self.odom_x[index], self.odom_y[index], self.odom_angles[index], self.linear_interp[index]))

        self.fig.canvas.draw_idle()


    def run(self):
        plt.show()


def main():
    plotter = Plotter()

    plotter.run()



if __name__ == '__main__':
    main()
