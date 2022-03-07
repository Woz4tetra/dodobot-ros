import math
import numpy as np
import matplotlib.pyplot as plt

# physical_parameters = dict(
#     la=0.029,
#     lb=0.01775,
#     lc=0.024,
#     ld=0.001,
# )

# physical_parameters = dict(
#     la=0.042,
#     lb=0.01775,
#     lc=0.024,
#     ld=0.001,
# )


# Measured from main motor spring axle
physical_parameters = dict(
    la=0.05,
    lb=0.01775,
    lc=0.024,
    ld=0.001,
)

# spring_parameters = dict(
#     k=1353  # N/m
# )

# spring_parameters = dict(
#     m=1.895,  # kg
#     x1=0.03008,  # m
#     x0=0.01195,  # m
# )

spring_parameters = dict(
    m=2.23,  # kg
    x1=0.03008,  # m
    x0=0.01195,  # m
)

robot_mass = 3.856  # kg
# robot_mass = 3.544  # kg
payload_mass = 0.5  # kg

g = 9.81

robot_force = robot_mass * g
robot_payload_force = (robot_mass + payload_mass) * g


def get_spring_len(theta, params):
    # meters:
    la = params["la"]
    lb = params["lb"]
    lc = params["lc"]
    ld = params["ld"]

    x0 = -ld
    y0 = -lc
    x1 = la * math.cos(theta) - lb * math.sin(theta)
    y1 = -la * math.sin(theta) - lb * math.cos(theta)

    theta_spring = math.atan2(y1 - y0, x1 - x0)
    l = (x1 - x0) / math.cos(theta_spring)

    return l, theta_spring, ((x0, y0), (x1, y1))




def motor_normal_force(l, y1):
    x0 = spring_parameters["x0"]

    if "k" in spring_parameters:
        k = spring_parameters["k"]
    else:
        m = spring_parameters["m"]
        x1 = spring_parameters["x1"]
        k = m * g / (x1 - x0)

    lm = 0.065

    return k * (l - x0) * (abs(y1) / lm)


def print_results(theta):
    l, theta_spring, ((x0, y0), (x1, y1)) = get_spring_len(math.radians(theta), physical_parameters)
    motor_F = motor_normal_force(l, y1)
    dual_motor_F = motor_F * 2
    net_force = dual_motor_F - robot_force
    net_force_payload = dual_motor_F - robot_payload_force

    print("%0.1f\t%0.3f\t%0.3f\t%0.3f\t%0.3f" % (theta, l, dual_motor_F, net_force, net_force_payload))



def main():
    print_results(9.0)
    print_results(12.1)
    print_results(22.0)

    # plt.figure(1)
    # plt.plot([0.0], [0.0], 'x')
    # plt.plot([0.065], [0.0], 'x')

    thetas = np.linspace(0.0, math.radians(22.0), 50)
    ls = []
    spring_spacing = np.linspace(0.048, 0.052, 10)
    # spring_spacing = [physical_parameters["la"]]

    plt.figure(2)
    for length in spring_spacing:
        forces = []
        physical_parameters["la"] = length
        for theta in thetas:
            l, theta_spring, ((x0, y0), (x1, y1)) = get_spring_len(theta, physical_parameters)
            motor_F = motor_normal_force(l, y1)
            dual_motor_F = motor_F * 2

            ls.append(l)
            forces.append(dual_motor_F)

            # plt.plot([x0, x1], [y0, y1])

        plt.plot(np.degrees(thetas), forces, label="%0.4f" % length)
        plt.xlabel("Assembly angle (deg)")
        plt.ylabel("Force (N)")

    factor_of_safety = 0.5
    plt.plot([12.1, 12.1], [robot_force - factor_of_safety, robot_payload_force - factor_of_safety])
    plt.legend()
    # plt.plot(thetas, ls)

    plt.show()

main()
