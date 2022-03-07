import math
import numpy as np
# import matplotlib.pyplot as plt


def cartesian(arrays, out=None):
    """
    Generate a cartesian product of input arrays.

    Parameters
    ----------
    arrays : list of array-like
        1-D arrays to form the cartesian product of.
    out : ndarray
        Array to place the cartesian product in.

    Returns
    -------
    out : ndarray
        2-D array of shape (M, len(arrays)) containing cartesian products
        formed of input arrays.

    Examples
    --------
    >>> cartesian(([1, 2, 3], [4, 5], [6, 7]))
    array([[1, 4, 6],
           [1, 4, 7],
           [1, 5, 6],
           [1, 5, 7],
           [2, 4, 6],
           [2, 4, 7],
           [2, 5, 6],
           [2, 5, 7],
           [3, 4, 6],
           [3, 4, 7],
           [3, 5, 6],
           [3, 5, 7]])

    """

    arrays = [np.asarray(x) for x in arrays]
    dtype = arrays[0].dtype

    n = np.prod([x.size for x in arrays])
    if out is None:
        out = np.zeros([n, len(arrays)], dtype=dtype)

    m = int(n / arrays[0].size)
    out[:,0] = np.repeat(arrays[0], m)
    if arrays[1:]:
        cartesian(arrays[1:], out=out[0:m, 1:])
        for j in range(1, arrays[0].size):
            out[j*m:(j+1)*m, 1:] = out[0:m, 1:]
    return out

def calc_linear_mass(linear_type, linear_stroke_mm):
    if linear_type == "MTJ40":
        return 1.3 + 0.0024 * linear_stroke_mm
    elif linear_type == "MTJ65S":
        return 4.0 + 0.0055 * linear_stroke_mm
    elif linear_type == "MTJ65L":
        return 4.6 + 0.0055 * linear_stroke_mm
    else:
        raise ValueError("Not valid linear type: %s" % linear_type)

def does_it_tip(scenario):
    if "deceleration" not in scenario:
        scenario["deceleration"] = scenario["initial_velocity"] / scenario["stopping_time"]

    if "m_linear" not in scenario:
        m_linear = calc_linear_mass(scenario["linear_type"], scenario["linear_stroke_mm"])
    else:
        m_linear = scenario["m_linear"]
    total_m = scenario["m_base"] + m_linear + scenario["m_payload"]
    x_cm = (
        scenario["m_base"] * scenario["lx_base"] +
        m_linear * scenario["lx_linear"] +
        scenario["m_payload"] * scenario["lx_payload"]
    ) / total_m
    y_cm = (
        scenario["m_base"] * scenario["ly_base"] +
        m_linear * scenario["ly_linear"] +
        scenario["m_payload"] * scenario["ly_payload"]
    ) / total_m

    l_cm = scenario["y_ground"] + y_cm
    l_nf = scenario["wheel_spacing_base"] / 2.0 - x_cm

    f_g = total_m * scenario["g"]
    moment_masses = f_g * l_nf
    moment_decel = total_m * scenario["deceleration"] * l_cm
    f_nb = (moment_masses - moment_decel) / scenario["wheel_spacing_base"]

    return f_nb

def max_load_exceeded(scenario):
    m_linear = calc_linear_mass(scenario["linear_type"], scenario["linear_stroke_mm"])
    total_m = m_linear + scenario["m_payload"]
    # return total_m > scenario.get("max_base_payload", 0.0)
    return total_m

def test_points():
    deceleration = 5.0

    short_omron_ld = dict(
        deceleration=deceleration,  # m/s^2
        # initial_velocity=0.25,  # m/s
        # stopping_time=1.0,  # s
        # m_base=70,  # kg
        m_base=90,  # kg
        m_payload=10,  # kg
        linear_stroke_mm=750,  # mm
        linear_type="MTJ40",
        g=9.81,
        wheel_spacing_base=0.699,  # m

        # dimensions relative base geometric center
        lx_base=0.0,
        ly_base=0.0,
        lx_linear=0.0,
        ly_linear=0.383 + 0.375,  # m
        lx_payload=0.35 + 0.1,  # m
        ly_payload=0.383 + 0.7,  # m
        y_ground=0.35,  # m
    )

    tall_omron_ld = dict(
        deceleration=deceleration,  # m/s^2
        # initial_velocity=0.25,  # m/s
        # stopping_time=1.0,  # s
        # m_base=70.0,  # kg
        m_base=90,  # kg
        m_payload=10.0,  # kg
        linear_stroke_mm=1250,  # mm
        linear_type="MTJ65L",
        g=9.81,
        wheel_spacing_base=0.699,  # m

        lx_base=0.0,
        ly_base=0.0,
        lx_linear=0.0,
        ly_linear=0.383 + 0.625,  # m
        lx_payload=0.35 + 0.1,  # m
        ly_payload=0.383 + 1.2,  # m
        y_ground=0.35,  # m
    )

    clearpath_dingo = dict(
        deceleration=deceleration,  # m/s^2
        # initial_velocity=0.25,  # m/s
        # stopping_time=1.0,  # s
        m_base=9.1,  # kg
        m_payload=10.0,  # kg
        linear_stroke_mm=750,  # mm
        linear_type="MTJ40",
        g=9.81,
        wheel_spacing_base=0.5,  # m
        max_base_payload=20,  # kg

        lx_base=0.0,
        ly_base=0.0,
        lx_linear=0.0,
        ly_linear=0.06 + 0.375,  # m
        lx_payload=0.25 + 0.1,  # m
        ly_payload=0.06 + 0.7,  # m
        y_ground=0.06,  # m
    )

    clearpath_boxer = dict(
        deceleration=deceleration,  # m/s^2
        m_base=127,  # kg
        m_payload=10.0,  # kg
        linear_stroke_mm=1250,  # mm
        linear_type="MTJ65L",
        g=9.81,
        wheel_spacing_base=0.55,  # m
        max_base_payload=100,  # kg

        lx_base=0.0,
        ly_base=0.0,
        lx_linear=0.0,
        ly_linear=0.15 + 0.625,  # m
        lx_payload=0.375 + 0.1,  # m
        ly_payload=0.15 + 0.12,  # m
        y_ground=0.15,  # m
    )

    short_mir100 = dict(
        deceleration=deceleration,  # m/s^2
        m_base=70.0,  # kg
        m_payload=10.0,  # kg
        linear_stroke_mm=750,  # mm
        linear_type="MTJ40",
        g=9.81,
        wheel_spacing_base=0.8,  # m
        max_base_payload=100,  # kg

        lx_base=0.0,
        ly_base=0.0,
        lx_linear=0.0,
        ly_linear=0.175 + 0.375,  # m
        lx_payload=0.445 + 0.1,  # m
        ly_payload=0.175 + 0.7,  # m
        y_ground=0.175,  # m
    )

    tall_mir100 = dict(
        deceleration=deceleration,  # m/s^2
        m_base=70.0,  # kg
        m_payload=10.0,  # kg
        linear_stroke_mm=1250,  # mm
        linear_type="MTJ65L",
        g=9.81,
        wheel_spacing_base=0.8,  # m
        max_base_payload=100,  # kg

        lx_base=0.0,
        ly_base=0.0,
        lx_linear=0.0,
        ly_linear=0.175 + 0.625,  # m
        lx_payload=0.445 + 0.1,  # m
        ly_payload=0.175 + 1.2,  # m
        y_ground=0.175,  # m
    )

    dodobot = dict(
        # deceleration=deceleration,  # m/s^2
        initial_velocity=0.33,
        stopping_time=0.09,
        m_base=3.6,  # kg (total: 4.427)
        m_payload=0.6,  # kg
        m_linear=0.827,
        g=9.81,
        wheel_spacing_base=0.125,  # m

        lx_base=0.0,
        ly_base=0.0,
        lx_linear=0.0,
        ly_linear=0.1,  # m
        lx_payload=0.25,  # m
        ly_payload=0.15,  # m
        y_ground=0.05,  # m
    )

    print("short_omron_ld\t", does_it_tip(short_omron_ld))
    print("tall_omron_ld\t", does_it_tip(tall_omron_ld))
    print("clearpath_dingo\t", does_it_tip(clearpath_dingo), max_load_exceeded(clearpath_dingo))
    print("clearpath_boxer\t", does_it_tip(clearpath_boxer), max_load_exceeded(clearpath_boxer))
    print("short_mir100\t", does_it_tip(short_mir100), max_load_exceeded(short_mir100))
    print("tall_mir100\t", does_it_tip(tall_mir100), max_load_exceeded(tall_mir100))
    print("dodobot\t", does_it_tip(dodobot))


def test_combinations():
    # MiR
    base_config = dict(
        deceleration=0.0,  # m/s^2
        m_base=0.0,  # kg
        m_payload=10.0,  # kg
        linear_stroke_mm=1250,  # mm
        # linear_stroke_mm=750,  # mm
        # linear_type="MTJ40",
        linear_type="MTJ65L",
        g=9.81,
        wheel_spacing_base=0.8,  # m
        max_base_payload=100,  # kg
        safety_offset=150.0,

        lx_base=0.0,
        ly_base=0.0,
        lx_linear=0.0,
        # ly_linear=0.175 + 0.375,  # m
        ly_linear=0.175 + 0.625,  # m
        lx_payload=0.445 + 0.1,  # m
        # ly_payload=0.175 + 0.7,  # m
        ly_payload=0.175 + 0.12,  # m
        y_ground=0.175,  # m
    )
    # omron
    # base_config = dict(
    #     deceleration=0.0,  # m/s^2
    #     m_base=0.0,  # kg
    #     m_payload=10,  # kg
    #     linear_stroke_mm=1250,  # mm
    #     # linear_stroke_mm=750,  # mm
    #     linear_type="MTJ40",
    #     # linear_type="MTJ65S",
    #     # linear_type="MTJ65L",
    #     g=9.81,
    #     wheel_spacing_base=0.699,  # m
    #     safety_offset=150.0,
    #
    #     lx_base=0.0,
    #     ly_base=0.0,
    #     lx_linear=0.0,
    #     ly_linear=0.383 + 0.625,  # m
    #     lx_payload=0.30,  # m
    #     ly_payload=0.383 + 1.2,  # m
    #
    #     y_ground=0.35,  # m
    # )

    m_base = np.linspace(70, 120, 30)
    # m_base = 90

    deceleration = np.linspace(4.0, 6.0, 5)
    # deceleration = 0.5

    # lx_payload = np.linspace(0.4, 1.0, 10)
    lx_payload = np.linspace(0.4, 0.6, 5)
    # lx_payload = 0.3

    # m_payload = np.linspace(10.0, 20.0, 2)
    m_payload = 10.0

    # base_config["m_base"] = m_base
    # base_config["deceleration"] = deceleration
    # base_config["lx_payload"] = lx_payload
    # base_config["m_payload"] = m_payload
    # print(does_it_tip(base_config))

    # combinations = cartesian([m_base, deceleration, lx_payload, m_payload])
    combinations = cartesian([m_base, deceleration, lx_payload])
    for combination in combinations:
        base_config["m_base"] = combination[0]
        base_config["deceleration"] = combination[1]
        base_config["lx_payload"] = combination[2]
        # base_config["m_payload"] = combination[3]
        rear_force = does_it_tip(base_config)

        # print(rear_force > 0, rear_force, combination)
        if rear_force > base_config["safety_offset"]:
            print(rear_force, combination)

test_points()
# test_combinations()
# print(calc_linear_mass("MTJ40", np.linspace(750, 1250, 25)))
