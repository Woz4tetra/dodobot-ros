import os
import scipy.signal
import scipy.optimize
import pickle
import numpy as np
import pandas as pd
from datetime import datetime
import matplotlib.pyplot as plt


def parse_msg(msg):
    line_data = msg.split("\t")
    identifier = line_data[0]
    if identifier == "power":
        try:
            timestamp, voltage_V, current_mA = list(map(float, line_data[1:]))
            if timestamp == 0.0:
                data = None
            else:
                data = timestamp, voltage_V, current_mA
        except ValueError as e:
            print(e, msg)
            data = None
    else:
        data = None
    return identifier, data


def parse_contents(df: pd.DataFrame, contents, time_interval=None):
    prev_date = None
    for line in contents.splitlines():
        line = line.replace("\x00", "")
        if len(line) == 0:
            continue

        date_str, msg = line.split(":\t", 1)
        date = datetime.strptime(date_str, "%Y-%m-%dT%H:%M:%S,%f")
        if time_interval is not None:
            if prev_date is None:
                prev_date = date
            else:
                date_diff = date - prev_date
                if date_diff.total_seconds() < time_interval:
                    continue
                prev_date = date

        identifier, data = parse_msg(msg)
        if data is None:
            continue

        df = df.append({
            "date"      : date,
            "timestamp" : data[0],
            "current_mA": data[1],
            "voltage_V" : data[2],
        }, ignore_index=True)
    return df


def create_data_frame(input_dir, output_dir, time_interval=None):
    input_dir = os.path.expanduser(input_dir)
    output_dir = os.path.expanduser(output_dir)
    df = pd.DataFrame(columns=["timestamp", "voltage_V", "current_mA"])

    for dirpath, dirnames, filenames in os.walk(input_dir):
        for filename in filenames:
            if filename.startswith("data"):
                path = os.path.join(dirpath, filename)
                print("Parsing %s" % path)

                with open(path) as file:
                    contents = file.read()
                    df = parse_contents(df, contents, time_interval)

    print(df)
    output_path = os.path.join(output_dir, "data.pkl")
    with open(output_path, 'wb') as file:
        pickle.dump(df, file)


def open_pickled_df(path):
    path = os.path.expanduser(path)
    with open(path, 'rb') as file:
        df = pickle.load(file)
    return df


def noise_filter(xn, Wn=0.1):
    b, a = scipy.signal.butter(3, Wn)
    zi = scipy.signal.lfilter_zi(b, a)
    z, _ = scipy.signal.lfilter(b, a, xn, zi=zi * xn[0])
    z2, _ = scipy.signal.lfilter(b, a, z, zi=zi * z[0])
    y = scipy.signal.filtfilt(b, a, xn)
    return y


def estimate_capacity(df, start_date=None, stop_date=None):
    if start_date is not None:
        start_mask = df["date"] > start_date
    else:
        start_mask = None
    if stop_date is not None:
        stop_mask = df["date"] <= stop_date
    else:
        stop_mask = None

    if start_mask is not None and stop_mask is not None:
        mask = start_mask & stop_mask
    elif start_mask is None:
        mask = stop_mask
    elif stop_mask is None:
        mask = start_mask
    else:
        mask = None

    if mask is not None:
        df = df.loc[mask]

    start_V = df["voltage_V"].iloc[0]
    stop_V = df["voltage_V"].iloc[-1]
    cutoff_V = 8.0

    ratio = (start_V - cutoff_V) / (start_V - stop_V)

    # dates = np.array(df["date"], dtype=np.float64)
    # avg_diff = np.mean(np.diff(dates)) * 1E-9

    avg_diff = np.mean(np.diff(df["timestamp"]))

    energy_used = np.sum(df["current_mA"]) * avg_diff * 1E-3
    energy_total = ratio * energy_used

    print(energy_total - energy_used)
    return energy_total

def main():
    input_dir = "~/Desktop/data"
    output_dir = "~/Desktop"
    # create_data_frame(input_dir, output_dir, time_interval=60.0)

    df = open_pickled_df("~/Desktop/data.pkl")
    df = df.sort_values("timestamp")
    df['date'] = pd.to_datetime(df['date'])

    start_date = datetime(2020, 11, 19, 20, 34)
    stop_date = datetime(2020, 11, 20, 3, 3)

    mask = (df['date'] > start_date) & (df['date'] <= stop_date)
    df = df.loc[mask]

    df["timestamp"] = df["timestamp"] - df["timestamp"].iloc[0]

    filtered_V = noise_filter(np.array(df["voltage_V"]))
    weights_V = np.polyfit(df["timestamp"], filtered_V, 1)
    model_V = np.poly1d(weights_V)
    model_V_data = model_V(df["timestamp"])
    print("weights_V:", weights_V)

    # print(estimate_capacity(df, stop_date=stop_date))

    # fn = lambda t, a, b: a + b * t ** 3
    # results = scipy.optimize.curve_fit(fn, df["timestamp"], filtered_V)
    # model_V_data = fn(df["timestamp"], results[0][0], results[0][1])

    fig, ax1 = plt.subplots()
    ax2 = ax1.twinx()

    ax1.plot(df["timestamp"], df["voltage_V"])
    ax1.plot(df["timestamp"], model_V_data)
    ax1.plot(df["timestamp"], filtered_V)

    ax2.plot(df["timestamp"], df["current_mA"] * df["voltage_V"])
    # ax2.plot(df["date"], noise_filter(np.array(df["current_mA"]), 0.04))

    # plt.plot(df["timestamp"], df["current_mA"])
    plt.show()


main()
