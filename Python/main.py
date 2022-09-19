import os.path

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

def smoothen_baro(t, p):
    cov = np.polyfit(t, p, 5)
    return np.polyval(cov, t)


def read_data(dir):
    types = np.dtype([
        ("time", np.uint32),
        ("phase", np.uint8),
        ("ax", np.int16),
        ("ay", np.int16),
        ("az", np.int16),
        ("gx", np.int16),
        ("gy", np.int16),
        ("gz", np.int16),
        ("at", np.int16),
        ("a_ok", np.uint8),
        ("b1d1", np.uint32),
        ("b1d2", np.uint32),
        ("b1_ok", np.uint8),
        ("b2d1", np.uint32),
        ("b2d2", np.uint32),
        ("b2_ok", np.uint8),
        ("tc", np.int16),
        ("th", np.int16),
        ("t_ok", np.uint8),
    ])
    i = 1
    data = np.fromfile(f"{dir}/FD0000.BIN", dtype=types)
    while True:
        path = f"{dir}/FD{i:04}.BIN"
        if os.path.exists(path):
            print(f"opening {path}")
            data_i = np.fromfile(path, dtype=types)
            data = np.append(data, data_i)
            i += 1
        else:
            break
    return data


def read_cal(dir):
    types = np.dtype([
        ("1c1", np.uint16),
        ("1c2", np.uint16),
        ("1c3", np.uint16),
        ("1c4", np.uint16),
        ("1c5", np.uint16),
        ("1c6", np.uint16),
        ("2c1", np.uint16),
        ("2c2", np.uint16),
        ("2c3", np.uint16),
        ("2c4", np.uint16),
        ("2c5", np.uint16),
        ("2c6", np.uint16),
        ("ac", np.uint16),
        ("gy", np.uint16),
    ])
    cal = np.fromfile(f"{dir}/CAL.BIN", dtype=types)
    return cal


class Accelerometer:
    def __init__(self, ac, gy):
        self.ac = ac
        self.gy = gy / 10.0

    def get_acc(self, a):
        return a / self.ac * 9.81

    def get_gy(self, g):
        return g / self.gy

    def get_t(self, t):
        return t / 100


class Barometer:
    def __init__(self, c1, c2, c3, c4, c5, c6):
        self.c1 = c1
        self.c2 = c2
        self.c3 = c3
        self.c4 = c4
        self.c5 = c5
        self.c6 = c6

    def get_baro(self, D1, D2):
        dT = D2 - (np.int32(self.c5) << 8)
        TEMP = 2000 + (np.int64(dT) * self.c6 >> 23)

        T2 = np.zeros(TEMP.shape, dtype=np.int64)
        OFF2 = np.zeros(TEMP.shape, dtype=np.int64)
        SENS2 = np.zeros(TEMP.shape, dtype=np.int64)
        T2[TEMP < 2000] = 3 * ((dT[TEMP < 2000] * dT[TEMP < 2000]) >> 33)
        T2[TEMP >= 2000] = 7 * (np.int64(dT * dT)[TEMP >= 2000]) / 137438953472
        OFF2[TEMP < 2000] = 3 * (TEMP[TEMP < 2000] - 2000.0) * (TEMP[TEMP < 2000] - 2000.0) / 2
        OFF2[TEMP >= 2000] = ((TEMP[TEMP >= 2000] - 2000) * (TEMP[TEMP >= 2000] - 2000)) / 16
        SENS2[TEMP < 2000] = 5 * (TEMP[TEMP < 2000] - 2000.0) * (TEMP[TEMP < 2000] - 2000.0) / 8
        OFF2[TEMP < -1500] = OFF2[TEMP < -1500] + 7 * (TEMP[TEMP < -1500] + 1500) * (TEMP[TEMP < -1500] + 1500.0)
        SENS2[TEMP < -1500] = SENS2[TEMP < -1500] + 4 * (TEMP[TEMP < -1500] + 1500) * (TEMP[TEMP < -1500] + 1500.0)

        OFF = (np.int64(self.c2) << 16) + ((self.c4 * np.int64(dT)) >> 7)
        SENS = (np.int64(self.c1) << 15) + ((self.c3 * np.int64(dT)) >> 8)

        TEMP -= T2
        OFF -= OFF2
        SENS -= SENS2
        P = ((D1 * SENS) / 2097152 - OFF) / 32768

        t = TEMP / 100.0
        p = P / 10.0
        return t, p


if __name__ == '__main__':
    dir = "HELVETIA_FLIGHT"
    cal = read_cal(dir)
    Acc = Accelerometer(cal['ac'], cal['gy'])
    B1 = Barometer(cal['1c1'], cal['1c2'], cal['1c3'], cal['1c4'], cal['1c5'], cal['1c6'])
    B2 = Barometer(cal['2c1'], cal['2c2'], cal['2c3'], cal['2c4'], cal['2c5'], cal['2c6'])

    data = read_data(dir)
    t1, p1 = B1.get_baro(data['b1d1'], data['b1d2'])
    t2, p2 = B2.get_baro(data['b2d1'], data['b2d2'])
    ax = Acc.get_acc(data['ax'])
    ay = Acc.get_acc(data['ay'])
    az = Acc.get_acc(data['az'])
    gx = Acc.get_gy(data['gx'])
    gy = Acc.get_gy(data['gy'])
    gz = Acc.get_gy(data['gz'])
    at = Acc.get_t(data['at'])

    tc = data["tc"] / 100
    th = data["th"] / 100

    d = {"timestamp" : data["time"],
         "ax": ax,
         "ay": ay,
         "az": az,
         "gx": gx,
         "gy": gy,
         "gz": gz,
         "t_tip":th,
         "t_pcb":tc,
         "t_static":t2,
         "t_dyn":t1,
         "p_static": p2,
         "p_static_filteres": smoothen_baro(data["time"], p2),
         "p_dyn" : p1
         }
    df = pd.DataFrame(data=d)
    df.to_csv("helvetia_pitot.csv",index=False)

    print(f"avg logging frequency: {1000/np.diff(data['time']).mean():.2f} Hz")
    plt.step(range(1, len(data["time"])), np.diff(data["time"]), color="black", where='pre')
    plt.ylabel(r"$\Delta t$ [ms]")
    plt.xlabel("measurement")
    plt.show()

    plt.step(data["time"], ax, color="black", where='pre')
    plt.step(data["time"], ay, color="red", where='pre')
    plt.step(data["time"], az, color="blue", where='pre')
    plt.ylabel(r"$a$ [m/s$^2$]")
    plt.xlabel(r"$t$ [ms]")
    plt.show()

    plt.step(data["time"], gx, color="black", where='pre')
    plt.step(data["time"], gy, color="red", where='pre')
    plt.step(data["time"], gz, color="blue", where='pre')
    plt.ylabel(r"$\omega$ [dps]")
    plt.xlabel(r"$t$ [ms]")
    plt.show()

    plt.step(data["time"], p1, color="black", where='pre')
    plt.step(data["time"], p2, color="red", where='pre')
    plt.ylabel(r"$p [hPa]")
    plt.xlabel(r"$t$ [ms]")
    plt.show()

    plt.step(data["time"], tc, color="black", where='pre')
    plt.step(data["time"], th, color="red", where='pre')
    plt.step(data["time"], at, color="blue", where='pre')
    plt.step(data["time"], t1, color="orange", where='pre')
    plt.step(data["time"], t2, color="brown", where='pre')
    plt.ylabel(r"$T$ [°C]")
    plt.xlabel(r"$t$ [ms]")
    plt.show()
