import pandas as pd
from numpy.fft import rfft, rfftfreq

from main import read_cal, Accelerometer, Barometer, read_data
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import root


def v_bernoulli(p_dynamic, p_static, T):
    M = 0.029
    R = 8.314
    rho = p_static * M / (R * (T + 273.15))
    speed_of_sound = 20.05 * np.sqrt(T + 273.15)
    return np.sqrt(2 * np.abs(p_dynamic - p_static) / rho) / speed_of_sound


def mach_helper(M, p_dynamic, p_static):
    g = 1.4
    return (g + 1) / (1 + g * M ** 2) * ((2 / (g + 1)) * (1 + (g - 1) / 2 * M ** 2)) ** (
            g / (g - 1)) - p_static / p_dynamic


def mach_helper2(M, p_dynamic, p_static):
    g = 1.4
    return (1 - g + 2 * g * M ** 2) / (
            g + 1) * ((g + 1) ** 2 * M ** 2 / (4 * g * M ** 2 - 2 * (g - 1))) ** (g / (g - 1)) - p_dynamic / p_static


def mach_helper3(M, p_dynamic, p_static):
    k = 1.4
    dp = p_dynamic - p_static
    return p_static * ((((k + 1) / 2 * M ** 2) ** (
            k / (k - 1))) / ((2 * k / (k + 1) * M ** 2 - (k - 1) / (k + 1)) ** (1 / (k - 1))) - 1) - dp


def sub_helper(M, p_dynamic, p_static):
    k = 1.4
    dp = p_dynamic - p_static
    return p_static * ((1 + (k - 1) / 2 * M ** 2) ** (k / (k - 1)) - 1) - dp


def v_supersonic(p_dynamics, p_statics, Ts):
    v = []
    res = []
    for p_static, p_dynamic, T in zip(p_statics, p_dynamics, Ts):
        print(f"{len(v)}/{len(Ts)}")
        zero = root(mach_helper3, np.array([v_bernoulli(p_dynamic, p_static, T)]),
                    args=(p_dynamic, p_static))
        v.append(zero.x)
        res.append(mach_helper3(zero.x, p_dynamic, p_static))
    return np.array(v), np.array(res)


def v_subsonic(p_dynamics, p_statics, Ts):
    v = []
    res = []
    for p_static, p_dynamic, T in zip(p_statics, p_dynamics, Ts):
        print(f"{len(v)}/{len(Ts)}")
        zero = root(sub_helper, np.array([v_bernoulli(p_dynamic, p_static, T)]),
                    args=(p_dynamic, p_static))
        v.append(zero.x)
        res.append(sub_helper(zero.x, p_dynamic, p_static))
    return np.array(v), np.array(res)


def make_fft(t, ax, ay, az):
    SAMPLE_RATE = 1000 / np.diff(t).mean()
    N = len(ay)

    a = rfft(ay)
    f = rfftfreq(N, 1 / SAMPLE_RATE)
    freq_1y = f
    freq_1_ampy = np.abs(a)

    a = rfft(az)
    f = rfftfreq(N, 1 / SAMPLE_RATE)
    freq_1z = f
    freq_1_ampz = np.abs(a)

    a = rfft(ax)
    f = rfftfreq(N, 1 / SAMPLE_RATE)
    freq_1x = f
    freq_1_ampx = np.abs(a)

    plt.plot(freq_1x, freq_1_ampx, label="x")
    plt.plot(freq_1y, freq_1_ampy, label="y")
    plt.plot(freq_1z, freq_1_ampz, label="z")
    plt.xlabel("f [Hz]")
    plt.ylabel("amplitude [a.u.]")
    plt.legend()
    plt.show()


def smoothen_baro(t, p):
    cov = np.polyfit(t, p, 5)
    return np.polyval(cov, t)


if __name__ == "__main__":
    dir = "HELVETIA_FLIGHT"
    cal = read_cal(dir)
    Acc = Accelerometer(cal['ac'], cal['gy'])
    B1 = Barometer(cal['1c1'], cal['1c2'], cal['1c3'], cal['1c4'], cal['1c5'], cal['1c6'])
    B2 = Barometer(cal['2c1'], cal['2c2'], cal['2c3'], cal['2c4'], cal['2c5'], cal['2c6'])

    data = read_data(dir)[:17000]
    t = data["time"] - 600

    t1, p1 = B1.get_baro(data['b1d1'], data['b1d2'])
    t2, p2 = B2.get_baro(data['b2d1'], data['b2d2'])

    p2_raw = p2
    p2 = smoothen_baro(t, p2)

    ax = Acc.get_acc(data['ax'])
    ay = Acc.get_acc(data['ay'])
    az = Acc.get_acc(data['az'])
    gx = Acc.get_gy(data['gx'])
    gy = Acc.get_gy(data['gy'])
    gz = Acc.get_gy(data['gz'])
    at = Acc.get_t(data['at'])

    tc = data["tc"] / 100
    th = data["th"] / 100

    df = pd.read_csv("HELVETIA_FLIGHT/Helvetia - flight_info_processed.csv")

    fig, (axv, axa, axp, axt) = plt.subplots(4, 1, sharex=True)

    v, r = v_supersonic(p1, p2, th)
    v_, r_ = v_subsonic(p1, p2, th)

    # axr.plot(t, r,color="orange")
    # axr.plot(t, r_,color="green")
    # axr.set_ylabel("residuals (supersonic calc)")

    speed_of_sound = 20.05 * np.sqrt(tc + 273.15)
    axv.plot(t, v_bernoulli(p1, p2, th) * speed_of_sound, label="bernoulli")
    # axv.plot(t, v[:, 0] * speed_of_sound, label="supersonic")
    axv.plot(t, v_[:, 0] * speed_of_sound, label="supersonic")
    axv.plot(np.array(df["ts"])[:5000] * 1000, df["velocity"][:5000], label="CATS")
    axv.plot(t, speed_of_sound, ls="--", color="black")
    axv.axvline(6540 - 600, ls="--", color="red")
    axv.axvline(10180 - 600, ls="--", color="red")
    axv.axvline(14560 - 600, ls="--", color="red")
    axv.axvline(22590 - 600, ls="--", color="red")
    axv.set_ylabel(r"$v$ [m/s]")
    axv.legend()

    axa.step(t, -ax, color="black", where='pre', label="ax")
    axa.step(t, -ay, color="red", where='pre', label="ay")
    axa.step(t, -az, color="blue", where='pre', label="az")
    axa.plot(np.array(df["ts"])[:5000] * 1000, df["acceleration"][:5000], label="CATS")

    axa.set_ylabel(r"$a$ [m/s$^2$]")
    axa.legend()

    axp.step(t, p1, color="black", where='pre', label="p_dyn (baro1)")
    axp.step(t, p2_raw, color="blue", where='pre', label="p_stat raw (baro2)")
    axp.step(t, p2, color="red", where='pre', label="p_stat (baro2)")
    axp.axvline(6540 - 600, ls="--", color="red", label="shocks")
    axp.axvline(10180 - 600, ls="--", color="red")
    axp.axvline(14560 - 600, ls="--", color="red")
    axp.axvline(22590 - 600, ls="--", color="red")
    axp.set_ylabel(r"$p [hPa]")
    axp.legend()

    axt.step(t, tc, color="black", where='pre', label="T_pcb")
    axt.step(t, th, color="red", where='pre', label="T_tip")
    axt.step(t, t1, color="blue", where='pre', label="T_baro1")
    axt.step(t, t2, color="brown", where='pre', label="T_baro2")
    axt.set_ylabel(r"$T$ [°C]")
    axt.set_xlabel(r"$t$ [ms]")
    axt.legend()
    plt.show()
