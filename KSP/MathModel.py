import math
import matplotlib.pyplot as plt
from scipy import integrate

PI = math.pi
H_0 = 83
turn_start_time = 20
turn_end_time = 175
P = PI / 360
K = 0.75


# Угол между вертикалью и вектором скорости ракеты
def alpha(t: int):
    return (t < turn_start_time) * 0 + (turn_start_time <= t <= turn_end_time) * K * PI/2 * (t - turn_start_time)/(turn_end_time - turn_start_time) + (t > turn_end_time) * K * PI/2


# Угол между вектором силы тяги двигателя и вектором местного ускорения свободного падения в момент времени t
def gamma(t: int):
    return (t < turn_start_time) * PI + (turn_start_time <= t <= turn_end_time) * max(PI/2 + P, PI - PI/2 * (t - turn_start_time)/(turn_end_time - turn_start_time)) + (t > turn_end_time) * (PI/2 + P)


# Местное ускорение свободного падения в момент времени t
def g(t: int):
    return 9.81


# Гравитационные потери скорости в момент времени t
def deltaV_g(t: int):
    def f(t: int):
        return g(t) * (-math.cos(gamma(t)))

    return integrate.quad(f, 0, t)[0]


# Формула Циолковского - характеристическая скорость в момент времени t
def V_ch(t: int):
    v = 0
    for i in range(1, N + 1):
        numerator = M0 + sum(m0[j] for j in range(i, N + 1))
        denominator = M0 + max(m1[i], m0[i] - m[i] * (t - sum(t_i[j] for j in range(1, i)))) + sum(m0[j] for j in range(i + 1, N + 1))
        v += (t > sum(t_i[j] for j in range(1, i))) * (I[i] * math.log1p(-1 + numerator / denominator))
    return v


# Итоговая скорость ракеты в момент времени t
def V(t: int):
    return V_ch(t) - deltaV_g(t)


# Скорость набора высоты ракетой в момент времени t
def V_h(t: int):
    return math.cos(alpha(t)) * V(t)


# Горизонтальная скорость ракеты в момент времени t
def V_l(t: int):
    return math.cos(PI/2 - alpha(t)) * V(t)


# Высота в момент времени t
def h(xV_h: list, yV_h: list, t: int):
    return H_0 + integrate.trapezoid(yV_h[:t], xV_h[:t])


# Число ступеней
N = 3
# Масса полезной нагрузки
M0 = 870
# Масса заправленной ступени
m0 = [None, 202_960, 83_551, 3_908]
# Масса ступени без топлива
m1 = [None, 50_960, 17_851, 1308]
# Удельный импульс ступени в м/с
I = [None, 2797.8, 3089.1, 3481.4]
# Расход топлива ступени
m = [None, 1431, 304, 16]
# Время работы ступени
t_i = [None, 106, 216, 163]


def createGraphics(kspx, kspyV, kspyVh, kspyH, printTime):
    x = []
    yVch = []  # Характеристическая скорость
    yV = []  # Итоговая скорость
    yVh = []  # Вертикальная проекция скорости
    yVl = []  # Горизонтальная проекция скорости
    yH = []  # Высота
    ygamma = []
    ydeltaV_g = []  # Гравитационные потери скорости
    t = 0
    i = 0
    while t < sum(t_i[1:]):
        t += 0.5
        i += 1
        x.append(t)
        yVch.append(V_ch(t))
        yV.append(V(t))
        yVh.append(V_h(t))
        yVl.append(V_l(t))
        yH.append(h(x, yVh, i))
        ygamma.append(gamma(t))
        ydeltaV_g.append(t)
    print(x[x.index(printTime)], yV[x.index(printTime)])
    print(x[x.index(printTime)], yH[x.index(printTime)])
    plt.subplot(211)
    plt.title("Скорость")
    plt.plot(x, yV)
    plt.plot(kspx, kspyV)
    plt.legend([
        "Мат. модель",
        "KSP",
    ])
    plt.xlabel("Время, с")
    plt.ylabel("Скорость, м/с")

    plt.subplot(212)
    plt.title("Набранная высота")
    plt.plot(x, yH)
    plt.plot(kspx, kspyH)
    plt.legend([
        "Мат. модель",
        "KSP",
    ])
    plt.xlabel("Время, с")
    plt.ylabel("Высота, м")

    plt.show()

runMathModel = False
if runMathModel:
    createGraphics([], [], [], [], 188)
