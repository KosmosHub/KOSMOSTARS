import math
import matplotlib.pyplot as plt
from scipy import integrate

FILENAME_RESULTS_MATHMOD = "resultsMathmod.txt"
END_TIME = 200

PI = math.pi

# Начальные параметры
g = -9.81
H_0 = 83
turn_start_time = 20
turn_end_time = 175
K = 0.75


def saveResults(filename, x, yV, yH):
    with open(filename, "wt", encoding="UTF-8") as file:
        file.write("Time; Velocity; Height\n")
        for i in range(len(x)):
            file.write(f"{x[i]}; {yV[i]}; {yH[i]}\n")


# Угол между вектором тяги двигателя и вертикалью по программе полета
def u(t: int):
    return (t < turn_start_time) * 0 + (turn_start_time <= t <= turn_end_time) * min(PI / 2, PI / 2 * (t - turn_start_time) / (turn_end_time - turn_start_time)) + (t > turn_end_time) * (PI / 2)


# Угол между вертикалью и вектором скорости ракеты
def alpha(t: int):
    return K * u(t)


# Угол между вектором силы тяги двигателя и вектором местного ускорения свободного падения в момент времени t
def gamma(t: int):
    return PI - u(t)


# Гравитационные потери скорости в момент времени t
def deltaV_g(t: int):
    def f(t: int):
        return math.cos(gamma(t))

    return g * integrate.quad(f, 0, t)[0]


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
def h(x: list, yV_h: list, t_ind: int):
    return H_0 + integrate.trapezoid(yV_h[:t_ind], x[:t_ind])


# Число ступеней
N = 3
# Масса полезной нагрузки
M0 = 870
# Масса заправленной i-й ступени
m0 = [None, 202_960, 83_551, 3_908]
# Масса i-й ступени без топлива
m1 = [None, 50_960, 17_851, 1308]
# Удельный импульс i-й ступени в м/с
I = [None, 2796, 3090, 3483]
# Расход топлива i-й ступени
m = [None, 1430, 304, 16]
# Время работы i-й ступени
t_i = [None, 106, 216, 163]


def createGraphics(kspx=tuple(x for x in range(END_TIME)),
                   kspyV=tuple(0 for x in range(END_TIME)),
                   kspyH=tuple(0 for x in range(END_TIME))):
    yV = []  # Итоговая скорость
    yVh = []  # Вертикальная проекция скорости
    yH = []  # Высота
    for i in range(len(kspx)):
        t = kspx[i]
        yV.append(round(V(t)))
        yVh.append(V_h(t))
        yH.append(round(h(kspx, yVh, i + 1)))
    saveResults(FILENAME_RESULTS_MATHMOD, kspx, yV, yH)
    calc_error(kspx, yV, yH, kspyV, kspyH)

    plt.subplot(211)
    plt.title("Скорость")
    plt.plot(kspx, yV)
    plt.plot(kspx, kspyV)
    plt.legend([
        "Мат. модель",
        "KSP",
    ])
    plt.xlabel("Время, с")
    plt.ylabel("Скорость, м/с")

    plt.subplot(212)
    plt.title("Набранная высота")
    plt.plot(kspx, yH)
    plt.plot(kspx, kspyH)
    plt.legend([
        "Мат. модель",
        "KSP",
    ])
    plt.xlabel("Время, с")
    plt.ylabel("Высота, м")

    plt.show()


def calc_error(x, yV_m, yH_m, yV_ksp, yH_ksp):
    V_abs_err = 0
    V_rel_err = None
    V_point = None
    H_abs_err = 0
    H_rel_err = None
    H_point = None
    for i in range(len(x)):
        V_var = abs(yV_m[i] - yV_ksp[i])
        H_var = abs(yH_m[i] - yH_ksp[i])
        if V_var > V_abs_err:
            V_abs_err = V_var
            V_rel_err = V_abs_err / yV_m[i] * 100
            V_point = x[i]
        if H_var > H_abs_err:
            H_abs_err = H_var
            H_rel_err = H_abs_err / yH_m[i] * 100
            H_point = x[i]
    print("Погрешность в точке максимального расхождения графиков")
    print(f"Скорость ({V_point} с)")
    print(f"    Абсолютная погрешность {V_abs_err} м/с")
    print(f"    Относительная погрешность {V_rel_err} %")
    print(f"Высота ({H_point} с)")
    print(f"    Абсолютная погрешность {H_abs_err} м")
    print(f"    Относительная погрешность {H_rel_err} %")


runMathModel = False
if runMathModel:
    createGraphics()
