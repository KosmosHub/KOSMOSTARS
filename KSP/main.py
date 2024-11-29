import math
import time
import krpc
import MathModel

PI = math.pi
P = math.degrees(MathModel.P)

def print_report(mess):
    print(f"Время {round(TIME): >3} c | Высота {round(ALTITUDE): >6} м | {mess}")


x = []
yV = []
yVh = []
yVl = []
yH = []

target_altitude = 230000
turn_start_time = MathModel.turn_start_time
turn_end_time = MathModel.turn_end_time

conn = krpc.connect(name='Launch into orbit')
vessel = conn.space_center.active_vessel

# Установление потоков телеметрии
# Реальное время
ut = conn.add_stream(getattr, conn.space_center, 'ut')
# Скорость
speed = conn.add_stream(getattr, vessel.flight(vessel.orbit.body.reference_frame), 'speed')
# Скорость вертикальная
Vh = conn.add_stream(getattr, vessel.flight(vessel.orbit.body.reference_frame), 'vertical_speed')
# Скорость горизонтальная
Vl = conn.add_stream(getattr, vessel.flight(vessel.orbit.body.reference_frame), 'horizontal_speed')
# Высота
altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
# Высота апогея орбиты
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')

# Первая ступень
stage_7_resources = vessel.resources_in_decouple_stage(stage=5)
stage_7_fuel = conn.add_stream(stage_7_resources.amount, name='LiquidFuel')
# Вторая ступень
stage_5_resources = vessel.resources_in_decouple_stage(stage=1)
stage_5_fuel = conn.add_stream(stage_5_resources.amount, name='LiquidFuel')
# Третья ступень
stage_2_resources = vessel.resources_in_decouple_stage(stage=0)
stage_2_fuel = conn.add_stream(stage_2_resources.amount, name='LiquidFuel')

stage7_activated = False  # Двигатели первой ступени
stage6_activated = False  # Двигатель второй ступени
stage5_activated = False  # Сброс первой ступени
stage4_activated = False  # Сброс обтекателя
stage3_activated = False  # Сброс второй ступени
stage2_activated = False  # Двигатель третьей ступени
stage1_activated = False  # Сброс третьей ступени
stage0_activated = False  # Парашют

# Настройки перед запуском
vessel.control.sas = False
vessel.control.rcs = False

START_TIME = ut()
TIME = ut() - START_TIME
ALTITUDE = altitude()
APOAPSIS = apoapsis()
print_report("Запуск! Тяга 1.0")

vessel.control.throttle = 1.0
vessel.control.activate_next_stage()
stage7_activated = True

# Запуск автопилота для установки нужного наклона ракеты при полете
vessel.auto_pilot.engage()
vessel.auto_pilot.target_pitch_and_heading(90, 90)

turn_angle = 0

flag1 = False
flag2 = False
px = ut() - START_TIME
while True:
    TIME = ut() - START_TIME
    ALTITUDE = altitude()
    APOAPSIS = apoapsis()

    # Запись данных для графика
    if TIME - px >= 1:
        x.append(TIME)
        yV.append(speed())
        yVh.append(Vh())
        yVl.append(Vl())
        yH.append(ALTITUDE)
        px = TIME

    if turn_start_time < TIME < turn_end_time:
        if not flag1:
            flag1 = True
            print_report("Начало разворота")
        frac = ((TIME - turn_start_time) /
                (turn_end_time - turn_start_time))
        new_turn_angle = frac * 90
        if abs(new_turn_angle - turn_angle) > 0.5:
            turn_angle = new_turn_angle
            vessel.auto_pilot.target_pitch_and_heading(max(P, 90-turn_angle), 90)
    elif turn_end_time < TIME:
        if not flag2:
            flag2 = True
            print_report("Окончание разворота")
        vessel.auto_pilot.target_pitch_and_heading(P, 90)

    if not stage6_activated:
        if stage_7_fuel() == 0:
            vessel.control.activate_next_stage()
            stage6_activated = True
            vessel.control.activate_next_stage()
            stage5_activated = True
            print_report("Сброс первой ступени")

    if APOAPSIS >= target_altitude:
        break

vessel.control.activate_next_stage()
stage4_activated = True
print_report("Сброс обтекателя")
vessel.control.throttle = 0.0
vessel.auto_pilot.target_pitch_and_heading(0, 90)

TIME = ut() - START_TIME
ALTITUDE = altitude()
print_report(f"Апогей {round(APOAPSIS)} м достигнут. Тяга 0.0")
print(f"Время {round(x[-1])} с | Скорость {round(yV[-1])} м/с | Высота {round(yH[-1])} м")

MathModel.createGraphics(x, yV, yVh, yH, round(x[-1]))

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Plan circularization burn (using vis-viva equation)
print_report('Планирование выхода на орбиту')
mu = vessel.orbit.body.gravitational_parameter
r = vessel.orbit.apoapsis
a1 = vessel.orbit.semi_major_axis
a2 = r
v1 = math.sqrt(mu*((2./r)-(1./a1)))
v2 = math.sqrt(mu*((2./r)-(1./a2)))
delta_v = v2 - v1
node = vessel.control.add_node(
    ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v)

# Calculate burn time (using rocket equation)
F = vessel.available_thrust
Isp = vessel.specific_impulse * 9.82
m0 = vessel.mass
m1 = m0 / math.exp(delta_v/Isp)
flow_rate = F / Isp
burn_time = (m0 - m1) / flow_rate

print_report("Ориентирование корабля")
vessel.auto_pilot.reference_frame = node.reference_frame
vessel.auto_pilot.target_direction = (0, 1, 0)

print_report("Ожидание маневра")
burn_ut = ut() + vessel.orbit.time_to_apoapsis - (burn_time/2.)
lead_time = 5
conn.space_center.warp_to(burn_ut - lead_time)

time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
while time_to_apoapsis() - (burn_time/2.) > 0:
    pass
vessel.control.throttle = 1.0
print_report("Начало маневра. Тяга 1.0")
time.sleep(burn_time - 0.1)
vessel.control.throttle = 0.15
print_report("Тяга 0.15")
remaining_burn = conn.add_stream(node.remaining_burn_vector, node.reference_frame)
while remaining_burn()[1] > 4:
    pass
vessel.control.throttle = 0.0
node.remove()
print_report(f"Корабль выведен на орбиту. Апогей {apoapsis()} м. Тяга 0.0")
print(f"Время {ut() - START_TIME} с | Скорость {round(speed())} м/с | Высота {round(altitude())} м")
