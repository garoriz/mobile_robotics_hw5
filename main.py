#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import math

# Create your objects here.
ev3 = EV3Brick()

left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
ultra = UltrasonicSensor(Port.S4)

X = 25
Y = 25

start_X = 0
start_Y = 0

# Укажите реальные размеры ваших колёс/шасси (мм)
WHEEL_DIAMETER_MM = 55.5    # мм
AXLE_TRACK_MM = 175       # мм (расстояние между центрами колёс)

drive = DriveBase(left_motor, right_motor, wheel_diameter=WHEEL_DIAMETER_MM, axle_track=AXLE_TRACK_MM)
drive.settings(straight_speed=150, straight_acceleration=300, turn_rate=200, turn_acceleration=400)

FORWARD_SPEED = 120            # mm/s при движении к цели
WALL_FOLLOW_SPEED = 80         # mm/s при следовании стене
OBSTACLE_DIST_MM = 150         # порог обнаружения препятствия (мм)
WALL_TARGET_MM = 80            # желаемая дистанция до стены при следовании (мм)
MLINE_TOLERANCE_MM = 40        # допуска вхождения в M-line (мм)
GOAL_TOLERANCE_MM = 60         # расстояние до цели, считаем достигнутым (мм)
KP_TURN = 3.0

wheel_circ_mm = math.pi * WHEEL_DIAMETER_MM
deg_to_mm = wheel_circ_mm / 360.0

odom_x = 0.0
odom_y = 0.0
odom_theta_deg = 0.0  # 0 = +X
_last_left_deg = left_motor.angle()
_last_right_deg = right_motor.angle()

def reset_odometry(x_mm=0.0, y_mm=0.0, theta_deg=0.0):
    global odom_x, odom_y, odom_theta_deg, _last_left_deg, _last_right_deg
    odom_x = x_mm
    odom_y = y_mm
    odom_theta_deg = theta_deg
    _last_left_deg = left_motor.angle()
    _last_right_deg = right_motor.angle()

def update_odometry():
    global odom_x, odom_y, odom_theta_deg, _last_left_deg, _last_right_deg
    left_deg = left_motor.angle()
    right_deg = right_motor.angle()
    dl_deg = left_deg - _last_left_deg
    dr_deg = right_deg - _last_right_deg
    _last_left_deg = left_deg
    _last_right_deg = right_deg

    dl_mm = dl_deg * deg_to_mm
    dr_mm = dr_deg * deg_to_mm
    ds = (dl_mm + dr_mm) / 2.0
    dtheta_rad = (dr_mm - dl_mm) / AXLE_TRACK_MM
    theta_rad = math.radians(odom_theta_deg) + dtheta_rad

    odom_x += ds * math.cos(theta_rad)
    odom_y += ds * math.sin(theta_rad)
    odom_theta_deg = math.degrees(theta_rad)
    # normalize
    odom_theta_deg = (odom_theta_deg + 180.0) % 360.0 - 180.0

def get_pose_mm():
    update_odometry()
    return odom_x, odom_y, odom_theta_deg

def dist(a, b):
    return ((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5

def normalize_angle_deg(a):
    a = (a + 180.0) % 360.0 - 180.0
    return a

def project_to_mline(pos, start, goal):
    # возвращает проекцию точки pos на прямую start->goal (параметр t и координаты proj)
    gx = goal[0] - start[0]
    gy = goal[1] - start[1]
    denom = gx*gx + gy*gy
    if denom == 0:
        return 0.0, start
    px = pos[0] - start[0]
    py = pos[1] - start[1]
    t = (px*gx + py*gy) / denom
    proj_x = start[0] + t * gx
    proj_y = start[1] + t * gy
    return t, (proj_x, proj_y)

def bug2_go_to_goal(goal_x_cm, goal_y_cm, start_x_cm, start_y_cm):
    # перевод в мм
    start_mm = (start_x_cm*10.0, start_y_cm*10.0)
    goal_mm = (goal_x_cm*10.0, goal_y_cm*10.0)

    # M-line угол (радианы / градусы)
    mline_angle_rad = math.atan2(goal_mm[1] - start_mm[1], goal_mm[0] - start_mm[0])
    mline_angle_deg = math.degrees(mline_angle_rad)

    reset_odometry(x_mm=start_mm[0], y_mm=start_mm[1], theta_deg=0.0)

    # состояния
    STATE_GO_TO_GOAL = 0
    STATE_FOLLOW_WALL = 1
    state = STATE_GO_TO_GOAL

    hit_point = None            # (x,y) mm of Hi
    hit_proj_goal_dist = None   # d(Hi_proj, T)
    wall_follow_start_time = None
    WALL_LOOP_DETECT_DIST = 60  # mm: если вернулись к Hi ближе этой дистанции -> замкнутый обход

    ev3.screen.clear()
    ev3.screen.print("Bug2 start")
    ev3.speaker.beep()

    # Вспомогательные локальные функции
    def at_goal(pos):
        return dist(pos, goal_mm) <= GOAL_TOLERANCE_MM

    def on_mline_and_clear(pos):
        t_proj, proj_point = project_to_mline(pos, start_mm, goal_mm)
        on_mline = (abs(dist(pos, proj_point)) <= MLINE_TOLERANCE_MM) and (0.0 <= t_proj <= 1.0)
        # проверим, что впереди свободно (примерно)
        try:
            d = ultra.distance()
        except Exception:
            d = 9999
        return on_mline and (d > OBSTACLE_DIST_MM), proj_point, t_proj

    # Повернуться к направлению M-line перед началом
    #drive.stop()
    # При старте задаём направление движения по M-line
    # Поворачиваем на угол mline_angle_deg относительно 0 (предполагается 0 = +X)
    drive.turn(-mline_angle_deg)

    loop_counter = 0
    max_iterations = 20000

    while loop_counter < max_iterations:
        loop_counter += 1

        x, y, theta = get_pose_mm()
        pos = (x, y)

        # 1) проверка цели в любом состоянии
        if at_goal(pos):
            ev3.screen.clear()
            ev3.screen.print("Goal reached")
            ev3.speaker.beep(1000,200)
            return True

        # чтение ультразвука
        try:
            obs_dist = ultra.distance()  # mm
        except Exception:
            obs_dist = 9999

        if state == STATE_GO_TO_GOAL:
            # Движемся по прямой S-T (M-line). Поддерживаем направление mline_angle_deg.
            # Если встретили препятствие - зафиксировать Hi и перейти в FOLLOW_WALL
            if obs_dist <= OBSTACLE_DIST_MM:
                # hit point Hi = текущая позиция
                hit_point = pos
                # запомним расстояние от проекции Hi на M-line до цели
                _, hit_proj = project_to_mline(hit_point, start_mm, goal_mm)
                hit_proj_goal_dist = dist(hit_proj, goal_mm)
                wall_follow_start_time = StopWatch()
                wall_follow_start_time.reset()
                state = STATE_FOLLOW_WALL
                drive.stop()
                wait(150)
                # начнём следовать границе вправо: делаем плавный разворот вправо чтобы прижаться к стене
                # небольшой шаг назад, затем поворот вправо ~30deg чтобы начало обхода было стабильным
                drive.drive(WALL_FOLLOW_SPEED, 120)  # краткий манёвр
                wait(250)
                continue

            # иначе едем вдоль M-line
            # Скорость по прямой; корректируем heading к mline_angle_deg если сильно ушли
            heading_err = normalize_angle_deg(mline_angle_deg - theta)
            turn_rate = KP_TURN * heading_err
            if turn_rate > 200: turn_rate = 200
            if turn_rate < -200: turn_rate = -200
            drive.drive(FORWARD_SPEED, turn_rate)

        else:  # STATE_FOLLOW_WALL
            # Простое правое следование стене: поддерживаем расстояние WALL_TARGET_MM
            # Если ультразвук смотрит вперёд, то используем его как боковой измеритель: простая P-структура
            error = obs_dist - WALL_TARGET_MM
            Kp_wall = 1.6
            turn_rate_wall = -Kp_wall * error
            if turn_rate_wall > 200: turn_rate_wall = 200
            if turn_rate_wall < -200: turn_rate_wall = -200
            drive.drive(WALL_FOLLOW_SPEED, turn_rate_wall)

            # 2a) Если на ходу дошли до цели
            if at_goal(pos):
                drive.stop()
                ev3.screen.clear()
                ev3.screen.print("Goal reached")
                ev3.speaker.beep(1000,200)
                return True

            # 2b) Проверка встречи M-line в точке Q с условием d(Q,T) < d(Hi,T) и линия из Q к T свободна
            on_mline, proj_now, t_now = on_mline_and_clear(pos)
            if on_mline and (0.0 <= t_now <= 1.0):
                proj_now_goal_dist = dist(proj_now, goal_mm)
                # убедиться, что проекция ближе к цели, чем проекция Hi и впереди нет препятствия
                try:
                    front_clear = ultra.distance() > OBSTACLE_DIST_MM
                except Exception:
                    front_clear = True
                if hit_proj_goal_dist is not None and (proj_now_goal_dist + 1.0 < hit_proj_goal_dist) and front_clear:
                    # leaving point Li = proj_now (Q)
                    # переходим в GO_TO_GOAL, перед этим повернёмся к M-line
                    state = STATE_GO_TO_GOAL
                    drive.stop()
                    ev3.screen.clear()
                    ev3.screen.print("Leave wall-follow")
                    wait(120)
                    # поворот к направлению M-line
                    drive.turn(mline_angle_deg - theta)
                    wait(50)
                    continue

            # 2c) проверка возвращения к Hi (замкнутый обход) -> цель недостижима
            if hit_point is not None and dist(pos, hit_point) <= WALL_LOOP_DETECT_DIST and wall_follow_start_time is not None and wall_follow_start_time.time() > 800:
                drive.stop()
                ev3.screen.clear()
                ev3.screen.print("Trapped: cannot reach")
                ev3.speaker.beep(300,600)
                return False

        wait(60)

    # safety fallback
    drive.stop()
    ev3.screen.clear()
    ev3.screen.print("Timeout")
    ev3.speaker.beep(300,200)
    return False

ev3.screen.clear()
ev3.screen.print("Press center")
ev3.screen.print("button to start")

while Button.CENTER not in ev3.buttons.pressed():
    wait(10)

# Вызов навигации
bug2_go_to_goal(X, Y, start_X, start_Y)
