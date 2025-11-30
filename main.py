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

def dist(a,b):
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

    reset_odometry(x_mm=start_mm[0], y_mm=start_mm[1], theta_deg=0.0)

    # начальные значения
    wall_following = False
    hit_proj_goal_dist = None

    ev3.speaker.beep()

    while True:
        # обновление и получение позы
        x, y, theta = get_pose_mm()
        pos = (x, y)

        # Проверка достижения цели
        if dist(pos, goal_mm) <= GOAL_TOLERANCE_MM:
            drive.stop()
            ev3.screen.clear()
            ev3.screen.print("Goal reached")
            ev3.speaker.beep(1000,200)
            return True

        # измерение дистанции до препятствия
        try:
            obs_dist = ultra.distance()  # mm
        except Exception:
            obs_dist = 9999

        # вычисление направления на цель
        dx = goal_mm[0] - x
        dy = goal_mm[1] - y
        angle_to_goal_deg = math.degrees(math.atan2(dy, dx))
        heading_error = normalize_angle_deg(angle_to_goal_deg - theta)

        # критерий на M-line
        t_proj, proj_point = project_to_mline(pos, start_mm, goal_mm)
        on_mline = (abs(dist(pos, proj_point)) <= MLINE_TOLERANCE_MM) and (0.0 <= t_proj <= 1.0)

        if not wall_following:
            # если увидели препятствие впереди — переключаемся в wall-follow
            if obs_dist <= OBSTACLE_DIST_MM:
                wall_following = True
                # запомним расстояние от проекции на goal для точки касания (hit)
                _, hit_proj = project_to_mline(pos, start_mm, goal_mm)
                hit_proj_goal_dist = dist(hit_proj, goal_mm)
                ev3.screen.clear()
                ev3.screen.print("Hit obstacle, start wall follow")
                wait(200)
                # небольшая резкая разворот-манёвр, чтобы прижаться к стене
                drive.stop()
                wait(50)
                # продолжим в режиме wall-following на малой скорости
                continue

            # движение прямо к цели: пропорциональное управление углом
            turn_rate = KP_TURN * heading_error
            # ограничение turn_rate
            if turn_rate > 200: turn_rate = 200
            if turn_rate < -200: turn_rate = -200
            drive.drive(FORWARD_SPEED, turn_rate)
        else:
            # Wall-following (правостороннее): поддерживаем дистанцию WALL_TARGET_MM
            # простая P-структура для правой стороны (если ультразвук смотрит вперед, может корректировать влево/вправо)
            # Для правого следования: если расстояние больше желаемого, поворачиваем вправо (отрицательный поворот)
            error = obs_dist - WALL_TARGET_MM
            Kp_wall = 1.2
            turn_rate_wall = -Kp_wall * error  # знак подобран экспериментально
            # ограничение
            if turn_rate_wall > 200: turn_rate_wall = 200
            if turn_rate_wall < -200: turn_rate_wall = -200
            drive.drive(WALL_FOLLOW_SPEED, turn_rate_wall)

            # Условие выхода: на M-line и проекция ближе к цели, чем точка касания
            if on_mline:
                _, proj_now = project_to_mline(pos, start_mm, goal_mm)
                proj_now_goal_dist = dist(proj_now, goal_mm)
                if hit_proj_goal_dist is not None and proj_now_goal_dist < hit_proj_goal_dist:
                    # выходим из wall-follow и идём к цели
                    wall_following = False
                    ev3.screen.clear()
                    ev3.screen.print("Leave wall follow, on M-line")
                    wait(200)
                    drive.stop()
                    continue

        wait(50)

ev3.screen.clear()
ev3.screen.print("Press center")
ev3.screen.print("button to start")

while Button.CENTER not in ev3.buttons.pressed():
    wait(10)

# Вызов навигации
bug2_go_to_goal(X, Y, start_X, start_Y)
