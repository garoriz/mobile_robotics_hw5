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

# Добавлено: моторы и навигация к цели
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Укажите реальные размеры ваших колёс/шасси (мм)
WHEEL_DIAMETER_MM = 56    # мм
AXLE_TRACK_MM = 115       # мм (расстояние между центрами колёс)

drive = DriveBase(left_motor, right_motor, wheel_diameter=WHEEL_DIAMETER_MM, axle_track=AXLE_TRACK_MM)
drive.settings(straight_speed=150, straight_acceleration=300, turn_rate=200, turn_acceleration=400)

def navigate_to_goal(Xg_cm, Yg_cm, X0_cm, Y0_cm):
    # перевод в мм
    dx_mm = (Xg_cm - X0_cm) * 10.0
    dy_mm = (Yg_cm - Y0_cm) * 10.0

    # расстояние и угол до цели (в градусах)
    distance_mm = math.hypot(dx_mm, dy_mm)
    angle_deg = math.degrees(math.atan2(dy_mm, dx_mm))

    # Поворот к целевому углу (предполагается начальный heading = 0 по +X)
    ev3.screen.clear()
    ev3.screen.print("Turn {:.1f} deg".format(angle_deg))
    drive.turn(angle_deg)
    wait(200)

    # Поездка вперёд на distance_mm
    ev3.screen.clear()
    ev3.screen.print("Go {:.0f} mm".format(distance_mm))
    drive.straight(distance_mm)
    wait(200)

    ev3.speaker.beep()

# Вызов навигации
navigate_to_goal(X, Y, start_X, start_Y)
