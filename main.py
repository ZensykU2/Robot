#!/usr/bin/env pybricks-micropython
# Wächter-Roboter (Security Bot) - Quadrat-Patrouille

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import UltrasonicSensor, GyroSensor, Motor
from pybricks.parameters import Port, Color, Direction, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from math import pi

ev3 = EV3Brick()
ultra = UltrasonicSensor(Port.S4)
gyro = GyroSensor(Port.S2, direction=Direction.CLOCKWISE)

medium_motor = Motor(Port.A)
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

wheel_diameter = 56
axle_track = 114
drive = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)

PATROL_DISTANCE = 400  # mm pro Seite
ALARM_DISTANCE = 100   # mm (10 cm)
SPEED = 150            # mm/s
TURN_SPEED = 100       # Drehgeschwindigkeit °/s

gyro.reset_angle(0)
medium_motor.reset_angle(45)

def check_intruder():
    return ultra.distance() < ALARM_DISTANCE

def trigger_alarm():
    drive.stop()
    
    for i in range(5):
        ev3.screen.clear()
        ev3.screen.draw_text(20, 30, "!!! WARNUNG !!!")
        ev3.screen.draw_text(10, 50, "Eindringling!")
        ev3.screen.draw_text(15, 70, "Abstand: {} cm".format(ultra.distance() // 10))
        
        ev3.light.on(Color.RED)
        ev3.speaker.beep(frequency=1000, duration=200)
        wait(200)
        
        ev3.light.off()
        wait(200)
    
    for i in range(2):
        medium_motor.run_target(
        speed=500,          
        target_angle=90,    
        then=Stop.HOLD,     
        wait=True
        )
        medium_motor.run_target(
            speed=500,
            target_angle=45,
            then=Stop.COAST,
            wait=True
        )
    
    medium_motor.reset_angle(45)
    ev3.screen.clear()
    ev3.screen.print("Weiche zurück...")
    drive.straight(-100)
    
    ev3.light.on(Color.ORANGE)
    wait(1000)

def turn_90_degrees():
    target_angle = gyro.angle() + 90
    
    left_motor.run(TURN_SPEED)
    right_motor.run(-TURN_SPEED)
    
    while gyro.angle() < target_angle:
        wait(10)
    
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    
    wait(200) 

def patrol_one_side(side_number):

    distance_cm = ultra.distance() // 10
    ev3.screen.clear()
    ev3.screen.print("Patrouille")
    ev3.screen.print("Seite: {}".format(side_number))
    ev3.screen.print("Abstand: {} cm".format(distance_cm))
    
    left_motor.reset_angle(0)
    
    drive.drive(SPEED, 0)
    
    while True:
        angle = abs(left_motor.angle())
        driven_mm = (angle / 360) * (wheel_diameter * pi)
        
        if driven_mm >= PATROL_DISTANCE:
            break
        
        if check_intruder():
            trigger_alarm()
            break
        
        wait(50)
    
    drive.stop()
    wait(300)

ev3.screen.clear()
ev3.screen.print("Wächter aktiv")
ev3.screen.print("Starte Patrouille...")
ev3.light.on(Color.GREEN)
ev3.speaker.beep()
wait(1000)

side_count = 0

while True:
    patrol_one_side(side_count + 1)
    
    side_count = (side_count + 1) % 4
    
    ev3.screen.clear()
    ev3.screen.print("Wende...")
    ev3.screen.print("Gyro: {}°".format(gyro.angle()))
    ev3.light.on(Color.GREEN)
    
    turn_90_degrees()
    
    wait(500)