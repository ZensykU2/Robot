#!/usr/bin/env pybricks-micropython
# Wächter-Roboter (Security Bot) - Quadrat-Patrouille

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import UltrasonicSensor, GyroSensor, Motor
from pybricks.parameters import Port, Color, Direction
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# Initialisierung
ev3 = EV3Brick()
ultra = UltrasonicSensor(Port.S4)
gyro = GyroSensor(Port.S2, direction=Direction.CLOCKWISE)

left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

wheel_diameter = 56
axle_track = 114
drive = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)

PATROL_DISTANCE = 400  # mm pro Seite
ALARM_DISTANCE = 100   # mm (10 cm)
SPEED = 150            # mm/s

gyro.reset_angle(0)

def check_intruder():
    return ultra.distance() < ALARM_DISTANCE

def trigger_alarm():
    drive.stop()
    
    ev3.screen.clear()
    ev3.screen.draw_text(20, 40, "!!! WARNUNG !!!")
    ev3.screen.draw_text(10, 60, "Eindringling erkannt!")
    
    ev3.light.on(Color.RED)
    ev3.speaker.beep(frequency=1000, duration=300)
    wait(500)

# Hauptschleife
ev3.screen.clear()
ev3.screen.print("Wächter aktiv")
ev3.light.on(Color.GREEN)

side_count = 0 

while True:
    # Vorwärts patrouillieren
    distance_cm = ultra.distance() // 10
    ev3.screen.clear()
    ev3.screen.print("Patrouille")
    ev3.screen.print("Seite: {}".format(side_count + 1))
    ev3.screen.print("Abstand: {} cm".format(distance_cm))
    
    drive.drive(SPEED, 0)
    
    distance_traveled = 0
    check_interval = 50
    
    # Eine Seite des Quadrats abfahren
    while distance_traveled < PATROL_DISTANCE:
        if check_intruder():
            trigger_alarm()
            break
        
        wait(check_interval)
        distance_traveled += SPEED * check_interval / 1000
    
    drive.stop()
    wait(300)
    
    # Immer zur nächsten Seite (auch nach Alarm)
    side_count += 1
    
    if side_count >= 4:
        side_count = 0
    
    ev3.screen.clear()
    ev3.screen.print("Wende...")
    ev3.light.on(Color.GREEN)
    drive.turn(90)  
    wait(500)