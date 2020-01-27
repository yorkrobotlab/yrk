import yrk.motors as motors
import time

def forwards(speed):
      motors.set_motor_speed(0,speed)
      motors.set_motor_speed(3,speed)

def turn(speed):
      motors.set_motor_speed(0,speed)
      motors.set_motor_speed(3,-speed)

def brake_motors():
      motors.brake_motor(0)
      motors.brake_motor(3)

for i in range(4):
      forwards(0.5)
      time.sleep(0.5)
      brake_motors()
      time.sleep(0.1)
      turn(0.5)
      time.sleep(0.5)
      brake_motors()
      time.sleep(0.1)

motors.stop_all_motors()
