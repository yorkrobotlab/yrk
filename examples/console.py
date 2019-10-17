#!/usr/bin/python
#
# York Robotics Kit Python API
# Version 0.1
# Curses console: A console program for the York Robotics Kit
# James Hilder, York Robotics Laboratory, Oct 2019

"""
.. module:: console
   :synopsis: A curses terminal-based console allowing hardware control

.. moduleauthor:: James Hilder <github.com/jah128>

A terminal-based console that provides feedback on sensor information and allows
keyboard based control of actuators. The console.py example uses the curses
library to allow console over-writing.  It uses line-drawing characters, which
may not render correctly on default settings on PuTTy; there are work-arounds
for this [KiTTy is worth looking at as an alternative].


To run::

   python console.py

"""

import yrk.settings as s,yrk.utils as utils, yrk.adc as adc, yrk.motors as motors
import yrk.power as power, yrk.switch as switch, yrk.gpio as gpio
import curses 	  # For fancy console over-writing
import os         # For call i2cdetect for sensor detection
import threading  # Run the sensor polling as a thread
import sys        # For clean system exit
import argparse	  # Command line arguments
import math       # For math functions
import time       # For timer\sleep functions
from array import *

running = True
selected_index = 0
gpio.setup_user_gpio()
switch.setup_switch_gpio()
motors.stop_all_motors()

# Function to exit gracefully one curses started
def end_curses(message):
  curses.nocbreak()
  curses.echo()
  curses.endwin()
  end_program(message)

#Function to exit program outside of curses
def end_program(message):
  running = False
  print (message)
  sys.exit(0)

# Function to read all sensors and update display
def take_readings():
  while(running == True):
    count = 0
    power.read_all_values()
    power_box.addstr(1,5,"%02.2f" % power.get_battery_voltage());
    power_box.addstr(1,16,"%1.2f" % power.get_pi_voltage());
    power_box.addstr(1,27,"%1.2f" % power.get_aux_voltage());
    power_box.addstr(1,38,"%1.2f" % power.get_pi_current());
    power_box.addstr(1,49,"%1.2f" % power.get_aux_current());
    power_box.addstr(1,60,"%2.1f" % power.get_pcb_temperature());
    power_box.addstr(1,70,"%2.1f" % utils.get_cpu_temp());
    for i in range(8):
        adc_box.addstr(1,4+(i*9),"%03d" % adc.read_adc(i));
        time.sleep(0.02)
    switch_register = switch.read_input_registers()
    if(switch_register & 0x400): sw_1_box.addstr(1,1,'PUSH 1',curses.A_STANDOUT)
    else: sw_1_box.addstr(1,1,'PUSH 1')
    if(switch_register & 0x200): sw_0_box.addstr(1,1,'PUSH 0',curses.A_STANDOUT)
    else: sw_0_box.addstr(1,1,'PUSH 0')
    if(switch_register & 0x100): sw_push_box.addstr(1,1,'CENTER',curses.A_STANDOUT)
    else: sw_push_box.addstr(1,1,'CENTER')
    if(switch_register & 0x080): sw_right_box.addstr(1,1,'RIGHT',curses.A_STANDOUT)
    else: sw_right_box.addstr(1,1,'RIGHT')
    if(switch_register & 0x040): sw_left_box.addstr(1,1,'LEFT',curses.A_STANDOUT)
    else: sw_left_box.addstr(1,1,'LEFT')
    if(switch_register & 0x020): sw_down_box.addstr(1,1,'DOWN',curses.A_STANDOUT)
    else: sw_down_box.addstr(1,1,'DOWN')
    if(switch_register & 0x010): sw_up_box.addstr(1,1,'UP',curses.A_STANDOUT)
    else: sw_up_box.addstr(1,1,'UP')
    if(switch_register & 0x008): sw_dip3_box.addstr(1,1,'DIP 3',curses.A_STANDOUT)
    else: sw_dip3_box.addstr(1,1,'DIP 3')
    if(switch_register & 0x004): sw_dip2_box.addstr(1,1,'DIP 2',curses.A_STANDOUT)
    else: sw_dip2_box.addstr(1,1,'DIP 2')
    if(switch_register & 0x002): sw_dip1_box.addstr(1,1,'DIP 1',curses.A_STANDOUT)
    else: sw_dip1_box.addstr(1,1,'DIP 1')
    if(switch_register & 0x001): sw_dip0_box.addstr(1,1,'DIP 0',curses.A_STANDOUT)
    else: sw_dip0_box.addstr(1,1,'DIP 0')
    if(motors.get_brake_state(0)): motor_box.addstr(1,10,'BRAKE')
    elif motors.get_motor_speed(0) == 0: motor_box.addstr(1,10,'COAST')
    else: motor_box.addstr(1,10,'%+1.2f' % (motors.get_motor_speed(0)))
    if(motors.get_brake_state(1)): motor_box.addstr(1,29,'BRAKE')
    elif motors.get_motor_speed(1) == 0: motor_box.addstr(1,29,'COAST')
    else: motor_box.addstr(1,29,'%+1.2f' % (motors.get_motor_speed(1)))
    if(motors.get_brake_state(2)): motor_box.addstr(1,48,'BRAKE')
    elif motors.get_motor_speed(2) == 0: motor_box.addstr(1,48,'COAST')
    else: motor_box.addstr(1,48,'%+1.2f' % (motors.get_motor_speed(2)))
    if(motors.get_brake_state(3)): motor_box.addstr(1,67,'BRAKE')
    elif motors.get_motor_speed(3) == 0: motor_box.addstr(1,67,'COAST')
    else: motor_box.addstr(1,67,'%+1.2f' % (motors.get_motor_speed(3)))
    motor_fault = gpio.read_motor_fault()
    if(motor_fault & 0x01): motor_box.addstr(1,16,'F',curses.A_STANDOUT)
    else: motor_box.addstr(1,16,' ')
    if(motor_fault & 0x02): motor_box.addstr(1,35,'F',curses.A_STANDOUT)
    else: motor_box.addstr(1,35,' ')
    if(motor_fault & 0x04): motor_box.addstr(1,54,'F',curses.A_STANDOUT)
    else: motor_box.addstr(1,54,' ')
    if(motor_fault & 0x08): motor_box.addstr(1,73,'F',curses.A_STANDOUT)
    else: motor_box.addstr(1,73,' ')
    if(selected_index == 0): motor_box.addstr(1,1,"MOTOR 0",curses.A_STANDOUT)
    else: motor_box.addstr(1,1,"MOTOR 0")
    if(selected_index == 1): motor_box.addstr(1,20,"MOTOR 1",curses.A_STANDOUT)
    else: motor_box.addstr(1,20,"MOTOR 1")
    if(selected_index == 2): motor_box.addstr(1,39,"MOTOR 2",curses.A_STANDOUT)
    else: motor_box.addstr(1,39,"MOTOR 2")
    if(selected_index == 3): motor_box.addstr(1,58,"MOTOR 3",curses.A_STANDOUT)
    else: motor_box.addstr(1,58,"MOTOR 3")


#Program code
if __name__ == "__main__":
    #Parse command line using Argument Parser
    parser = argparse.ArgumentParser("curses_console.py : Display YRK data in a curses-based console")
    parser.add_argument("-s","--silent",help="Disable non-error std-out messages",action="store_true")
    args = parser.parse_args()

    #Now initial setup is complete, we can create the main curses window
    stdscr = curses.initscr() # Create the curses console window
    curses.cbreak()	# Lets program react to keypresses without Enter
    curses.noecho()	# Turn off automatic echoing of keys to screen
    curses.curs_set(False) # Turn off cursor
    curses.start_color() # Enable colour mode in console
    curses.use_default_colors() # Use curses defaults
    stdscr.box() #Box arond whole screen
    stdscr.immedok(True)
    stdscr.refresh()

    #Create a box at the top of page for title
    title_box = curses.newwin(3,76,1,2) #Height,width,Y,X
    title_box.box()
    title_box.addstr (1,1,'York Robotics Kit Console                    York Robotics Laboratory 2019',curses.A_STANDOUT)
    title_box.immedok(True)
    title_box.refresh()

    #Create a box to display power information
    power_box = curses.newwin(3,76,4,2) #Height,width,Y,X
    power_box.box()
    power_box.addstr (1,1,'Vin:--.--V Vpi:-.--V Vaux:-.--V  Ipi:-.--A Iaux:-.--A  PCB:--.-C CPU:--.-C')
    power_box.immedok(True)
    power_box.refresh()
    stdscr.addstr (4,3,'Power Status')

    #Create a box to display ADC information
    adc_box = curses.newwin(3,76,7,2)
    adc_box.box()
    adc_box.addstr (1,1,'0: ---   1: ---   2: ---   3: ---   4: ---   5: ---   6: ---   7: ---')
    adc_box.immedok(True)
    adc_box.refresh()
    stdscr.addstr (7,3,'Raw ADC Values')
    adc_box.refresh()

    sw_0_box = curses.newwin(3,8,19,2)
    sw_dip0_box = curses.newwin(3,7,19,10)
    sw_dip1_box = curses.newwin(3,7,19,17)
    sw_dip2_box = curses.newwin(3,7,19,24)
    sw_dip3_box = curses.newwin(3,7,19,31)

    sw_up_box = curses.newwin(3,4,19,38)
    sw_down_box = curses.newwin(3,6,19,42)
    sw_left_box = curses.newwin(3,6,19,48)
    sw_right_box = curses.newwin(3,7,19,54)
    sw_push_box = curses.newwin(3,8,19,61)
    sw_1_box = curses.newwin(3,8,19,69)

    sw_0_box.box()
    sw_dip0_box.box()
    sw_dip1_box.box()
    sw_dip2_box.box()
    sw_dip3_box.box()
    sw_up_box.box()
    sw_down_box.box()
    sw_left_box.box()
    sw_right_box.box()
    sw_push_box.box()
    sw_1_box.box()

    sw_0_box.immedok(True)
    sw_0_box.refresh()
    sw_dip0_box.immedok(True)
    sw_dip1_box.immedok(True)
    sw_dip2_box.immedok(True)
    sw_dip3_box.immedok(True)
    sw_up_box.immedok(True)
    sw_down_box.immedok(True)
    sw_left_box.immedok(True)
    sw_right_box.immedok(True)
    sw_push_box.immedok(True)
    sw_1_box.immedok(True)

    sw_dip0_box.refresh()
    sw_dip1_box.refresh()
    sw_dip2_box.refresh()
    sw_dip3_box.refresh()
    sw_up_box.refresh()
    sw_down_box.refresh()
    sw_left_box.refresh()
    sw_right_box.refresh()
    sw_push_box.refresh()
    sw_1_box.refresh()

    #Create a box to display servo information
    servo_box = curses.newwin(3,76,10,2)
    servo_box.box()
    servo_box.addstr (1,1,'S0:XXXX  S1:XXXX  S2: XXXX  S3: XXXX  S4:XXXX  S5:XXXX  S6: XXXX  S7: XXXX')
    servo_box.immedok(True)
    servo_box.refresh()
    stdscr.addstr (10,3,'Servo Periods')



    #Create a box to display motor information
    motor_box = curses.newwin(3,76,13,2)
    motor_box.box()
    motor_box.addstr (1,1,'Motor 0: XX.XX     Motor 1: XX.XX     Motor 2: XX.XX     Motor 3: XX.XX')
    motor_box.immedok(True)
    motor_box.refresh()
    stdscr.addstr (13,3,'Motor Speeds')

    #Start the sensor polling thread
    sensor_thread = threading.Thread(target=take_readings,args=())
    sensor_thread.start()

    #Keyboard listener
    while (running == True):
      c=stdscr.getch()
      stdscr.addstr(16,10,'Key %s    ' % (c))
      if c==ord('q'): running = False
      elif c==curses.KEY_RIGHT or c==67:
          selected_index += 1
          if selected_index == 4: selected_index = 0
      elif c==curses.KEY_LEFT or c==68:
          selected_index -= 1
          if selected_index < 0: selected_index = 3
      elif c==curses.KEY_UP or c==65:
          if(selected_index > -1 and selected_index < 4):
              motors.set_motor_speed(selected_index,motors.get_motor_speed(selected_index)+0.1)
      elif c==curses.KEY_DOWN or c==66:
          if(selected_index > -1 and selected_index < 4):
              motors.set_motor_speed(selected_index,motors.get_motor_speed(selected_index)-0.1)

    end_curses("FIN.") # Graceful exit; restores terminal modes
