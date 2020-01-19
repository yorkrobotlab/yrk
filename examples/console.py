#!/usr/bin/python
#
# York Robotics Kit Python API
# Version 0.21
# Curses console: A console program for the York Robotics Kit
# James Hilder, York Robotics Laboratory, Jan 2020

"""
.. module:: console
   :synopsis: A curses terminal-based console allowing hardware control

.. moduleauthor:: James Hilder <github.com/jah128>

A terminal-based console that provides feedback on sensor information and allows
keyboard based control of actuators. The console.py example uses the **curses**
library to allow console over-writing.  It uses line-drawing characters, which
may not render correctly on default settings on Windows, *kitty* (a fork of *putty*)
has an ``Allow ACS line drawing in UTF`` setting which allows the
correct rendering.


It provides a useful quick test for hardware and also a useful example of how to
do many low-level API calls.  The console should be run without any other code
(include ``core.py``) running.  It can be run as follows::

   cd ~/yrk/examples
   python console.py


.. figure:: /images/console.png
     :width: 550px
     :height: 347px
     :alt: Screen shot of console.py python program

     Screen shot of ``console.py``



The cursor keys on the keyboard can be used to move between the different motor, LED and servo options
(*the console sets the PWM driver up for 1.5mS analogue servos*).  As the console can enable and motors
and servos it should be used with caution.


"""

import yrk.settings as s,yrk.utils as utils, yrk.adc as adc, yrk.motors as motors
import yrk.power as power, yrk.switch as switch, yrk.gpio as gpio, yrk.led as led
import yrk.pwm as pwm
import curses 	  # For fancy console over-writing
from curses import wrapper
import logging    # For {pre-curses} error logging
import os         # For call i2cdetect for sensor detection
import threading  # Run the sensor polling as a thread
import sys        # For clean system exit
import argparse	  # Command line arguments
import math       # For math functions
import time       # For timer\sleep functions
from array import *

pwm_enabled = True
running = True
selected_index = 0
led_index = 0
led_brightness = 3
switched_out_5V = True #Init as true but will be toggled when toggle_5V_SO is called
switched_out_12V = True
display_pwm_as_microseconds = True
pwm_periods = 0



def update_servo_period(output,increment):
    global pwm_periods
    pwm_periods[output] += increment
    if pwm_periods[output] > 4095: pwm_periods[output] = 4095
    if pwm_periods[output] < 0: pwm_periods[output] = 0
    display_value = pwm_periods[output]
    if display_pwm_as_microseconds: display_value = int(1000000 * pwm.estimate_on_time(pwm_periods[output]))
    servo_box.addstr(1,(8*output) + 4,"%4d" % display_value, curses.A_BOLD)
    if pwm_enabled:
        pwm.set_duty_cycle_raw(output,pwm_periods[output])


def update_led_brightness(increment):
    global led_brightness
    led_brightness += increment
    if led_brightness > 15 : led_brightness = 15
    if led_brightness < 0 : led_brightness = 0
    led_box.addstr(1,46,"%02d [%03d%%]" % (led_brightness, int(6.67 * led_brightness) ) )
    led.set_brightness(led_brightness)
    update_led(0)

def toggle_5V_SO():
    global switched_out_5V
    switched_out_5V = not switched_out_5V
    gpio.set_switched_output_5V(switched_out_5V)
    if(switched_out_5V): so_box.addstr(1,16,"ON ")
    else: so_box.addstr(1,16,"OFF")

def toggle_12V_SO():
    global switched_out_12V
    switched_out_12V = not switched_out_12V
    gpio.set_switched_output_12V(switched_out_12V)
    if(switched_out_12V): so_box.addstr(1,6,"ON ")
    else: so_box.addstr(1,6,"OFF")

def update_led(increment):
    global led_index
    led_index += increment
    led_options = len(led.solid_colours) + len(led.body_animations) - 1
    if led_index < 0 : led_index = led_options
    if led_index > led_options : led_index = 0
    if led_index < len(led.solid_colours) : led.set_colour_solid(led_index)
    else: led.animation(led_index - len(led.solid_colours))

def get_led_str():
    if led_index < len(led.solid_colours) : ret_str = led.solid_colours[led_index][0]
    else: ret_str = led.body_animations[led_index - len(led.solid_colours)][0]
    return ret_str.ljust(20)

# Function to read all sensors and update display
def take_readings():
  while(running == True):
    count = 0
    power.read_all_values()
    power_box.addstr(1,5,"%02.2fV" % power.get_battery_voltage(),curses.A_BOLD);
    power_box.addstr(1,16,"%1.2fV" % power.get_pi_voltage(),curses.A_BOLD);
    power_box.addstr(1,27,"%1.2fV" % power.get_aux_voltage(),curses.A_BOLD);
    power_box.addstr(1,38,"%1.2fA" % power.get_pi_current(),curses.A_BOLD);
    power_box.addstr(1,49,"%1.2fA" % power.get_aux_current(),curses.A_BOLD);
    power_box.addstr(1,60,"%2.1fC" % power.get_pcb_temperature(),curses.A_BOLD);
    power_box.addstr(1,70,"%2.1fC" % utils.get_cpu_temp(),curses.A_BOLD);
    for i in range(8):
        adc_box.addstr(1,4+(i*9),"%03d" % adc.read_adc(i),curses.A_BOLD);
        time.sleep(0.01)
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
    if(selected_index == 4): led_box.addstr(1,1,get_led_str(),curses.A_STANDOUT)
    else: led_box.addstr(1,1,get_led_str())
    if(selected_index == 5): led_box.addstr(1,34,"Brightness:",curses.A_STANDOUT)
    else: led_box.addstr(1,34,"Brightness:")
    if(selected_index == 7): so_box.addstr(1,12,"5V:",curses.A_STANDOUT)
    else: so_box.addstr(1,12,"5V:")
    if(selected_index == 6): so_box.addstr(1,1,"12V:",curses.A_STANDOUT)
    else: so_box.addstr(1,1,"12V:")
    if(selected_index == 8): servo_box.addstr(1,1,"S0:",curses.A_STANDOUT)
    else: servo_box.addstr(1,1,"S0:")
    if(selected_index == 9): servo_box.addstr(1,9,"S1:",curses.A_STANDOUT)
    else: servo_box.addstr(1,9,"S1:")
    if(selected_index == 10): servo_box.addstr(1,17,"S2:",curses.A_STANDOUT)
    else: servo_box.addstr(1,17,"S2:")
    if(selected_index == 11): servo_box.addstr(1,25,"S3:",curses.A_STANDOUT)
    else: servo_box.addstr(1,25,"S3:")
    if(selected_index == 12): servo_box.addstr(1,33,"S4:",curses.A_STANDOUT)
    else: servo_box.addstr(1,33,"S4:")
    if(selected_index == 13): servo_box.addstr(1,41,"S5:",curses.A_STANDOUT)
    else: servo_box.addstr(1,41,"S5:")
    if(selected_index == 14): servo_box.addstr(1,49,"S6:",curses.A_STANDOUT)
    else: servo_box.addstr(1,49,"S6:")
    if(selected_index == 15): servo_box.addstr(1,57,"S7:",curses.A_STANDOUT)
    else: servo_box.addstr(1,57,"S7:")
    if(selected_index == 16): servo_box.addstr(1,65,"Freq:",curses.A_STANDOUT)
    else: servo_box.addstr(1,65,"Freq:")

def main(stdscr):
    #Don't really like this long list of globals but was needed adapt older version to use curses wrapper
    global title_box,pwm_periods,power_box,adc_box,servo_box,motor_box,so_box,led_box,sw_push_box,sw_0_box,sw_dip0_box,sw_dip1_box,sw_dip2_box,sw_dip3_box,sw_left_box,sw_right_box,sw_up_box,sw_down_box,sw_1_box
    global pwm_enabled,running, selected_index
    try:
        pwm.set_pwm_frequency(50.0)
    except OSError:
        logging.warning("PWM I2C Error: Disabling PWM functionality")
        pwm_enabled = False
    default_servo_period_raw = pwm.calculate_nearest_duty_cycle_to_period(0.0015)
    pwm_periods = [default_servo_period_raw] * 8
    gpio.setup_user_gpio()
    switch.setup_switch_gpio()
    motors.stop_all_motors()
    #Now initial setup is complete, we can create the main curses window
    curses.cbreak()	# Lets program react to keypresses without Enter
    curses.noecho()	# Turn off automatic echoing of keys to screen
    curses.curs_set(False) # Turn off cursor
    curses.start_color() # Enable colour mode in console
    curses.use_default_colors() # Use curses defaults
    curses.init_pair(1, curses.COLOR_RED, curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_GREEN, curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    curses.init_pair(4, curses.COLOR_MAGENTA, curses.COLOR_BLACK)
    stdscr.attron(curses.color_pair(3)  | curses.A_BOLD)
    stdscr.box() #Box arond whole screen
    stdscr.immedok(True)
    stdscr.refresh()

    #Create a box at the top of page for title
    title_box = curses.newwin(3,76,1,2) #Height,width,Y,X
    title_box.attron(curses.color_pair(3)  )
    title_box.box()
    title_box.addstr (1,1,'York Robotics Kit Console V0.2               York Robotics Laboratory 2020',curses.A_BOLD)
    title_box.immedok(True)
    title_box.refresh()

    #Create a box to display power information
    power_box = curses.newwin(3,76,4,2) #Height,width,Y,X
    power_box.attron(curses.color_pair(2))
    power_box.box()
    power_box.addstr (1,1,'Vin:--.--V Vpi:-.--V Vaux:-.--V  Ipi:-.--A Iaux:-.--A  PCB:--.-C CPU:--.-C',curses.color_pair(0))
    power_box.immedok(True)
    power_box.refresh()
    stdscr.addstr (4,3,'Power Status',curses.color_pair(2) | curses.A_BOLD)

    #Create a box to display ADC information
    adc_box = curses.newwin(3,76,7,2)
    adc_box.attron(curses.color_pair(2))
    adc_box.box()
    adc_box.addstr (1,1,'0: ---   1: ---   2: ---   3: ---   4: ---   5: ---   6: ---   7: ---',curses.color_pair(0))
    adc_box.immedok(True)
    adc_box.refresh()
    stdscr.addstr (7,3,'Raw ADC Values',curses.color_pair(2) | curses.A_BOLD)
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

    sw_0_box.attron(curses.color_pair(4))
    sw_dip0_box.attron(curses.color_pair(4))
    sw_dip1_box.attron(curses.color_pair(4))
    sw_dip2_box.attron(curses.color_pair(4))
    sw_dip3_box.attron(curses.color_pair(4))
    sw_up_box.attron(curses.color_pair(4))
    sw_down_box.attron(curses.color_pair(4))
    sw_left_box.attron(curses.color_pair(4))
    sw_right_box.attron(curses.color_pair(4))
    sw_push_box.attron(curses.color_pair(4))
    sw_1_box.attron(curses.color_pair(4))

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
    servo_box.attron(curses.color_pair(3))
    servo_box.box()
    #servo_box.addstr (1,1,'S0:XXXX  S1:XXXX  S2:XXXX  S3:XXXX  S4:XXXX  S5:XXXX  S6:XXXX  S7:XXXX',curses.color_pair(0))
    servo_box.immedok(True)
    servo_box.refresh()
    if display_pwm_as_microseconds: stdscr.addstr (10,3,'Servo Period uS')
    else: stdscr.addstr (10,3,'Servo Raw Value')

    #Create a box to display motor information
    motor_box = curses.newwin(3,76,13,2)
    motor_box.attron(curses.color_pair(3))
    motor_box.box()
    motor_box.addstr (1,1,'Motor 0: XX.XX     Motor 1: XX.XX     Motor 2: XX.XX     Motor 3: XX.XX',curses.color_pair(0))
    motor_box.immedok(True)
    motor_box.refresh()
    stdscr.addstr (13,3,'Motor Speeds')

    led_box = curses.newwin(3,56,16,2)
    led_box.attron(curses.color_pair(3))
    led_box.box()
    led_box.addstr (1,34,'Brightness: -- [---%]',curses.color_pair(0))
    led_box.immedok(True)
    led_box.refresh()
    stdscr.addstr (16,3,'LED Mode')

    so_box = curses.newwin(3,20,16,58)
    so_box.attron(curses.color_pair(3))
    so_box.box()
    so_box.addstr (1,1,'12V: ---   5V: ---',curses.color_pair(0))
    so_box.immedok(True)
    so_box.refresh()
    stdscr.addstr (16,59,'Switched Outputs')

    toggle_5V_SO()
    toggle_12V_SO()
    update_led_brightness(0)

    #Start the sensor polling thread
    sensor_thread = threading.Thread(target=take_readings,args=())
    sensor_thread.daemon = True #This stops the thread running when main thread stops
    sensor_thread.start()

    #Keyboard listener
    while (running == True):
      c=stdscr.getch()
      #stdscr.addstr(16,10,'Key %s    ' % (c))
      if c==ord('q'):
          running = False
      elif c==curses.KEY_RIGHT or c==67:
          selected_index += 1
          if selected_index == 17: selected_index = 0
      elif c==curses.KEY_LEFT or c==68:
          selected_index -= 1
          if selected_index < 0: selected_index = 16
      elif c==curses.KEY_UP or c==65:
          if(selected_index > -1 and selected_index < 4):
              motors.set_motor_speed(selected_index,motors.get_motor_speed(selected_index)+0.1)
          if selected_index == 4: update_led(1)
          if selected_index == 5: update_led_brightness(1)
          if selected_index == 6: toggle_12V_SO()
          if selected_index == 7: toggle_5V_SO()
          if selected_index > 7 and selected_index < 16: update_servo_period(selected_index - 8, 1)


      elif c==curses.KEY_DOWN or c==66:
          if(selected_index > -1 and selected_index < 4):
              motors.set_motor_speed(selected_index,motors.get_motor_speed(selected_index)-0.1)
          if selected_index == 4: update_led(-1)
          if selected_index == 5: update_led_brightness(-1)
          if selected_index == 6: toggle_12V_SO()
          if selected_index == 7: toggle_5V_SO()
          if selected_index > 7 and selected_index < 16: update_servo_period(selected_index - 8, -1)

#Program code
if __name__ == "__main__":
    #Parse command line using Argument Parser
    parser = argparse.ArgumentParser("console.py : Display YRK data in a curses-based console")
    parser.add_argument("-s","--silent",help="Disable non-error std-out messages",action="store_true")
    args = parser.parse_args()
    wrapper(main)
