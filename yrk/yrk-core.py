#!/usr/bin/python
#
# York Robotics Kit Python API
# Version 0.1
# Core services
#
# In normal use, a single instance of this should be run as a program when
# using the York Robotics Kit.  This is done automatically on boot by the
# .bashrc and yrk/bootscript.sh scripts.
#
# James Hilder, York Robotics Laboratory, Jan 2020

"""
.. module:: yrk-core
   :synopsis: Core services module

.. moduleauthor:: James Hilder <github.com/jah128>

The yrk-core module provides the low level hypervisory functions for normal
use of the York Robotics Kit.  This includes the following list of operations,
which are periodically checked:

 * Low and critical battery level monitoring
 * High and critical temperature monitoring
 * Fault conditions and low-voltage on the aux- and pi- supply rails
 * Monitoring of DIP-switches and kill-switch

The module is responsible for handling DIP-switch changes.  It is by default
run automatically by the bootscript.sh script if DIP switch 1 is enabled.
Assuming it is, the module then handles launching the ROS services if DIP switch
2 is enabled [and terminating them if is disabled], and likewise launching and
closing the DASH-DAQ web interface is DIP switch 3 is enabled and disabled.

"""

#Setup settings file and log
import yrk.settings as settings, logging, os

settings.init()
settings.setup_logger("test.log")

import threading,time
import yrk.power as power
import yrk.switch as switch
import yrk.utils as utils
import yrk.led as led
import yrk.display as display
import yrk.audio as audio

logging.info("York Robotics Kit Core Server [yrk-core.py] Started - Version %s" % (settings.VERSION_STRING))

#Setup GPIO library and interrupt pins
import RPi.GPIO as GPIO


#Globals
core_running = True
battery_check_requested = False
temperature_check_requested = False
switch_check_requested = True
previous_switch_state = 0
battery_warning_state = 0
temperature_warning_state = 0

def switch_interrupt_callback(pin):
  global switch_check_requested
  switch_check_requested = True

GPIO.setmode(GPIO.BCM)
GPIO.setup(settings.SWITCH_INTERRUPT_PIN,GPIO.IN,pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(settings.SWITCH_INTERRUPT_PIN , GPIO.FALLING, switch_interrupt_callback)

audio.setup_audio()


def shutdown():
    logging.critical("Battery voltage critically low, shutting down.")
    time.sleep(0.5)
    os.system("shutdown")

def dip_switch_handler(current_dip_state,previous_dip_state):
  #The DIP switch has changed state...
  logging.info('DIP Switch state altered')
  if current_dip_state & 1 == 1 and previous_dip_state & 1 == 0:
    logging.info("DIP 1 enabled")
  if current_dip_state & 1 == 0 and previous_dip_state & 1 == 1:
    demo_mode_enabled = False
    logging.info("DIP 1 disabled")
  if current_dip_state & 2 == 2 and previous_dip_state & 2 == 0:
    logging.info("DIP 2 enabled: Starting ROS services")
  if current_dip_state & 2 == 0 and previous_dip_state & 2 == 2:
    logging.info("DIP 2 disabled: Stopping ROS services")
  if current_dip_state & 4 == 4 and previous_dip_state & 4 == 0:
    logging.info("DIP 3 enabled: Starting DASH-DAQ server")
  if current_dip_state & 4 == 0 and previous_dip_state & 4 == 4:
    logging.info("DIP 3 disabled: Stopping DASH-DAQ server")
  if current_dip_state & 8 == 8 and previous_dip_state & 8 == 0:
    logging.info("DIP 4 enabled: Starting demo mode")
  if current_dip_state & 8 == 0 and previous_dip_state & 8 == 8:
    logging.info("DIP 4 disabled: Stopping demo mode")


def check_switch_state():
    global previous_switch_state
    current_switch_state = switch.read_input_registers()
    #Check if the switches have changed state (false interrupts eg bounce possible...)
    if previous_switch_state != current_switch_state:
        logging.debug("Switch handler called [switch state %X]" % (current_switch_state))
        current_dip_state = current_switch_state % 16
        previous_dip_state = previous_switch_state % 16
        if current_dip_state != previous_dip_state:
          #DIP switch has been changed...
          logging.debug('DIP switch changed')
          switch.set_dip_leds(current_dip_state)
          if settings.USE_DIP_FUNCTIONS:
            dip_switch_handler(current_dip_state,previous_dip_state)
        previous_switch_state = current_switch_state
    else:
        logging.debug("Switch handler called; switch state unchanged")



def check_battery_state():
     global battery_warning_state
     voltage = power.read_battery_voltage()
     if(voltage < settings.BATTERY_SHUTDOWN_VOLTAGE and settings.BATTERY_CRITICAL_SHUTDOWN):
         logging.critical("Battery voltage critically low [%2.2f V], starting forced shutdown." % (voltage))
         shutdown()
     if(voltage < settings.BATTERY_CRITICAL_VOLTAGE and battery_warning_state != 1):
         battery_warning_state = 1
         led.animation(8)
         display.warning("BATTERY EMPTY")
         audio.play_audio_file(settings.AUDIO_FILEPATH + "batterycriticallylow.wav")
         logging.warning("Battery voltage critical warning [%2.2f V]" % (voltage))
     elif (voltage < settings.BATTERY_LOW_VOLTAGE and battery_warning_state != 2):
         battery_warning_state = 2
         display.warning("BATTERY LOW")
         audio.play_audio_file(settings.AUDIO_FILEPATH + "batterylow.wav")
         logging.warning("Battery voltage low warning [%2.2f V]" % (voltage))
     #If voltage recovers to +0.3V above low state restore normal state
     if(battery_warning_state != 0 and voltage>(settings.BATTERY_LOW_VOLTAGE + 0.3)):
         battery_warning_state = 0
         led.animation(0)
         logging.info("Battery voltage recovered [%2.2f V]" % (voltage))

def check_temperature():
     global temperature_warning_state
     pcb_temp = power.read_pcb_temperature()
     cpu_temp = utils.get_cpu_temp()
     logging.debug("CPU Temp: %2.2f C, PCB Temp: %2.2f C" % (cpu_temp, pcb_temp))
     if((cpu_temp > settings.CPU_SHUTDOWN_TEMP or pcb_temp>settings.PCB_SHUTDOWN_TEMP) and settings.TEMPERATURE_CRITICAL_SHUTDOWN):
         logging.critical("Temperature too high [CPU:%2.2f C, PCB:%2.2f], starting forced shutdown." % (cpu_temp, pcb_temp))
         shutdown()
     if((cpu_temp > settings.CPU_CRITICAL_TEMP or pcb_temp >settings.PCB_CRITICAL_TEMP) and temperature_warning_state != 1):
         temperature_warning_state = 1
         led.animation(8)
         display.warning("OVERHEATING")
         audio.play_audio_file(settings.AUDIO_FILEPATH + "overheating.wav")
         logging.warning("Temperature critically high warning [CPU:%2.2f C, PCB:%2.2f]" % (cpu_temp, pcb_temp))
     elif ((cpu_temp > settings.CPU_WARNING_TEMP or pcb_temp > settings.PCB_WARNING_TEMP) and temperature_warning_state != 2):
         temperature_warning_state = 2
         display.warning("TEMP HIGH")
         audio.play_audio_file(settings.AUDIO_FILEPATH + "warning.wav")
         logging.warning("Temperature high warning [CPU:%2.2f C, PCB:%2.2f]" % (cpu_temp, pcb_temp))
     #If temperatures recover to 2C below warning state restore normal state
     if(temperature_warning_state != 0 and cpu_temp<(settings.CPU_WARNING_TEMP - 2) and pcb_temp<(settings.PCB_WARNING_TEMP - 2)):
         temperature_warning_state = 0
         led.animation(0)
         logging.info("Temperatures recovered [CPU:%2.2f C, PCB:%2.2f]" % (cpu_temp, pcb_temp))

def battery_check_thread():
    global battery_check_requested
    while core_running:
        time.sleep(settings.BATTERY_CHECK_PERIOD)
        battery_check_requested = True

def temperature_check_thread():
    global temperature_check_requested
    while core_running:
        time.sleep(settings.TEMPERATURE_CHECK_PERIOD)
        temperature_check_requested = True

#Initialise battery voltage checking thread [unless disabled in settings]
if (settings.ENABLE_BATTERY_MONITOR):
    battery_check_requested=True
    battery_checkThread = threading.Thread(target=battery_check_thread)
    battery_checkThread.daemon = True
    battery_checkThread.start()

#Initialise temperature checking thread [unless disabled in settings]
if (settings.ENABLE_TEMPERATURE_MONITOR):
    temperature_check_requested=True
    temperature_checkThread = threading.Thread(target=battery_check_thread)
    temperature_checkThread.daemon = True
    temperature_checkThread.start()

#Main loop
def core_loop():
  """Core services loop


  """
  global battery_check_requested, switch_check_requested, temperature_check_requested
  while core_running:
      if battery_check_requested:
           check_battery_state()
           battery_check_requested = False
      if temperature_check_requested:
           check_temperature()
           temperature_check_requested = False
      if switch_check_requested:
           check_switch_state()
           switch_check_requested = False
      time.sleep(0.001)

core_loop()
logging.info("Core loop terminated")
