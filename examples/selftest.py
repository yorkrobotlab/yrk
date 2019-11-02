#!/usr/bin/python
# York Robotics Kit Python API
# Version 0.1
# Self-test Python Script
# James Hilder, York Robotics Laboratory, Oct 2019

"""
.. module:: selftest
   :synopsis: Self-test routine.  Run at boot time in standard setup.

.. moduleauthor:: James Hilder <github.com/jah128>

"""

import logging, sys
import yrk.settings as settings

settings.init()
log_filename = "selftest"
settings.setup_logger(log_filename)


import RPi.GPIO as GPIO, subprocess, time, os
import yrk.display as display, yrk.led as led, yrk.audio as audio
import yrk.gpio as gpio, yrk.switch as switch, yrk.utils as utils
import yrk.motors as motors, yrk.pwm as pwm, yrk.adc as adc
import yrk.power as power
from subprocess import call

major_fail = False
dip_switch_state = 0
logging.info("York Robotics Kit Self Test - Ver  %s" % (settings.VERSION_STRING))
logging.info("HOSTNAME      : %s" % (utils.get_hostname()))
logging.info("IP ADDRESS    : %s" % (utils.get_ip()))
if settings.BUS_ERROR:
    logging.info("Self-test failed [i2c bus fault].  I2C switch may be faulty or wrongly configured.")
    os._exit(3)

GPIO.setmode(GPIO.BCM)

#Test (and initialise) display
disp_status = "DISABLED"
if(settings.HAS_DISPLAY):
    try:
        display.init_display()
        disp_status = "PASSED"
    except (IOError):
        disp_status = "FAILED"
        settings.HAS_DISPLAY=False
logging.info("DISPLAY       : %s" % (disp_status))

try:
    gpio.setup_user_gpio()
    aux_gpio_status = "PASSED"
except IOError:
    aux_gpio_status = "FAILED"
    major_fail = True
logging.info("AUX GPIO      : %s" % (aux_gpio_status))

try:
    switch.setup_switch_gpio()
    dip_switch_state = switch.read_dip_switch() % 2
    switch.set_dip_leds(0x1)
    time.sleep(0.1)
    switch.set_dip_leds(0x2)
    time.sleep(0.1)
    switch.set_dip_leds(0x4)
    time.sleep(0.1)
    switch.set_dip_leds(0x8)
    time.sleep(0.1)
    switch.set_dip_leds(0xF)
    time.sleep(0.1)
    switch.set_dip_leds(0x0)
    switch_gpio_status = "PASSED"
except IOError:
    switch_gpio_status = "FAILED"
    major_fail = True

logging.info("SWITCH GPIO   : %s" % (switch_gpio_status))

try:
    led.timed_animation(3,1)
    led_status = "PASSED"
except IOError:
    led_status = "FAILED"
    major_fail = True

logging.info("LED DRIVER    : %s" % (led_status))

try:
    pwm.set_pwm_frequency(settings.PWM_FREQUENCY)
    pwm_status = "PASSED"
except IOError:
    pwm_status = "FAILED"
    major_fail = True

logging.info("PWM DRIVER    : %s" % (pwm_status))

try:
    adc.read_adc(0)
    adc_status = "PASSED"
except IOError:
    adc_status = "FAILED"
    major_fail = True

logging.info("A\D CONVERTER : %s" % (adc_status))

motors.silent=True
for motor in range(4):
    try:
        motors.set_motor_speed(motor,0)
        motor_status = "PASSED"
    except IOError:
        motor_status = "FAILED"
        major_fail = True
    logging.info("MOTOR %d       : %s" % (motor,motor_status))


try:
    power.read_all_values()
    if(power.get_battery_voltage() < settings.BATTERY_CRITICAL_VOLTAGE):
        yrl039_status = "FAILED: V_IN LOW [%2.2f]" % (power.get_battery_voltage())
        major_fail = True
    elif (power.get_pi_voltage()<4.8):
        yrl039_status = "FAILED: V_PI LOW [%2.2f]" % (power.get_pi_voltage())
        major_fail = True
    elif (power.get_aux_voltage()<4.8):
        yrl039_status = "FAILED: V_AUX LOW [%2.2f]" % (power.get_aux_voltage())
        major_fail = True
    else: yrl039_status = "PASSED    [V_IN=%2.2f  V5_PI=%2.2f  V5_AUX=%2.2f]" % (power.get_battery_voltage(),power.get_pi_voltage(),power.get_aux_voltage())
except IOError:
    yrl039_status = "FAILED"
    major_fail = True
logging.info("POWER BOARD   : %s" % (yrl039_status))



audio.setup_audio()
audio.play_audio_file(settings.AUDIO_FILEPATH + "york-robotics-kit.wav")

display.display_image_file(settings.IMAGE_FILEPATH + "yrl-white.pbm")
time.sleep(0.8)
if (settings.SHOW_HOSTNAME): display.two_line_text_wrapped(utils.get_hostname(),utils.get_ip())
else: display.two_line_text_wrapped("IP Address:",utils.get_ip())
time.sleep(0.5)
if major_fail:
    logging.info("Self-test failed.  Consult {0}/{1}.log for more information.".format(settings.RAMDISK_FILEPATH, log_filename) )
    os._exit(2)
logging.info("Self-test passed.")
os._exit(1-dip_switch_state)
