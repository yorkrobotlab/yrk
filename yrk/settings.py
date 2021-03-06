#!/usr/bin/python
# York Robotics Kit Python API
#
# Version 0.21
#
# Settings and Constants File
#
# James Hilder, York Robotics Laboratory, Jan 2020



# Settings with docstrings are ones a user is likely to want\need to change

import logging,sys

VERSION_STRING="0.21.200120"

FILE_LOGGING_MODE = logging.DEBUG
"""Set the Python logging level for saved log files.

The recommend setting is logging.INFO for deployment and logging.DEBUG for debugging

"""

CONSOLE_LOGGING_MODE = logging.INFO  #Cannot be a lower level than FILE_LOGGING_MODE
"""Set the Python logging level for console output.

The recommend setting is logging.INFO for deployment and logging.DEBUG for debugging.
Cannot be at a lower level than FILE_LOGGING_MODE.

"""


#BUS Settings
#Note about busses:  RPi4 models have extra i2c busses so numbering for switch starts at I2C_7 [I2C_3 on Pi3 and earlier]

RASPBERRY_PI_MODEL = 4
"""Sets the Raspberry Pi version being used

   Set to 4 if using Raspberry Pi 4 (recommended)
   If set to different value, default BUS values are overwritten with values compatible with the earlier Pis.


"""

I2C_5V_BUS = 11
'''The PWM driver and the Arduino are on the 5V I2C Bus [bus 4 on switch]'''

OLED_BUS = 12
'''The /dev/i2c_XX bus which the OLED module is attached to'''

YRL040_BUS = 13
'''The /dev/i2c_XX bus the YRL040 [3.3V] I2C devoices are attached to'''

YRL039_BUS = 14
'''The /dev/i2c_XX bus the YRL039 Power Supply board's ATMega is attached to'''

#Use Raspberry Pi 3 bus settings
if (RASPBERRY_PI_MODEL != 4):
  I2C_5V_BUS = 7
  OLED_BUS = 8
  YRL040_BUS = 9
  YRL039_BUS = 10

#Display settings

HAS_DISPLAY = True
'''Set to True is OLED module is being used'''

DISPLAY_ROTATED = False
'''Set to True if the OLED module is rotated to flip image'''

#Audio Settings
AUDIO_VOLUME = 100
'''Volume to set Alsa mixer to on setup of audio (range 0-100, but 80+ recommended)'''

#PWM settings
PWM_FREQUENCY = 50
'''PWM (Analog Servo) target frequency in hertz'''

#ADC setting
ADC_MODELS = ['voltage','voltage','voltage','voltage','voltage','voltage','inv_pct','voltage']
"""List of model types for the 8 ADC inputs

   Sensor type in this list.  Should have 8 entries.  Note 7th is for potentiometer.
   Valid models are voltage, pct, inv_pct, raw, 2y0a21 and 2Y0a41

   Eg: ADC_MODELS              = ['2y0a21','2y0a41','voltage','raw','raw','raw','inv_pct','raw']


"""

ADC_NAMES = ['ADC 0','ADC 1','ADC 2','ADC 3','ADC 4','ADC 5','POT','ADC 7']

#GPIO settings
USER_GPIO_MODE = 0xFF
'''Initialisation mode for the 8 user GPIO pins [1=Input [with pull-up], 0=Output].'''

USER_GPIO_INVERTED = 0x00
'''Initialisation inversion state for the 8 user GPIO pins [1=Invert input]'''

USER_GPIO_OUTPUT_STATE = 0x00
'''Initialisation output state for the 8 user GPIO pins'''

#YRK-core settings
BATTERY_CHECK_PERIOD = 2.0
'''Period (s) between battery state checks'''

TEMPERATURE_CHECK_PERIOD = 2.0
'''Period (s) between temperature checks'''

ENABLE_BATTERY_MONITOR = True
'''If enabled yrk-core.py will display visual+audible warnings when battery low'''


ENABLE_TEMPERATURE_MONITOR = True
'''If enabled yrk-core.py will display visual+audible warnings when cpu\pcb temperature high'''

BATTERY_CRITICAL_SHUTDOWN = True
'''If enabled, system will force shutdown when Vbatt<BATTERY_SHUTDOWN_VOLTAGE'''

TEMPERATURE_CRITICAL_SHUTDOWN = True
'''If enabled, system will force shutdown when CPU Temp<CPU_SHUTDOWN_TEMP or PCB Temp<PCB_SHUTDOWN_TEMP'''

BATTERY_CELLS = 2
"""Number of Li-Ion or Li-Po cells in battery (2,3 or 4)

   Sets the BATTERY_LOW_VOLTAGE, BATTERY_CRITICAL_VOLTAGE and BATTERY_SHUTDOWN_VOLTAGE parameters
   based on the number of cells described.  The values for 2 cell batteries are a little more generous
   to get decent usable life of of battery, although voltage drop may be too great in high current
   drain use cases.  If using other battery technology (eg Ni-Mh) set values manually within settings.py


 """

CPU_WARNING_TEMP = 65
'''CPU warning temperature, recommend ~65C for Pi 4 and ~60C for Pi 3'''

CPU_CRITICAL_TEMP = 75
'''CPU critical temperature, recommend ~75C for Pi 4 and ~65C for Pi 3'''

CPU_SHUTDOWN_TEMP = 82
'''CPU shutdown temperature, recommend ~82C for Pi 4 and ~72C for Pi 3'''

PCB_WARNING_TEMP = 40
'''PCB warning temperature, recommend ~40C'''

PCB_CRITICAL_TEMP = 45
'''PCB critical temperature, recommend ~45C'''

PCB_SHUTDOWN_TEMP = 50
'''PCB forced shutdown temperature, recommend ~50C'''

USE_DIP_FUNCTIONS = True
'''If True, yrk-core uses DIP 2 for ROS, DIP 3 for DASH server and DIP 4 for DEMO'''

USE_DIP_LEDS = True
'''If true, yrk-core will set DIP LEDs based on program state'''

YRL039_ADDRESS = 0x39
'''The I2C address for the ATMega on the YRL039 Power Supply Board (could be reprogrammed to different address if needed).'''


#Set battery levels [if necessary set manual overrides here]
if BATTERY_CELLS == 2:
    BATTERY_LOW_VOLTAGE = 6.2                                                   #Voltage at which low voltage warning given
    BATTERY_CRITICAL_VOLTAGE = 5.8                                              #Voltage at which critical voltage warning given
    BATTERY_SHUTDOWN_VOLTAGE = 5.6                                              #Enforced shutdown voltage
elif BATTERY_CELLS == 4:
    BATTERY_LOW_VOLTAGE = 13.6                                                  #Voltage at which low voltage warning given
    BATTERY_CRITICAL_VOLTAGE = 12.3                                             #Voltage at which critical voltage warning given
    BATTERY_SHUTDOWN_VOLTAGE = 11.8                                             #Enforced shutdown voltage
else:
    BATTERY_LOW_VOLTAGE = 10.2                                                  #Voltage at which low voltage warning given
    BATTERY_CRITICAL_VOLTAGE = 9.2                                              #Voltage at which critical voltage warning given
    BATTERY_SHUTDOWN_VOLTAGE = 8.9                                              #Enforced shutdown voltage


#Address Settings [user is unlikely to want to change these]
MOTOR_ADDRESSES         = [0x60,0x62,0x66,0x68]                                 #List of I2C addresses for the DRV8830 motor driver ICs
PWM_ADDRESS             = 0x40                                                  #PCA9685 - Address 0x40 Bus 4 [5V bus]
AUDIO_ON_GPIO_PIN = 16                                                          #Audio On [ie unmute] is pin 16 on R.Pi GPIO
SWITCH_GPIO_ADDRESS     = 0x20                                                  #I2C Address of U4 PCA9555 GPIO expender used for switches etc
USER_GPIO_ADDRESS       = 0x21                                                  #I2C Address of U13 PCA9555 GPIO expender used for user GPIO etc
RGB_LED_ADDRESS         = 0x45                                                  #I2C Address of U14 TCA6507 LED driver
ADC_ADDRESS             = 0x48                                                  #The I2C address for the ADS7830
SWITCH_INTERRUPT_PIN    = 5                                                     #GPIO pin connected to interrupt out of U4-PCA9555 GPIO [switches]
GPIO_INTERRUPT_PIN      = 6                                                     #GPIO pin connected to interrupt out of U13-PCA9555 GPIO [user]
DEBUG_LED_PIN = 17                                                              #GPIO pin connected to CATHODE of RED PI_DATA LED on YRL039
RAMDISK_FILEPATH = "/mnt/ramdisk"
AUDIO_FILEPATH = "/home/pi/yrk/wav/"
IMAGE_FILEPATH = "/home/pi/yrk/images/"
SHOW_HOSTNAME		= True                                                      #Show hostname with IP address

#ROS settings
ROS_POWER_PUBLISHER_RATE = 2                                                    #Refresh rate [Hz] of power status messages on ROS
ROS_ADC_PUBLISHER_RATE = 5                                                      #Refresh rate [Hz] of ADC messages on ROS
ROS_SWITCH_PUBLISHER_RATE = 5							#Refresh rate [Hz] of button messages
ROS_PID_FILE = RAMDISK_FILEPATH+"/roslaunch.pid"
ROS_LAUNCH_COMMAND = ["roslaunch","--pid="+ROS_PID_FILE,"yrk_ros","yrk.launch"] #Command to be executed [using subprocess.Popen()] when ROS is launched by YRK-Core
DASH_LAUNCH_COMMAND = ["python","--pid="+ROS_PID_FILE,"yrk_ros","yrk.launch"] #Command to be executed [using subprocess.Popen()] when ROS is launched by YRK-Core


SWITCH_STATUS_FILENAME = RAMDISK_FILEPATH+"/switch.dat"                         #The most recent switch state [when read following interrupt] is saved to this file for processing by ROS publisher
GPIO_STATUS_FILENAME = RAMDISK_FILEPATH+"/gpio.dat"                             #The most recent GPIO state [when read following interrupt] is saved to this file for processing by ROS publisher


#I2C Lock settings
I2C_LOCK_FILENAME = RAMDISK_FILEPATH+"/i2c_lock"
I2C_TIMEOUT = 0.05

#Selftest [examples/selftest.py] override settings
IGNORE_PWM_FAILURE = True                                                       #If true, fault in PWM I2C will be ignored



#Fonts [Good resource for fonts to try: http://www.dafont.com/bitmap.php]
SMALL_MONO = '/home/pi/yrk/font/mono_small.ttf'
SMALL_FONT = '/home/pi/yrk/font/small_font.ttf'
SMALL_BOLD = '/home/pi/yrk/font/small_bold.ttf'
LARGE_FONT = '/home/pi/yrk/font/yrk_font.ttf'


BUS_ERROR       = False                                                         #Will be set to true if there is a problem initialising i2c busses
ENABLE_PROGRAMS = True
DEFAULT_PROGRAM = None

PROGRAM_FILEPATH = "user_programs"
ENABLE_ROBOT_TAB = True                                                         #Enable if a robot sensor model and motor model is defined to enable components in DASH server
# Define the robot sensors for DASH server, list of list of 4 elements: sensor ID, sensor type, theta, +-view angle
ROBOT_SENSOR_MODEL = [
    ['analog-1','2y0a21',48,14],
    ['analog-2','2y0a21',18,14],
    ['analog-3','2y0a21',312,14],
    ['analog-4','2y0a21',342,14]
]

# The locations for the sensor and system file logs
system_datafilename="/ramdisk/system.csv"
sensor_datafilename="/ramdisk/sensor"
program_request_filename="/ramdisk/progrequest"
program_info_filename="/ramdisk/proginfo"
program_state_filename="/ramdisk/progstate"
camera_request_filename="/ramdisk/camerarequest"

import logging

def setup_logger(filename):
    rootLogger = logging.getLogger()
    #rootLogger.setLevel(LOGGING_MODE)
    rootLogger.setLevel(FILE_LOGGING_MODE)

    logFormatter = logging.Formatter("%(asctime)s [%(threadName)-12.12s] [%(levelname)-5.5s]  %(message)s")
    consoleFormatter = logging.Formatter("[%(levelname)-1.1s] %(message)s")
    fileHandler = logging.FileHandler("{0}/{1}.log".format(RAMDISK_FILEPATH, filename))
    fileHandler.setFormatter(logFormatter)
    rootLogger.addHandler(fileHandler)
    consoleHandler = logging.StreamHandler()
    consoleHandler.setFormatter(consoleFormatter)
    consoleHandler.setLevel(CONSOLE_LOGGING_MODE)
    rootLogger.addHandler(consoleHandler)
