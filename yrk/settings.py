#!/usr/bin/python
# York Robotics Kit Python API
#
# Version 0.1
#
# Settings and Constants File
#
# James Hilder, York Robotics Laboratory, Oct 2019

import logging,sys

VERSION_STRING="0.1.241019"

# Set the Python logging level for log files; recommend INFO for deployment and DEBUG for debugging
FILE_LOGGING_MODE = logging.DEBUG
CONSOLE_LOGGING_MODE = logging.INFO  #Cannot be a lower level than FILE_LOGGING_MODE


YRL039_ADDRESS          = 0x39                                                  #The I2C address for the ATMega on the YRL039 Power Supply Board

#Note about busses:  RPi4 models have extra i2c busses so numbering for switch starts at I2C_7 [I2C_3 on Pi3 and earlier]
YRL039_BUS              = 14                                                    #Bus YRL039 Power Supply board's ATMega is attached to
YRL040_BUS              = 13                                                    #Bus YRL040 [3.3V] I2C devoices are attached to
I2C_5V_BUS              = 11                                                    #The PWM driver and the Arduino are on the 5V I2C Bus [bus 4 on switch]

#Display settings
HAS_DISPLAY             = True
OLED_BUS                = 12                                                    #The bus which the OLED module is attached to [i2c_12 on YRL040 with Pi 4]
DISPLAY_ROTATED         = False                                                 #Set to true if the OLED module is rotated to flip image

#Fonts [Good resource for fonts to try: http://www.dafont.com/bitmap.php]
SMALL_MONO = '/home/pi/yrk/font/mono_small.ttf'
SMALL_FONT = '/home/pi/yrk/font/small_font.ttf'
SMALL_BOLD = '/home/pi/yrk/font/small_bold.ttf'
LARGE_FONT = '/home/pi/yrk/font/yrk_font.ttf'

#Audio settings
AUDIO_VOLUME = 100
AUDIO_ON_GPIO_PIN = 16                                                          #Audio On [ie unmute] is pin 16 on R.Pi GPIO

#Motor Settings
MOTOR_ADDRESSES         = [0x60,0x62,0x66,0x68]
ADC_NAMES               = ['ADC 0','ADC 1','ADC 2','ADC 3','ADC 4','ADC 5','POT','ADC 7']

#PWM settings
PWM_ADDRESS             = 0x40                                                  #PCA9685 - Address 0x40 Bus 4 [5V bus]
PWM_FREQUENCY           = 50                                                    #PWM (Analog Servo) frequency in hertz

#Define sensor type in this list.  Should have 8 entries.  Note 7th is for potentiometer
#ADC_MODELS              = ['2y0a21','2y0a41','voltage','raw','raw','raw','inv_pct','raw']
ADC_MODELS              = ['voltage','voltage','voltage','voltage','voltage','voltage','inv_pct','voltage']

SWITCH_GPIO_ADDRESS     = 0x20                                                  #I2C Address of U4 PCA9555 GPIO expender used for switches etc
USER_GPIO_ADDRESS       = 0x21                                                  #I2C Address of U13 PCA9555 GPIO expender used for user GPIO etc
RGB_LED_ADDRESS         = 0x45                                                  #I2C Address of U14 TCA6507 LED driver

#Initialisation state of user GPIO pins
USER_GPIO_MODE          = 0xFF                                                  #Init. mode for user GPIO pins [1=Input [with pull-up], 0=Output].
USER_GPIO_INVERTED      = 0x00                                                  #Init inversion state for user GPIO pins [1=Invert input]
USER_GPIO_OUTPUT_STATE  = 0x00                                                  #Init output state for user GPIO pins

SWITCH_INTERRUPT_PIN    = 5                                                     #GPIO pin connected to interrupt out of U4-PCA9555 GPIO [switches]
GPIO_INTERRUPT_PIN      = 6                                                     #GPIO pin connected to interrupt out of U13-PCA9555 GPIO [user]

RAMDISK_FILEPATH = "/mnt/ramdisk"
AUDIO_FILEPATH = "/home/pi/yrk/wav/"
IMAGE_FILEPATH = "/home/pi/yrk/images/"
SHOW_HOSTNAME		= True                                                      #Show hostname with IP address


#Unchecked
LED_DRIVER_BUS          = 9                                                     #The bus which the TCA6507 LED driver is attached to [i2c_9 on YRL028]
SWITCH_BUS              = 9                                                     #The bus which the YRL015 switch board is attached to [i2c_9 on YRL028]
ADC_BUS                 = 9                                                     #The bus which the ADS7830 ADC is connected to [i2c_9 on YRL028]
ADC_ADDRESS             = 0x48                                                  #The I2C address for the ADS7830
MOTORS_BUS              = 7
ARDUINO_BUS             = 10
ARDUINO_ADDRESS         = 0x57

BATTERY_CRITICAL_VOLTAGE = 9.2                                                  #Voltage at which critical voltage warning given

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


SENSOR_PORT_LIST        = [3,4,5]                                               #The I2C ports which will be scanned for sensors

POLL_PERIOD             = 1.0                                                   #Period (seconds) at which to update sensor readings

ENABLE_BATTERY_MONITOR    = True                                                #If enabled core.py will display visual+audible warnings when battery low
BATTERY_CRITICAL_SHUTDOWN = True                                                #If enabled, system will shutdown when Vbatt<BATTERY_SHUTDOWN_VOLTAGE
BATTERY_LOW_VOLTAGE = 10.2                                                      #Voltage at which low voltage warning given
BATTERY_SHUTDOWN_VOLTAGE = 8.9                                                  #Enforced shutdown voltage

# The following settings are only used when a YRL015 switch board is NOT connected
ENABLE_DEMO_MODE        = False
ENABLE_STATS_MODE       = False
ENABLE_AUTOSENSE_MODE   = True



# The following are I2C and register addresses and other parameters for the YRL013 and YLR019 sensor boards
GRIDEYE_ADDRESS         = 0x69
GAS_SENSOR_GPIO_ADDRESS = 0x41
GAS_SENSOR_ADC_ADDRESS  = 0x23
HUMIDITY_SENSOR_ADDRESS = 0x27
SENSOR_LED_ADDRESS_1    = 0x6A                                                  # I2C Address of the TLC59116 LED Driver on the YRL019 Sensor
SENSOR_LED_ADDRESS_2    = 0x69
TOF_SENSOR_ADDRESS      = 0x29                                                  # I2C Address of the VL53L0X Time-of-flight Sensor
IR_ADDRESS 	            = 0x60                                                  # I2C Address of the VCNL4040 IR Sensor
SENSOR_ADC_ADDRESS      = 0x4d                                                  # I2C Address of the MCP3221 ADC
EPROM_ADDRESS           = 0x50                                                  # I2C Address of the EEPROM
COLOUR_ADDRESS          = 0x10                                                  # I2C Address of the VEML6040 Colour Sensor
ALS_CONF_REG            = 0x00
PS_CONF1_REG            = 0x03
PS_CONF3_REG            = 0x04
PS_DATA_REG             = 0x08
ALS_DATA_REG            = 0x09
WHITE_DATA_REG          = 0x0A
RED_REGISTER 	        = 0x08
GREEN_REGISTER 	        = 0x09
BLUE_REGISTER 	        = 0x0A
WHITE_REGISTER 	        = 0x0B
WHITE_BYTE 	            = 0x40
COMMAND_BYTE 	        = 0x04
COLOUR_SENSOR_SENSITIVITY = 0x00                                                #Range 0 - 5 [40ms, 0.25168 lux/count  to 1280ms,
NOISE_FLOOR = 4000

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

def init():
    global use_built_in_dip_functions
    global initial_led_brightness
    global max_brightness
    global sensor_list
    global default_poll_period
    use_built_in_dip_functions = True                                             #Enable to use the built-in functions for the DIP switches
    sensor_list = []
    initial_led_brightness = 0.3                                                  #Multiplied by max_brightness
    max_brightness = 0.3                                                          #Brightness limit for LEDs.  Range 0-1.  1 is very bright and may cause power\heat issues if not used carefully...
    default_poll_period = POLL_PERIOD
