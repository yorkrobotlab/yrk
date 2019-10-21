#!/usr/bin/python
#
# York Robotics Kit Python API - Version 0.1
# Functions for the ADS7830 8-bit Analog:Digital Converter
# Datasheet: http://www.ti.com/lit/ds/symlink/ads7830.pdf
# James Hilder, York Robotics Laboratory, Oct 2019

"""
.. module:: adc
   :synopsis: Functions for reading from the ADS7830 8-bit Analog:Digital Converter

.. moduleauthor:: James Hilder <github.com/jah128>

This module provides functions for reading from the ADS7830 8-bit Analog:Digital
Converter.  8-bit raw values can be read using the ``read_adc`` function and
these values can be converted to distance measurements for various Sharp IR
distance sensors based on look-up table values using the ``get_model_value``
function.

The ADC has 8 input channels.  Channels 0-5 are broken out on the PCB to 3-pin
JST-PH headers.  +5V[aux] is the uppermost pin, GND in middle and V_sense at the
bottom.  The ADC is set to range from 0V [0] to 2.50V [255], approx 0.01V/scale.
Using the ``2y0a21`` or ``2y0a41`` sensor models for the


GP2Y0A21YK0F distance sensor [10 - 80cm]
https://global.sharp/products/device/lineup/data/pdf/datasheet/gp2y0a21yk_e.pdf

GP2Y0A41SK0F distance sensor [4 - 30cm]
https://global.sharp/products/device/lineup/data/pdf/datasheet/gp2y0a41sk_e.pdf


ADC channel six is connected to the potentiometer at the top-left of the board.
This will produce a raw output of 255-0 as it is turned clockwise. Using the
``inv_pct`` sensor model will convert this from 0 - 100.0 %.  ADC channel 7 is
connected to the left auxiliary ADC pin, GND to the right.  These pins are 2mm
pitch [jumpers leads are available to convert to 0.1" pitch].

.. _ADS7830 Datasheet:
   http://www.ti.com/lit/ds/symlink/ads7830.pdf

"""


import yrk.settings as s
import logging, os, smbus2

DISTANCE_2Y0A21 = [80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,80.00,79.97,79.88,79.51,78.48,76.74,74.71,72.68,70.72,68.84,67.06,65.35,63.72,62.16,60.67,59.24,57.87,56.56,55.30,54.09,52.93,51.81,50.73,49.70,48.70,47.73,46.81,45.91,45.04,44.21,43.40,42.62,41.86,41.13,40.42,39.73,39.06,38.42,37.79,37.18,36.59,36.02,35.46,34.92,34.39,33.88,33.38,32.90,32.43,31.97,31.52,31.08,30.66,30.24,29.84,29.44,29.06,28.68,28.32,27.96,27.61,27.27,26.94,26.61,26.29,25.98,25.67,25.38,25.09,24.80,24.52,24.25,23.98,23.72,23.46,23.21,22.96,22.72,22.48,22.25,22.02,21.80,21.58,21.37,21.16,20.95,20.75,20.55,20.35,20.16,19.97,19.78,19.60,19.42,19.25,19.08,18.91,18.74,18.57,18.41,18.26,18.10,17.95,17.79,17.65,17.50,17.36,17.22,17.08,16.94,16.80,16.67,16.54,16.41,16.29,16.16,16.04,15.92,15.80,15.68,15.56,15.45,15.34,15.23,15.12,15.01,14.90,14.80,14.69,14.59,14.49,14.39,14.29,14.20,14.10,14.01,13.92,13.82,13.73,13.65,13.56,13.47,13.39,13.30,13.22,13.13,13.05,12.97,12.89,12.81,12.74,12.66,12.59,12.51,12.44,12.36,12.29,12.22,12.15,12.08,12.01,11.94,11.88,11.81,11.75,11.68,11.62,11.55,11.49,11.43,11.37,11.31,11.25,11.19,11.13,11.07,11.01,10.96,10.90,10.85,10.79,10.74,10.68,10.63,10.58,10.52,10.47,10.42,10.37,10.32,10.27,10.22,10.17,10.13,10.08,10.03,9.99,9.94,9.89,9.85,9.80,9.76,9.71,9.67,9.63,9.59,9.54,9.50,9.46,9.42,9.38,9.34,9.30,9.26,9.22,9.18,9.15]

#This is potentially wrong, just been quickly adapted from 2Y0A21 model, please check
DISTANCE_2Y0A41 = [40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 39.985, 39.94, 39.755, 39.24, 38.37, 37.355, 36.34, 35.36, 34.42, 33.53, 32.675, 31.86, 31.08, 30.335, 29.62, 28.935, 28.28, 27.65, 27.045, 26.465, 25.905, 25.365, 24.85, 24.35, 23.865, 23.405, 22.955, 22.52, 22.105, 21.7, 21.31, 20.93, 20.565, 20.21, 19.865, 19.53, 19.21, 18.895, 18.59, 18.295, 18.01, 17.73, 17.46, 17.195, 16.94, 16.69, 16.45, 16.215, 15.985, 15.76, 15.54, 15.33, 15.12, 14.92, 14.72, 14.53, 14.34, 14.16, 13.98, 13.805, 13.635, 13.47, 13.305, 13.145, 12.99, 12.835, 12.69, 12.545, 12.4, 12.26, 12.125, 11.99, 11.86, 11.73, 11.605, 11.48, 11.36, 11.24, 11.125, 11.01, 10.9, 10.79, 10.685, 10.58, 10.475, 10.375, 10.275, 10.175, 10.08, 9.985, 9.89, 9.8, 9.71, 9.625, 9.54, 9.455, 9.37, 9.285, 9.205, 9.13, 9.05, 8.975, 8.895, 8.825, 8.75, 8.68, 8.61, 8.54, 8.47, 8.4, 8.335, 8.27, 8.205, 8.145, 8.08, 8.02, 7.96, 7.9, 7.84, 7.78, 7.725, 7.67, 7.615, 7.56, 7.505, 7.45, 7.4, 7.345, 7.295, 7.245, 7.195, 7.145, 7.1, 7.05, 7.005, 6.96, 6.91, 6.865, 6.825, 6.78, 6.735, 6.695, 6.65, 6.61, 6.565, 6.525, 6.485, 6.445, 6.405, 6.37, 6.33, 6.295, 6.255, 6.22, 6.18, 6.145, 6.11, 6.075, 6.04, 6.005, 5.97, 5.94, 5.905, 5.875, 5.84, 5.81, 5.775, 5.745, 5.715, 5.685, 5.655, 5.625, 5.595, 5.565, 5.535, 5.505, 5.48, 5.45, 5.425, 5.395, 5.37, 5.34, 5.315, 5.29, 5.26, 5.235, 5.21, 5.185, 5.16, 5.135, 5.11, 5.085, 5.065, 5.04, 5.015, 4.995, 4.97, 4.945, 4.925, 4.9, 4.88, 4.855, 4.835, 4.815, 4.795, 4.77, 4.75, 4.73, 4.71, 4.69, 4.67, 4.65, 4.63, 4.61, 4.59, 4.575]

adc_bus = smbus2.SMBus(s.YRL040_BUS)

def read_adc(channel):
    """A function that reads the raw 8-bit value [8-bit] from given channel of ADC

    Args:
        channel (int): The ADC channel to read from [0-7]. 6 is potentiometer on YRL040.

    Returns:
        int: ADC value [range 0-255]
    """

    channel_values = [0x88,0xC8,0x98,0xD8,0xA8,0xE8,0xB8,0xF8]
    adc_bus.write_byte(s.ADC_ADDRESS,channel_values[channel])
    return adc_bus.read_byte(s.ADC_ADDRESS)

def get_model_value(raw_value,sensor_model):
    """A function that converts a raw ADC value to distance for Sharp IR sensor

    Args:
        raw_value (int):  The 8-bit raw ADC value [eg from read_adc()]
        sensor_model (str): The sensor model to use.  '2y0a21' or '2y0a41' are
        valid sensors; other options are 'voltage','raw' and 'inv_pct'

    Returns:
        float: Converted distance in mm based on look-up tables
    """

    if(sensor_model == '2y0a21'): return DISTANCE_2Y0A21[raw_value]
    if(sensor_model == '2y0a41'): return DISTANCE_2Y0A41[raw_value]
    if(sensor_model == 'voltage'): return round(raw_value * 0.00977,2)
    if(sensor_model == 'raw'): return raw_value
    if(sensor_model == 'inv_pct'): return round((255-raw_value) * 0.392157,1)
    return 0

#Test code
if __name__ == "__main__":
 s.init()
 s.setup_logger("adc")
 logging.info("York Robotics Kit: A\D Converter test code")
 for channel in range (8):
     raw_value = read_adc(channel)
     logging.info(f"Channel {channel} [{s.ADC_NAMES[channel]}]:{raw_value} [{s.ADC_MODELS[channel]}:{get_model_value(raw_value,s.ADC_MODELS[channel])}]")
 os._exit(1)
