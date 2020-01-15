#!/bin/bash
arduino --upload --port /dev/ttyUSB0 --board arduino:avr:pro:cpu=16MHzatmega328 --verbose-upload encoder.ino 
