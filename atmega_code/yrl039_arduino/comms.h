// Arduino Code for YRL039 - Power Supply Board for York Robotics Kit
//
// Copyright (C) 2019 James Hilder, York Robotics Laboratory
//
// comms.h  : Header for i2c and serial communications
//
// Copyright (C) 2019 James Hilder, York Robotics Laboratory
// This is free software, with no warranty, and you are welcome to redistribute it
// under certain conditions.  See GNU GPL v3.0.

#ifndef COMMS_H
#define COMMS_H

#include "yrl039.h"

void write_int16_t(int16_t value);
void write_int32_t(int32_t value);
void i2c_request_event(void);
void i2c_receive_event(int howMany);
void check_serial_for_data(void);
void process_serial_data(void);
void process_i2c_data(void);
boolean parse_integer_from_serial(int low_limit, int high_limit);

#endif
