// Arduino Code for YRL039 - Power Supply Board for York Robotics Kit
//
// Copyright (C) 2019 James Hilder, York Robotics Laboratory
//
// comms.cpp  : Source for i2c and serial communications
//
// Copyright (C) 2019 James Hilder, York Robotics Laboratory
// This is free software, with no warranty, and you are welcome to redistribute it
// under certain conditions.  See GNU GPL v3.0.

#include "yrl039.h"

//Serial variables
const byte numChars = 32;  //Size of serial string buffer (-1)
char received[numChars];  //Stores the received serial data
char i2c_received[numChars];
char i2c_received_length = 0;
char i2c_register;
boolean new_i2c_data = false;
boolean non_displayed_i2c_data = false;

boolean new_data = false;
int parsed_value = 0;

void i2c_receive_event(int howMany) {
  //static byte ndx = 0;
  byte ndx = 0;
  i2c_register = Wire.read();
  if (howMany > 1) {
    while (Wire.available()) { // loop through all but the last
      i2c_received[ndx] = Wire.read();
      ndx++;
    }
  }
  i2c_received_length = ndx;
  new_i2c_data = true;
  non_displayed_i2c_data = true;
  switch (i2c_register) {

    case 7:
      if (i2c_received_length == 2) {
        //        int speed1 = ((int)i2c_received[0]+128) << 1;
        //        int speed2 = ((int)i2c_received[1]+128) << 1;
        //        if(speed1 > 255) speed1 = speed1 - 512;
        //        if(speed2 > 255) speed2 = speed2 - 512;
        //        set_motor_speeds(speed1,speed2);
        //        Serial.print("M1:");
        //        Serial.print(speed1);
        //        Serial.print(" M2:");
        //        Serial.println(speed2);
        //      }
        break;
      };
  }
}

void process_i2c_data() {
  //  if (non_displayed_i2c_data) {
  //
  //    }
  //    Serial.print("I2C Receive - ");
  //    Serial.print(i2c_register, HEX);
  //    if (i2c_received_length == 0) {
  //      Serial.println(" - No data");
  //    } else {
  //      Serial.print(" - ");
  //      Serial.print(i2c_received_length, HEX);
  //      Serial.print(" [0x");
  //      for (byte i = 0; i < i2c_received_length; i++) {
  //        Serial.print(i2c_received[i], HEX);
  //        if (i + 1 < i2c_received_length) {
  //          Serial.print(",0x");
  //        }
  //      }
  //      Serial.println("]");
  //    }
  //  }
  non_displayed_i2c_data = false;
}

void write_int16_t(int16_t value) {
  byte m_array[2];
  m_array[0] = (value >> 8) & 0xFF;
  m_array[1] = value & 0xFF;
  Wire.write(m_array, 2);
}

void write_int32_t(int32_t value) {
  byte m_array[4];
  m_array[0] = (value >> 24) & 0xFF;
  m_array[1] = (value >> 16) & 0xFF;
  m_array[2] = (value >> 8) & 0xFF;
  m_array[3] = value & 0xFF;
  Wire.write(m_array, 4);
}

void i2c_request_event() {
  if (!new_i2c_data || i2c_received_length > 0) {
    Serial.println("Invalid i2c request");
  } else {
    switch (i2c_register) {
      case 0: Wire.write("yrl039"); break;
      case 1: write_int16_t(stored_adc_value[0]); break;      //Multiple by 0.0172 for V
      case 2: write_int16_t(stored_adc_value[1]); break;        //Multiple by 0.00539 for V
      case 3: write_int16_t(stored_adc_value[2]); break;       //Multiple by 0.00539 for V
      case 4: write_int16_t(stored_adc_value[3]); break;  //Multiple by 0.0043 for I
      case 5: write_int16_t(stored_adc_value[4]); break; //Multiple by 0.0043 for I
      case 6: write_int16_t(stored_adc_value[5]); break;    //Subtract 395 then multiply by 0.171875 for C 
      case 8: //Compact all readings into 8-byte data block
              unsigned long stat_block_0 = ((unsigned long) stored_adc_value[0] << 22) + ((unsigned long) stored_adc_value[1] << 12) + (stored_adc_value[2] << 2);
              unsigned long stat_block_1 = ((unsigned long) stored_adc_value[3] << 22) + ((unsigned long) stored_adc_value[4] << 12) + (stored_adc_value[5] << 2);
              byte w_array[8];
              w_array[0] = (stat_block_0 >> 24) & 0xFF;
              w_array[1] = (stat_block_0 >> 16) & 0xFF;
              w_array[2] = (stat_block_0 >> 8) & 0xFF;
              w_array[3] = (stat_block_0) & 0xFF;
              w_array[4] = (stat_block_1 >> 24) & 0xFF;
              w_array[5] = (stat_block_1 >> 16) & 0xFF;
              w_array[6] = (stat_block_1 >> 8) & 0xFF;
              w_array[7] = (stat_block_1) & 0xFF; 
              Wire.write(w_array,8);
              break;             
    }
    new_i2c_data = false;
  }
}

void check_serial_for_data() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;
  while (Serial.available() > 0 && new_data == false) {
    rc = Serial.read();
    if (rc != endMarker) {
      received[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    }
    else {
      received[ndx] = '\0'; // terminate the string
      ndx = 0;
      new_data = true;
      Serial.print("Data received:");
      Serial.println(received);
    }
  }
}

boolean parse_integer_from_serial(int low_limit, int high_limit) {
  char * submessage = received + 1;
  int value;
  value = atoi (submessage); //Read integer from string [note this will like return 0 for invalid strings]
  if (value <= high_limit && value >= low_limit) {
    parsed_value = value;
    return true;
  }
  return false;
}

void process_serial_data() {
  if (new_data == true) {
    Serial.println("Received");
    boolean ok = true;
    boolean show_value = true;
    switch (received[0])
    {
      case 't': if (parse_integer_from_serial(30, 10000)) tone(buzzer, parsed_value); else ok = false; break;
      case 'y': noTone(buzzer); show_value = false; break;
      default: ok = false;
    }

    /**
       Serial commands:
       t [30 to 10000]: Play a tone at given frequency
       y [-----------]: Stop playing tone
    */

    if (debug) {
      if (ok) {
        Serial.print(F("Message: "));
        Serial.print(received[0]);
        if (show_value)Serial.print(parsed_value);
        Serial.println();
      } else {
        Serial.print(F("ERROR: Invalid message "));
        Serial.println(received);
      }
    }
    new_data = false;
  }
}
