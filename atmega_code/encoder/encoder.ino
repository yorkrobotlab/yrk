/*
 * Wheel Encoder example Arduino Code for the York Robotics Kit
 *
 * Copyright (C) 2020 James Hilder, York Robotics Laboratory,
 * University of York
 *
 * MIT License
 *
 * The York Robotics Kit main PCB [YRL040] includes an ATMega328P
 * microcontroller.  Its is effectively the equivalent of an
 * Arduino Nano, although with a different pin configuration.
 * It is intended to provide additional functionality, such as
 * counting wheel encoders and allowing the use of digital servo
 * motors.
 *
 * The microcontroller is connected to the I2C switch on the YRK,
 * on channel 4 [maps to /dev/i2c_10 on Pi 4].  The bus and
 * microcontroller operate at 5V.  The microcontroller has a 16 MHz
 * clock speed.
 *
 * This code is a simple example of its use for 2-motor wheel encoding.
 * It is designed for 5V tolerant wheel encoders such as the Pololu
 * magnetic and optical encoders for micro-metal gear motors.  If
 * using optical encoders ensure they are optimised for 5V operation.
 *
 * The Arduino has (only) two interrupt inputs [pins D2 and D3]. This
 * code assumes the (A) outputs of the encoders are connected to D2 and
 * D3, with the corresponding (B) outputs connected to A2 and A3.
 *
 * Power for the encoders can be taken from one of the PWM banks [ensure
 * that the jumper which connects the internal supply is attached].
 *
 */

#include <Wire.h>

const int i2c_slave_address = 0x57;
const int red_led = 12;
const int green_led = 13;

const int activity_led = A6;
const int motor_1_int = 2;
const int motor_1_dir = 3; 
const int motor_2_int = A2;
const int motor_2_dir = A3;
unsigned long startMillis;
volatile long enc_1_count = 0;
volatile long enc_2_count = 0;
volatile long enc_1_cumulative = 0;
volatile long enc_2_cumulative = 0;


void i2c_receive_event(int howMany) {
  Serial.println(howMany);
  while (1 < Wire.available()) { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
    Serial.print(c);
  }
  Serial.println(" - i2c data");
}

void i2c_request_event() {
  Serial.println("Request event");
}

void enc_1_ISR() {
  boolean a = digitalRead(motor_1_int);
  boolean b = digitalRead(motor_1_dir);
  if(a==b) enc_1_count--;
  else enc_1_count++;
  enc_1_cumulative++;
}

void enc_2_ISR() {
  boolean a = digitalRead(motor_2_int);
  boolean b = digitalRead(motor_2_dir);
  if(a==b) enc_2_count++;
  else enc_2_count--;
  enc_2_cumulative++;
}

void reset_encoders() {
  enc_1_count=0;
  enc_2_count=0;
  enc_1_cumulative=0;
  enc_2_cumulative=0;
}


void setup() {
  // Setup serial console
  Serial.begin(57600);
  Serial.println(F("YRK-Arduino Wheel Encoder Code"));

  // Setup LEDs
  pinMode(red_led, OUTPUT);
  pinMode(green_led, OUTPUT);

  // Setup encoder pins and interrupts
  pinMode(motor_1_int, INPUT);
  pinMode(motor_1_dir, INPUT);
  pinMode(motor_2_int, INPUT);
  pinMode(motor_2_dir, INPUT);
  attachInterrupt(digitalPinToInterrupt(motor_1_int),enc_1_ISR,CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor_2_int),enc_2_ISR,CHANGE);

  // Setup i2c connection
  Wire.begin(i2c_slave_address);
  Wire.onRequest(i2c_request_event);
  Wire.onReceive(i2c_receive_event);
  startMillis = millis();
}

void loop() {
  unsigned long currentMillis = millis() - startMillis;
  if(currentMillis > 1000) startMillis = millis();
  //Heartbeat blink with red led, 100ms every second
  if(currentMillis < 100) {if (!digitalRead(red_led)) digitalWrite(red_led,1);}
  else {if (digitalRead(red_led)) digitalWrite(red_led,0);}
}
