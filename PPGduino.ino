/*******************************************************************************
* Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************/

//#define USE_16BIT

#include <Arduino.h>
#include "algorithm.h"
#include "max30102.h"

#define PIN_INT 10

#if defined(USE_16BIT)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated.  Samples become 16-bit data.
uint16_t aun_ir_buffer[100]; //infrared LED sensor data
uint16_t aun_red_buffer[100];  //red LED sensor data
#else
uint32_t aun_ir_buffer[100]; //infrared LED sensor data
uint32_t aun_red_buffer[100];  //red LED sensor data
#endif

int32_t n_ir_buffer_length; //data length
int32_t n_spo2;  //SPO2 value
int8_t ch_spo2_valid;  //indicator to show if the SPO2 calculation is valid
int32_t n_heart_rate; //heart rate value
int8_t  ch_hr_valid;  //indicator to show if the heart rate calculation is valid
uint8_t uch_dummy;

uint8_t mode = 0;

// the setup routine runs once when you press reset:
void setup() {
  maxim_max30102_reset(); //resets the MAX30102
  
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  pinMode(10, INPUT);  //pin D10 connects to the interrupt output pin of the MAX30102
  delay(1000);
  
  maxim_max30102_read_reg(REG_INTR_STATUS_1,&uch_dummy);  //Reads/clears the interrupt status register
  
  Serial.println(F("Input 0 for getting directly. (default)"));
  Serial.println(F("Input 1 for getting with calc."));
  
  uint32_t t = millis();
  while(Serial.available() == 0) {
    if (millis() - t > 3000) break;
  }
  delay(1000);
  
  uch_dummy=Serial.read();
  switch (uch_dummy) {
    case '0': mode = 0; break;
    case '1': mode = 1; break;
    default: mode = 0; break;
  }
  
  maxim_max30102_init();  //initialize the MAX30102
}

void get() {
    Serial.println(F("START GET"));
    delay(1000);
    
    while (1) {
        while(digitalRead(PIN_INT) == 1);
        maxim_max30102_read_fifo(aun_red_buffer, aun_ir_buffer);
        Serial.print(aun_red_buffer[0], DEC);
        Serial.print(F(","));
        Serial.println(aun_ir_buffer[0], DEC);
    }
}

void getWithCalc() {
  Serial.println(F("START GETWITHCALC"));
  delay(1000);
  
  int32_t i;
  float f_temp;
  
  n_ir_buffer_length=100;  //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for(i=0;i<n_ir_buffer_length;i++)
  {
    while(digitalRead(PIN_INT)==1);  //wait until the interrupt pin asserts
    maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));  //read from MAX30102 FIFO
    
    Serial.print(F("red="));
    Serial.print(aun_red_buffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(aun_ir_buffer[i], DEC);
  }
  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while(1)
  {
    i=0;

    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for(i=25;i<100;i++)
    {
      aun_red_buffer[i-25]=aun_red_buffer[i];
      aun_ir_buffer[i-25]=aun_ir_buffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for(i=75;i<100;i++)
    {
      while(digitalRead(PIN_INT)==1);
      digitalWrite(9, !digitalRead(9));
      maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));

      //send samples and calculation result to terminal program through UART
      Serial.print(F("red="));
      Serial.print(aun_red_buffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.print(aun_ir_buffer[i], DEC);
      
      Serial.print(F(", HR="));
      Serial.print(n_heart_rate, DEC);
      
      Serial.print(F(", HRvalid="));
      Serial.print(ch_hr_valid, DEC);
      
      Serial.print(F(", SPO2="));
      Serial.print(n_spo2, DEC);

      Serial.print(F(", SPO2Valid="));
      Serial.println(ch_spo2_valid, DEC);
    }
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
  }
}

// the loop routine runs over and over again forever:
void loop() {
    switch (mode) {
        case 0: get(); break;
        case 1: getWithCalc(); break;
        default: get(); break;
    }
}
 
