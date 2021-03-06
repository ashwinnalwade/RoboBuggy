/**
 * @file lib_encoder.c
 * @author Joseph Paetz (rpaetz)
 */
#include <Arduino.h>
#include "lib_encoder.h"

//-------local Variables-----------------
//required to be volatile for interrupts
volatile unsigned long encCount = 0;

//-------local function prototypes-----------
void increment_encoder();

//------------global functions---------------
//sets up the encoder with the given Pin
void encoder_init(){
  pinMode(ENC_PIN, INPUT);
  attachInterrupt(INTERRUPT_NUM, increment_encoder, RISING); 
}

unsigned long encoder_get_count(){
  return encCount;
}

void encoder_reset(){
  encCount = 0; 
}

//-----------local Functions------------------
//this will count every time the encoder goes from 
//high to low or low to high
void increment_encoder(){
	encCount++;
}
