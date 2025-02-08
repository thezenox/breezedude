#pragma once
#include <Arduino.h>
#include "display.h"

#define DEBUGSER Serial1
extern int div_cpu;
extern bool usb_connected;
extern bool errors_enabled;
extern bool debug_enabled;

void log_i(const char * msg){
  if(debug_enabled){
    DEBUGSER.print(msg);
    DEBUGSER.flush();
    if(display_present()) {display_add_line(String(msg));}
  }
  if(usb_connected){
    Serial.print(msg);
  }
}
void log_i(const char * msg, uint32_t num){
  if(debug_enabled){
    DEBUGSER.print(msg);
    DEBUGSER.println(num);
    DEBUGSER.flush();
    if(display_present()) {display_add_line(String(msg) + String(num));}
  }
  if(usb_connected){
    Serial.print(msg);
    Serial.println(num);
  }
}
void log_i(const char * msg, int32_t num){
  if(debug_enabled){
    DEBUGSER.print(msg);
    DEBUGSER.println(num);
    DEBUGSER.flush();
    if(display_present()) {display_add_line(String(msg) + String(num));}
  }
  if(usb_connected){
    Serial.print(msg);
    Serial.println(num);
  }
}
void log_i(const char * msg, int num){
  if(debug_enabled){
    DEBUGSER.print(msg);
    DEBUGSER.println(num);
    if(display_present()) {display_add_line(String(msg) + String(num));}
  }
  if(usb_connected){
    Serial.print(msg);
    Serial.println(num);
  }
}
void log_i(const char * msg, float num){
  if(debug_enabled){
    DEBUGSER.print(msg);
    DEBUGSER.println(num);
    if(display_present()) {display_add_line(String(msg) + String(num));}
  }
  if(usb_connected){
    Serial.print(msg);
    Serial.println(num);
  }
}

void log_e(const char * msg){
  if(errors_enabled){
    DEBUGSER.print(msg);
    if(display_present()) {display_add_line(String(msg));}
  }
  if(usb_connected){
    Serial.print(msg);
  }
}

void log_flush(){
  if(debug_enabled){
    DEBUGSER.flush();
  }
}

void log_ser_begin(){
  if(debug_enabled){
    DEBUGSER.begin(115200*div_cpu);
  }
}