#include <LiquidCrystal.h>
#include "mpu.h"

#define LCD_D4  4
#define LCD_D5  5
#define LCD_D6  6
#define LCD_D7  7
#define LCD_EN  8
#define LCD_RS  9

#define TS        1     // sampling period in ms
#define MEAN_SIZE 1000  // Number of samples for mean power calculation
#define KI        26    // gain of the current AFE in 1/ohm

#define B   0.9995
#define A  -0.9990

LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
float voltage, current;
unsigned long curTime,lastTime;
int samplesNb;
float power;
bool newElecSample, newMpuSample;
VectorInt16 acc;

typedef enum {
  DISCONNECTED,
  CONNECTED
} state_t;
state_t state;


float readVoltage();
float readCurrent();

void setup() {
  mpuSetup();
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcd.print("P (mW)  : ");
  Serial.begin(115200);
  samplesNb = 0;
  power = 0;
  newElecSample = false;
  newMpuSample = false;
  state = DISCONNECTED;
  lastTime = millis();
}

void loop() {
  char c;
  
  if (Serial.available()) {
    c = Serial.read();
    if (c == 'S') {
      samplesNb = 0;
      power = 0;
      newElecSample = false;
      state = CONNECTED;
    }
    else if (c == 'E') {
      Serial.println();
      state = DISCONNECTED;
    }
  }
  
  curTime = millis();
  if (curTime >= lastTime + TS) {
    voltage = readVoltage();
    current = readCurrent();
    lastTime = curTime;
    newElecSample = true;
    samplesNb++;
    power += voltage*current;
    if (samplesNb >= MEAN_SIZE) {
      samplesNb = 0;
      power = (power*1000000)/MEAN_SIZE;
      lcd.setCursor(9, 0);
      lcd.print(power);
      power = 0;
    }
  }


  if (mpuAvailable()) {
    acc = mpuRead();
    newMpuSample = true;
  }

  if (state == DISCONNECTED) {
    // nothing to do;
  }
  else if (state == CONNECTED) {
    if (newElecSample) {
      newElecSample = false;
      Serial.write(0xF0);
      Serial.write((byte*)&voltage, 4);
      Serial.write((byte*)&current, 4);
    }
    if (newMpuSample) {
      newMpuSample = false;
      Serial.write(0x55);
      Serial.write((byte*)&aaReal.x, 2);
      Serial.write((byte*)&aaReal.y, 2);
      Serial.write((byte*)&aaReal.z, 2);
    }
  }
}


float readVoltage() {
  static float oldX = 0, oldY = 0;
  float x, y;
  
  x = (5.0*analogRead(0))/1024;
  y = B*(x-oldX) - A*oldY;
  oldX = x;
  oldY = y;

  return(y);
}


float readCurrent() {
  static float oldX = 0, oldY = 0;
  float x, y;
  
  x = (5.0*analogRead(1))/1024;
  y = B*(x-oldX) - A*oldY;
  oldX = x;
  oldY = y;

  return(y);
}
