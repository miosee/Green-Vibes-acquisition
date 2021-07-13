#include <LiquidCrystal.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define LCD_D4  4
#define LCD_D5  5
#define LCD_D6  6
#define LCD_D7  7
#define LCD_EN  8
#define LCD_RS  9

#define TS        10    // sampling period in ms
#define MEAN_SIZE 1000  // Number of samples for mean power calculation
#define KI        26    // gain of the current AFE in 1/ohm

#define B   0.995
#define A  -0.990

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


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

// MPU control/status vars
bool     dmpReady = false;  // set true if DMP init was successful
uint8_t  mpuIntStatus;      // holds actual interrupt status byte from MPU
uint8_t  devStatus;         // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;        // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;         // count of all bytes currently in FIFO
uint8_t  fifoBuffer[64];    // FIFO storage buffer

// orientation/motion vars
Quaternion  q;          // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
// b0 and b1 are used by devStatus
#define CONNECTION_ERROR  0x04
#define OVERFLOW_ERROR    0x08
uint8_t err = 0;
uint8_t test[4] = {0,1,2,3};
float testF = 1;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

bool newSample;

float readVoltage();
float readCurrent();

void mpuSetup();
bool mpuAvailable();
VectorInt16 mpuRead();



void setup() {
  mpuSetup();
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
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
  VectorInt16 acc;
  
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
    lastTime += TS;
    newElecSample = true;
    samplesNb++;
    power += voltage*current;
    if (samplesNb >= MEAN_SIZE) {
      samplesNb = 0;
      power = (power*1000)/MEAN_SIZE;
      if (power < 0) {
        power = 0;
      }
      lcd.clear();
      lcd.print("P = ");
      lcd.print(power);
      lcd.print(" mW");
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
      Serial.write((byte*)&acc.x, 2);
      Serial.write((byte*)&acc.y, 2);
      Serial.write((byte*)&acc.z, 2);
    }
  }
}


float readVoltage() {
  static float oldX = 2.5, oldY = 0;
  float x, y;
  
  x = (5.0*analogRead(0))/1024;
  y = B*(x-oldX) - A*oldY;
  oldX = x;
  oldY = y;

  return(y);
}


float readCurrent() {
  static float oldX = 2.5/26, oldY = 0;
  float x, y;
  
  x = (5.0*analogRead(1))/1024/26;
  y = B*(x-oldX) - A*oldY;
  oldX = x;
  oldY = y;

  return(y);
}


void mpuSetup() {  
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  // initialize device
  mpu.initialize();
  // verify connection
  if (!mpu.testConnection()) {
    err = CONNECTION_ERROR;
  }
  // load and configure the DMP
  devStatus = mpu.dmpInitialize();  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);
    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
      err |= devStatus;
  }
  newSample = false;
}

bool mpuAvailable() {
  return(mpuInterrupt);
}


VectorInt16 mpuRead() {
  // if there are no data to read
  if (!mpuInterrupt) {
    // put dummy values to fill the data packet
    aaReal.x = 0;
    aaReal.y = 0;
    aaReal.z = 0;
  } else {
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    err = mpuIntStatus;
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
      // get real acceleration, adjusted to remove gravity
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    }
  }
  return(aaReal);
}
