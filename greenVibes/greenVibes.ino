// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

#include <LiquidCrystal.h>

#define LCD_RS  4
#define LCD_EN  5
#define LCD_D4  6
#define LCD_D5  7
#define LCD_D6  8
#define LCD_D7  9

#define TEST1   10
#define TEST2   11
#define TEST3   12


LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);


#define TS        10    // sampling period in ms
#define MEAN_SIZE 50   // Number of samples for mean power calculation
#define KI        26    // gain of the current AFE in 1/ohm

#define B   0.995
#define A  -0.990

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


typedef enum {
  DISCONNECTED,
  CONNECTED
} state_t;
state_t state;

float voltage=0, current=0;
int samplesNb;
float power = 0;
float freq;

float getFreq(int16_t acc);
float readVoltage();
float readCurrent();

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // Timing test pins
  pinMode(TEST1, OUTPUT);
  pinMode(TEST2, OUTPUT);
  pinMode(TEST3, OUTPUT);
  
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  // initialize device
  lcd.clear();
  lcd.print("Initialising MPU");
  mpu.initialize();

  // verify connection
  lcd.clear();
  if (!mpu.testConnection()) {
    lcd.print("MPU init error");
    while(1);
  }

  // load and configure the DMP
  lcd.clear();
  lcd.print("Initialising DMP");
  devStatus = mpu.dmpInitialize();
    
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    lcd.clear();
    lcd.print("Enabling DMP");
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
    lcd.clear();
    lcd.print("P =       mW");
    lcd.setCursor(0, 1);
    lcd.print("f =       Hz");

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    lcd.clear();
    lcd.print("DMP error: ");
    lcd.print(devStatus);
    while(1);
  }
  state = DISCONNECTED;
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  char c;
  
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (Serial.available()) {
      c = Serial.read();
      if (c == 'S') {
        samplesNb = 0;
        power = 0;
        state = CONNECTED;
      }
      else if (c == 'E') {
        state = DISCONNECTED;
      }
    }
  }
  
  digitalWrite(TEST1, HIGH);

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  digitalWrite(TEST3, HIGH);

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  digitalWrite(TEST3, LOW);
  
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    lcd.clear();
    lcd.print("FIFO reset");
    while(1);
  }
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  else if (mpuIntStatus & 0x02) {
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

    freq = getFreq(aaReal.x);
    voltage = readVoltage();
    current = readCurrent();
    power += voltage*current;

    samplesNb++;
    if (samplesNb >= MEAN_SIZE) {
      digitalWrite(TEST2, HIGH);
      samplesNb = 0;
      power = (power*1000)/MEAN_SIZE;
      if (power < 0) {
        power = 0;
      }
      lcd.setCursor(4, 0);
      lcd.print(power);
      power = 0;
      lcd.setCursor(4, 1);
      lcd.print(freq);
      digitalWrite(TEST2, LOW);
    }

    if (state == DISCONNECTED) {
      // nothing to do;
    }
    else if (state == CONNECTED) {
      Serial.write(0x55);
      Serial.write((byte*)&aaReal.x, 2);
      Serial.write((byte*)&aaReal.y, 2);
      Serial.write((byte*)&aaReal.z, 2);
      Serial.write((byte*)&voltage, 4);
      Serial.write((byte*)&current, 4);
    }
  }
  digitalWrite(TEST1, LOW);
}

float getFreq(int16_t acc) {
  static int16_t minAcc = 0, maxAcc = 0;
  static uint32_t lastCrossingTime = 0;
  static int16_t lastSign = 1;
  static float freq = 0;
  uint32_t curTime;
  
  curTime = millis();

  if (acc > maxAcc) {
    maxAcc = acc;
  }
  else if (acc < minAcc) {
    minAcc = acc;
  }
    
  if ( (lastSign < 0) && (acc > 0) ) {
    lastSign = 1;
    if (lastCrossingTime != 0) {
      if ( (curTime - lastCrossingTime > 100) && (curTime - lastCrossingTime < 2000) && (maxAcc-minAcc > 1000) ) {
        freq = 1000.0/(curTime - lastCrossingTime);
      }
      else {
        freq = 0;
      }
      maxAcc = 0;
      minAcc = 0;
    }
    lastCrossingTime = curTime;
  }
  else if ( (lastSign > 0) && (acc < 0) ) {
    lastSign = -1;
  }
  return(freq);
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
