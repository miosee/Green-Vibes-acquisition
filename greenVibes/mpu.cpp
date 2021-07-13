#include "Wire.h"
#include "I2Cdev.h"
#include "mpu.h"

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

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


bool newSample;


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
// b0 and b1 are used by devStatus
#define CONNECTION_ERROR  0x04
#define OVERFLOW_ERROR    0x08
uint8_t err = 0;
uint8_t test[4] = {0,1,2,3};
float testF = 1;

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
