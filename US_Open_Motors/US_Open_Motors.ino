#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver reads raw data from the BNO055
  Same configuration with DRV2605
   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground
*/


// ACCELEROMETER DEFINITIONS -----------------------------------------------------------------------------
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055();


// MOTOR DEFINITIONS -----------------------------------------------------------------------------
#include "Adafruit_DRV2605.h"
Adafruit_DRV2605 drv;


// LED DEFINITIONS -----------------------------------------------------------------------------
int led = 13;


void setup(void)
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");
  pinMode(led, OUTPUT);
  drv.begin();
  drv.selectLibrary(1);

  // I2C trigger by sending 'go' command
  // default, internal trigger when sending GO command
  drv.setMode(DRV2605_MODE_INTTRIG);

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);
  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}

// MOTOR EFFECT -----------------------------------------------------------------------------
uint8_t effect = 7;


void loop(void)
{

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> acceleration = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  // set the effect to play
  drv.setWaveform(0, effect);  // play effect
  drv.setWaveform(1, 0);       // end waveform



  if (euler.y() < -5 && acceleration.x() > -.5 ) {
    digitalWrite(led, HIGH);
    // play the effect!
    drv.go();

    // wait a bit
    //delay(500);
  }
  else {
    digitalWrite(led, LOW);
  }
  printValues();
}

void printValues() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> acceleration = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  Serial.print(" X: Acceleration: ");
  Serial.print(acceleration.x());
  Serial.print("\t\t");
  Serial.print(" Y: ");
  Serial.print(euler.y());
  Serial.print("\t\t");

  
  //Serial.print("X: ");
  //Serial.print(euler.x());
  //Serial.print(" Z: ");
  //Serial.print(euler.z());

  /*
    // Quaternion data
    imu::Quaternion quat = bno.getQuat();
    Serial.print("qW: ");
    Serial.print(quat.w(), 4);
    Serial.print(" qX: ");
    Serial.print(quat.y(), 4);
    Serial.print(" qY: ");
    Serial.print(quat.x(), 4);
    Serial.print(" qZ: ");
    Serial.print(quat.z(), 4);
    Serial.print("\t\t");
  */

  /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}

