// User data functions.  Modify these functions for your data items.
#include "UserTypes.h"
#include "Wire.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);



//#include "I2Cdev.h"
//#include "MPU6050.h"
//------------------------------------------------------------------------------
//MPU6050 mpu;
static uint32_t startMicros;
// Acquire a data record.
void acquireData(data_t* data) {
  sensors_event_t eventAcc;
  accel.getEvent(&eventAcc);
  sensors_event_t eventMag;
  mag.getEvent(&eventMag);

  data->time = micros();
  getMotion6(&data->ax, &data->ay, &data->az,&data->gx, &data->gy, &data->gz);
}

void getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
  sensors_event_t eventAcc;
  accel.getEvent(&eventAcc);
  sensors_event_t eventMag;
  mag.getEvent(&eventMag);
  *ax = eventAcc.acceleration.x;
  *ay = eventAcc.acceleration.y;
  *az = eventAcc.acceleration.z;
  *gx = eventMag.magnetic.x;
  *gy = eventMag.magnetic.y;
  *gz = eventMag.magnetic.z;
//    I2Cdev::readBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 14, buffer);
//    *ax = (((int16_t)buffer[0]) << 8) | buffer[1];
//    *ay = (((int16_t)buffer[2]) << 8) | buffer[3];
//    *az = (((int16_t)buffer[4]) << 8) | buffer[5];
//    *gx = (((int16_t)buffer[8]) << 8) | buffer[9];
//    *gy = (((int16_t)buffer[10]) << 8) | buffer[11];
//    *gz = (((int16_t)buffer[12]) << 8) | buffer[13];
}
// setup Sensor
void userSetup() {


  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 Mag detected ... Check your wiring!");
    while(1);
  }
    /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 Accel detected ... Check your wiring!");
    while(1);
  }
  // Set scale to max accel
   accel.write8(LSM303_ADDRESS_ACCEL,LSM303_REGISTER_ACCEL_CTRL_REG4_A,0x30);
    /* Enable auto-gain */
  mag.enableAutoRange(true);
  
}

// Print a data record.
void printData(Print* pr, data_t* data) {
  if (startMicros == 0) {
    startMicros = data->time;
  }
  pr->print(data->time- startMicros);
  pr->write(',');
  pr->print(data->ax);
  pr->write(',');
  pr->print(data->ay);
  pr->write(',');
  pr->print(data->az);
  pr->write(',');
  pr->print(data->gx);
  pr->write(',');
  pr->print(data->gy);
  pr->write(',');
  pr->println(data->gz);
}

// Print data header.
void printHeader(Print* pr) {
  startMicros = 0;
  pr->println(F("micros,ax,ay,az,gx,gy,gz"));
}
