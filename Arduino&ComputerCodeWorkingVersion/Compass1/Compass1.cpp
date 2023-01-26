#include "Compass1.h"
#include <Wire.h>
#include "Arduino.h"

Compass1::Compass1(char address) {
 _address = address;
 _declination = -70;
//  CMPS2_init();
 
}

void Compass1::CMPS2_read_XYZ() {
  // initialize array for data

  // command internal control register 0 bit 0 (measure)
  Wire.beginTransmission(_address);
  Wire.write(0x07);
  Wire.write(0x01);
  Wire.endTransmission();
  delay(8);

  // wait for measurement to be completed
  bool flag = false;
  while (!flag)
  {
    // jump to status register
    Wire.beginTransmission(_address);
    Wire.write(0x06);
    Wire.endTransmission();

    // read its value
    Wire.requestFrom(_address, (uint8_t)1);
    int temporal = 0;
    if (Wire.available())
    {
      temporal = Wire.read();
    }

    // if the last bit is 1, data is ready
    temporal &= 1;
    if (temporal != 0)
    {
      flag = true;
    }
  }

  // move address pointer to first address
  Wire.beginTransmission(_address);
  Wire.write(0x00);
  Wire.endTransmission();

  // save data
  Wire.requestFrom(_address, (uint8_t)6);
  byte tmp[6] = {0, 0, 0, 0, 0, 0}; // array for raw data
  if (Wire.available())
  {
    for (int i = 0; i < 6; i++)
    {
      tmp[i] = Wire.read(); // save it
    }
  }

  // reconstruct raw data
  _raw[0] = tmp[1] << 8 | tmp[0]; // x
  _raw[1] = tmp[3] << 8 | tmp[2]; // y
  _raw[2] = tmp[5] << 8 | tmp[4]; // z

  // convert raw data to mG
  for (int i = 0; i < 3; i++)
  {
    _data[i] = 0.48828125 * (float)_raw[i];
  }
}

void Compass1::CMPS2_init() {
  float out1[3];
  float out2[3];
  int i;

  Wire.begin(); // initialization of I2C bus

  // calibration: SET
  Wire.beginTransmission(_address);
  Wire.write(0x07);
  Wire.write(0x80);
  Wire.endTransmission();
  delay(60);

  Wire.beginTransmission(_address);
  Wire.write(0x07);
  Wire.write(0x20);
  Wire.endTransmission();
  delay(10);

  CMPS2_read_XYZ();
  for (i = 0; i < 3; i++)
  {
    out1[i] = _data[i];
  }
  Serial.print("Raw SET = ");
  Serial.print(_raw[0]);
  Serial.print("\t");
  Serial.print(_raw[1]);
  Serial.print("\t");
  Serial.println(_raw[2]);

  // calibration: RESET
  Wire.beginTransmission(_address);
  Wire.write(0x07);
  Wire.write(0x80);
  Wire.endTransmission();
  delay(60);

  Wire.beginTransmission(_address);
  Wire.write(0x07);
  Wire.write(0x40);
  Wire.endTransmission();
  delay(10);

  CMPS2_read_XYZ();
  for (i = 0; i < 3; i++)
  {
    out2[i] = _data[i];
  }

  Serial.print("Raw RESET = ");
  Serial.print(_raw[0]);
  Serial.print("\t");
  Serial.print(_raw[1]);
  Serial.print("\t");
  Serial.println(_raw[2]);

  // offset calculation
  for (i = 0; i < 3; i++)
  {
    _offset[i] = (out1[i] + out2[i]) * 0.5;
  }

  // command internal control register 0 for set operation
  Wire.beginTransmission(_address);
  Wire.write(0x07);
  Wire.write(0x40); // SET
  Wire.endTransmission();
  delay(10);

  // command internal control register 1 to 16 bit resolution, 8ms measurement time
  Wire.beginTransmission(_address);
  Wire.write(0x08);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(10);
}

float Compass1::CMPS2_getHeading() {
  // CMPS2_init();
  CMPS2_read_XYZ(); // read X, Y, Z data of the magnetic field

  // eliminate offset before continuing
  for (int i = 0; i < 3; i++) {
    _data[i] = _data[i] - _offset[i];
  }

  // variables for storing partial results
  float temp0 = 0;
  float temp1 = 0;
  // and for storing the final result
  float deg = 0;

  // calculate heading from data of the magnetic field
  // the formula is different in each quadrant
  if (_data[0] < 0) {
    if (_data[1] > 0) {
      // Quadrant 1
      temp0 = _data[1];
      temp1 = -_data[0];
      deg = 90 - atan(temp0 / temp1) * (180 / 3.14159);
    }
    else {
      // Quadrant 2
      temp0 = -_data[1];
      temp1 = -_data[0];
      deg = 90 + atan(temp0 / temp1) * (180 / 3.14159);
    }
  }
  else {
    if (_data[1] < 0) {
      // Quadrant 3
      temp0 = -_data[1];
      temp1 = _data[0];
      deg = 270 - atan(temp0 / temp1) * (180 / 3.14159);
    }
    else {
      // Quadrant 4
      temp0 = _data[1];
      temp1 = _data[0];
      deg = 270 + atan(temp0 / temp1) * (180 / 3.14159);
    }
  }
  // correct heading
  deg += _declination;
  if (_declination > 0) {
    if (deg > 360) {
      deg -= 360;
    }
  }
  else {
    if (deg < 0) {
      deg += 360;
    }
  }
  return deg;
}