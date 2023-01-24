#include "Arduino.h"
#include "StepperMotorClosedLoop2.h"
#include "Wire.h"
#include "SparkFun_I2C_Mux_Arduino_Library.h" 
StepperMotorClosedLoop::StepperMotorClosedLoop(int stepPin, int dirPin, int port, int steps, float gearRatio, unsigned char address){
    _stepPin = stepPin;
    _dirPin = dirPin;
    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    _steps = steps;
    _port = port;
    _currentCount = 0;
    _desiredCount = 0;
    _stepsToGo = 0;
    _stepPin = stepPin;
    _dirPin = dirPin;
    _magnetStatus = 0; //value of the status register (MD, ML, MH)
    _numberofTurns = 0; //number of turns
    _correctedAngle = 0; //tared angle - based on the startup value
    _startAngle = 0; //starting angle
    _totalAngle = 0; //total absolute angular displacement
    _gearRatio = gearRatio;
    _resolution = (_steps/_gearRatio)/4096.0;
    _stepsNoGearbox = _steps/_gearRatio;
    _address = address;
    _pulseDelay = 100;
    
}

void StepperMotorClosedLoop::init() {
    Wire.begin(); //start i2C  
    Wire.setClock(10000); 
    _mux.begin();
    _mux.setPort(_port);
    _checkMagnetPresence(); //check the magnet (blocks until magnet is found)
    _readRawAngle(); //make a reading so the degAngle gets updated
    _startAngle = _degAngle; //update startAngle with degAngle - for taring
    _time = millis();
   
}

void StepperMotorClosedLoop::calculateSteps(float desiredPos) {
    _mux.setPort(_port);
    _desiredCount = (desiredPos * _steps/2)/PI;
    _desiredCount = _desiredCount % _steps;
    _readRawAngle();
    _correctAngle();
    _checkQuadrant();
    _currentCount = _totalAngle;
    if (abs(_currentCount - _desiredCount) > _steps / 2) {
        if (_currentCount > _desiredCount) {
            _stepsToGo = _steps + _desiredCount;
        } else {
            _stepsToGo = _desiredCount - _steps;
        }
    } else {
        _stepsToGo = _desiredCount;
    }
    // if (_stepsToGo < 0) {
    //   _stepsToGo = _steps-
    // } else if (_stepsToGo > _steps) {

    // }
}

// int StepperMotorClosedLoop::calculateStepsReversible(float desiredPos) {
//     _mux.setPort(_port);
//     int val = 1;
//     _desiredCount = (desiredPos * _steps/2)/PI;
//     _desiredCount = _desiredCount % _steps;
//     _readRawAngle();
//     _correctAngle();
//     _checkQuadrant();
//     _currentCount = _totalAngle;
//     if (abs(_currentCount - _desiredCount) > _steps / 2) {
//         if (_currentCount > _desiredCount) {
//             _stepsToGo = _steps + _desiredCount;
//         } else {
//             _stepsToGo = _desiredCount - _steps;
//         }
//     } else {
//         _stepsToGo = _desiredCount;
//     }
//     if (abs(_currentCount - _stepsToGo) > (_steps/4)) {
//         _stepsToGo = (_stepsToGo + _steps/2) % _steps;
//         val = -1;
//     } else {
//         val =  1;
//     }
//     if (abs(_currentCount - _stepsToGo) > _steps / 2) {
//         if (_currentCount > _stepsToGo) {
//             _stepsToGo = _steps + _stepsToGo;
//         } else {
//             _stepsToGo = _stepsToGo - _steps;
//         }
//     }

//     return val;
// }

int StepperMotorClosedLoop::calculateStepsReversible(float desiredPos) {
    _mux.setPort(_port);
    int val = 1;
    _desiredCount = (desiredPos * _steps/2)/PI;
    _desiredCount = _desiredCount % _steps;
    _readRawAngle();
    _correctAngle();
    _checkQuadrant();
    _currentCount = _totalAngle;

    if (_desiredCount > _steps/4) {
      _stepsToGo = _desiredCount-_steps/2;

      val = -1;
    } else if (_desiredCount < -_steps/4) {
      _stepsToGo = _desiredCount+_steps/2;
      val = -1;
    } else {
      _stepsToGo = _desiredCount;

    }

    return val;
}

void StepperMotorClosedLoop::turnToAngle() {
    
    // if (millis()-_time > 200){
    _mux.setPort(_port);
    _readRawAngle();
    _correctAngle();
    _checkQuadrant();
    // _time = millis();
    // }
    if (_totalAngle > _stepsToGo+_steps/200) {
        digitalWrite(_dirPin, LOW);
        digitalWrite(_stepPin, HIGH);
        delayMicroseconds(_pulseDelay);
        digitalWrite(_stepPin, LOW);
        // delayMicroseconds(_pulseDelay);
        
    }
    else if (_totalAngle < _stepsToGo-_steps/200) {
        digitalWrite(_dirPin, HIGH);
        digitalWrite(_stepPin, HIGH);
        delayMicroseconds(_pulseDelay);
        digitalWrite(_stepPin, LOW);
        // delayMicroseconds(_pulseDelay);  
    }
    // }
}

void StepperMotorClosedLoop::calibrateZero() {
    _mux.setPort(_port);
    _currentCount = 0;
    _desiredCount = 0;
    _readRawAngle();
    _startAngle = _degAngle;
}

void StepperMotorClosedLoop::_correctAngle() {
  //recalculate angle
  _correctedAngle = _degAngle; // - _startAngle; //this tares the position
  // _correctedAngle = _correctedAngle % _steps;
//   if(correctedAngle < 0){ //if the calculated angle is negative, we need to "normalize" it
//   _correctedAngle = _correctedAngle + _steps; //correction for negative numbers (i.e. -15 becomes +345)
//   }
//   else {
//     //do nothing
//   }
  //Serial.print("Corrected angle: ");
  //Serial.println(correctedAngle, 2); //print the corrected/tared angle  
}

void StepperMotorClosedLoop::_checkQuadrant() {
    /*
  //Quadrants:
  4  |  1
  ---|---
  3  |  2
  */

  //Quadrant 1
  if(_correctedAngle >= 0 && _correctedAngle <= _stepsNoGearbox/4.0)
  {
    _quadrantNumber = 1;
  }

  //Quadrant 2
  if(_correctedAngle > _stepsNoGearbox/4.0 && _correctedAngle <= _stepsNoGearbox/2.0)
  {
    _quadrantNumber = 2;
  }

  //Quadrant 3
  if(_correctedAngle > _stepsNoGearbox/2.0 && _correctedAngle <= 3.0*_stepsNoGearbox/4.0)
  {
    _quadrantNumber = 3;
  }

  //Quadrant 4
  if(_correctedAngle > 3.0*_stepsNoGearbox/4.0 && _correctedAngle < _stepsNoGearbox)
  {
    _quadrantNumber = 4;
  }
  //Serial.print("Quadrant: ");
  //Serial.println(quadrantNumber); //print our position "quadrant-wise"

  if(_quadrantNumber != _previousquadrantNumber) //if we changed quadrant
  {
    if(_quadrantNumber == 1 && _previousquadrantNumber == 4)
    {
      _numberofTurns++; // 4 --> 1 transition: CW rotation
    }

    if(_quadrantNumber == 4 && _previousquadrantNumber == 1)
    {
      _numberofTurns--; // 1 --> 4 transition: CCW rotation
    }
    //this could be done between every quadrants so one can count every 1/4th of transition

    _previousquadrantNumber = _quadrantNumber;  //update to the current quadrant
  
  }  
  //Serial.print("Turns: ");
  //Serial.println(numberofTurns,0); //number of turns in absolute terms (can be negative which indicates CCW turns)  

  //after we have the corrected angle and the turns, we can calculate the total absolute position
  _totalAngle = (_numberofTurns*_stepsNoGearbox + _correctedAngle); //number of turns (+/-) plus the actual angle within the 0-360 range
  //  _totalAngle = (_numberofTurns*_stepsNoGearbox + _degAngle)%_steps;
  //Serial.print("Total angle: ");
  //Serial.println(totalAngle, 2); //absolute position of the motor expressed in degree angles, 2 digits  
}

void StepperMotorClosedLoop::_checkMagnetPresence() {
      //This function runs in the setup() and it locks the MCU until the magnet is not positioned properly

  while((_magnetStatus & 32) != 32) //while the magnet is not adjusted to the proper distance - 32: MD = 1
  {
    _magnetStatus = 0; //reset reading

    Wire.beginTransmission(_address); //connect to the sensor
    Wire.write(0x0B); //figure 21 - register map: Status: MD ML MH
    Wire.endTransmission(); //end transmission
    Wire.requestFrom(_address, 1); //request from the sensor

    while(Wire.available() == 0); //wait until it becomes available 
    _magnetStatus = Wire.read(); //Reading the data after the request

    //Serial.print("Magnet status: ");
    //Serial.println(magnetStatus, BIN); //print it in binary so you can compare it to the table (fig 21)      
  }      
  
  //Status register output: 0 0 MD ML MH 0 0 0  
  //MH: Too strong magnet - 100111 - DEC: 39 
  //ML: Too weak magnet - 10111 - DEC: 23     
  //MD: OK magnet - 110111 - DEC: 55

  //Serial.println("Magnet found!");
  //delay(1000);  
}

void StepperMotorClosedLoop::_readRawAngle() {
      //7:0 - bits
  Wire.beginTransmission(_address); //connect to the sensor
  Wire.write(0x0D); //figure 21 - register map: Raw angle (7:0)
  Wire.endTransmission(); //end transmission
  Wire.requestFrom(_address, 1); //request from the sensor
  
  while(Wire.available() == 0); //wait until it becomes available 
  _lowbyte = Wire.read(); //Reading the data after the request
 
  //11:8 - 4 bits
  Wire.beginTransmission(_address);
  Wire.write(0x0C); //figure 21 - register map: Raw angle (11:8)
  Wire.endTransmission();
  Wire.requestFrom(_address, 1);
  
  while(Wire.available() == 0);  
  _highbyte = Wire.read();
  
  //4 bits have to be shifted to its proper place as we want to build a 12-bit number
  _highbyte = _highbyte << 8; //shifting to left
  //What is happening here is the following: The variable is being shifted by 8 bits to the left:
  //Initial value: 00000000|00001111 (word = 16 bits or 2 bytes)
  //Left shifting by eight bits: 00001111|00000000 so, the high byte is filled in
  
  //Finally, we combine (bitwise OR) the two numbers:
  //High: 00001111|00000000
  //Low:  00000000|00001111
  //      -----------------
  //H|L:  00001111|00001111
  _rawAngle = _highbyte | _lowbyte; //int is 16 bits (as well as the word)

  //We need to calculate the angle:
  //12 bit -> 4096 different levels: 360° is divided into 4096 equal parts:
  //360/4096 = 0.087890625
  //Multiply the output of the encoder with 0.087890625
  _degAngle = _rawAngle * _resolution; 
  
  //Serial.print("Deg angle: ");
  //Serial.println(degAngle, 2); //absolute position of the encoder within the 0-360 circle
}

int StepperMotorClosedLoop::returnAngle() {
  return _totalAngle;
}

int StepperMotorClosedLoop::returnCommand() {
  return _stepsToGo;
}
