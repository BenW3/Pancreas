#ifndef StepperMotorClosedLoop_h
#define StepperMotorClosedLoop_h
#include "Arduino.h"
#include "Wire.h"
#include "SparkFun_I2C_Mux_Arduino_Library.h" 

class StepperMotorClosedLoop {
public:
    StepperMotorClosedLoop(int stepPin, int dirPin,int port, int steps, float gearRatio, unsigned char address);
    void turnToAngle();
    int calculateStepsReversible(float desiredPos);
    void calculateSteps(float desiredPos);
    void calibrateZero();
    void init();
    int returnAngle();
    int returnCommand();
     
private:
    void _readRawAngle();
    void _correctAngle();
    void _checkQuadrant();
    void _checkMagnetPresence();
    long int _currentCount;
    long int _desiredCount;
    long int _steps;
    long int _stepsToGo;
    int _stepPin;
    int _dirPin;
    //Magnetic sensor things
    int _magnetStatus; //value of the status register (MD, ML, MH)
    int _lowbyte; //raw angle 7:0
    word _highbyte; //raw angle 7:0 and 11:8
    int _rawAngle; //final raw angle 
    float _degAngle; //raw angle in degrees (360/4096 * [value between 0-4095])
    int _quadrantNumber, _previousquadrantNumber; //quadrant IDs
    int _numberofTurns; //number of turns
    int _correctedAngle; //tared angle - based on the startup value
    float _startAngle; //starting angle
    int _totalAngle; //total absolute angular displacement
    float _resolution;
    unsigned char _address;
    float _gearRatio;
    long int _stepsNoGearbox;
    QWIICMUX _mux;
    int _port;
    long int _time;
    int _pulseDelay;
};

#endif