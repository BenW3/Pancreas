#ifndef PowerSensors_h
#define PowerSensors_h
#include "Arduino.h"

class PowerSensors {
public:
    PowerSensors();
    long readVref();
    float readDCCurrent(int Pin);
    void init();
    int VoltValue;  // value read from the pot
    float Voltage;  // calculated Voltage from sensorValue
    float Power;    // calculated Voltage from sensorValue
    float Power2;   // calculated Voltage from sensorValue
    float PowerNet; // calculated Voltage from sensorValue
    float CurrentValue;
    float CurrentValue2;
private:
    int _mVperAmp;         // use 185 for 5A Module, and 66 for 30A Module
    float _Vref;     // read your Vcc voltage,typical voltage should be 5000mV(5.0V)

};
#endif