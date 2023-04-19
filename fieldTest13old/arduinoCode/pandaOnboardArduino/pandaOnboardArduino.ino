#include <Compass1.h>
#include <PowerSensors.h>
#define DECLINATION -70
unsigned char compassAddress = 0x30;
PowerSensors sensors;
Compass1 compass(compassAddress);
const int currentSensorPin = A2;    // define sensor pin
const int voltageSensorPin = A1;
const int currentSensorPin2 = A0; // define sensor pin
String Comm = "";
char command;
String line = "";

void setup() {
  Serial.begin(115200); // serial initialization
  delay(10);
  compass.CMPS2_init();                  // initialize the compass
  delay(10);
  sensors.init();
  delay(100);
  Serial.println("1");
  Serial.setTimeout(2);

}

void loop() {
  if (Serial.available()) {
    line = Serial.readString();
    command = line[0];
    switch (command) {
      
      case 'C': //compass
        Comm = String(compass.CMPS2_getHeading());
        Serial.println(Comm);
//        delay(100);
        break;

      case 'P': //power
        sensors.CurrentValue = sensors.readDCCurrent(currentSensorPin);
        sensors.CurrentValue2 = sensors.readDCCurrent(currentSensorPin2);
        sensors.VoltValue = analogRead(voltageSensorPin);
        sensors.Voltage = (sensors.VoltValue - 512) * 0.073170;
        sensors.Power = sensors.Voltage * sensors.CurrentValue;   // Power from the batteries
        sensors.Power2 = 36 * sensors.CurrentValue2; // Power from solar
        sensors.PowerNet = sensors.Power2 - sensors.Power;   // Solar Power minus Power used from Batteries
        Comm = String(sensors.Power) + "," + String(sensors.Power2) + "," + String(sensors.Voltage);
        Serial.println(Comm);
        break;
        
      default:
        break;
        
      command = "";
    }
  }
}
