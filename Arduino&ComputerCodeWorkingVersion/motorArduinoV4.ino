
//-----------------------
//   DEFINES AND INCLUDES
//-----------------------
// Motor and steering
#define LWHEELMOTOR 2
#define RWHEELMOTOR 3
#define LRWHEELMOTOR 4
#define RRWHEELMOTOR 5
// #define WHEELMOTOR 1
#define LSTEPPIN 9
#define LDIRPIN 10
#define RSTEPPIN 11
#define RDIRPIN 12
#define LRSTEPPIN 13
#define LRDIRPIN 8
#define RRSTEPPIN 6
#define RRDIRPIN 7
//Power compass and communication
#define DECLINATION -70
#include <Servo.h>
#include <StepperMotorClosedLoop2.h>



//--------------------
//GLOBAL VARIABLES / OBJECTS
//--------------------
// Motor and steering
int steps = 850;
float gearRatio = 4.25;
unsigned char steeringAddress = 0x36;
int setPoint = 0;
float turnAngle;
float roboWidth = 10.0;
float roboLength = 4.0;
float leftAngle;
float rightAngle;
float rightRearAngle;
float leftRearAngle;
Servo LWheel;
Servo RWheel;
Servo LRWheel;
Servo RRWheel;
Servo Wheel;
StepperMotorClosedLoop lMotor(LSTEPPIN, LDIRPIN, 0, steps, gearRatio, steeringAddress);
StepperMotorClosedLoop rMotor(RSTEPPIN, RDIRPIN, 1, steps, gearRatio, steeringAddress);
StepperMotorClosedLoop lrMotor(LRSTEPPIN, LRDIRPIN, 2, steps, gearRatio, steeringAddress);
StepperMotorClosedLoop rrMotor(RRSTEPPIN, RRDIRPIN, 3, steps, gearRatio, steeringAddress);
String Comm = "";
char command;
String line = "";
String steeringString = "";
String forwardString = "";
String setpointString = "";
int i;
int j;
float steeringSignal;
int forwardSignal;
int signalStore;

//-------------------
//   SETUP
//-------------------

void setup() {
  Serial.begin(115200); // serial initialization
  delay(10);
  Serial.println("Setting up drive motors . . .");
  Serial.setTimeout(10);
  pinMode(LWHEELMOTOR, OUTPUT);
  pinMode(RWHEELMOTOR, OUTPUT);
  pinMode(LRWHEELMOTOR, OUTPUT);
  pinMode(RRWHEELMOTOR, OUTPUT);
  // pinMode(WHEELMOTOR, OUTPUT);
  // Wheel.attach(WHEELMOTOR);
  // Wheel.writeMicroseconds(1500);
  LWheel.attach(LWHEELMOTOR);
  LWheel.writeMicroseconds(1500);
  RWheel.attach(RWHEELMOTOR);
  RWheel.writeMicroseconds(1500);
  LRWheel.attach(LRWHEELMOTOR);
  LRWheel.writeMicroseconds(1500);
  RRWheel.attach(RRWHEELMOTOR);
  RRWheel.writeMicroseconds(1500);
  Serial.println(F("Connecting steering motors . . ."));
  lMotor.init();
  Serial.println(F("lmotor running"));
  delay(10);
  rMotor.init();
  Serial.println(F("rmotor running"));
  delay(10);
  lrMotor.init();
  Serial.println(F("lrmotor running"));
  delay(10);
  rrMotor.init();
  Serial.println(F("rrmotor running"));
  delay(10);

}

//-------------------
//   LOOP
//-------------------

void loop() {
  if (Serial.available()) {
    command = "";
    line = Serial.readString();
    Serial.flush();
    command = line[0];
    //    Serial.println(line);
    switch (command) {

      case 'S': //steering
        steeringString = "";
        forwardString = "";
        setpointString = "";
        j = line.length();
        if (j > 2) {
          i = 1;
          while (i < j) {
            if (line[i] == char(',')) {
              break;
            }
            steeringString += line[i];
            i++;
          }
          i++;
          while (i < j) {
            if (line[i] == char(',')) {
              break;
            }
            forwardString += line[i];
            i++;
          }
          i++;
          while (i < j) {
            setpointString += line[i];
            i++;
          }
          steeringSignal = steeringString.toFloat();
          forwardSignal = forwardString.toInt();
          forwardSignal = forwardSignal - 1500;
          signalStore = forwardSignal;
          setPoint = setpointString.toInt();
          steering(steeringSignal, setPoint);
          // Wheel.writeMicroseconds(forwardSignal+1500);
//                    lMotor.calculateSteps(leftAngle);
//                    rMotor.calculateSteps(rightAngle);
//                    lrMotor.calculateSteps(leftRearAngle);
//                    rrMotor.calculateSteps(rightRearAngle);
          forwardSignal = signalStore * lMotor.calculateStepsReversible(leftAngle);
          LWheel.writeMicroseconds(forwardSignal + 1500);
          forwardSignal = signalStore * rMotor.calculateStepsReversible(rightAngle);
          RWheel.writeMicroseconds(forwardSignal + 1500);
          forwardSignal = signalStore * lrMotor.calculateStepsReversible(leftRearAngle);
          LRWheel.writeMicroseconds(forwardSignal + 1500);
          forwardSignal = signalStore * rrMotor.calculateStepsReversible(rightRearAngle);
          RRWheel.writeMicroseconds(forwardSignal + 1500);
          //          Comm = String(steeringSignal)+","+String(setPoint)+","+String(leftAngle)+","+String(lMotor.returnCommand())+ "," + String(lMotor.returnAngle());
          //          Serial.println(Comm);
        }
        break;

      default:
        break;
    }
  }
  lMotor.turnToAngle();
//  Serial.println(lMotor.returnAngle());
  rMotor.turnToAngle();
  lrMotor.turnToAngle();
  rrMotor.turnToAngle();

}


//--------------------
//STEERING
//--------------------

void steering(float controlSignal, int setpoint) {
  float Offset = float(setpoint) * PI / 1000.0;
  turnAngle = controlSignal;
  leftAngle = atan(roboLength * sin(turnAngle) / (roboLength * cos(turnAngle) + roboWidth * sin(turnAngle)));
  rightAngle = atan(roboLength * sin(turnAngle) / (roboLength * cos(turnAngle) - roboWidth * sin(turnAngle)));
  if (turnAngle > 0 && rightAngle < 0) {
    rightAngle = (PI) + rightAngle;
  }
  if (turnAngle < 0 && leftAngle > 0) {
    leftAngle = -(PI) + leftAngle;
  }
  leftRearAngle = leftAngle;
  rightRearAngle = rightAngle;
  leftAngle = -leftAngle;
  rightAngle = -rightAngle;
  leftAngle = leftAngle - Offset;
  rightAngle = rightAngle - Offset;
  leftRearAngle = leftRearAngle - Offset;
  rightRearAngle = rightRearAngle - Offset;
}
