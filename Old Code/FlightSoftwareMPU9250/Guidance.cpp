#include <PID_v1.h>
#include "Guidance.h"
#include "Arduino.h"
#include "Filters.h"
#include <Servo.h>
Servo Servo1, Servo2, Servo3, Servo4;


Guidance::Guidance(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* mx, float* my, float* mz, float* DeltaT, int filterMode){ 
pax = ax; pay = ay; paz = az;//Pointers to each of these values
pgx = gx; pgy = gy; pgz = gz;//so we can get them when needed
pmx = mx; pmy = my; pmz = mz;  
pdeltat = DeltaT;
/*********Customization Values***********/ //Set these values for launch parameters
YawKp = 1; YawKi = 0.05; YawKd = 0.25;
PitchKp = 1; PitchKi = 0.05; PitchKd = 0.25;//Also set some preliminary conservative tunings
RollKp = 1; RollKi = 0.05; RollKd = 0.25;//Have not been tested tho
/*Rocket Heading*/
YawSetpoint = 0;
PitchSetpoint = 90;
RollSetpoint = 0;
/*Servo Settings*/
//Offsets
Servo1Offset = 0;//apply offsets to servo
Servo2Offset = 0;//to get make them straight
Servo3Offset = 0;
Servo4Offset = 0;
Servo1Pin = 3; Servo2Pin = 5; Servo3Pin = 6; Servo4Pin = 9;// Change this to correct pins for your board
/*Servo Numbering and axis
 *     Pitch
 *       1
 *     4   2  Roll
 *       3
 * Yaw is rotation in the rocket spin.
 * Obv if you want to move them around you can in the code below to assign one to another
 */

/*************************/
YawPID = new PID(&YawInput, &YawOutput, &YawSetpoint, YawKp, YawKi, YawKd, DIRECT);
PitchPID = new PID(&PitchInput, &PitchOutput, &PitchSetpoint, PitchKp, PitchKi, PitchKd, DIRECT);
RollPID = new PID(&RollInput, &RollOutput, &RollSetpoint, RollKp, RollKi, RollKd, DIRECT);
/*Start the PIDs*/
YawPID->SetMode(AUTOMATIC);
PitchPID->SetMode(AUTOMATIC);
RollPID->SetMode(AUTOMATIC);
/* Create Servos */
Servo1 = new Servo; Servo2 = new Servo; Servo3 = new Servo; Servo4 = new Servo;

/*Attach Servos*/
//Servo1->attach(Servo1Pin);  //Uncomment to actually use servos, my current dev board can't right now, but am adding anyways for your testing
//Servo2->attach(Servo2Pin);
//Servo3->attach(Servo3Pin);
//Servo4->attach(Servo4Pin);


};


void Guidance::PIDGuidance(){
unsigned long currentMillis = millis();

AcX = *pax; AcY = *pay; AcZ = *paz;
GyX = *pgx; GyY = *pgy; GyZ = *pgz;
MgX = *pmx; MgY = *pmy; MgZ = *pmz;
deltat = *pdeltat;

if (filterMode == 0){
MahonyQuaternionUpdate(AcX, AcY, AcZ, GyX, GyY, GyZ, MgX, MgY, MgZ, deltat);
}
else if(filterMode == 1){
MadgwickQuaternionUpdate(AcX, AcY, AcZ, GyX, GyY, GyZ, MgX, MgY, MgZ, deltat);
}
Yaw = *getypr(); //Get YPR
Pitch = *(getypr()+1);
Roll = *(getypr()+2);
YawInput = (double) Yaw;//Change to double to put in the PID controller
PitchInput = (double) Pitch;
RollInput = (double) Roll;

if(Roll < -50 && currentMillis > 8000 || Roll > 50 && currentMillis > 5000){
 aborten = true;
}
if(Pitch < -50 && currentMillis > 8000 || Pitch > 50 && currentMillis > 5000){
 aborten = true;
}
if (aborten == true){


Servo1->write(90+Servo1Offset); //write nominal straight position if it gets out of control
Servo2->write(90+Servo2Offset);
Servo3->write(90+Servo3Offset);
Servo4->write(90+Servo4Offset);
}
else{

YawPID->Compute(); //Compute PID
PitchPID->Compute();
RollPID->Compute();

/*Write to the Servos*/
//Pitch
Servo1->write(PitchOutput + Servo1Offset);//Write the PID output and the oppisite for the servo on the other side
PitchDiff = 90 - PitchOutput;
PitchOpp = 90 + PitchDiff;
Servo2->write(PitchOpp + Servo2Offset);

//Roll
Servo3->write(RollOutput + Servo3Offset);
RollDiff = 90 - RollOutput;
RollOpp = 90 + RollDiff;
Servo4->write(90 + Servo4Offset);
}
//Yaw is not yet enabled, have to do major PID turning to get expected results from yaw correction
//When enabled it will be added to all 4 servos to correct for yaw turning, stay tuned
};


int Guidance::SensorDetect(int address){
Wire.beginTransmission(address);
error = Wire.endTransmission();
return error;
};


