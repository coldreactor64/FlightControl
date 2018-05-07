#include <PID_v1.h>
#include "Guidance.h"
#include "Arduino.h"
#include "Filters.h"
#include <Servo.h>

/*Servo Numbering and axis
 *     Pitch
 *       1
 *     4   2  Roll
 *       3
 * Yaw is rotation in the rocket spin.
 * Obv if you want to move them around you can in the code below to assign one to another
 */

Servo Servo1, Servo2, Servo3, Servo4;

Guidance::Guidance(int imusensor){
IMUSensor = imusensor;
if (IMUSensor == 0){
IMU = new MPU6050HMC();
}
if (IMUSensor == 1){
MPU9250IMU = new MPU9250();
}


/*********Customization Values***********/ //Set these values for launch parameters
YawKp = 1; YawKi = 0.05; YawKd = 0.25;
PitchKp = 1; PitchKi = 0.05; PitchKd = 0.25;//Also set some preliminary conservative tunings
RollKp = 1; RollKi = 0.05; RollKd = 0.25;//Have not been tested tho
/*Rocket Heading*/
YawSetpoint = 0;
PitchSetpoint = 90;
RollSetpoint = 0;
/*Servo Settings*/
Servo1Offset = 0; Servo2Offset = 0; Servo3Offset = 0; Servo4Offset = 0;
Servo1Pin = 3; Servo2Pin = 5; Servo3Pin = 6; Servo4Pin = 9;

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

if (IMUSensor == 0){
IMU->Update();
}

if (IMUSensor == 1){
MPU9250IMU->Update();  
}

Yaw = *getypr(); //Get YPR
Pitch = *(getypr()+1);
Roll = *(getypr()+2);
YawInput = (double) Yaw; PitchInput = (double) Pitch; RollInput = (double) Roll;//Change to double to put in the PID controller



if(Roll < -50 && currentMillis > 8000 || Roll > 50 && currentMillis > 8000){
 aborten = true;
}
if(Pitch < -50 && currentMillis > 8000 || Pitch > 50 && currentMillis > 8000){
 aborten = true;
}
if (aborten == true){


Servo1->write(90+Servo1Offset); Servo2->write(90+Servo2Offset); 
Servo3->write(90+Servo3Offset); Servo4->write(90+Servo4Offset); //write nominal straight position if it gets out of control


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


