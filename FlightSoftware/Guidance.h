#include <Arduino.h>
#include "Sensors.h"


class PID;
class Servo;
class String;
class Serial;
class MPU6050HMC;
class MPU9250;
class Guidance  {
public:
    PID *YawPID;
    PID *PitchPID;
    PID *RollPID;
    Servo *Servo1, *Servo2, *Servo3, *Servo4;
    MPU6050HMC *IMU;
    MPU9250 *MPU9250IMU;
    Guidance(int);//Initialization Method
    void PIDGuidance();
    float Yaw, Pitch, Roll;
    double YawOutput, PitchOutput, RollOutput, PitchOpp, RollOpp, PitchDiff, RollDiff;
    double YawKp, YawKi, YawKd, PitchKp, PitchKi, PitchKd, RollKp, RollKi, RollKd;
    double YawSetpoint, PitchSetpoint, RollSetpoint; //Setpoints to the direction you want the rocket to fly 
    double YawInput, PitchInput, RollInput;
    float AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ,DeltaT;
    int IMUSensor;
private:
    int filterMode;
    int Servo1Offset,Servo2Offset,Servo3Offset,Servo4Offset;
    int Servo1Pin, Servo2Pin, Servo3Pin, Servo4Pin;
    bool aborten;
};
