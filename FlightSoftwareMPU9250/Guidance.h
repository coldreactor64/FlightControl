#include <Arduino.h>

class PID;
class Servo;
class String;
class Serial;
class Guidance  {
public:    
    PID *YawPID;
    PID *PitchPID;
    PID *RollPID;
    Servo *Servo1, *Servo2, *Servo3, *Servo4;
    double YawInput, PitchInput, RollInput;
    Guidance(float*, float*, float*, float*, float*, float*, float*, float*, float*, float*, int);//Initialization Method
    double YawSetpoint, PitchSetpoint, RollSetpoint; //Setpoints to the direction you want the rocket to fly 
    void PIDGuidance();
    float Yaw, Pitch, Roll;
    double YawOutput, PitchOutput, RollOutput, PitchOpp, RollOpp, PitchDiff, RollDiff;
    float AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ,deltat;
    float *pax, *pay, *paz, *pgx, *pgy, *pgz, *pmx, *pmy, *pmz, *pdeltat; //the pointers to the main values
    double YawKp, YawKi, YawKd, PitchKp, PitchKi, PitchKd, RollKp, RollKi, RollKd;
private:
    int filterMode;
    int Servo1Offset,Servo2Offset,Servo3Offset,Servo4Offset;
    int Servo1Pin, Servo2Pin, Servo3Pin, Servo4Pin;
    bool aborten;
};
