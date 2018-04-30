#include "Arduino.h"
#include <Wire.h>
#include "Filters.h"

class MPU6050HMC {
public:
float deltat;
float Yaw, Pitch, Roll;
    MPU6050HMC();
    void Update();
private:
uint8_t RawData[14];
uint8_t RawMagData[14];
float lastUpdate, Now;
float AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ,Tmp;
int16_t rAcX,rAcY,rAcZ,rGyX,rGyY,rGyZ,rMgX,rMgY,rMgZ,rTmp;
    bool startSensor();

};



class MPU9250 {   
public:
float deltat;
float Yaw, Pitch, Roll;
MPU9250();
void Update();
float AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ,Tmp;
int16_t rAcX,rAcY,rAcZ,rGyX,rGyY,rGyZ,rMgX,rMgY,rMgZ,rTmp;
private:
uint8_t ST1;
uint8_t RawData[14];
uint8_t RawMagData[14];
float lastUpdate, Now;

    bool startSensor();

};

void I2CReadNByte(uint8_t, uint8_t, uint8_t, uint8_t*);
void I2CwriteByte(uint8_t, uint8_t, uint8_t);
