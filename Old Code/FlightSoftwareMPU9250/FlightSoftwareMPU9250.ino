#include <PID_v1.h>
#include<Wire.h>
#include <SPI.h>
#include "Filters.h"
#include "Guidance.h"

#define MPU9250
//#define MPU6050HMC
#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C
const int MPU=0x68;  // I2C address of the MPU-6050
const int Mag = 0x1E;
const int Ahk = 0x0C;
int16_t rAcX,rAcY,rAcZ,rTmp,rGyX,rGyY,rGyZ,rMgX,rMgY,rMgZ,mx,my,mz;
float AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ,MgX,MgY,MgZ;
float lastUpdate = 0;    // used to calculate integration interval
float Now = 0;           // used to calculate integration interval
float deltat = 0.0f;        // integration interval for both filter scheme
//#define OutputYawPitchRoll
//#define OutputJSON
//#define OutputQuat
#define SensorOutput

float yaw, pitch, roll;
float x = 8;
Guidance Guidance(&AcX,&AcY,&AcZ,&GyX,&GyY,&GyZ,&MgX,&MgY,&MgZ, &deltat, 1);
uint8_t ST1;
void setup(){
Wire.begin();
Serial.begin(9600);
if(!setupSensors()){
  Serial.print(".");
}
Serial.println("Sensors Started");
}


void loop(){
/**Read Sensors**/
readGyroAccel();
readMag();
/*Get time btween updates*/
Now = micros();
deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
lastUpdate = micros();

/*Do the actual filtering and Guidance*/
Guidance.PIDGuidance();

/*Debug Outputs*/
#ifdef OutputJSON
String json;
json = "{\"q0\":";
json = json + *getQ();
json = json + ",\"q1\":";
json = json + *(getQ()+1);
json = json + ",\"q2\":";
json = json + *(getQ()+2);
json = json + ",\"q3\":";
json = json + *(getQ()+3);
json = json + "}";
Serial.println(json);
#endif
Serial.print(Guidance.Yaw);
Serial.print("\t");
Serial.print(Guidance.Pitch);
Serial.print("\t");
Serial.println(Guidance.Roll);
/*
#ifdef SensorOutput
Serial.print(Guidance.Yaw);
Serial.print("\t");
Serial.print(Guidance.Pitch);
Serial.print("\t");
Serial.println(Guidance.Roll);
#endif
*/
}

bool setupSensors() {
  int error = Guidance.SensorDetect(MPU);
  if (error == 0){
  Serial.println("MPU is found");
  delay(1000);
  }
  else{
  Serial.println("MPU not found");
  }
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
  Wire.beginTransmission(MPU);
  Wire.write(0x6A);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
  Wire.beginTransmission(MPU);
  Wire.write(0x37);  // PWR_MGMT_1 register
  Wire.write(0x02);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
  Wire.beginTransmission(Ahk);
  error = Wire.endTransmission();
  if (error == 0){
  Serial.println("AHK is found");
  }
  Wire.beginTransmission(Ahk); //open communication with AHK Magnetometer
  Wire.write(0x0A); //select mode register
  Wire.write(0x16); //continuous measurement mode and 16bit resolutin
  Wire.endTransmission();

  Wire.beginTransmission(MPU); //Set Acceleration to +-16g
  Wire.write(0x1C);//Register
  Wire.write(0x18);//Afs = 3
  Wire.endTransmission(); 
  return true;
}

void readGyroAccel(){
  //Start Recieving Data
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,14,true);  // request a total of 14 registers
  rAcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  rAcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  rAcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  rTmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  rGyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  rGyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  rGyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  AcX = (float) rAcX;//Convert all the int16_t's to floats for input
  AcY = (float) rAcY;
  AcZ = (float) rAcZ;
  Tmp = (float) rTmp;
  GyX = (float) rGyX;
  GyY = (float) rGyY;
  GyZ = (float) rGyZ;

}

void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}

void readMag()
{
  uint8_t ST1;
  do
  {
   // I2Cread(MAG_ADDRESS,0x02,1,&ST1);
    Wire.beginTransmission(MAG_ADDRESS);
    Wire.write(0x02);
    Wire.endTransmission();
    Wire.requestFrom(MAG_ADDRESS, 1);
    ST1 = Wire.read();
    Serial.println(".");
  }
  while (!(ST1&0x01));

  // Read magnetometer data  
  uint8_t Mag[7];  
  //I2Cread(MAG_ADDRESS,0x03,7,Mag);
    Wire.beginTransmission(MAG_ADDRESS);
    Wire.write(0x03);
    Wire.endTransmission();
    Wire.requestFrom(MAG_ADDRESS, 7);
    mx=Wire.read()<<8|Wire.read();   
    my=Wire.read()<<8|Wire.read();
    mz=Wire.read()<<8|Wire.read();
  MgX = (float) mx; //Convert all the int16_t's to floats for input
  MgY = (float) my;
  MgZ = (float) mz;
}

