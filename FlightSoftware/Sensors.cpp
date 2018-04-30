#include "Sensors.h"

#define MPU 0x68
#define HMC 0x1E
#define AK 0x0C

/*****MPU6050 and HMC5883L on AUX I2C/or same I2C Line******/

MPU6050HMC::MPU6050HMC(){ //Setup the Sensors to be read
while (!(startSensor()));
};

bool MPU6050HMC::startSensor(){ //Initialize the sensors
Wire.begin();
I2CwriteByte(MPU, 0x6B, 0);   // PWR_MGMT_1 register set to zero (wakes up the MPU-6050)
I2CwriteByte(MPU, 0x6A, 0);   //Enable Slave mode prerequrisite to I2C Bypass mode
I2CwriteByte(MPU, 0x37, 0x02);//I2C_Bypass Enable
I2CwriteByte(HMC, 0x02, 0x00);//continuous measurement mode enable
I2CwriteByte(MPU, 0x1C, 0x18);//Set Acceleration to +-16g by setting AFS = 3
digitalWrite(LED_BUILTIN, HIGH);
return true;
};

void MPU6050HMC::Update(){ //read the sensors and put them into the filters and return the values
Now = micros();
deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
lastUpdate = micros();
/****************Read Gyro and Accelerometer from MPU6050***************/
I2CReadNByte(MPU,0x3B,14,RawData);
  rAcX=-(RawData[0]<<8 | RawData[1]);
  rAcY=-(RawData[2]<<8 | RawData[3]);
  rAcZ=RawData[4]<<8 | RawData[5];
  rGyX=-(RawData[8]<<8 | RawData[9]);
  rGyY=-(RawData[10]<<8 | RawData[11]);
  rGyZ=RawData[12]<<8 | RawData[13];
  AcX = (float) rAcX;//Convert all the int16_t's to floats for input
  AcY = (float) rAcY;
  AcZ = (float) rAcZ;
  Tmp = (float) rTmp;
  GyX = (float) rGyX;
  GyY = (float) rGyY;
  GyZ = (float) rGyZ;
/****************Read Magnetometer***************/
I2CReadNByte(HMC,0x03,7,RawMagData);
    rMgX=-(RawMagData[3]<<8 | RawMagData[2]);
    rMgY=-(RawMagData[1]<<8 | RawMagData[0]);
    rMgZ=-(RawMagData[5]<<8 | RawMagData[4]);
    MgX = (float) rMgX; //Convert all the int16_t's to floats for input
    MgY = (float) rMgY;
    MgZ = (float) rMgZ;
/****************Filter Data***************/
MahonyQuaternionUpdate(AcX, AcY, AcZ, GyX, GyY, GyZ, MgX, MgY, MgZ, deltat);
};





/*****************************MPU9250 Class*******************************/


MPU9250::MPU9250(){ //Setup the Sensors to be read
while (!(startSensor()));
};

bool MPU9250::startSensor(){ //Initialize the sensors
Wire.begin();
I2CwriteByte(MPU, 0x6B, 0);   // PWR_MGMT_1 register set to zero (wakes up the MPU-6050)
I2CwriteByte(MPU, 0x6A, 0);   //Enable Slave mode prerequrisite to I2C Bypass mode
I2CwriteByte(MPU, 0x37, 0x02);//I2C_Bypass Enable
I2CwriteByte(AK,0x0A,0x16);//Set Continuous Measurement Mode
I2CwriteByte(MPU, 0x1C, 0x18);//Set Acceleration to +-16g by setting AFS = 3
Serial.println("Sensors Started");
return true;
};

void MPU9250::Update(){ //read the sensors and put them into the filters and return the values
Now = micros();
deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
lastUpdate = micros();
/****************Read Gyro and Accelerometer from MPU6050***************/
I2CReadNByte(MPU,0x3B,14,RawData);
  rAcX=-(RawData[0]<<8 | RawData[1]);
  rAcY=-(RawData[2]<<8 | RawData[3]);
  rAcZ=RawData[4]<<8 | RawData[5];
  rGyX=-(RawData[8]<<8 | RawData[9]);
  rGyY=-(RawData[10]<<8 | RawData[11]);
  rGyZ=RawData[12]<<8 | RawData[13];
  AcX = (float) rAcX;//Convert all the int16_t's to floats for input
  AcY = (float) rAcY;
  AcZ = (float) rAcZ;
  Tmp = (float) rTmp;
  GyX = (float) rGyX;
  GyY = (float) rGyY;
  GyZ = (float) rGyZ;
/****************Read Magnetometer***************/
do
{
I2CReadNByte(AK,0x02,1,&ST1);
}
while (!(ST1&0x01));

I2CReadNByte(AK,0x03,7,RawMagData);
    rMgX=-(RawMagData[3]<<8 | RawMagData[2]);
    rMgY=-(RawMagData[1]<<8 | RawMagData[0]);
    rMgZ=-(RawMagData[5]<<8 | RawMagData[4]);
    MgX = (float) rMgX; //Convert all the int16_t's to floats for input
    MgY = (float) rMgY;
    MgZ = (float) rMgZ;
/****************Filter Data***************/
MahonyQuaternionUpdate(AcX, AcY, AcZ, GyX, GyY, GyZ, MgX, MgY, MgZ, deltat);
};


void I2CReadNByte(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
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


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}
