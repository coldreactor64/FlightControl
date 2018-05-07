#include <PID_v1.h>
#include<Wire.h>
#include <SPI.h>
#include "Filters.h"
#include "Guidance.h"

//#define OutputYawPitchRoll
//#define OutputJSON
//#define OutputQuat
#define SensorOutput
//#define CSVOutput

Guidance *GNCGuidance;
int MPU6050HMC = 0;
int MPU9250 = 1;
void setup(){
Serial.begin(250000);
pinMode(LED_BUILTIN, OUTPUT);
//Serial.println("Sensors Started");
GNCGuidance = new Guidance(MPU9250); // Make it 1 for MPU9250, 0 for MPU6050/HMC5883L
}


void loop(){
/*Do the actual filtering and Guidance*/
GNCGuidance->PIDGuidance();

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

#ifdef SensorOutput
Serial.print(GNCGuidance->Yaw);
Serial.print("\t");
Serial.print(GNCGuidance->Pitch);
Serial.print("\t");
Serial.print(GNCGuidance->Roll);
Serial.print("\t");
Serial.print(GNCGuidance->DeltaT);
Serial.println("\t");
#endif


#ifdef CSVOutput
long timeCurrent = millis();

Serial.print(GNCGuidance->Yaw);
Serial.print(",");
Serial.print(GNCGuidance->Pitch);
Serial.print(",");
Serial.print(GNCGuidance->Roll);
Serial.print(",");
Serial.print(timeCurrent);
Serial.println("\t");
#endif

}

