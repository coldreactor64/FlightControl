#include <PID_v1.h>
#include<Wire.h>
#include <SPI.h>
#include "Filters.h"
#include "Guidance.h"


//#define SensorOutput
Guidance *Guidance1;

void setup(){
Serial.begin(9600);
pinMode(LED_BUILTIN, OUTPUT);
//Serial.println("Sensors Started");
Guidance1 = new Guidance(1); // Make it 1 for MPU9250, 0 for MPU6050/HMC5883L
}


void loop(){

/*Do the actual filtering and Guidance*/
Guidance1->PIDGuidance();

#ifdef SensorOutput
Serial.print(Guidance1->Yaw);
Serial.print("\t");
Serial.print(Guidance1->Pitch);
Serial.print("\t");
Serial.print(Guidance1->Roll);
Serial.print("\t");
Serial.print(Guidance1->DeltaT);
Serial.println("\t");
#endif

}
