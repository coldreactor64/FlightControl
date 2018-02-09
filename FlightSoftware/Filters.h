
#include <Arduino.h>
void MadgwickQuaternionUpdate(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy,
                              int16_t gz, float mx, float my, float mz,
                              float deltat);
void MahonyQuaternionUpdate(int16_t Iax, int16_t Iay, int16_t Iaz, int16_t Igx, int16_t Igy,
                            int16_t Igz, float mx, float my, float mz,
                            float deltat);
const float * getQ();
const float * getYPR();
