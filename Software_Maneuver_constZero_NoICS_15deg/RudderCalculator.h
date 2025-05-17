// ラダー計算に関する定義

#ifndef RUDDERCALCULATOR_H
#define RUDDERCALCULATOR_H

#include <Arduino.h>

class RudderCalculator {
  public: 
    RudderCalculator(int potL, int potR, int potN);
    int setPotRange(float potL_zero, float potL_full, float potR_zero, float potR_full);
    int setSensitivity(float sensitivity);
    int setDeadzone(float deadzone);
    int setRudderAngleDeg(float rudderAngleDeg);
    int begin();
    float floatMap (float val, float in_min, float in_max, float out_min, float out_max, float sensitivity = 1.0, float deadzone = 0.0);
    int getPos();

  private:
    int _potL, _potR, _potN;
    float _potL_zero, _potL_full, _potR_zero, _potR_full;
    float _sensitivity, _deadzone, _rudderAngleDeg;
};

#endif // RUDDERCALCULATOR_H