// ラダー計算に関する実装

#include "RudderCalculator.h"

RudderCalculator::RudderCalculator (int potL, int potR, int potN) {
  _potL = potL;
  _potR = potR;
  _potN = potN;
}

int RudderCalculator::setPotRange (float potL_zero, float potL_full, float potR_zero, float potR_full) {
  _potL_zero = potL_zero;
  _potL_full = potL_full;
  _potR_zero = potR_zero;
  _potR_full = potR_full;
  return 1;
}

int RudderCalculator::setSensitivity (float sensitivity) {
  _sensitivity = sensitivity; // default: 1.0
  return 1;
}

int RudderCalculator::setDeadzone (float deadzone) {
  _deadzone = deadzone; // range: 0.0 ~ 1.0
  return 1;
}

int RudderCalculator::setRudderAngleDeg (float rudderAngleDeg) {
  _rudderAngleDeg = rudderAngleDeg;
  return 1;
}

int RudderCalculator::begin() {
  analogReadResolution(12); // analogReadの分解能を12bitに設定
  pinMode(_potL, INPUT);
  pinMode(_potR, INPUT);
  pinMode(_potN, INPUT);
  return 1;
}

// map関数のfloat型版 + 最大・最小値判定 + 感度適用 + 不感帯適用
// in_min: 変換前最小値
// in_max: 変換前最大値
// out_min: 変換後最小値
// out_max: 変換後最大値
float RudderCalculator::floatMap (float val, float in_min, float in_max, float out_min, float out_max, float sensitivity, float deadzone) {
  float _in_min = 0.0;
  if (in_min < in_max) {
    _in_min = in_min + (in_max - in_min) * deadzone;
  }
  else if (in_max < in_min) {
    _in_min = in_min - (in_max - in_min) * deadzone;
  }

  float _out = out_min + (val - _in_min)*(out_max - out_min)/(in_max - _in_min)*sensitivity;
  if (_out < out_min) {
    _out = out_min;
  }
  else if (out_max < _out) {
    _out = out_max;
  }

  return _out;
}


int RudderCalculator::getPos () {
  // サーボ
  float servo_neutral = (float)(7500 + analogRead(_potN) - 2000); // ニュートラル
  float servo_min = servo_neutral - 4000.0*_rudderAngleDeg/135.0; // 最小値
  float servo_max = servo_neutral + 4000.0*_rudderAngleDeg/135.0; // 最大値

  //Serial.print("NEUTRAL: ");
  //Serial.println(servo_neutral);

  // ポテンショメータの値
  float potL_val = (float)analogRead(_potL); // L
  float potR_val = (float)analogRead(_potR); // R

  // 百分率に変換 + 感度適用 + 不感帯適用
  float potL_percentage = floatMap(potL_val, _potL_zero, _potL_full, 0, 100, _sensitivity, _deadzone);
  float potR_percentage = floatMap(potR_val, _potR_zero, _potR_full, 0, 100, _sensitivity, _deadzone);

  //Serial.print("POT_L/ POT_R: ");
  //Serial.print(potL_percentage);
  //Serial.print("%/ ");
  //Serial.print(potR_percentage);
  //Serial.println("%");

  // ポテンショメータの値を左右統合
  float rudder = potL_percentage - potR_percentage;

  //Serial.print("RUDDER: ");
  //Serial.print(rudder);
  //Serial.println("%");

  // サーボの値域に変換
  float servo_pos = servo_neutral;
  if (rudder < 0.0) {
    servo_pos = floatMap(rudder, -100, 0, servo_min, servo_neutral);
  }
  else if (0.0 < rudder) {
    servo_pos = floatMap(rudder, 0, 100, servo_neutral, servo_max);
  }

  return servo_pos;
}