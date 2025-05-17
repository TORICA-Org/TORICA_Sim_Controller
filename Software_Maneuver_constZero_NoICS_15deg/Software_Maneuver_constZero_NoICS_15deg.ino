// 最終更新日: 20250517

// 直感的な入力量評価のため，ラダー入力を百分率に変更
// 挙動を安定させるため，フル入力時・無入力時の値を定数化
// ラダーの計算に関する部分をrudderCalculatorに分離

// 入力値の変化は
// Lが減少
// Rが増加
// servo_pos:
// 135°(L)<=>  30°<=> -15°<=> 0°(N)<=> -15°<=> -30°<=> -135°(R)
// 11500  <=> 8389<=> 7944<=> 7500 <=> 7056<=> 6611<=> 3500

#include <Arduino.h>
//#include <IcsHardSerialClass.h>
#include "RudderCalculator.h"

// ピン指定
#define POT_L 26 // ポテンショメータL
#define POT_R 27 // ポテンショメータR
#define OFFSET 28 // ニュートラル調整用可変抵抗

// サーボモータの初期設定
const byte EN_PIN = 29; // ENピンの指定
const long BAUDRATE = 115200; // 通信速度
const int TIMEOUT = 10; // タイムアウト

// 感度
const float sensitivity = 1.5; // フルで切るのは構造的に厳しそうなので多少高感度に設定

// 操舵角
const float rudderAngleDeg = 15.0; // 5/6 守山さんの意見により試験的に設定(30→15)

// 不感帯割合
const float deadzone = 0.01;

// 各種定数
const float potL_zero = 1230.0;
const float potL_full = 790.0;
const float potR_zero = 2710.0;
const float potR_full = 3230.0;

// インスタンス化およびENピン(2番ピン)とUARTの指定
//IcsHardSerialClass krs(&Serial1,EN_PIN,BAUDRATE,TIMEOUT);

RudderCalculator rudder(POT_L, POT_R, OFFSET);

void setup() {
  // ピン割り当て
  pinMode(LED_BUILTIN, OUTPUT); // 動作確認用LED
  
  // サーボモータの通信初期設定
  //krs.begin();
  Serial1.begin(115200);
  
  // デバッグ用シリアルを開始
  Serial.begin(115200);
  Serial.println("SERIAL READY");

  rudder.setPotRange(potL_zero, potL_full, potR_zero, potR_full);
  rudder.setSensitivity(sensitivity);
  rudder.setDeadzone(deadzone);
  rudder.setRudderAngleDeg(rudderAngleDeg);
  rudder.begin();

  delay(5);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  int servo_pos = rudder.getPos();

  Serial.print("SERVO_POS: ");
  Serial.println(servo_pos);
  
  // サーボ（ID:0）をservo_posだけ駆動
  //krs.setPos(0, (int)servo_pos);
  Serial1.println((int)servo_pos);
  delay(10);
}
