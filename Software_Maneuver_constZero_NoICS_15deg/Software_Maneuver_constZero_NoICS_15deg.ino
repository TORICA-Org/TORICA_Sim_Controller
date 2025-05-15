// 5/15更新
// 直感的な入力量評価のため，ラダー入力を百分率に変更
// 挙動を安定させるため，フル入力時・無入力時の値を定数化

// ICSを使わず，UARTでの通信です

#include <Arduino.h>
//#include <IcsHardSerialClass.h>

// 20250213:note
// 入力値の変化は
// Lが減少
// Rが増加
// rudder:
//    L <=> R
// -500 <=> 500
// servo_pos:
// 135°(L)<=>  30°<=> -15°<=> 0°(N)<=> -15°<=> -30°<=> -135°(R)
// 11500  <=> 8389<=> 7944<=> 7500 <=> 7056<=> 6611<=> 3500

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

// 各種定数
const float potL_zero = 1230.0;
const float potL_full = 790.0;
const float potR_zero = 2710.0;
const float potR_full = 3230.0;

// インスタンス化およびENピン(2番ピン)とUARTの指定
//IcsHardSerialClass krs(&Serial1,EN_PIN,BAUDRATE,TIMEOUT);

// プロトタイプ宣言
float float_map (float val, float in_min, float in_max, float out_min, float out_max);

void setup() {
  // ピン割り当て
  pinMode(POT_L, INPUT);
  pinMode(POT_R, INPUT);
  pinMode(OFFSET, INPUT);
  pinMode(LED_BUILTIN, OUTPUT); // 動作確認用LED
  
  // analogReadの分解能を12bitに設定
  analogReadResolution(12);
  
  // サーボモータの通信初期設定
  //krs.begin();

  Serial1.begin(115200);
  
  // デバッグ用シリアルを開始
  Serial.begin(115200);
  Serial.println("SERIAL READY");

  delay(5);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {  
  // 不感帯（この値は左右それぞれ適用する）
  float dead_zone = 1.0; // 百分率

  // サーボ
  float servo_neutral = (float)(7500 + analogRead(OFFSET) - 2000); // ニュートラル
  float servo_min = servo_neutral - 4000.0*rudderAngleDeg/135.0; // 最小値
  float servo_max = servo_neutral + 4000.0*rudderAngleDeg/135.0; // 最大値

  Serial.print("NEUTRAL: ");
  Serial.println(servo_neutral);

  // ポテンショメータの値
  float potL = (float)analogRead(POT_L); // L
  float potR = (float)analogRead(POT_R); // R

  // 百分率に変換 + 感度適用
  float potL_input = float_map(potL, potL_zero, potL_full, 0, 100)*sensitivity;
  float potR_input = float_map(potR, potR_zero, potR_full, 0, 100)*sensitivity;

  Serial.print("POT_L/ POT_R: ");
  Serial.print(potL_input);
  Serial.print("%/ ");
  Serial.print(potR_input);
  Serial.println("%");

  //ポテンショメータの値を左右統合
  float rudder = 0.0; //統合先変数
  if (potL_input > dead_zone) {
    rudder += potL_input - dead_zone;
  }
  if (potR_input > dead_zone) {
    rudder -= potR_input - dead_zone;
  }

  Serial.print("RUDDER: ");
  Serial.print(rudder);
  Serial.println("%");

  //サーボの値域に変換
  float servo_pos = servo_neutral;
  if (rudder < 0.0) {
    servo_pos = float_map(rudder, -100, 0, servo_min, servo_neutral);
  }
  else if (0.0 < rudder) {
    servo_pos = float_map(rudder, 0, 100, servo_neutral, servo_max);
  }

  //サーボの最小値・最大値を判定
  if (servo_pos < servo_min) {
    servo_pos = servo_min;
  }
  else if (servo_max < servo_pos) {
    servo_pos = servo_max;
  }
  
  Serial.print("SERVO_POS: ");
  Serial.println(servo_pos);
  
  //サーボ（ID:0）をservo_posだけ駆動
  //krs.setPos(0, (int)servo_pos);
  Serial1.println((int)servo_pos);
  delay(10);
}

//map関数のfloat型版 + 最大・最小値判定
//val: 変換したい値
//in_min: 変換前最小値
//in_max: 変換前最大値
//out_min: 変換後最小値
//out_min: 変換後最大値
float float_map (float val, float in_min, float in_max, float out_min, float out_max) {
  float out = out_min + (val - in_min)*(out_max - out_min)/(in_max - in_min);
  if (out < out_min) {
    out = out_min;
  }
  else if (out_max < out) {
    out = out_max;
  }
  return out;
}
