//4/1Simテスト用
//ICSを使わず，UARTでの通信です
//5/6作成
//ニュートラル調整用可変抵抗省略版です

#include <Arduino.h>
//#include <IcsHardSerialClass.h>

//20250213:note
//入力値の変化は
//Lが減少
//Rが増加
//rudder:
//   L <=> R
//-500 <=> 500
//servo_pos:
//135°(L)<=>  30°<=> 0°(N)<=> -30°<=> -135°(R)
//11500  <=> 8389<=> 7500 <=> 6611<=> 3500

//ピン指定
#define POT_L 26 //ポテンショメータL
#define POT_R 27 //ポテンショメータR
#define OFFSET 28 //ニュートラル調整用可変抵抗

//サーボモータの初期設定
const byte EN_PIN = 29; //ENピンの指定
const long BAUDRATE = 115200; //通信速度
const int TIMEOUT = 10; //タイムアウト

//感度
const float sensitivity = 1.0;

//操舵角
const float rudderAngleDeg = 15.0; // 5/6 守山さんの意見により試験的に設定(30→15)

//ゼロ入力
float potL_default = 0.0; //L
float potR_default = 0.0; //R

//インスタンス化およびENピン(2番ピン)とUARTの指定
//IcsHardSerialClass krs(&Serial1,EN_PIN,BAUDRATE,TIMEOUT);

//プロトタイプ宣言
float float_map (float val, float in_min, float in_max, float out_min, float out_max);

void setup() {
  //ピン割り当て
  pinMode(POT_L, INPUT);
  pinMode(POT_R, INPUT);
  pinMode(OFFSET, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);//動作確認用LED
  
  //analogReadの分解能を12bitに設定
  analogReadResolution(12);
  
  //サーボモータの通信初期設定
  //krs.begin();

  Serial1.begin(115200);
  
  //デバッグ用シリアルを開始
  Serial.begin(115200);
  Serial.println("SERIAL READY");
  
  //電源投入時にポテンショメータの値を複数回読み平均をとる
  for (int i = 0; i < 5; i++) {
    potL_default += analogRead(POT_L);
    potR_default += analogRead(POT_R);
  }
  potL_default /= 5;
  potR_default /= 5;

  delay(5);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  
  //不感帯（この値は左右それぞれ適用する）
  float dead_zone = 50.0;
  
  //サーボ
  //float servo_neutral = (float)(7500 + analogRead(OFFSET) - 2000); //ニュートラル
  float servo_neutral = 7500.0;
  float servo_min = servo_neutral - 4000.0*rudderAngleDeg/135.0; //最小値
  float servo_max = servo_neutral + 4000.0*rudderAngleDeg/135.0; //最大値
  
  Serial.print("NEUTRAL: ");
  Serial.println(servo_neutral);
  
  //ポテンショメータの値
  float potL = (float)analogRead(POT_L); //L
  float potR = (float)analogRead(POT_R); //R
  
  Serial.print("POT_L/ POT_R: ");
  Serial.print(potL);
  Serial.print("/ ");
  Serial.println(potR);

  //ポテンショメータの値を左右統合
  float rudder = 0.0; //統合先変数
  if (potL < potL_default - dead_zone) {
    rudder += potL - (potL_default - dead_zone);
  }
  if (potR_default + dead_zone < potR) {
    rudder += potR - (potR_default + dead_zone);
  }
  rudder *= -1; //LR反転
  rudder *= sensitivity; //感度適用

  if (rudder < -500) {
    rudder = -500;
  }
  else if (500 < rudder) {
    rudder = 500;
  }
  
  Serial.print("RUDDER: ");
  Serial.println(rudder);

  //サーボの値域に変換
  float servo_pos = servo_neutral;
  if (rudder < 0.0) {
    servo_pos = float_map(rudder, -500, 0, servo_min, servo_neutral);
  }
  else if (0.0 < rudder) {
    servo_pos = float_map(rudder, 0, 500, servo_neutral, servo_max);
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

//map関数のfloat型版
//val: 変換したい値
//in_min: 変換前最小値
//in_max: 変換前最大値
//out_min: 変換後最小値
//out_min: 変換後最大値
float float_map (float val, float in_min, float in_max, float out_min, float out_max) {
  return out_min + (val - in_min)*(out_max - out_min)/(in_max - in_min);
}
