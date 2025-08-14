//4/1Simテスト用
//こちらは周期的にラダーが左右に切れます
//ICSを使わず，UARTでの通信です

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

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);//動作確認用LED

  //ラダー用UARTを開始
  Serial1.begin(115200);
  
  //デバッグ用シリアルを開始
  Serial.begin(115200);
  Serial.println("SERIAL READY");

  digitalWrite(LED_BUILTIN, HIGH);
  delay(5);
}

void loop() {
  static int count = 0;
  if (2*PI < count*0.1) {
    count = 0;
  }
  int servo_pos = 7500+(4000*sin(count*0.1));
  
  Serial.println(servo_pos);
  Serial1.println(servo_pos);
  
  delay(10);
}
