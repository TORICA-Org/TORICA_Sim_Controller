#include <Arduino.h>
//#include <TORICA_ICS.h>

//RUN(30)ピンをLOWにすることでリセット可能
#define READY_LED 22

//プロトタイプ宣言
long LoadCell_Read(int pin_slk, int pin_dout);
long LoadCell_Averaging(long adc,char num);
float LoadCell_getGram(char num);

// 使用しているロードセルは定格出力0.1mV 定格容量50kg （秋月のサンプルには選択肢がないので注意）
#define OUT_VOL 0.001f //定格出力 [V]
#define LOAD 50000.0f //定格容量 [g]

float offset[4];

//forward, forward, backward, backward
int pin_slk[4] = {2, 4, 6, 8};
int pin_dout[4] = {3, 5, 7, 9}; 

//TORICA_ICSインスタンス化
//TORICA_ICS maneuver(&Serial1);

//=====サーボのポジションのグローバル変数=====//

//servo_pos:
// (L)30°<=> 0°(N)<=> -30°(R)
//   8389<=> 7500 <=> 6611

//以下のどちらかをコメントアウト

int servo_pos = 7500; //こちらでは最適化される影響で値が更新されにくい可能性あり
//volatile int servo_pos = 7500; //こちらでは最適化を抑制するため，確実に更新されるが推奨されない

//=====================================//

void setup() {
  //pinMode設定（サンプルのAE_HX711_Initに対応）
  for(int i = 0; i < 4; ++i){
    pinMode(pin_slk[i],OUTPUT);
    pinMode(pin_dout[i],INPUT);
  }

  //Reset（サンプルのAE_HX711_Resetに対応）
  for(int i = 0; i < 1; ++i){
    digitalWrite(pin_slk[i],1);
    delayMicroseconds(100);
    digitalWrite(pin_slk[i],0);
    delayMicroseconds(100); 
  }
  
  for(int i = 0; i < 4; ++i){
    offset[i] = AE_HX711_getGram(30,2*i,2*i+1);
  } 
  
  //ピン割り当て
  pinMode(LED_BUILTIN, OUTPUT); // 動作確認用LED
  pinMode(READY_LED, OUTPUT);
  
  //デバッグ用シリアルを開始
  Serial.begin(115200);
  
  //ICS用UARTを開始
  Serial1.setTX(12);
  Serial1.setRX(13);
  Serial1.begin(115200);

  delay(5);
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(READY_LED, HIGH);
}

void loop() {
  float data[5];
  char trans_data[100];

  for(int i = 0; i < 4; ++i) {
    data[i] = (AE_HX711_getGram(10, pin_slk[i], pin_dout[i])-offset[i])*0.6125;
  }

  data[4] = (float)servo_pos;

  sprintf(trans_data,"%f,%f,%f,%f,%f\n",data[0],data[1],data[2],data[3],data[4]);
  Serial.print(trans_data);
  Serial.println();
  
  delay(10);
}

long AE_HX711_Read(int pin_slk, int pin_dout) {
  long data=0;
  while(digitalRead(pin_dout)!=0);
  delayMicroseconds(10);
  for(int i=0;i<24;i++)
  {
    digitalWrite(pin_slk,1);
    delayMicroseconds(5);
    digitalWrite(pin_slk,0);
    delayMicroseconds(5);
    data = (data<<1)|(digitalRead(pin_dout));
  }
  //Serial.println(data,HEX);   
  digitalWrite(pin_slk,1);
  delayMicroseconds(10);
  digitalWrite(pin_slk,0);
  delayMicroseconds(10);
  return data^0x800000; 
}

long AE_HX711_Averaging(long adc,char num,int pin_slk, int pin_dout) {
  long sum = 0;
  for (int i = 0; i < num; i++) sum += AE_HX711_Read(pin_slk, pin_dout);
  return sum / num;
}

float AE_HX711_getGram(char num, int pin_slk, int pin_dout) {
  #define HX711_R1  20000.0f
  #define HX711_R2  8200.0f
  #define HX711_VBG 1.25f
  #define HX711_AVDD      4.2987f//(HX711_VBG*((HX711_R1+HX711_R2)/HX711_R2))
  #define HX711_ADC1bit   HX711_AVDD/16777216 //16777216=(2^24)
  #define HX711_PGA 128
  #define HX711_SCALE     (OUT_VOL * HX711_AVDD / LOAD *HX711_PGA)
  
  float data;

  //data = AE_HX711_Averaging(AE_HX711_Read(pin_slk,pin_dout),num,pin_slk,pin_dout)*HX711_ADC1bit; 
  data = AE_HX711_Read(pin_slk,pin_dout)*HX711_ADC1bit;
  
  //Serial.println( HX711_AVDD);   
  //Serial.println( HX711_ADC1bit);   
  //Serial.println( HX711_SCALE);   
  //Serial.println( data);   
  data =  data / HX711_SCALE;

  return data;
}

//====以下，ラダー取得のための処理====

void setup1() {}

void loop1() {
  String str = Serial1.readStringUntil('\n');
  servo_pos = str.toInt();
}
