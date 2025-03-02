#include <Arduino.h>
#include <TORICA_ICS.h>

//RUN(30)ピンをLOWにすることでリセット可能

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
TORICA_ICS maneuver;

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
  pinMode(LED_BUILTIN, OUTPUT);//動作確認用LED
  
  //デバッグ用シリアルを開始
  Serial.begin(115200);
  
  //ICS用UARTを開始
  Serial1.setTX(12);
  Serial1.setRX(13);
  Serial1.begin(115200);

  delay(5);
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(22, HIGH); //READY_LED
}

void loop() {
  float data[5];
  char trans_data[100];

  for(int i = 0; i < 4; ++i) {
    data[i] = (AE_HX711_getGram(10, pin_slk[i], pin_dout[i])-offset[i])*0.6125;
  }

  data[4] = (float)maneuver.read_Angle();

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

//====以下，スピーカー関連の処理====
#include <TORICA_UART.h>
TORICA_UART Sim_UART;

enum {
  FAST,
  NORMAL,
  SLOW
} speed_level = NORMAL;

enum {
 PLATFORM,
 HIGH_LEVEL,
 MID_LEVEL,
 LOW_LEVEL 
} flight_phase = PLATFORM;

float airspeed = 0;
float altitude = 0;

void setup1() {
  pinMode(21, OUTPUT); //O_SPK
}

void loop1() {
  int readnum = Sim_UART.readUART();
  int sim_data_num = 2;
  if (readnum == sim_data_num) {
    airspeed = Sim_UART.UART_data[0];
    altitude = Sim_UART.UART_data[1];
  }
  speed_level_check();
  determine_flight_phase();
  speaker();
}

void speed_level_check() {
  if (airspeed > 1.5) {
    speed_level = FAST;
  } else if (airspeed > 1.0) {
    speed_level = NORMAL;
  } else {
    speed_level = SLOW;
  }
}

void determine_flight_phase() {
  switch (flight_phase) {
    case PLATFORM:
      {
        static int over_urm_range_count = 0;
        if (filtered_under_urm_altitude_m.get() > 9.0) {
          over_urm_range_count++;
        } else {
          over_urm_range_count = 0;
        }
        bool over_urm_range = false;
        // 超音波が測定不能な状態が2秒以上続いたとき
        if (over_urm_range_count >= 100) {
          over_urm_range = true;
        }
        // 気圧センサにより下降したと判断したとき
        bool descending = estimated_altitude_lake_m < 10.2;
        if ((over_urm_range || descending) && millis() > 15000) {
          flight_phase = TAKEOFF;
          takeoff_time_ms = millis();
        }
        if (over_urm_range && millis() > 15000) {
          SerialWireless.print("\n\nover_urm_range\n\n");
        }
        if (descending && millis() > 15000) {
          SerialWireless.print("\n\ndescending\n\n");
        }
      }
      break;
    case TAKEOFF:
      if (millis() - takeoff_time_ms > 3000) {
        flight_phase = HIGH_LEVEL;
      }
      break;
    case HIGH_LEVEL:
      // 超音波が測定できるようになったとき
      if (filtered_under_urm_altitude_m.get() < 1.0) {
        flight_phase = MID_LEVEL;
      }
      break;
    case MID_LEVEL:
      // 高度が1m以下になったとき
      if (filtered_under_urm_altitude_m.get() < 0.3) {
        flight_phase = LOW_LEVEL;
      }
      break;
    case LOW_LEVEL:
      break;
    default:
      break;
  }
}

void speaker() {

  static int sound_freq = 440;
  static int spk_flag = 0;
  static uint32_t speaker_last_change_time = millis();
  static uint32_t  sound_duration = 100;  //音が出ている時間

  switch (airspeed) {
    case SLOW:
      sound_freq = 440;
      break;
    case NORMAL:
      sound_freq = 880;
      break;
    case FAST:
      sound_freq = 1320;
      break;
    default:
      break;
  }

  int interval;
  switch (flight_phase) {
    case PLATFORM:
      interval = 1000;
      break;
    case HIGH_LEVEL:
      interval = 500;
      break;
    case MID_LEVEL:
      interval = 250;
      break;
    case LOW_LEVEL:
      interval = 125;
      break;
    default:
      interval = 0;
      break;
  }

  if (flight_phase != PLATFORM) {
    tone(O_SPK, sound_freq, 100);
    delay(interval);
  }
}
