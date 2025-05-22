//Analog UARTの宣言
#define SerialSD Serial1
#define SerialCondition Serial2

#include <TORICA_SD.h>
int cs_SD = 17;
TORICA_SD sd(cs_SD, false);
char SD_BUF[256];

#include <TORICA_UART.h>
TORICA_UART Condition_UART(&SerialCondition);

const int O_SPK = 14;
const int L_SD = 15;

volatile int flight_phase;
volatile int speed_level;
volatile float filtered_under_urm_altitude_m;

//float用のmap関数
float float_map (float val, float in_min, float in_max, float out_min, float out_max) {
  return out_min + (val - in_min) * (out_max - out_min) / (in_max - in_min);}


void setup() {
  SerialSD.setRX(1);
  SerialCondition.setRX(5);
  SerialSD.setFIFOSize(1024);
  SerialCondition.setFIFOSize(1024);
  SerialSD.begin(115200); 
  SerialCondition.begin(9600);
  Serial.begin(115200);

  pinMode(L_SD,OUTPUT);
  pinMode(25,OUTPUT);

  sd.begin();

}

void setup1(){

}

const int readUART_BUF_SIZE = 256;
char readUART_BUF[256];
void loop() {
  while (SerialSD.available()) {
    digitalWrite (L_SD,!digitalRead(L_SD));
    int read_length = SerialSD.available();
    if (read_length >= readUART_BUF_SIZE - 1) {
      read_length = readUART_BUF_SIZE - 1;
    }

    SerialSD.readBytes(readUART_BUF, read_length);
    readUART_BUF[read_length] = '\0';

    sd.add_str(readUART_BUF);
  
    if (!SerialSD.available()) {
      delay(1);
    }
  }
  
  digitalWrite(25,HIGH);
  int readnum = Condition_UART.readUART();
  int condition_data_num = 3;
  if (readnum == condition_data_num) {
    flight_phase = Condition_UART.UART_data[0];
    speed_level = Condition_UART.UART_data[1];
    filtered_under_urm_altitude_m = Condition_UART.UART_data[2];
    //Serial.print(filtered_under_urm_altitude_m);
  }
  digitalWrite(25,LOW);


  sd.flash();
  //Serial.println(readnum);


}

void loop1() {
  static int sound_freq = 440;
  //static int last_flight_phase = -1;
  static int spk_flag = 0;
  static uint32_t speaker_last_change_time = millis();
  static uint32_t sound_duration = 100; // 音が出ている時間

  switch (speed_level) {
    //speed_level 0=FAST(機体速度8m/s~) 1=NORMAL(機体速度3~8m/s) 2=SLOW(機体速度~3m/s)
    case 0: //FAST
      sound_freq = 440;
      break;
    case 1: //NORMAL
      sound_freq = 880;
      break;
    case 2: //SLOW
      sound_freq = 1320;
      break;
    default:
      break;
  }

  float interval;
  float altitude_max = 1.0;
  float altitude_min = 0.0;
  switch (flight_phase) {
    //flight_phase 0=PLATFORM(TAKEOFF判断　(over_urm_range || descending) && (x_accelerated || y_accelerated) && millis() > 10000)
    //flight_phase 1=HIGH_LEVEL(超音波高度1m~) 2=MID_LEVEL(超音波高度0.3~1m) 3=LOW_LEVEL(超音波高度~0.3m)
    case 0: //PLATFORM
      interval = 1000;
      break;  
    case 1: //HIGH_LEVEL
      interval = 900;
      break;
    case 2: //MID_LEVEL
      interval = float_map(filtered_under_urm_altitude_m, altitude_min, altitude_max, 125, 900);
      //Serial.println(interval);
      break;
    case 3: //LOW_LEVEL
      interval = float_map(filtered_under_urm_altitude_m, altitude_min, altitude_max, 125, 900);
      //Serial.println(interval);

      break;
    default:
      interval = 0;
      break;
  }

  uint32_t current_time = millis();
  uint32_t off_duration = interval - sound_duration;
  
  //Serial.print("flight_phase:");
  //Serial.println(flight_phase);
  //Serial.print("spk_flag:");
  //Serial.println(spk_flag);
  
  


  if (flight_phase != 0) {
    if (spk_flag == 0 && (current_time - speaker_last_change_time) > off_duration) {
      tone(O_SPK, sound_freq);
      speaker_last_change_time = current_time;
      spk_flag = 1;
    } else if (spk_flag == 1 && (current_time - speaker_last_change_time) > sound_duration) {
      noTone(O_SPK);
      speaker_last_change_time = current_time;
      spk_flag = 0;
    }
  }

  
  //last_flight_phase = flight_phase;


}

