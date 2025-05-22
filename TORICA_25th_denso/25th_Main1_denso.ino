//Analog UARTの宣言
#define SerialAir Serial1 //AirのUART
#define SerialUnder Serial2 //UnderのUART

//PIO UARTの宣言 
SerialPIO SerialWireless(2, 3); //GPS,TWELITE
SerialPIO SerialSD_ICS(10, 11); 
SerialPIO SerialCondition(12,13);

//GPS
#include <TinyGPSPlus.h>
TinyGPSPlus gps;

//SD送信用バッファ
char UART_SD[512];

//TWELITE送信用バッファ
char TWE_BUF[256];

//flight_phase,speed_level送信用バッファ
char UART_Condition[256];

//TORICA_UARTインスタンス化
#include <TORICA_UART.h>
TORICA_UART Under_UART(&SerialUnder);
TORICA_UART Air_UART(&SerialAir);
TORICA_UART SD_ICS_UART(&SerialSD_ICS);

//動作確認用LED
const int L_Air = 18;
const int L_Under = 19;
const int L_ICS = 20;

//ICS初期化
#include <TORICA_ICS.h>
TORICA_ICS ics(&SerialSD_ICS);

//I2Cパッケージ読み込み
#include <Wire.h>

//BNO055インスタンス化(あとでGeometry.hを使ったプログラムを考える)
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <Geometry.h>
using namespace Geometry;
using namespace BLA;
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire1);

//DPS310初期化
#include <Adafruit_DPS310.h>
Adafruit_DPS310 dps;
sensors_event_t temp_event, pressure_event;

//Flight-Phaseの定義
enum {
  PLATFORM,
  HIGH_LEVEL,
  MID_LEVEL,
  LOW_LEVEL
} flight_phase = PLATFORM;//"PLATFORM"で初期化

bool TAKEOFF = false;

enum {
  FAST,
  NORMAL,
  SLOW,
} speed_level = NORMAL;

// filtered:移動平均
// lake:対地高度, 無印:気圧基準海抜高度
// dps:気圧高度
// urm:超音波高度

const float const_platform_m = 10.6; //プラットフォームの高さ+機体の高さ

#include <TORICA_MoveAve.h>
// 対気速度
TORICA_MoveAve<5> filtered_airspeed_ms(0);

// 現在の気圧高度(気圧基準)
TORICA_MoveAve<5> filtered_main_dps_altitude_m(0);
TORICA_MoveAve<5> filtered_under_dps_altitude_m(0);
TORICA_MoveAve<5> filtered_air_dps_altitude_m(0);
// プラホの高度(気圧基準)
TORICA_MoveAve<50> main_dps_altitude_platform_m(0);
TORICA_MoveAve<50> under_dps_altitude_platform_m(0);
TORICA_MoveAve<50> air_dps_altitude_platform_m(0);


// 気圧センサを用いた信頼できる対地高度
// 3つの気圧高度にそれぞれ移動平均をとってプラホを10mとし，中央値をとった値
#include <QuickStats.h>
float dps_altitude_lake_array_m[3];
QuickStats dps_altitude_lake_m;

// 超音波高度(対地高度)
TORICA_MoveAve<3> filtered_under_urm_altitude_m(0);

#include<TORICA_MoveMedian.h>
// 気圧での対地高度と超音波での対地高度の差
// 100Hz(calculate)*4s = 400
TORICA_MoveMedian<400> altitude_dps_urm_offset_m(0);


// 気圧と超音波から推定した対地高度
float estimated_altitude_lake_m = const_platform_m;

//エアデータと機体下電装部の生存確認
bool air_is_alive = false;
bool under_is_alive = false;

uint32_t time_ms;

// ---- sensor data value  ----
//    data_マイコン名_センサー名_データ種類_単位
//main電装
//BNO055
volatile float data_main_bno_accx_mss = 0;
volatile float data_main_bno_accy_mss = 0;
volatile float data_main_bno_accz_mss = 0;
volatile float data_main_bno_qw = 0;
volatile float data_main_bno_qx = 0;
volatile float data_main_bno_qy = 0;
volatile float data_main_bno_qz = 0;
volatile float data_main_bno_roll = 0;
volatile float data_main_bno_pitch = 0;
volatile float data_main_bno_yaw = 0;
//DPS310
volatile float data_main_dps_pressure_hPa = 0;
volatile float data_main_dps_temperature_deg = 0;
volatile float data_main_dps_altitude_m = 0;
//GPS
volatile uint8_t data_main_gps_hour = 0;
volatile uint8_t data_main_gps_minute = 0;
volatile uint8_t data_main_gps_second = 0;
volatile uint8_t data_main_gps_centisecond = 0;
volatile double data_main_gps_latitude_deg = 0;
volatile double data_main_gps_longitude_deg = 0;
volatile double data_main_gps_altitude_m = 0;
volatile double data_main_gps_groundspeed_kmph = 0;

//Under電装部
volatile float data_under_dps_pressure_hPa = 0;
volatile float data_under_dps_temperature_deg = 0;
volatile float data_under_dps_altitude_m = 0;
volatile float data_under_urm_altitude_m = 0;

//Airdata電装部
volatile float data_air_dps_pressure_hPa = 0;
volatile float data_air_dps_temperature_deg = 0;
volatile float data_air_dps_altitude_m = 0;
volatile float data_air_sdp_differentialPressure_Pa = 0;
volatile float data_air_sdp_airspeed_ms = 0;
volatile float data_air_AoA_angle_deg = 0;
volatile float data_air_AoS_angle_deg = 0;
volatile float data_air_bno_accx_mss = 0;
volatile float data_air_bno_accy_mss = 0;
volatile float data_air_bno_accz_mss = 0;
volatile float data_air_bno_qw = 0;
volatile float data_air_bno_qx = 0;
volatile float data_air_bno_qy = 0;
volatile float data_air_bno_qz = 0;
volatile float data_air_bno_roll = 0;
volatile float data_air_bno_pitch = 0;
volatile float data_air_bno_yaw = 0;
//ICS基盤
volatile int data_ics_angle = 0;

// ----------------------------

void setup() {
  //LED初期化
  pinMode(L_ICS, OUTPUT);
  pinMode(L_Under, OUTPUT);
  pinMode(L_Air, OUTPUT);
  pinMode(25,OUTPUT);

  // USBケーブルを差した時の起動猶予時間
  for (int i = 0; i < 3; i++) {
    digitalWrite(L_ICS, HIGH);
    digitalWrite(L_Under, HIGH);
    digitalWrite(L_Air, HIGH);
    delay(400);
    digitalWrite(L_ICS, LOW);
    digitalWrite(L_Under, LOW);
    digitalWrite(L_Air, LOW);
    delay(100);
  }

  delay(2000);

  //UARTピン設定
  SerialUnder.setTX(4);
  SerialUnder.setRX(5);
  SerialAir.setTX(0);
  SerialAir.setRX(1);

  // UART初期化
  //SerialGPS.setFIFOSize(1024);
  SerialAir.setFIFOSize(1024);
  SerialUnder.setFIFOSize(1024);
  //SerialSupport_ICS.setFIFOSize(1024);

  //通信速度設定
  SerialWireless.begin(9600);  //GPS+TWELITE
  SerialSD_ICS.begin(115200);  //SD+ICS
  SerialAir.begin(460800);
  SerialUnder.begin(460800);
  SerialCondition.begin(9600); //Flight_phase+Speed_level
  Serial.begin(115200);
  SerialWireless.print("loading...\n\n");

  //I2C wire1ピン設定
  Wire1.setSDA(6);
  Wire1.setSCL(7);
  Wire1.begin();

  //DPS310初期化
  Wire1.setClock(400000);
  //DPS310と通信できているかの判定 DPS310が通信できていない場合はL_Airが点滅する
  if (!dps.begin_I2C(0x77, &Wire1)) {
    Serial.println("Failed to find DPS");
    while (1) {
      SerialWireless.println("Failed to find DPS");
      digitalWrite(L_Air, HIGH);
      delay(100);
      digitalWrite(L_Air, LOW);
      delay(100);
    }
  }
  dps.configurePressure(DPS310_32HZ, DPS310_16SAMPLES);
  dps.configureTemperature(DPS310_32HZ, DPS310_2SAMPLES);
  Serial.println("DPS OK!");

  // BNO055初期化　BNO055が通信できていない場合はL_Underが点滅する
  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1) {
      digitalWrite(L_Under, HIGH);
      delay(100);
      digitalWrite(L_Under, LOW);
      delay(100);
    }
  }

  // センサー・各基板の起動を待機
  for (int i = 0; i < 3; i++) {
    digitalWrite(L_ICS, HIGH);
    digitalWrite(L_Under, HIGH);
    digitalWrite(L_Air, HIGH);
    delay(100);
    digitalWrite(L_ICS, LOW);
    digitalWrite(L_Under, LOW);
    digitalWrite(L_Air, LOW);
    delay(400);
  }
}


void loop() {
  uint32_t ISR_now_time = millis(); //100Hzで測定
  static uint32_t ISR_last_time = 0;
  if (ISR_now_time - ISR_last_time >= 10) {
    ISR_last_time = millis();
    func_100Hz();
  }

  // テレメトリダウンリンク(コア1で処理するとダウンリンクするタイミングでSDの書き込み周期がずれてしまうのでコア2で行う)
  //TWE_downlink();

  //Serial.println(filtered_under_urm_altitude_m.get());


}

void setup1(){
  
}

void loop1(){
  TWE_downlink();
}


//各関数の実行
void func_100Hz() {
  //uint32_t time_us = micros();

  // GPS・操舵角・機体下電装部・エアデータ電装部読み取り
  polling_UART();

  // フライトフェーズ判断(メインの加速度も測定)
  determine_flight_phase();

  // 気圧センサ×3・超音波センサから高度推定
  calculate_altitude();

  // SDに記録
  send_SD();

  // Flight_phaseとSpeed_levelを2層目に送る
  send_Condition();

  // if (micros() - time_us > 9900) {  //MAX10000=100Hz
  //   Serial.print("ISR100Hz_overrun!!!");
  // }
  // Serial.print("ISR_us:");
  // Serial.println(micros() - time_us);

}

void polling_UART() {
  //ICS
  data_ics_angle = ics.read_Angle();
  if (data_ics_angle > 0) {
    digitalWrite(L_ICS, !digitalRead(L_ICS));
  }

  //UnderSide
  static unsigned long int last_under_time_ms = 0;
  int readnum = Under_UART.readUART();
  int under_data_num = 4;
  if (readnum == under_data_num) {
    last_under_time_ms = millis();
    digitalWrite(L_Under, !digitalRead(L_Under));
    data_under_dps_pressure_hPa = Under_UART.UART_data[0];
    data_under_dps_temperature_deg = Under_UART.UART_data[1];
    data_under_dps_altitude_m = Under_UART.UART_data[2];
    data_under_urm_altitude_m = Under_UART.UART_data[3];
    filtered_under_dps_altitude_m.add(data_under_dps_altitude_m);
    if (flight_phase == PLATFORM) {
      under_dps_altitude_platform_m.add(data_under_dps_altitude_m);
    }
    filtered_under_urm_altitude_m.add(data_under_urm_altitude_m);
  }
  if (millis() - last_under_time_ms > 1000) {
    // 超音波高度のみ冗長系がないため，データが来なければ8mとして高度推定に渡す．
    // 測定範囲外のときは10mになり，9m以上でテイクオフ判断をするため故障時は8m
    // filtered_under_urm_altitude_m.add(8.0);
    // ToDo 明示的にis_aliveを作るべき．値の処理によって7変わる．
    under_is_alive = false;
  } else {
    under_is_alive = true;
  }

  //AirData
  static unsigned long int last_air_time_ms = 0;
  readnum = Air_UART.readUART();
  int air_data_num = 17;
  if (readnum == air_data_num) {
    last_air_time_ms = millis();
    digitalWrite(L_Air, !digitalRead(L_Air));
    data_air_dps_pressure_hPa = Air_UART.UART_data[0];
    data_air_dps_temperature_deg = Air_UART.UART_data[1];
    data_air_dps_altitude_m = Air_UART.UART_data[2];
    data_air_sdp_differentialPressure_Pa = Air_UART.UART_data[3];
    data_air_sdp_airspeed_ms = Air_UART.UART_data[4];
    data_air_AoA_angle_deg = Air_UART.UART_data[5];
    data_air_AoS_angle_deg = Air_UART.UART_data[6];
    data_air_bno_accx_mss = Air_UART.UART_data[7];
    data_air_bno_accy_mss = Air_UART.UART_data[8];
    data_air_bno_accz_mss = Air_UART.UART_data[9];
    data_air_bno_qw = Air_UART.UART_data[10];
    data_air_bno_qx = Air_UART.UART_data[11];
    data_air_bno_qy = Air_UART.UART_data[12];
    data_air_bno_qz = Air_UART.UART_data[13];
    data_air_bno_roll = Air_UART.UART_data[14];
    data_air_bno_pitch = Air_UART.UART_data[15];
    data_air_bno_yaw = Air_UART.UART_data[16];
    filtered_airspeed_ms.add(data_air_sdp_airspeed_ms);
    filtered_air_dps_altitude_m.add(data_air_dps_altitude_m);
    if (flight_phase == PLATFORM) {
      air_dps_altitude_platform_m.add(data_air_dps_altitude_m);
    }
  }
   if (millis() - last_air_time_ms > 1000) {
    air_is_alive = false;
  } else {
    air_is_alive = true;
  }

  //GPS
  while (SerialWireless.available() > 0) {
    if (gps.encode(SerialWireless.read())) {
      data_main_gps_hour = gps.time.hour();
      data_main_gps_minute = gps.time.minute();
      data_main_gps_second = gps.time.second();
      data_main_gps_centisecond = gps.time.centisecond();
      data_main_gps_latitude_deg = gps.location.lat();
      data_main_gps_longitude_deg = gps.location.lng();
      data_main_gps_altitude_m = gps.altitude.meters();
      data_main_gps_groundspeed_kmph = gps.speed.kmph();
    }
  }
}

void determine_flight_phase() {
  //発進判定のため，IMU測定はここで行う
  read_main_bno();
  Serial.print("距離:");
  Serial.println(filtered_under_urm_altitude_m.get());
  static unsigned long int takeoff_time_ms = 0;
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
        // x軸方向の加速度またはy軸方向の急激な加速と超音波センサと気圧センサによる下降判断そしてマイコンが起動してから10秒後
        if ((over_urm_range || descending) && millis() > 15000){
          TAKEOFF = true;
          flight_phase = HIGH_LEVEL;
        }
        if (over_urm_range && millis() > 15000) {
          SerialWireless.print("\n\nover_urm_range\n\n");
        }
        if (descending && millis() > 15000) {
          SerialWireless.print("\n\ndescending\n\n");
        }
      }
      break;
    case HIGH_LEVEL:
      if ( 0.3 <= filtered_under_urm_altitude_m.get() < 1.0){
        flight_phase = MID_LEVEL;
      }
      else if ( filtered_under_urm_altitude_m.get() < 0.3){
        flight_phase = LOW_LEVEL;
      }
      else{
        flight_phase = HIGH_LEVEL;
      }
      break;
    case MID_LEVEL:
      if ( 0.3 <= filtered_under_urm_altitude_m.get() && filtered_under_urm_altitude_m.get() < 1.0){
        flight_phase = MID_LEVEL;
      }
      else if ( filtered_under_urm_altitude_m.get() < 0.3){
        flight_phase = LOW_LEVEL;
      }
      else{
        flight_phase = HIGH_LEVEL;
      }
      break;
    case LOW_LEVEL:
      if ( 0.3 <= filtered_under_urm_altitude_m.get() && filtered_under_urm_altitude_m.get() < 1.0){
        flight_phase = MID_LEVEL;
      }
      else if ( filtered_under_urm_altitude_m.get() < 0.3){
        flight_phase = LOW_LEVEL;
      }
      else{
        flight_phase = HIGH_LEVEL;
      }
      break;
    default:
      break;
  }
  //速度レベル判断
  if (filtered_airspeed_ms.get() > 1100) {
    speed_level = FAST;
  }
  else if (filtered_airspeed_ms.get() > 1000) {
    speed_level = NORMAL;
  }
  else {
    speed_level = SLOW;
  }
}

void calculate_altitude() {
  // 100Hzで関数呼び出し
  read_main_dps();

  dps_altitude_lake_array_m[0] = filtered_main_dps_altitude_m.get() - main_dps_altitude_platform_m.get() + const_platform_m;
  dps_altitude_lake_array_m[1] = filtered_under_dps_altitude_m.get() - under_dps_altitude_platform_m.get() + const_platform_m;
  dps_altitude_lake_array_m[2] = filtered_air_dps_altitude_m.get() - air_dps_altitude_platform_m.get() + const_platform_m;
  estimated_altitude_lake_m = dps_altitude_lake_m.median(dps_altitude_lake_array_m, 3);

  // 関数は100Hzで呼び出される
  // 中央値が出始めるのに2秒，そこから3秒間で気圧から超音波に情報源を切り替え
  static int transtion_count = 0;
  if ( filtered_under_urm_altitude_m.get() < 8.0) {
    // 気圧センサが本来より低い値ならオフセットは正
    altitude_dps_urm_offset_m.add(filtered_under_urm_altitude_m.get() - estimated_altitude_lake_m );

    if (transtion_count < 500) {
      transtion_count++;
    }
    float ratio = 1;
    if (transtion_count < 500) {
      ratio = 0;
    }
    if (transtion_count > 200) {
      ratio = (float)(transtion_count - 200) / 300.0;
    }
    // 気圧センサが本来より低い値なら正のオフセットを足す
    estimated_altitude_lake_m += altitude_dps_urm_offset_m.get() * ratio;

  }
}

void send_SD() {
  time_ms = millis();
  static int loop_count = 0;
  static uint32_t SD_last_send_time = 0;
  if (loop_count == 0) {
    sprintf(UART_SD, "%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,",  //10個
            time_ms, data_main_bno_accx_mss, data_main_bno_accy_mss, data_main_bno_accz_mss, 
            data_main_bno_qw, data_main_bno_qx, data_main_bno_qy, data_main_bno_qz, 
            data_main_bno_roll, data_main_bno_pitch);
  } else if (loop_count == 1) {
    sprintf(UART_SD, "%.2f,%.2f,%.2f,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,", //10個
            data_main_bno_yaw, estimated_altitude_lake_m, altitude_dps_urm_offset_m.get(), flight_phase, speed_level,
            data_main_dps_pressure_hPa, data_main_dps_temperature_deg, data_main_dps_altitude_m, data_under_dps_pressure_hPa,
            data_under_dps_temperature_deg);
  
  } else if (loop_count == 2) {
    sprintf(UART_SD, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,",  //10個
            data_under_dps_altitude_m, data_under_urm_altitude_m, data_air_dps_pressure_hPa, data_air_dps_temperature_deg, data_air_dps_altitude_m,
            data_air_sdp_differentialPressure_Pa, data_air_sdp_airspeed_ms, data_air_AoA_angle_deg,
            data_air_AoS_angle_deg, data_ics_angle);
  } else if (loop_count == 3) {
    sprintf(UART_SD, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,",   //10個
            data_air_bno_accx_mss, data_air_bno_accy_mss, data_air_bno_accz_mss,
            data_air_bno_qw, data_air_bno_qx, data_air_bno_qy, data_air_bno_qz,
            data_air_bno_roll, data_air_bno_pitch, data_air_bno_yaw);
  } else {
    sprintf(UART_SD, "%u,%u,%u,%u,%10.7lf,%10.7lf,%5.2lf,5.2lf\n",    //7個
            data_main_gps_hour, data_main_gps_minute, data_main_gps_second, data_main_gps_centisecond,
            data_main_gps_latitude_deg, data_main_gps_longitude_deg, data_main_gps_altitude_m,data_main_gps_groundspeed_kmph);
    loop_count = -1;
  }
  SD_last_send_time = millis();
  loop_count++;
  //バッファをクリアしてから新しいデータを書き込み
  SerialAir.flush();
  SerialSD_ICS.flush();
  SerialUnder.flush();
  SerialAir.print(UART_SD);
  SerialUnder.print(UART_SD);
  SerialSD_ICS.print(UART_SD);
}

void send_Condition() {
  digitalWrite(25,HIGH);
  sprintf(UART_Condition, "%d,%d,%f\n", flight_phase, speed_level,filtered_under_urm_altitude_m.get());
  SerialCondition.flush();
  SerialCondition.print(UART_Condition);
  digitalWrite(25,LOW);
}


void read_main_bno() {
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); //加速度ベクトルを取得（センサーの生の値）
  imu::Quaternion quat = bno.getQuat(); //クォータニオンを取得（センサーの生の値）

  data_main_bno_accx_mss = accel.x(); //x方向の加速度を格納
  data_main_bno_accy_mss = accel.y(); //y方向の加速度を格納
  data_main_bno_accz_mss = accel.z(); //z方向の加速度を格納

  data_main_bno_qw = quat.w(); //クォータニオン(回転量w)を格納
  data_main_bno_qx = quat.x(); //クォータニオン(x軸)を格納
  data_main_bno_qy = quat.y(); //クォータニオン(y軸)を格納
  data_main_bno_qz = quat.z(); //クォータニオン(z軸)を格納

  Quaternion qua(data_main_bno_qx, data_main_bno_qy, data_main_bno_qz, data_main_bno_qw);
  EulerAngles euler(qua.to_rotation_matrix());
  data_main_bno_roll = euler.second() * 180 / 3.1415;
  data_main_bno_pitch = -euler.first() * 180 / 3.1415;
  data_main_bno_yaw = euler.third() * 180 / 3.1415;
/*
  Serial.print("roll:");
  Serial.println(data_main_bno_roll);
  Serial.print("pitch");
  Serial.println(data_main_bno_pitch);
  Serial.print("yaw");
  Serial.println(data_main_bno_yaw);
*/
  /*参考
  https://cdn-learn.adafruit.com/downloads/pdf/adafruit-bno055-absolute-orientation-sensor.pdf
  */
}

void read_main_dps() {
  if (!(dps.temperatureAvailable() && dps.pressureAvailable())) {
    return;
  }
  dps.getEvents(&temp_event, &pressure_event);
  data_main_dps_pressure_hPa = pressure_event.pressure;
  data_main_dps_temperature_deg = temp_event.temperature;
  data_main_dps_altitude_m = (powf(1013.25 / data_main_dps_pressure_hPa, 1 / 5.257) - 1) * (data_main_dps_temperature_deg + 273.15) / 0.0065;
  filtered_main_dps_altitude_m.add(data_main_dps_altitude_m);
  if (flight_phase == PLATFORM) {
    main_dps_altitude_platform_m.add(data_main_dps_altitude_m);
  }
}


void TWE_downlink() {
  static uint8_t TWE_downlink_type = 0;
  static uint32_t TWE_last_send_time = millis() - 1000;
  if (TWE_downlink_type == 0 && millis() - TWE_last_send_time >= 2000) {
    SerialWireless.print("\n\n\n");
    SerialWireless.print("MAIN\n");
    sprintf(TWE_BUF, "accX    accY    accZ\n%+06.2f  %+06.2f  %+06.2f\n", data_main_bno_accx_mss, data_main_bno_accy_mss, data_main_bno_accz_mss);
    SerialWireless.print(TWE_BUF);
    sprintf(TWE_BUF, "roll(left+)   pitch   yaw\n %+06.2f        %+06.2f  %+06.2f\n", data_main_bno_roll, data_main_bno_pitch, data_main_bno_yaw);
    SerialWireless.print(TWE_BUF);
    TWE_downlink_type++;
    TWE_last_send_time = millis();
  } else if (TWE_downlink_type == 1 && millis() - TWE_last_send_time >= 400) {
    sprintf(TWE_BUF, "pressure        temp    alt\n%+06.2f        %+06.2f  %+06.2f\n", data_main_dps_pressure_hPa, data_main_dps_temperature_deg, data_main_dps_altitude_m);
    SerialWireless.print(TWE_BUF);
    SerialWireless.print("\n");
    TWE_downlink_type++;
    TWE_last_send_time = millis();
  } else if (TWE_downlink_type == 2 && millis() - TWE_last_send_time >= 400) {
    SerialWireless.print("UNDER : ");
    if (under_is_alive) {
      SerialWireless.print("alive\n");
    } else {
      SerialWireless.print("dead\n");
    }
    sprintf(TWE_BUF, "pressure        temp    alt\n%+08.2f        %+06.2f  %+06.2f\n", data_under_dps_pressure_hPa, data_under_dps_temperature_deg, data_under_dps_altitude_m);
    SerialWireless.print(TWE_BUF);
    sprintf(TWE_BUF, "sonic_alt\n%+06.2f\n", data_under_urm_altitude_m);
    SerialWireless.print(TWE_BUF);
    //sprintf(TWE_BUF, "%+06.2f\n", filtered_under_urm_altitude_m.get());
    //SerialWireless.print(TWE_BUF);
    SerialWireless.print("\n");
    TWE_downlink_type++;
    TWE_last_send_time = millis();
  } else if (TWE_downlink_type == 3 && millis() - TWE_last_send_time >= 400) {
    SerialWireless.print("AIR : ");
    if (air_is_alive) {
      SerialWireless.print("alive\n");
    } else {
      SerialWireless.print("dead\n");
    }
    sprintf(TWE_BUF, "accX    accY    accZ\n%+06.2f  %+06.2f  %+06.2f\n", data_air_bno_accx_mss, data_air_bno_accy_mss, data_air_bno_accz_mss);
    SerialWireless.print(TWE_BUF);
    sprintf(TWE_BUF, "roll(left+)   pitch   yaw\n %+06.2f        %+06.2f  %+06.2f\n", data_air_bno_roll, data_air_bno_pitch, data_air_bno_yaw);
    SerialWireless.print(TWE_BUF);
    sprintf(TWE_BUF, "AoA   AoS\n %+06.2f        %+06.2f\n", data_air_AoA_angle_deg, data_air_AoS_angle_deg);
    SerialWireless.print(TWE_BUF);
    sprintf(TWE_BUF, "pressure        temp    alt\n%+08.2f        %+06.2f  %+06.2f\n", data_air_dps_pressure_hPa, data_air_dps_temperature_deg, data_air_dps_altitude_m);
    SerialWireless.print(TWE_BUF);
    sprintf(TWE_BUF, "diffPressure    AirSpeed\n%+09.3f       %+06.2f\n", data_air_sdp_differentialPressure_Pa, data_air_sdp_airspeed_ms);
    SerialWireless.print(TWE_BUF);
    //sprintf(TWE_BUF, "                %+06.2f\n", filtered_airspeed_ms.get());
    //SerialWireless.print(TWE_BUF);
    SerialWireless.print("\n");
    TWE_downlink_type++;
    TWE_last_send_time = millis();
  } else if (TWE_downlink_type == 4 && millis() - TWE_last_send_time >= 400) {
    SerialWireless.print("ICS(joystick)\n");
    sprintf(TWE_BUF, "angle (center=7500)\n%d\n", data_ics_angle);
    SerialWireless.print(TWE_BUF);

    SerialWireless.print("estimated_altitude_lake_m\n");
    sprintf(TWE_BUF, "%+08.3f\n", estimated_altitude_lake_m);
    SerialWireless.print(TWE_BUF);

    SerialWireless.print("Flight Phase\n");
    sprintf(TWE_BUF, "%d\n",flight_phase);
    SerialWireless.print(TWE_BUF);

   SerialWireless.print("Speed Level\n");
    sprintf(TWE_BUF, "%d\n",speed_level);
    SerialWireless.print(TWE_BUF);


    sprintf(TWE_BUF, "latitude:%10.7lf longitude:%10.7lf", data_main_gps_latitude_deg, data_main_gps_longitude_deg);
    SerialWireless.println(TWE_BUF);

    //SerialWireless.flush();

    //Reset downlink type
    TWE_downlink_type = 0;
    TWE_last_send_time = millis();
  }
}
