#include <Adafruit_DPS310.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <Geometry.h>
#include <AS5600.h>
#include <SensirionI2CSdp.h>
//#include <SPI.h>
#include <TORICA_SD.h>

using namespace Geometry;
using namespace BLA;

#define SerialIN Serial1
#define SerialOUT Serial1

int cs_SD = 6;
TORICA_SD sd(cs_SD, false);
//char SD_BUF[256];

Adafruit_DPS310 dps;
sensors_event_t temp_event, pressure_event;

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

AS5600 AoS(&Wire);
AS5600 AoA(&Wire1);

SensirionI2CSdp sdp;
volatile uint16_t error;
char errorMessage[256];

//NeoPixcel
#include <Adafruit_NeoPixel.h>
int NEO_Power = 11;
int NEO_PIN = 12;
#define NUMPIXELS 1

Adafruit_NeoPixel pixels(NUMPIXELS, NEO_PIN, NEO_GRB + NEO_KHZ800);

volatile float dps_altitude_m = 0;
volatile float dps_pressure_hPa = 0;
volatile float dps_temperature_deg = 0;
volatile float sdp_airspeed_ms = 0;
volatile float sdp_differentialPressure_Pa = 0;
volatile float sdp_temperature_deg = 0;
volatile float bno_accx_mss = 0;
volatile float bno_accy_mss = 0;
volatile float bno_accz_mss = 0;
volatile float bno_qw = 0;
volatile float bno_qx = 0;
volatile float bno_qy = 0;
volatile float bno_qz = 0;
volatile float bno_roll = 0;
volatile float bno_pitch = 0;
volatile float bno_yaw = 0;
volatile float AoS_angle_deg = 0;
volatile float AoA_angle_deg = 0;

void setup() {
  //while (!Serial) delay(10);
  Serial1.setFIFOSize(1024);

  Serial1.setTX(0);
  Serial1.setRX(1);  
  Serial.begin(460800);
  Serial1.begin(460800);


  Wire.setSDA(26); 
  Wire.setSCL(27);
  Wire1.setSDA(28); 
  Wire1.setSCL(29);  //AOA
  Wire.begin();
  Wire1.begin();
  Wire.setClock(400000);
  Wire1.setClock(400000);

  //DPS310
  Serial.println("DPS310");
  while (!dps.begin_I2C(DPS310_I2CADDR_DEFAULT, &Wire)) {  // Can pass in I2C address here
    //if (! dps.begin_SPI(DPS310_CS)) {  // If you want to use SPI
    Serial.println("Failed to find DPS");
    //delay(100);
  }
  Serial.println("DPS OK!");
  dps.configurePressure(DPS310_32HZ, DPS310_16SAMPLES);
  dps.configureTemperature(DPS310_32HZ, DPS310_2SAMPLES);

  //BNO055
  if (!bno.begin()) {
    Serial.print("no BNO055 detected"); 
    while(1);
  }
  bno.setExtCrystalUse(true);

  //SDP(差圧センサ)
  sdp.begin(Wire1, SDP8XX_I2C_ADDRESS_0);
  uint32_t productNumber;
  uint8_t serialNumber[8];
  uint8_t serialNumberSize = 8;
  
  sdp.stopContinuousMeasurement();
  error = sdp.readProductIdentifier(productNumber, serialNumber, serialNumberSize);
  if (error) {
    Serial.print("Error trying to execute readProductIdentifier(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else {
    Serial.print("ProductNumber:");
    Serial.print(productNumber);
    Serial.print("\t");
    Serial.print("SerialNumber:");
    Serial.print("0x");
    for (size_t i = 0; i < serialNumberSize; i++) {
      Serial.print(serialNumber[i], HEX);
    }
    Serial.println();
  }
  error = sdp.startContinuousMeasurementWithDiffPressureTCompAndAveraging();
  if (error) {
    Serial.print(
      "Error trying to execute "
      "startContinuousMeasurementWithDiffPressureTCompAndAveraging(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }

  //AoS
  if(AoS.isConnected() == 0){
    Serial.print("Connect: ");
    Serial.println("AoS error");
  }
  else if(AoS.isConnected() == 1){
    Serial.print("Connect: ");
    Serial.println("AoS OK!");
  }

  //AoA
  if(AoA.isConnected() == 0){
    Serial.print("Connect: ");
    Serial.println("AoA error");
  }
  else if(AoA.isConnected() ==1){
    Serial.print("Connect: ");
    Serial.println("AoA OK!");
  }

  //NeoPixcel
  pinMode(NEO_Power, OUTPUT);
  digitalWrite(NEO_Power, HIGH);
  pixels.begin();

  sd.begin();

}

const int readUART_BUF_SIZE = 256;
char readUART_BUF[256];
char sendUART_BUF[256];
void loop() {
  //UART メインからデータを受け取り，SDに記録
  while (SerialIN.available()) {
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.show();
    int read_length = SerialIN.available();
    if (read_length >= readUART_BUF_SIZE - 1) {
      read_length = readUART_BUF_SIZE - 1;
    }
    
    SerialIN.readBytes(readUART_BUF, read_length);
    readUART_BUF[read_length] = '\0';

    sd.add_str(readUART_BUF);
  
    if (!SerialIN.available()) {
      delay(1);
    }
    pixels.clear();
    pixels.show();
  }

  if (dps.temperatureAvailable() && dps.pressureAvailable()) {
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(0, 0, 255));
    pixels.show();

    dps.getEvents(&temp_event, &pressure_event);
    dps_pressure_hPa = pressure_event.pressure;
    dps_temperature_deg = temp_event.temperature;

    dps_altitude_m = (powf(1013.25 / dps_pressure_hPa, 1 / 5.257) - 1) * (dps_temperature_deg + 273.15) / 0.0065;  //高度の計算

    //SDP(差圧センサ)による差圧と機体速度の計算
    float _sdp_differentialPressure_Pa = 0;
    float _sdp_temperature_deg = 0;
    error = sdp.readMeasurement(_sdp_differentialPressure_Pa, _sdp_temperature_deg);
    sdp_differentialPressure_Pa = _sdp_differentialPressure_Pa;
    sdp_temperature_deg = _sdp_temperature_deg;
    if (error) {
      Serial.print("Error trying to execute readMeasurement(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
    } else {
      sdp_airspeed_ms = sqrt(abs(2.0 * sdp_differentialPressure_Pa / (0.0034837 * 101325.0 / (dps_temperature_deg + 273.5))));  //機体速度
    }

    //AoS
    if(AoS.detectMagnet()){
      float AoS_rawAngle = AoS.rawAngle();
      AoS_angle_deg = (AoS_rawAngle * AS5600_RAW_TO_DEGREES);
    }

    //AoA
    if(AoA.detectMagnet()){
      float AoA_rawAngle = AoA.rawAngle();
      AoA_angle_deg = (AoA_rawAngle * AS5600_RAW_TO_DEGREES);
    }

    //BNO055
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); //加速度ベクトルを取得（センサーの生の値）
    imu::Quaternion quat = bno.getQuat(); //クォータニオンを取得（センサーの生の値）

    bno_accx_mss = accel.x(); //x方向の加速度を格納
    bno_accy_mss = accel.y(); //y方向の加速度を格納
    bno_accz_mss = accel.z(); //z方向の加速度を格納

    bno_qw = quat.w(); //クォータニオン(回転量w)を格納
    bno_qx = quat.x(); //クォータニオン(x軸)を格納
    bno_qy = quat.y(); //クォータニオン(y軸)を格納
    bno_qz = quat.z(); //クォータニオン(z軸)を格納

    Quaternion qua(bno_qx, bno_qy, bno_qz, bno_qw); //クォータニオンを構築
    EulerAngles euler(qua.to_rotation_matrix()); //回転行列に変換
    bno_pitch = -(euler.first() * 180 / 3.1415); //ロール角を計算して格納
    bno_roll = euler.second() * 180 / 3.1415; //ピッチ角を計算して格納
    bno_yaw = euler.third() * 180 / 3.1415; //ヨー角を計算して格納



    //文字列の作成+メイン基板にデータを送る
    sprintf(sendUART_BUF, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", dps_pressure_hPa, dps_temperature_deg, dps_altitude_m, sdp_differentialPressure_Pa, sdp_airspeed_ms,
            AoA_angle_deg, AoS_angle_deg, bno_accx_mss, bno_accy_mss, bno_accz_mss, bno_qw, bno_qx, bno_qy, bno_qz, bno_roll, bno_pitch, bno_yaw);
    SerialOUT.print(sendUART_BUF);


    pixels.clear();
    pixels.show();
  }

  sd.flash();
  
  //確認用後でなくす
  /*Serial.print("dps_pressure_hPa: ");
  Serial.println(dps_pressure_hPa);
  Serial.print("dps_temperature_deg: ");
  Serial.println(dps_temperature_deg);
  Serial.print("dps_altitude_m: ");
  Serial.println(dps_altitude_m);
  Serial.print("sdp_differentialPressure_Pa: ");
  Serial.println(sdp_differentialPressure_Pa);
  Serial.print("sdp_airspeed_ms: ");
  Serial.println(sdp_airspeed_ms);
  Serial.print("bno_accx_mss: ");
  Serial.println(bno_accx_mss);
  Serial.print("bno_accy_mss: ");
  Serial.println(bno_accy_mss);
  Serial.print("bno_accz_mss: ");
  Serial.println(bno_accz_mss);
  Serial.print("bno_roll: ");
  Serial.println(bno_roll);
  Serial.print("bno_pitch: ");
  Serial.println(bno_pitch);
  Serial.print("bno_yaw: ");
  Serial.println(bno_yaw);
  Serial.print("AS5600_AOS_angle_deg: ");
  Serial.println(AoS_angle_deg);
  Serial.print("AS5600_AOA_angle_deg: ");
  Serial.println(AoA_angle_deg);

  //delay(10);*/
}
