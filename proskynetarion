#include <MPU6050_tockn.h>
#include <Wire.h>
#include <PID_v1.h>
MPU6050 mpu6050(Wire);
#include <SPI.h>
#include <RH_NRF24.h>
#include <math.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <Arduino.h>
#include <EEPROM.h>
#include <WiFiClient.h>
WebServer server(80);

#ifndef _ESP32_ANALOG_WRITE_
#define _ESP32_ANALOG_WRITE_
#include <Arduino.h>
typedef struct analog_write_channel {
  int8_t pin;
  double frequency;
  uint8_t resolution;
} analog_write_channel_t;
int analogWriteChannel(uint8_t pin);
void analogWriteFrequency(double frequency);
void analogWriteFrequency(uint8_t pin, double frequency);
void analogWriteResolution(uint8_t resolution);
void analogWriteResolution(uint8_t pin, uint8_t resolution);
void analogWrite(uint8_t pin, uint32_t value, uint32_t valueMax = 255);
#endif
analog_write_channel_t _analog_write_channels[16] = {
  { -1, 5000, 13 },
  { -1, 5000, 13 },
  { -1, 5000, 13 },
  { -1, 5000, 13 },
  { -1, 5000, 13 },
  { -1, 5000, 13 },
  { -1, 5000, 13 },
  { -1, 5000, 13 },
  { -1, 5000, 13 },
  { -1, 5000, 13 },
  { -1, 5000, 13 },
  { -1, 5000, 13 },
  { -1, 5000, 13 },
  { -1, 5000, 13 },
  { -1, 5000, 13 },
  { -1, 5000, 13 }
};
int analogWriteChannel(uint8_t pin) {
  int channel = -1;
  // Check if pin already attached to a channel
  for (uint8_t i = 0; i < 16; i++) {
    if (_analog_write_channels[i].pin == pin) {
      channel = i;
      break;
    }
  }
  // If not, attach it to a free channel
  if (channel == -1) {
    for (uint8_t i = 0; i < 16; i++) {
      if (_analog_write_channels[i].pin == -1) {
        _analog_write_channels[i].pin = pin;
        channel = i;
        ledcSetup(channel, _analog_write_channels[i].frequency, _analog_write_channels[i].resolution);
        ledcAttachPin(pin, channel);
        break;
      }
    }
  }
  return channel;
}
void analogWriteFrequency(double frequency) {
  for (uint8_t i = 0; i < 16; i++) {
    _analog_write_channels[i].frequency = frequency;
  }
}
void analogWriteFrequency(uint8_t pin, double frequency) {
  int channel = analogWriteChannel(pin);
  // Make sure the pin was attached to a channel, if not do nothing
  if (channel != -1 && channel < 16) {
    _analog_write_channels[channel].frequency = frequency;
  }
}
void analogWriteResolution(uint8_t resolution) {
  for (uint8_t i = 0; i < 16; i++) {
    _analog_write_channels[i].resolution = resolution;
  }
}
void analogWriteResolution(uint8_t pin, uint8_t resolution) {
  int channel = analogWriteChannel(pin);
  // Make sure the pin was attached to a channel, if not do nothing
  if (channel != -1 && channel < 16) {
    _analog_write_channels[channel].resolution = resolution;
  }
}
void analogWrite(uint8_t pin, uint32_t value, uint32_t valueMax) {
  int channel = analogWriteChannel(pin);
  // Make sure the pin was attached to a channel, if not do nothing
  if (channel != -1 && channel < 16) {
    uint8_t resolution = _analog_write_channels[channel].resolution;
    uint32_t levels = pow(2, resolution);
    uint32_t duty = ((levels - 1) / valueMax) * min(value, valueMax);
    // write duty to LEDC
    ledcWrite(channel, duty);
  }
}
long gyrotimer = 0;
#define DEXRET 25  //dexter retrograde
#define DEXPRG 26  //dexter prograde
#define SNSRET 33  //sinister retrograde
#define SNSPRG 32  //sinister prograde
#define DEXHLA 14  //dexter hall alpha
#define DEXHLB 15
#define SNSHLA 27
#define SNSHLB 13
#define INTRPT 2  //gyro interrupt
#define CHIPEN 4  //nrf24 CE
#define CHIPSL 0  //nrf24 CSN

RH_NRF24 nrf24(CHIPEN, CHIPSL);  //CE,CSN

int motor_dead_zone = 160;  //電機死區
long counter_s = 0;         //left hall encoder counter
long counter_d = 0;
const double vel_ratio = 20;                 //分辨率  即减速比*电机转一圈编码器转脉冲数
double vel, e_vel, sum_e_vel, V_Set, V_out;  //实际速度，设定速度,速度偏差，速度偏差积分
double acc_angle, balance_output;            //角度，陀螺仪
double eKpB, eKdB, eKpV, eKiV = 0;           //eeprom config

void eeprom_init() {
  EEPROM.begin(128);
  Serial.println("Reading EEPROM");
  String KpB = "";
  for (int i = 0; i < 32; ++i) {
    KpB += char(EEPROM.read(i));
  }
  eKpB = KpB.toFloat();
  String KdB = "";
  for (int i = 32; i < 64; ++i) {
    KdB += char(EEPROM.read(i));
  }
  eKdB = KdB.toFloat();
  String KpV = "";
  for (int i = 64; i < 96; ++i) {
    KpV += char(EEPROM.read(i));
  }
  eKpV = KpV.toFloat();
  String KiV = "";
  for (int i = 64; i < 96; ++i) {
    KiV += char(EEPROM.read(i));
  }
  eKiV = KpV.toFloat();
  Serial.print("Loaded config from EEPROM: ");
  Serial.print(eKpB);
  Serial.print(", ");
  Serial.print(eKdB);
  Serial.print(", ");
  Serial.print(eKpV);
  Serial.print(", ");
  Serial.println(eKiV);
}

String content;
bool param_reset = 0;
int statusCode;
void createWebServer() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.softAP("Apo_Proskynetarion", "");
  server.on("/", []() {
    content = "<!DOCTYPE HTML>\r\n<html>Proskynetarion Configuration Interface";
    content += "</br><b1>";
    content += "Current parameters: KpB=" + String(eKpB) + ", KdB=" + String(eKdB) + ", KpV=" + String(eKpV);
    content += "</b1></br><form method='get' action='setting'><label>KpB: </label><input name='KpB' length=32></br><label>KdB: </label><input name='KdB' length=32></br><label>KpV: </label><input name='KpV' length=32></br><label>KiV: </label><input name='KiV' length=32></br><input type='submit'></form>";
    content += "</html>";
    server.send(200, "text/html", content);
  });
  server.on("/setting", []() {
    arrt();
    String KpB = server.arg("KpB");
    String KdB = server.arg("KdB");
    String KpV = server.arg("KpV");
    String KiV = server.arg("KiV");
    if (KpB.length() > 0 && KdB.length() > 0) {
      Serial.println("clearing eeprom");
      for (int i = 0; i < 128; ++i) {
        EEPROM.write(i, 0);
      }
      Serial.println(KpB);
      Serial.println("");
      Serial.println(KdB);
      Serial.println("");
      for (int i = 0; i < KpB.length(); ++i) {
        EEPROM.write(i, KpB[i]);
        Serial.print("Wrote: ");
        Serial.println(KpB[i]);
      }
      for (int i = 0; i < KdB.length(); ++i) {
        EEPROM.write(32 + i, KdB[i]);
        Serial.print("Wrote: ");
        Serial.println(KdB[i]);
      }
      for (int i = 0; i < KpV.length(); ++i) {
        EEPROM.write(64 + i, KpV[i]);
        Serial.print("Wrote: ");
        Serial.println(KpV[i]);
      }
      for (int i = 0; i < KiV.length(); ++i) {
        EEPROM.write(96 + i, KiV[i]);
        Serial.print("Wrote: ");
        Serial.println(KiV[i]);
      }
      EEPROM.commit();
      content = "{\"Success\":\"saved to eeprom... reset to boot into new wifi\"}";
      statusCode = 200;
      param_reset = 1;
    } else {
      content = "{\"Error\":\"404 not found\"}";
      statusCode = 404;
      Serial.println("Sending 404");
    }
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(statusCode, "application/json", content);
  });
}

void gyro_init() {
  Wire.begin();
  mpu6050.begin();
  //mpu6050.calcGyroOffsets(true);
  mpu6050.setGyroOffsets(-0.17, -0.37, 1.43);  //vehicle specific
}
void nrf_init() {
  if (!nrf24.init())
    Serial.println("init failed");
  if (!nrf24.setChannel(1))
    Serial.println("setChannel failed");
  if (!nrf24.setRF(RH_NRF24::DataRate250kbps, RH_NRF24::TransmitPower0dBm))
    Serial.println("setRF failed");
}

bool button = 1;
bool balance = 0;
long command_rescission_timer = 0;
bool timeout() {
  if (!balance) return nrf24.waitAvailableTimeout(1);
  else return (1);
}
void nrf_audit() {
  uint8_t data[5];
  if (nrf24.waitAvailableTimeout(1)) {
    uint8_t len = sizeof(data);
    if (nrf24.recv(data, &len)) {
      command_rescission_timer = millis();
      if (button == 1 && data[4] == 0) {
        balance = 1 - balance;
        delay(50);
      }
      button = data[4];
      int x = data[0];
      int y = data[1];
      if (data[2]) x = 0 - x;
      if (data[3]) y = 0 - y;
      if (!balance) curro(y, x);
    } else {
      if (!balance && command_rescission_timer - millis() > 500) arrt();
    }
  } else {
    if (!balance && command_rescission_timer - millis() > 500) arrt();
    //Serial.println("Disconnected from controller.");
  }
}

void prog_s(int v) {
  analogWrite(SNSRET, 0);
  analogWrite(SNSPRG, v);
}
void prog_d(int v) {
  analogWrite(DEXRET, 0);
  analogWrite(DEXPRG, v);
}
void retr_d(int v) {
  analogWrite(DEXRET, v);
  analogWrite(DEXPRG, 0);
}
void retr_s(int v) {
  analogWrite(SNSRET, v);
  analogWrite(SNSPRG, 0);
}
void arrt() {
  analogWrite(DEXRET, 0);
  analogWrite(DEXPRG, 0);
  analogWrite(SNSRET, 0);
  analogWrite(SNSPRG, 0);
}
void s(int v) {
  if (v > 255) v = 255;
  if (v < -255) v = -255;
  if (v > 0) {
    prog_s(v);
  } else {
    retr_s(abs(v));
  }
}
void d(int v) {
  if (v > 255) v = 255;
  if (v < -255) v = -255;
  if (v > 0) {
    prog_d(v);
  } else {
    retr_d(abs(v));
  }
}
//-------------------------------------------------------------
//编码器求速度部分
//-------------------------------------------------------------
//定时器中断函数，每10ms执行一次

void IRAM_ATTR hall_s_a_audit() {
  if (digitalRead(SNSHLA)) {
    if (digitalRead(SNSHLB)) {
      counter_s--;
    } else {
      counter_s++;
    }
  } else {
    if (digitalRead(SNSHLB)) {
      counter_s++;
    } else {
      counter_s--;
    }
  }
}

void IRAM_ATTR hall_s_b_audit() {
  if (digitalRead(SNSHLB)) {
    if (digitalRead(SNSHLA)) {
      counter_s++;
    } else {
      counter_s--;
    }
  } else {
    if (digitalRead(SNSHLA)) {
      counter_s--;
    } else {
      counter_s++;
    }
  }
}

void IRAM_ATTR hall_d_a_audit() {
  if (digitalRead(DEXHLA)) {
    if (digitalRead(DEXHLB)) {
      counter_d++;
    } else {
      counter_d--;
    }
  } else {
    if (digitalRead(DEXHLB)) {
      counter_d--;
    } else {
      counter_d++;
    }
  }
}

void IRAM_ATTR hall_d_b_audit() {
  if (digitalRead(DEXHLB)) {
    if (digitalRead(DEXHLA)) {
      counter_d--;
    } else {
      counter_d++;
    }
  } else {
    if (digitalRead(DEXHLA)) {
      counter_d++;
    } else {
      counter_d--;
    }
  }
}
void hall_interrupt_init() {
  attachInterrupt(SNSHLA, hall_s_a_audit, FALLING);
  attachInterrupt(SNSHLB, hall_s_b_audit, FALLING);
  attachInterrupt(DEXHLA, hall_d_a_audit, FALLING);
  attachInterrupt(DEXHLB, hall_d_b_audit, FALLING);
}

void curro(int x, int y) {
  int vel = (pow(abs(x), 2) + pow(abs(y), 2));
  if (vel > 900) vel = 900;
  vel = map(vel, 0, 900, 0, 256);
  if (x < 0 && y < 0) {  //右前
    prog_s(vel);
    prog_d(vel / 2);
  } else if (x > 5 && y < -5) {  //右後
    retr_s(vel);
    retr_d(vel / 2);
  } else if (x < -5 && y > 0) {  //左前
    prog_s(vel / 2);
    prog_d(vel);
  } else if (x > 5 && y > 5) {  //左後
    retr_s(vel / 2);
    retr_d(vel);
  } else if ((x < 5 && x > -5) && (y < 5 && y > -5)) {
    arrt();
  } else if (x < -5 && y == 0) {
    prog_s(vel);  //calibration
    prog_d(vel);
  } else if (x > 5 && y == 0) {
    retr_s(vel);
    retr_d(vel);
  } else if (y > 5 && x == 0) {
    retr_s(vel);
    prog_d(vel);
  } else if (y < -5 && x == 0) {
    prog_s(vel);
    retr_d(vel);
  }
}

void gyro_audit() {
  mpu6050.update();
  Serial.println("=======================================================");
  Serial.print("temp : ");
  Serial.print(mpu6050.getTemp());
  Serial.print(" accX : ");
  Serial.print(mpu6050.getAccX());
  Serial.print(" accY : ");
  Serial.print(mpu6050.getAccY());
  Serial.print(" accZ : ");
  Serial.print(mpu6050.getAccZ());

  Serial.print(" gyroX : ");
  Serial.print(mpu6050.getGyroX());
  Serial.print(" gyroY : ");
  Serial.print(mpu6050.getGyroY());
  Serial.print(" gyroZ : ");
  Serial.print(mpu6050.getGyroZ());

  Serial.print(" accAngleX : ");
  Serial.print(mpu6050.getAccAngleX());
  Serial.print(" accAngleY : ");
  Serial.print(mpu6050.getAccAngleY());

  Serial.print(" gyroAngleX : ");
  Serial.print(mpu6050.getGyroAngleX());
  Serial.print(" gyroAngleY : ");
  Serial.print(mpu6050.getGyroAngleY());
  Serial.print(" gyroAngleZ : ");
  Serial.print(mpu6050.getGyroAngleZ());

  Serial.print(" angleX : ");
  Serial.print(mpu6050.getAngleX());
  Serial.print(" angleY : ");
  Serial.print(mpu6050.getAngleY());
  Serial.print(" angleZ : ");
  Serial.print(mpu6050.getAngleZ());
}

void currate(int sv, int dv) {
  s(sv);
  d(dv);
}

int balance_control() {
  mpu6050.update();
  acc_angle = mpu6050.getAccAngleX();
  float Bias = 180 - abs(acc_angle) - 15;  //角度误差
  if (acc_angle < 0) Bias = 0 - Bias;
  //===求出平衡的角度中值和机械相关
  int balance_output = eKpB * Bias + eKdB * mpu6050.getGyroX();
  if (balance_output > 0)
    balance_output = map(balance_output, 0, 255, motor_dead_zone, 255);  //motor dependent
  if (balance_output < 0)
    balance_output = map(balance_output, -255, 0, -255, -1 * motor_dead_zone);
  //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数
  return balance_output;
}

long velocity_timer = 0;
/*
void velocity_audit() {
  double time_difference = millis() - velocity_timer;
  velocity_timer = millis();
  vel = (counter_s + counter_d) / time_difference;
  Serial.print(" Velocity: ");
  Serial.println(vel);
  counter_s = 0;
  counter_d = 0;
}
*/
int speed_control(int desired_speed) {
  float speed_output, speed_difference, average_speed, Movement;
  float speed_integral;
  double time_difference = millis() - velocity_timer;
  velocity_timer = millis();
  vel = (counter_s + counter_d) / time_difference;
  Serial.print(" Velocity: ");
  Serial.println(vel);
  //=============速度PI控制器=======================//
  speed_difference = (counter_s + counter_d) - desired_speed;
  average_speed = vel;
  //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零）                    //===一阶低通滤波器
  average_speed += speed_difference;         //===一阶低通滤波器
  speed_integral += average_speed;           //===积分出位移 积分时间：10ms
  constrain(speed_integral, -65536, 65536);  //===积分限幅
  speed_output = average_speed * eKpV + speed_integral * eKiV;
  //===速度控制
  if (vel == 0) speed_integral = 0;
  //===电机关闭后清除积分
  counter_s = 0;
  counter_d = 0;
  return speed_output;
}
void setup() {
  Serial.begin(115200);
  pinMode(DEXRET, OUTPUT);
  pinMode(DEXPRG, OUTPUT);
  pinMode(SNSRET, OUTPUT);
  pinMode(SNSPRG, OUTPUT);
  pinMode(SNSHLA, INPUT);
  pinMode(SNSHLB, INPUT);
  pinMode(DEXHLA, INPUT);
  pinMode(DEXHLB, INPUT);
  hall_interrupt_init();
  gyro_init();
  nrf_init();
  eeprom_init();
  createWebServer();
  server.begin();
  arrt();
}

void loop() {
  server.handleClient();
  if (param_reset) {
    eeprom_init();
    createWebServer();
    param_reset = 0;
  }
  nrf_audit();
  //gyro_audit();
  if (balance) {
    float s_throughput = balance_control() - speed_control(0);
    float d_throughput = balance_control() - speed_control(0);
    currate(s_throughput, d_throughput);
  }
}
