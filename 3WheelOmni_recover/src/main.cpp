#include <Arduino.h>
#include <math.h>
#include <PS4Controller.h>
#include <esp_bt_defs.h>
#include <esp_bt_main.h>
// #include <ESP32Motor.hpp>
// 制御周期系の設定
const int control_period = 2000;                         // us
const float control_freq = 1.0f / (float)control_period; // Hz
unsigned long prevtime = 0;                              // us

// モーターまわりのピン宣言
const uint8_t pwmPin1 = 4;
const int dirPin1_1 = 16;
const int dirPin1_2 = 17;
const int channel_motor1 = 1;
const uint8_t pwmPin2 = 15;
const int dirPin2_1 = 18;
const int dirPin2_2 = 19;
const int channel_motor2 = 2;
const uint8_t pwmPin3 = 25;
const int dirPin3_1 = 26;
const int dirPin3_2 = 27;
const int channel_motor3 = 3;
uint8_t pwmResolution = 8;

double motor1[4]; // 1番モーター
double motor2[4]; // 2番モーター
double motor3[4]; // 3番モーター

// モーターのセットアップmotornum[4] = {pwmPin, dirPin1, dirPin2, channel}
void motorSetup(double motornum[], int freq) //motornum[4] = {pwmPin, dirPin1, dirPin2, channel}
{
  PS4.begin("0C:B8:15:C1:3C:66");
  pinMode(motornum[0], OUTPUT); // pwm
  pinMode(motornum[1], OUTPUT); // IN1
  pinMode(motornum[2], OUTPUT); // IN2
  ledcSetup(motornum[3], freq, pwmResolution);
  ledcAttachPin(motornum[0], motornum[3]);
  Serial.printf("setup\n");
}

// ps4の値
int ps4_x_raw; //左スティック横方向
int ps4_y_raw; //左スティック縦方向
int ps4_yaw_raw; //右スティック横

// モーター駆動用関数
// void motorDrive(int channel_motor, int IN1, int IN2, double duty)
void motorDrive(double Drive_motornum[], double duty)
{
  bool dir = duty > 0; // 方向
  double DUTY = pow(2, pwmResolution) * abs(duty);
  digitalWrite(Drive_motornum[1], dir);  // 方向信号1
  digitalWrite(Drive_motornum[2], !dir); // 方向信号2
  ledcWrite(Drive_motornum[3], DUTY);    // pwm出力
  Serial.printf("DUTY=%lf",DUTY);
}

// モーター電圧解放
void motorFree(double Free_motornum[])
{
  digitalWrite(Free_motornum[1], 0); // 方向信号1
  digitalWrite(Free_motornum[2], 0); // 方向信号2
  ledcWrite(Free_motornum[3], 0);    // duty=0で出力
}

// モーターブレーキ
void motorStop(double motornum[])
{
  digitalWrite(motornum[1], 1); // 方向信号1
  digitalWrite(motornum[2], 1); // 方向信号2
  ledcWrite(motornum[3], 1);    // duty=0を出力
}

// コントローラの値からオムニ行列を計算し，モーター出力
void OmniDrive(int ps4x, int ps4y, int ps4yaw)
{
  double omniArray[3][3] = {{1.0, 0.0, 1.0}, {0.5, -0.8660254, 1.0}, {0.5, 0.8660254, 1.0}};
  double duty_calc[3] = {0, 0, 0};
  // double duty_motor1=0,duty_motor2=0,duty_motor1 =0;

  for (int i = 0; i < 2; i++) // dutyの計算
  {
    duty_calc[i] = omniArray[i][0] * ps4x + omniArray[i][1] * ps4y + omniArray[i][2] * ps4yaw;
  }

  motorDrive(motor1, duty_calc[0]);
  motorDrive(motor2, duty_calc[1]);
  motorDrive(motor3, duty_calc[2]);
}

void setup()
{
  Serial.begin(115200);
  PS4.begin("0C:B8:15:C1:3C:66");

  // モーターのピン
  int ledcfreq = 2000; // ledcWriteの周波数設定
  double motor1[4] = {pwmPin1, dirPin1_1, dirPin1_2, channel_motor1};
  motorSetup(motor1, ledcfreq);
  Serial.printf("motor1Setup\n");
  double motor2[4] = {pwmPin2, dirPin2_1, dirPin2_2, channel_motor2};
  motorSetup(motor2, ledcfreq);
  Serial.printf("motor2Setup\n");
  double motor3[4] = {pwmPin3, dirPin3_1, dirPin3_2, channel_motor3};
  motorSetup(motor3, ledcfreq);
  Serial.printf("motor3Setup\n");
}

void loop()
{
  // MDの動作確認   3秒正転,1秒フリー,3秒逆転，1秒フリー
  Serial.printf("start_cw\n");
  double testduty = 0.1;
  delay(500);
  motorDrive(motor1, testduty);
  motorDrive(motor2, testduty);
  motorDrive(motor3, testduty);
  delay(3000);
  // Serial.printf("switch_ccw\n");
  // motorFree(motor1);
  // delay(500);

  // Serial.printf("start_ccw\n");
  // testduty = -0.5;
  // delay(500);
  // motorDrive(motor1, testduty);
  // motorDrive(motor2, testduty);
  // motorDrive(motor3, testduty);
  // delay(3000);
  // Serial.printf("switch_cw\n");
  // delay(500);

  ps4_x_raw = PS4.LStickX();
  ps4_y_raw = PS4.LStickY();
  ps4_yaw_raw = PS4.RStickX();
  Serial.printf("x=%d,y=%d,yaw=%d\n",ps4_x_raw,ps4_y_raw,ps4_yaw_raw);

  // unsigned long TIMER = micros() - prevtime;
  // while (TIMER < control_period)
  // {
  //   TIMER = micros() - prevtime;
  // }
  // prevtime = micros();
}