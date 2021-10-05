/*///////////////////////////////////////////////
//ピン配置
//SDA&SCL:I2C
//A6~A11:tpr-r,tpr-m,tpr-l,silver,tpr-back-r,tpr-back-l
//D2,D3:Motor Driver(PWM)
//D4~D7:Motor Driver(Digital)
//D22,D24:Sonic Sensor(Trig)
//D23,D25:Sonic Sensor(Echo)
//D26~D30:Touch Sensor
//
//
//
//
//
///////////////////////////////////////////////*/
#include <Arduino.h>
#include <Wire.h>
#include <PCA9685.h>

//ピン指定
#define P_TPR_R A6
#define P_TPR_M A7
#define P_TPR_L A8
#define P_SILVER A9
#define P_TPR_BR A10
#define P_TPR_BL A11
#define P_M_APWM 2
#define P_M_A1 4
#define P_M_A2 5
#define P_M_BPWM 3
#define P_M_B1 6
#define P_M_B2 7
#define P_S_RT 22
#define P_S_LT 24
#define P_S_RE 23
#define P_S_LE 25
#define P_T_R 26
#define P_T_M 27
#define P_T_L 28
#define P_T_BR 29
#define P_T_BL 30

//アドレス指定
#define S11059_ADDR 0x2A
#define CONTROL_MSB 0x00
#define CONTROL_1_LSB 0x89
#define CONTROL_2_LSB 0x09
#define SENSOR_REGISTER 0x03
//debug
#define COLOR_DEBUG

//Enum
enum Color{BLACK,WHITE,GREEN,RED,SILVER};

//int
int lower, higher, rr, rg, rb, rir, lr, lg, lb, lir,silver,tpr_l,tpr_m,tpr_r;

// カラーセンサー2個に対し2個i2cのチャネルを贅沢に作らないと行けないっぽい
void test_sensor_setup(){
  Wire.begin();
  Wire.beginTransmission(S11059_ADDR);
  Wire.write(CONTROL_MSB);
  Wire.write(CONTROL_1_LSB);
  Wire.endTransmission();
  Wire.beginTransmission(S11059_ADDR);
  Wire.write(CONTROL_MSB);
  Wire.write(CONTROL_2_LSB);
  Wire.endTransmission();
}
void test_sensor_loop(){
  int low,high,r,g,b,ir;
  delay(10);
  Wire.beginTransmission(S11059_ADDR);
  Wire.write(SENSOR_REGISTER);
  Wire.endTransmission();
  Wire.requestFrom(S11059_ADDR,8);
  if(Wire.available()){
    high = Wire.read();
    low = Wire.read();
    r = high << 8 | low;
    high = Wire.read();
    low = Wire.read();
    g = high << 8 | low;
    high = Wire.read();
    low = Wire.read();
    b = high << 8 | low;
    high = Wire.read();
    low = Wire.read();
    ir = high << 8 | low;
  }
  Wire.endTransmission();
  Serial.print(r);
  Serial.print(",");
  Serial.print(g);
  Serial.print(",");
  Serial.print(b);
  Serial.print(",");
  Serial.print(ir);
  Serial.print('\n');  
}

//TCA9548A
void change_i2c_port(int byte){
  Wire.beginTransmission(0x70);
  Wire.write(1 << byte);
  Wire.endTransmission();
}

void setup() {
  //ピンリセット
  pinMode(P_TPR_R,INPUT);
  pinMode(P_TPR_M,INPUT);
  pinMode(P_TPR_L,INPUT);
  pinMode(P_SILVER,INPUT);
  pinMode(P_TPR_BR,INPUT);
  pinMode(P_TPR_BL,INPUT);
  pinMode(P_M_APWM,OUTPUT);
  pinMode(P_M_A1,OUTPUT);
  pinMode(P_M_A2,OUTPUT);
  pinMode(P_M_BPWM,OUTPUT);
  pinMode(P_M_B1,OUTPUT);
  pinMode(P_M_B2,OUTPUT);
  pinMode(P_S_RT,OUTPUT);
  pinMode(P_S_LT,OUTPUT);
  pinMode(P_S_RE,INPUT);
  pinMode(P_S_LE,INPUT);
  pinMode(P_T_R,INPUT);
  pinMode(P_T_M,INPUT);
  pinMode(P_T_L,INPUT);
  pinMode(P_T_BR,INPUT);
  pinMode(P_T_BL,INPUT);
  // シリアル開始
  Serial.begin(9600);
  //テスト
  //test_sensor_setup();
  //return;
  //I2Cスタート
  Wire.begin();
  change_i2c_port(0);
  Wire.beginTransmission(S11059_ADDR);
  Wire.write(CONTROL_MSB);
  Wire.write(CONTROL_1_LSB);
  Wire.endTransmission();
  change_i2c_port(1);
  Wire.beginTransmission(S11059_ADDR);
  Wire.write(CONTROL_MSB);
  Wire.write(CONTROL_1_LSB);
  Wire.endTransmission();
  change_i2c_port(0);
  Wire.beginTransmission(S11059_ADDR);
  Wire.write(CONTROL_MSB);
  Wire.write(CONTROL_2_LSB);
  Wire.endTransmission();
  change_i2c_port(1);
  Wire.beginTransmission(S11059_ADDR);
  Wire.write(CONTROL_MSB);
  Wire.write(CONTROL_2_LSB);
  Wire.endTransmission();

}

void color_read(){
  //カラーセンサー読み取り
  change_i2c_port(0);
  Wire.beginTransmission(S11059_ADDR);
  Wire.write(SENSOR_REGISTER);
  Wire.endTransmission();
  change_i2c_port(1);
  Wire.beginTransmission(S11059_ADDR);
  Wire.write(SENSOR_REGISTER);
  Wire.endTransmission();
  change_i2c_port(0);
  Wire.requestFrom(S11059_ADDR,8);
  if(Wire.available()){
    //right-red
    higher = Wire.read();
    lower = Wire.read();
    rr = higher << 8 | lower;
    //right-green
    higher = Wire.read();
    lower = Wire.read();
    rg = higher << 8 | lower;
    //right-blue
    higher = Wire.read();
    lower = Wire.read();
    rb = higher << 8 | lower;
    //right-ir
    higher = Wire.read();
    lower = Wire.read();
    rir = higher << 8 | lower;
  }
  Wire.endTransmission();
  change_i2c_port(1);
  Wire.requestFrom(S11059_ADDR,8);
  if(Wire.available()){
    //left-red
    higher = Wire.read();
    lower = Wire.read();
    lr = higher << 8 | lower;
    //left-green
    higher = Wire.read();
    lower = Wire.read();
    lg = higher << 8 | lower;
    //left-blue
    higher = Wire.read();
    lower = Wire.read();
    lb = higher << 8 | lower;
    //left-ir
    higher = Wire.read();
    lower = Wire.read();
    lir = higher << 8 | lower;
  }
  Wire.endTransmission();

  //銀検知読み取り

  //TPR105読み取り

}

void judge_color(){
  #define r_border
  #define g_border
  #define b_border
  #define ir_border
}

void loop() {
  //カラーセンサー読み取り
  #ifdef COLOR_DEBUG
  color_read();
  Serial.print(lr);
  Serial.print(',');
  Serial.print(lg);
  Serial.print(',');
  Serial.print(lb);
  Serial.print(',');
  Serial.print(lir);
  Serial.print(',');
  Serial.print(rr);
  Serial.print(',');
  Serial.print(rg);
  Serial.print(',');
  Serial.print(rb);
  Serial.print(',');
  Serial.print(rir);
  Serial.print('\n');
  return;
  #endif
  color_read();
  
}