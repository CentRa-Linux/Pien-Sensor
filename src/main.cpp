#include <Arduino.h>
#include <AsyncDelay.h>
#include <SoftWire.h>
<<<<<<< HEAD
#include <Wire.h>
=======
#include <wire.h>
#include <SoftwareSerial.h>
>>>>>>> 5d2f04fa6c74b17e5229131400e1cba89914b603

// アドレス指定
#define S11059_ADDR 0x2A
#define CONTROL_MSB 0x00
#define CONTROL_1_LSB 0x89
#define CONTROL_2_LSB 0x09
#define SENSOR_REGISTER 0x03
// しきい値
// 左右、検出する色、読む色
// 白は全部多いと白だと検知
#define L_W_R 5000
#define L_W_G 5000
#define L_W_B 5800
#define R_W_R 5000
#define R_W_G 5000
#define R_W_B 5000
// 黒は全部小さいと黒だと検知
#define L_B_R 5000
#define L_B_G 5000
#define L_B_B 5000
#define R_B_R 5000
#define R_B_G 5000
#define R_B_B 5000
// 緑は緑のみしきい値より多く、その他がしきい値より少なければ緑だと判定
#define L_G_R 5000
#define L_G_G 5000
#define L_G_B 5000
#define R_G_R 5000
#define R_G_G 5000
#define R_G_B 5000
// 赤は赤のみしきい値より多く、その他がしきい値より少なければ緑だと判定
#define L_R_R 5000
#define L_R_G 5000
#define L_R_B 5000
#define R_R_R 5000
#define R_R_G 5000
#define R_R_B 5000

// カラーセンサー2個に対し2個i2cのチャネルを贅沢に作らないと行けないっぽい
SoftWire LSWire(A4, A5);
SoftWire RSWire(SDA, SCL);
AsyncDelay readInterval;
<<<<<<< HEAD
void test_sensor_setup() {
=======
SoftwareSerial ss(2,3);
void test_ss_setup(){
  ss.begin(9600);
}
void test_ss_loop(){
  for(int i = 0;i < 256;i++){
    ss.write(i);
    delay(250);
  }
}
void test_sensor_setup(){
>>>>>>> 5d2f04fa6c74b17e5229131400e1cba89914b603
  Wire.begin();
  Wire.beginTransmission(S11059_ADDR);
  Wire.write(CONTROL_MSB);
  Wire.write(CONTROL_1_LSB);
  Wire.write(CONTROL_2_LSB);
  Wire.endTransmission();
}
<<<<<<< HEAD
void test_sensor_loop() {
  int low, high, r, g, b, ir;
  delay(10);
=======
void test_sensor_loop(){
  int low,high,r,g,b,ir;
  delay(1000);
>>>>>>> 5d2f04fa6c74b17e5229131400e1cba89914b603
  Wire.beginTransmission(S11059_ADDR);
  Wire.write(SENSOR_REGISTER);
  Wire.endTransmission();
  Wire.requestFrom(S11059_ADDR, 8);
  if (Wire.available()) {
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
void setup() {
<<<<<<< HEAD
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  // シリアル開始
  Serial.begin(9600);
  // テスト
  test_sensor_setup();
  return;
=======
  test_ss_setup();
  return;
  // シリアル開始
  Serial.begin(9600);
  //テスト
  //test_sensor_setup();
  //return;
>>>>>>> 5d2f04fa6c74b17e5229131400e1cba89914b603
  // ソフトウェアi2c開始
  LSWire.begin();
  RSWire.begin();
  // コントロールバイトを指定、スリープ解除
  LSWire.beginTransmission(S11059_ADDR);
  LSWire.write(CONTROL_MSB);
  LSWire.write(CONTROL_1_LSB);
  LSWire.endTransmission();
  RSWire.beginTransmission(S11059_ADDR);
  RSWire.write(CONTROL_MSB);
  RSWire.write(CONTROL_1_LSB);
  RSWire.endTransmission();
  // コントロールバイトを指定、バスリリース
  LSWire.beginTransmission(S11059_ADDR);
  LSWire.write(CONTROL_MSB);
  LSWire.write(CONTROL_2_LSB);
  LSWire.endTransmission();
  RSWire.beginTransmission(S11059_ADDR);
  RSWire.write(CONTROL_MSB);
  RSWire.write(CONTROL_2_LSB);
  RSWire.endTransmission();
}

void loop() {
  //テスト
  test_ss_loop();
  return;
  // 変数宣言
  int lower, higher, rr, rg, rb, rir, lr, lg, lb, lir;
  delay(10);
  // 出力データバイトを指定
  LSWire.beginTransmission(S11059_ADDR);
  LSWire.write(SENSOR_REGISTER);
  LSWire.endTransmission();
  RSWire.beginTransmission(S11059_ADDR);
  RSWire.write(SENSOR_REGISTER);
  RSWire.endTransmission();
  // 8バイトを要求し、大丈夫であれば上部、下部バイトと読み込み、それぞれの色の変数に入れる
  // 左側
  LSWire.requestFrom(S11059_ADDR, 8);
  if (LSWire.available()) {
    // 赤
    higher = LSWire.read();
    lower = LSWire.read();
    lr = higher << 8 | lower;
    // 緑
    higher = LSWire.read();
    lower = LSWire.read();
    lg = higher << 8 | lower;
    // 青
    higher = LSWire.read();
    lower = LSWire.read();
    lb = higher << 8 | lower;
    // 赤外線
    higher = LSWire.read();
    lower = LSWire.read();
    lir = higher << 8 | lower;
  }
  LSWire.endTransmission();
  //右側
  RSWire.requestFrom(S11059_ADDR, 8);
  if (RSWire.available()) {
    // 赤
    higher = RSWire.read();
    lower = RSWire.read();
    rr = higher << 8 | lower;
    // 緑
    higher = RSWire.read();
    lower = RSWire.read();
    rg = higher << 8 | lower;
    // 青
    higher = RSWire.read();
    lower = RSWire.read();
    rb = higher << 8 | lower;
    // 赤外線
    higher = RSWire.read();
    lower = RSWire.read();
    rir = higher << 8 | lower;
  }
  RSWire.endTransmission();
  // それぞれの値をもとに色を判別
  // 色は白=0、黒=1、緑=2、赤=3
  int left = 0;
  int right = 0;
  if (lr > L_W_R && lg > L_W_G && lb > L_W_B) {
    left = 0;
  }
  if (rr > R_W_R && rg > R_W_G && rb > R_W_B) {
    right = 0;
  }
  if (lr < L_B_R && lg < L_B_G && lb < L_B_B) {
    left = 1;
  }
  if (rr < R_B_R && rg < R_B_G && rb < R_B_B) {
    right = 1;
  }
  if (lr < L_G_R && lg > L_G_G && lb < L_G_B) {
    left = 2;
  }
  if (rr < R_G_R && rg > R_G_G && rb < R_G_B) {
    right = 2;
  }
  if (lr > L_R_R && lg < L_R_G && lb < L_R_B) {
    left = 3;
  }
  if (rr > R_R_R && rg < R_R_G && rb < R_R_B) {
    right = 3;
  }
  Serial.print(left * 10 + right); //左が10の位、右が1の位
  Serial.print('\n');
}