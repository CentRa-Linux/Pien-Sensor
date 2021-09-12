#include <Arduino.h>
#include <AsyncDelay.h>
#include <SoftWire.h>

//アドレス指定
#define S11059_ADDR 0x2A
#define CONTROL_MSB 0x00
#define CONTROL_1_LSB 0x89
#define CONTROL_2_LSB 0x09
#define SENSOR_REGISTER 0x03

// カラーセンサー2個に対し2個i2cのチャネルを贅沢に作らないと行けないっぽい
SoftWire RSWire(SDA, SCL);
SoftWire LSWire(A4, A5);
AsyncDelay readInterval;

void setup() {
  // シリアル開始
  Serial.begin(9600);
  // ソフトウェアi2c開始
  RSWire.begin();
  LSWire.begin();
  // コントロールバイトを指定、スリープ解除
  RSWire.beginTransmission(S11059_ADDR);
  RSWire.write(CONTROL_MSB);
  RSWire.write(CONTROL_1_LSB);
  RSWire.endTransmission();
  LSWire.beginTransmission(S11059_ADDR);
  LSWire.write(CONTROL_MSB);
  LSWire.write(CONTROL_1_LSB);
  LSWire.endTransmission();
  // コントロールバイトを指定、バスリリース
  RSWire.beginTransmission(S11059_ADDR);
  RSWire.write(CONTROL_MSB);
  RSWire.write(CONTROL_2_LSB);
  RSWire.endTransmission();
  LSWire.beginTransmission(S11059_ADDR);
  LSWire.write(CONTROL_MSB);
  LSWire.write(CONTROL_2_LSB);
  LSWire.endTransmission();
}

void loop() {
  // 変数宣言
  int lower, higher, rr, rg, rb, rir, lr, lg, lb, lir;
  delay(10);
  // 出力データバイトを指定
  RSWire.beginTransmission(S11059_ADDR);
  RSWire.write(SENSOR_REGISTER);
  RSWire.endTransmission();
  LSWire.beginTransmission(S11059_ADDR);
  LSWire.write(SENSOR_REGISTER);
  LSWire.endTransmission();
  // 8バイトを要求し、大丈夫であれば上部、下部バイトと読み込み、それぞれの色の変数に入れる
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
  // それぞれの値をスラッシュ区切りで改行までシリアルで送信
  Serial.print(rr);
  Serial.print('/');
  Serial.print(rg);
  Serial.print('/');
  Serial.print(rb);
  Serial.print('/');
  Serial.print(rir);
  Serial.print('/');
  Serial.print(lr);
  Serial.print('/');
  Serial.print(lg);
  Serial.print('/');
  Serial.print(lb);
  Serial.print('/');
  Serial.print(lir);
  Serial.print('\n');
}