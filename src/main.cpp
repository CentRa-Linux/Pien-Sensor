#include <Arduino.h>
#include <Wire.h>

//アドレス指定
#define S11059_ADDR 0x2A
#define CONTROL_MSB 0x00
#define CONTROL_1_LSB 0x89
#define CONTROL_2_LSB 0x09
#define SENSOR_REGISTER 0x03

//Enum
enum Color{black,white,green,red,silver};

//int
int lower, higher, rr, rg, rb, rir, lr, lg, lb, lir;

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
    //right-red
    higher = Wire.read();
    lower = Wire.read();
    lr = higher << 8 | lower;
    //right-green
    higher = Wire.read();
    lower = Wire.read();
    lg = higher << 8 | lower;
    //right-blue
    higher = Wire.read();
    lower = Wire.read();
    lb = higher << 8 | lower;
    //right-ir
    higher = Wire.read();
    lower = Wire.read();
    lir = higher << 8 | lower;
  }
  Wire.endTransmission();
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
}

void loop() {
  //テスト
  color_read();
  //test_sensor_loop();
  return;
  /*// 変数宣言
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
  // それぞれの値をカンマ区切りで改行までシリアルで送信
  // 右左それぞれ、赤、緑、青、赤外線と出す
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
  Serial.print('\n');*/
}