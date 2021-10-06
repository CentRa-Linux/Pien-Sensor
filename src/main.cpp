/*///////////////////////////////////////////////
//ピン配置
//SDA&SCL:I2C
//A6~A10:tpr-r,tpr-m,tpr-l,silver,tpr-back
//D2,D3:Motor Driver(PWM)
//D4~D7:Motor Driver(Digital)
//D22,D24:Sonic Sensor(Trig)
//D23,D25:Sonic Sensor(Echo)
//D26~D29:Touch Sensor
//motor_write()の最大値は256
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
#define P_TPR_B A10
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
#define P_T_B 29

//アドレス指定
#define S11059_ADDR 0x2A
#define CONTROL_MSB 0x00
#define CONTROL_1_LSB 0x89
#define CONTROL_2_LSB 0x09
#define SENSOR_REGISTER 0x03
#define BMX_MAG 0x13
//debug
#define COLOR_DEBUG

//Enum
enum Color{BLACK,WHITE,GREEN,RED,SILVER};

//int
int lower, higher, rr, rg, rb, rir, lr, lg, lb, lir,silver,tpr_l,tpr_m,tpr_r,xMag,yMag,zMag = 0;

//右から左へ
Color Line_Sensor[5];

//走行モード切替
bool isTraceing = true;

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

void bmx_init(){
  Wire.beginTransmission(BMX_MAG);
  Wire.write(0x4B);  // Select Mag register
  Wire.write(0x83);  // Soft reset
  Wire.endTransmission();
  Wire.beginTransmission(BMX_MAG);
  Wire.write(0x4B);  // Select Mag register
  Wire.write(0x01);  // Soft reset
  Wire.endTransmission();
  Wire.beginTransmission(BMX_MAG);
  Wire.write(0x4C);  // Select Mag register
  Wire.write(0x00);  // Normal Mode, ODR = 10 Hz
  Wire.endTransmission();
  Wire.beginTransmission(BMX_MAG);
  Wire.write(0x4E);  // Select Mag register
  Wire.write(0x84);  // X, Y, Z-Axis enabled
  Wire.endTransmission();
  Wire.beginTransmission(BMX_MAG);
  Wire.write(0x51);  // Select Mag register
  Wire.write(0x04);  // No. of Repetitions for X-Y Axis = 9
  Wire.endTransmission();
  Wire.beginTransmission(BMX_MAG);
  Wire.write(0x52);  // Select Mag register
  Wire.write(0x16);  // No. of Repetitions for Z-Axis = 15
  Wire.endTransmission();
}

void bmx_read(){
  unsigned int data[8];
  for(int i = 0;i < 8;i++){
    Wire.beginTransmission(BMX_MAG);
    Wire.write(0x42 + i);
    Wire.endTransmission();
    Wire.requestFrom(BMX_MAG,1);
    if(Wire.available() == 1){
      data[i] = Wire.read();
    }
  }
  xMag = ((data[1] << 5) | (data[0] >> 3));
  if(xMag > 4095)xMag -= 8192;
  yMag = ((data[3] << 5) | (data[2] >> 3));
  if(yMag > 4095)yMag -= 8192;
  zMag = ((data[5] << 7) | (data[4] >> 1));
  if(zMag > 16383)zMag -= 32768;
}

void setup() {
  #ifdef COLOR_DEBUG
  //ほげー
  #endif
  //ピンリセット
  pinMode(P_TPR_R,INPUT);
  pinMode(P_TPR_M,INPUT);
  pinMode(P_TPR_L,INPUT);
  pinMode(P_SILVER,INPUT);
  pinMode(P_TPR_B,INPUT);
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
  pinMode(P_T_B,INPUT);
  // シリアル開始
  Serial.begin(9600);
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
  silver = analogRead(P_SILVER);
  //TPR105読み取り
  tpr_r = analogRead(P_TPR_R);
  tpr_m = analogRead(P_TPR_M);
  tpr_l = analogRead(P_TPR_L);
}

void judge_color(){
  #define R_BORDER 0
  #define G_BORDER 0
  #define B_BORDER 0
  #define IR_BORDER 0
  #define TPR_BORDER 0
  #define SILVER_BORDER 0
  //I2C Color Sensor
  //right
  if(rr > R_BORDER){
    if(rg > G_BORDER){
      Line_Sensor[1] = WHITE;
    }else{
      Line_Sensor[1] = RED;
    }
  }else{
    if(rg > G_BORDER){
      Line_Sensor[1] = GREEN;
    }else{
      Line_Sensor[1] = BLACK;
    }
  }
  //left
  if(lr > R_BORDER){
    if(lg > G_BORDER){
      Line_Sensor[3] = WHITE;
    }else{
      Line_Sensor[3] = RED;
    }
  }else{
    if(lg > G_BORDER){
      Line_Sensor[3] = GREEN;
    }else{
      Line_Sensor[3] = BLACK;
    }
  }
  //TPR-r
  if(tpr_r > TPR_BORDER){
    Line_Sensor[0] = BLACK;
  }else{
    Line_Sensor[0] = WHITE;
  }
  //TPR-m
  if(tpr_r > TPR_BORDER){
    Line_Sensor[2] = BLACK;
  }else{
    Line_Sensor[2] = WHITE;
  }
  //TPR-l
  if(tpr_r > TPR_BORDER){
    Line_Sensor[4] = BLACK;
  }else{
    Line_Sensor[4] = WHITE;
  }
  //silver
  if(silver < SILVER_BORDER){
    isTraceing = false;
  }
}

void motor_write(int right,int left){
  //right
  if(right == 0){
    digitalWrite(P_M_A1,LOW);
    digitalWrite(P_M_A2,LOW);
  }else if(right > 0){
    digitalWrite(P_M_A1,HIGH);
    digitalWrite(P_M_A2,LOW);
    analogWrite(P_M_APWM,right);
  }else{
    digitalWrite(P_M_A1,LOW);
    digitalWrite(P_M_A2,HIGH);
    analogWrite(P_M_APWM,right);
  }
  //left
  if(left == 0){
    digitalWrite(P_M_B1,LOW);
    digitalWrite(P_M_B2,LOW);
  }else if(right > 0){
    digitalWrite(P_M_B1,HIGH);
    digitalWrite(P_M_B2,LOW);
    analogWrite(P_M_BPWM,left);
  }else{
    digitalWrite(P_M_B1,LOW);
    digitalWrite(P_M_B2,HIGH);
    analogWrite(P_M_BPWM,left);
  }
}

void detect_green(){
  bool isRightGreen,isLeftGreen;
      Line_Sensor[1] == GREEN ? isRightGreen = true : isRightGreen = false;
      Line_Sensor[3] == GREEN ? isLeftGreen = true : isLeftGreen = false;
      motor_write(0,0);
      delay(500);
      motor_write(64,64);
      while(Line_Sensor[1] == GREEN || Line_Sensor[3] == GREEN){
        color_read();
        judge_color();
      }
      motor_write(0,0);
      color_read();
      judge_color();
      if(Line_Sensor[2] == WHITE){
        Serial.println("What's wrong??? There is no detecton on middle sensor.");
      }
      if(Line_Sensor[0] == WHITE && Line_Sensor[4] == WHITE){
        //フェイク交差点だったとき
        Serial.println("An Fatal Error Has Occured.Program Will Exit.Stop Code:Black Line Not Found");
        return;
      }else{
        //THE☆交差点
        if(isRightGreen && isLeftGreen){
          //180ターン
        }else if(isRightGreen && !isLeftGreen){
          //右に90ターン
        }else if(!isRightGreen && isLeftGreen){
          //左に90ターン
        }
      }
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
  if(isTraceing){//ライントレースプログラム
    color_read();
    judge_color();
    if(!isTraceing)return;
    //特殊処理発生かどうか知りたい
    if((Line_Sensor[1] == WHITE || Line_Sensor[1] == BLACK) 
        && (Line_Sensor[3] == WHITE || Line_Sensor[3] == BLACK)){
      //白黒処理
      int right_value,left_value,right_colored,left_colored;
      right_colored = (rr + rg + rb) / 3;//だいたい最大値256-512くらい？
      left_colored = (lr + lg + lb) / 3;

      //右のP制御値
      //この辺は要検討
    }else if(Line_Sensor[1] == RED || Line_Sensor[3] == RED){
      //赤処理
      motor_write(0,0);
      delay(1000000);
    }else if(Line_Sensor[1] == GREEN || Line_Sensor[3] == GREEN){
      //緑処理
      detect_green();
    }else{
      Serial.println("Strange Error:No Color Found");
    }
  }
}