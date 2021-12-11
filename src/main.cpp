/*///////////////////////////////////////////////
//ピン配置
//SDA&SCL:I2C
//A12~A15:tpr-r,tpr-m,tpr-l,silver
//A11:tpr-back
//D2,D7:Motor Driver(PWM)
//D3~D6:Motor Driver(Digital)
//D23,D25:Sonic Sensor(Trig)
//D24,D26:Sonic Sensor(Echo)
//D44,D46,D48,D50,D52:Touch Sensor
//motor_write()の最大値は256
//タッチセンサーの値は反転するので注意
//調整必要項目
//・BMX055の中心点・拡大率など
//・ラインセンサー閾値
//・三角コーナーでどっちに回転するか
//・トレース制御部の定数項
//・救助コーナー内での転換方向
///////////////////////////////////////////////*/
#include <Arduino.h>
#include <Wire.h>
#include <PCA9685.h>

//ただのバカ
#define A 440
#define B 495
#define C 528
#define D 586
#define E 660
#define F 704
#define Fs 740
#define G 782
#define hiA 880
#define hiB B*2
#define hiC C*2
#define hiD D*2
#define hiE E*2
#define hiF F*2
#define hiG G*2
#define hiAf 830
#define off 1

//ピン指定
#define P_TPR_R A12
#define P_TPR_M A13
#define P_TPR_L A14
#define P_SILVER A15
#define P_TPR_B A11
#define P_M_APWM 5
#define P_M_A1 6
#define P_M_A2 7
#define P_M_BPWM 4
#define P_M_B1 3
#define P_M_B2 2
#define P_S_RT 53
#define P_S_LT 47
#define P_S_RE 51
#define P_S_LE 49
#define P_T_R 46
#define P_T_M 52
#define P_T_L 50
#define P_T_B 48
#define BZ 9

//アドレス指定
#define S11059_ADDR 0x2A
#define CONTROL_MSB 0x00
#define CONTROL_1_LSB 0x89
#define CONTROL_2_LSB 0x09
#define SENSOR_REGISTER 0x03
#define Addr_Accl 0x19
#define Addr_Gyro 0x69
#define Addr_Mag 0x13
#define ADDR_MAG 0x13
#define PCA9685_ADDR 0x40

//debug
//#define COLOR_DEBUG

//Servo's Hz Min And Max
#define SERVO_MIN 150
#define SERVO_MAX 480

//三角コーナー探すモードかどうか
#define CORNER_BORDER 10.0

//救助コーナーで曲がるほう
#define ROTATE_BORDER 20.0

//RCフィルタ(0 < x < 1)
const float RC = 0.5;
int before_x,before_y;

//救助コーナー内でライン検知しちゃったとき
#define BACK_LINE 1000

//Enum
enum Color{BLACK,WHITE,GREEN,RED,SILVER};
enum Bucket{RAISE,DOWN,RELEASE,HOLD};
enum State{TRACE,RESCUE,CORNER,SENPAN};

//int
int lower, higher, rr, rg, rb, rir, lr, lg, lb, lir,silver,tpr_l,tpr_m,tpr_r,xMag,yMag,zMag,count = 0;

//float
float xAccl,yAccl,zAccl;

//bmxのオフセット
const int bmx_x_os = -3;
const int bmx_y_os = 41;

//右から左へ
Color Line_Sensor[5];
Color Previous_Line_Sensor[5];

//走行モード切替
State state = TRACE;

//PCA9685変数
PCA9685 pwm = PCA9685(0x40);// PCA9685のI2Cアドレスを指定

//silver
bool isSilver = false;

//Timer
long timer;

//前回の値保持(P操作量)
int volume;
int previous_volume; 
 
 int Servo_Pin = 0;      // サーボ接続ピンを0番に
 int angle;

 //前回のオール白の時の時間
 long white_time;

//愚かだなぁ
int tune;
int t;
void oto(int tune,int t,int d) {
    tone(BZ,tune);
    delay(t);
    noTone(9);
    delay(d);
}

//TCA9548A
void change_i2c_port(int byte){
  Wire.beginTransmission(0x70);
  Wire.write(1 << byte);
  Wire.endTransmission();
}

void buzzer(int frq){
  tone(BZ,frq);
  delay(1000);
  noTone(BZ);
}

void motor_write(int right,int left){
  //right
  if(right == 0){
    digitalWrite(P_M_A1,LOW);
    digitalWrite(P_M_A2,LOW);
  }else if(right > 0){
    digitalWrite(P_M_A1,HIGH);
    digitalWrite(P_M_A2,LOW);
    analogWrite(P_M_APWM,abs(right));
  }else{
    digitalWrite(P_M_A1,LOW);
    digitalWrite(P_M_A2,HIGH);
    analogWrite(P_M_APWM,abs(right));
  }
  //left
  if(left == 0){
    digitalWrite(P_M_B1,LOW);
    digitalWrite(P_M_B2,LOW);
  }else if(left > 0){
    digitalWrite(P_M_B1,HIGH);
    digitalWrite(P_M_B2,LOW);
    analogWrite(P_M_BPWM,abs(left));
  }else{
    digitalWrite(P_M_B1,LOW);
    digitalWrite(P_M_B2,HIGH);
    analogWrite(P_M_BPWM,abs(left));
  }
}

void bmx_init(){
  change_i2c_port(2);
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x0F);
  Wire.write(0x03);
  Wire.endTransmission();

  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x10);
  Wire.write(0x08);
  Wire.endTransmission();

  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x11);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x0F);
  Wire.write(0x04);
  Wire.endTransmission();

  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x10);
  Wire.write(0x07);
  Wire.endTransmission();
  
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x11);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B);
  Wire.write(0x83);
  Wire.endTransmission();
  
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B);
  Wire.write(0x01);
  Wire.endTransmission();

  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4C);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4E);
  Wire.write(0x84);
  Wire.endTransmission();

  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x51);
  Wire.write(0x04);
  Wire.endTransmission();
}

void i2c_init(){
  Wire.begin();
  change_i2c_port(3);
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(1000);
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
  //地磁気センサーリセット
  bmx_init();
}

void judge_color(){
  #define RR_BORDER 250
  #define RG_BORDER 290
  #define RB_BORDER 260
  #define RIR_BORDER 200
  #define LR_BORDER 268
  #define LG_BORDER 340
  #define LB_BORDER 200
  #define LIR_BORDER 200
  #define R_TPR_BORDER 450
  #define M_TPR_BORDER 29
  #define L_TPR_BORDER 450
  #define B_TPR_BORDER 100
  #define SILVER_BORDER 570
  #define R_GpR_MIN 1.0
  #define R_GpR_MAX 1.7
  #define L_GpR_MIN 1.0
  #define L_GpR_MAX 1.7

  //前回の値を移動
  for(int i = 0;i < 5;i++) Previous_Line_Sensor[i] = Line_Sensor[i];
  //I2C Color Sensor
  //right
  if(rr > RR_BORDER){
    if(rg > RG_BORDER){
      Line_Sensor[1] = WHITE;
    }else{
      if((float)rg/rr > R_GpR_MIN && (float)rg/rr < R_GpR_MAX){
        Line_Sensor[1] = WHITE;
      }else{
        Line_Sensor[1] = RED;

      }
    }
  }else{
    if(rg > RG_BORDER){
      if((float)rg/rr > R_GpR_MIN && (float)rg/rr < R_GpR_MAX){
        Line_Sensor[1] = BLACK;
      }else{
        Line_Sensor[1] = GREEN;
      }
    }else{
      Line_Sensor[1] = BLACK;
    }
  }
  //left
  if(lr > LR_BORDER){
    if(lg > LG_BORDER){
      Line_Sensor[3] = WHITE;
    }else{
      if((float)lg/lr > L_GpR_MIN && (float)lg/lr < L_GpR_MAX){
        Line_Sensor[3] = WHITE;
      }else{
        Line_Sensor[3] = RED;
      }
    }
  }else{
    if(lg > LG_BORDER){
      if((float)lg/lr > L_GpR_MIN && (float)lg/lr < L_GpR_MAX){
        Line_Sensor[3] = BLACK;
      }else{
        Line_Sensor[3] = GREEN;
      }
    }else{
      Line_Sensor[3] = BLACK;
    }
  }
  //TPR-r
  if(tpr_r > R_TPR_BORDER){
    Line_Sensor[0] = BLACK;
  }else{
    Line_Sensor[0] = WHITE;
  }
  //TPR-m
  if(tpr_m > M_TPR_BORDER){
    Line_Sensor[2] = BLACK;
  }else{
    Line_Sensor[2] = WHITE;
  }
  //TPR-l
  if(tpr_l > L_TPR_BORDER){
    Line_Sensor[4] = BLACK;
  }else{
    Line_Sensor[4] = WHITE;
  }
  //silver
  /*if((rr > 1500 && rg > 1500) || (lr > 1500 && lg > 1500)){
    isSilver = true;
  }*/
  if(silver < SILVER_BORDER){
    isSilver = true;
  }
}

/*void judge_color(){
  //こっちは比を取る方
  #define RR_BORDER 250
  #define RG_BORDER 290
  #define RB_BORDER 260
  #define RIR_BORDER 200
  #define LR_BORDER 268
  #define LG_BORDER 340
  #define LB_BORDER 200
  #define LIR_BORDER 200
  #define R_TPR_BORDER 450
  #define M_TPR_BORDER 29
  #define L_TPR_BORDER 450
  #define B_TPR_BORDER 100
  #define SILVER_BORDER 300
  #define R_GpR_MIN 1.0
  #define R_GpR_MAX 1.7
  #define L_GpR_MIN 1.0
  #define L_GpR_MAX 1.7

  //前回の値を移動
  for(int i = 0;i < 5;i++) Previous_Line_Sensor[i] = Line_Sensor[i];
  //I2C Color Sensor
  //right
  float right_ratio = rg / rr;
  if(right_ratio > R_GpR_MIN && right_ratio < R_GpR_MAX){
    //白or黒
    if(rr < RR_BORDER && rg < RG_BORDER){
      //黒
      Line_Sensor[1] = BLACK;
    }else{
      //白
      Line_Sensor[1] = WHITE;
    }
  }else if(right_ratio > R_GpR_MAX){
    //緑
    Line_Sensor[1] = GREEN;
  }
  else if(right_ratio < R_GpR_MIN){
    //赤
    Line_Sensor[1] = RED;
  }
  //left
  float left_ratio = lg / lr;
  if(left_ratio > L_GpR_MIN && left_ratio < L_GpR_MAX){
    //白or黒
    if(lr < LR_BORDER && lg < LG_BORDER){
      //黒
      Line_Sensor[3] = BLACK;
    }else{
      //白
      Line_Sensor[3] = WHITE;
    }
  }else if(left_ratio > L_GpR_MAX){
    //緑
    Line_Sensor[3] = GREEN;
  }
  else if(left_ratio < L_GpR_MIN){
    //赤
    Line_Sensor[3] = RED;
  }
  //TPR-r
  if(tpr_r > R_TPR_BORDER){
    Line_Sensor[0] = BLACK;
  }else{
    Line_Sensor[0] = WHITE;
  }
  //TPR-m
  if(tpr_m > M_TPR_BORDER){
    Line_Sensor[2] = BLACK;
  }else{
    Line_Sensor[2] = WHITE;
  }
  //TPR-l
  if(tpr_l > L_TPR_BORDER){
    Line_Sensor[4] = BLACK;
  }else{
    Line_Sensor[4] = WHITE;
  }
  //silver
  if((rr > 1500 && rg > 1500) || (lr > 1500 && lg > 1500)){
    isSilver = true;
  }
  if(silver < SILVER_BORDER){
    isSilver = true;
  }
}*/

void color_read(){
  #define ERROR 15
  //カラーセンサー読み取り
  int trr,trg,trb,trir,tlr,tlg,tlb,tlir;
  change_i2c_port(1);
  Wire.beginTransmission(S11059_ADDR);
  Wire.write(SENSOR_REGISTER);
  Wire.endTransmission();
  change_i2c_port(0);
  Wire.beginTransmission(S11059_ADDR);
  Wire.write(SENSOR_REGISTER);
  Wire.endTransmission();
  change_i2c_port(1);
  Wire.requestFrom(S11059_ADDR,8);
  if(Wire.available()){
    //right-red
    higher = Wire.read();
    lower = Wire.read();
    trr = higher << 8 | lower;
    //right-green
    higher = Wire.read();
    lower = Wire.read();
    trg = higher << 8 | lower;
    //right-blue
    higher = Wire.read();
    lower = Wire.read();
    trb = higher << 8 | lower;
    //right-ir
    higher = Wire.read();
    lower = Wire.read();
    trir = higher << 8 | lower;
  }
  Wire.endTransmission();
  change_i2c_port(0);
  Wire.requestFrom(S11059_ADDR,8);
  if(Wire.available()){
    //left-red
    higher = Wire.read();
    lower = Wire.read();
    tlr = higher << 8 | lower;
    //left-green
    higher = Wire.read();
    lower = Wire.read();
    tlg = higher << 8 | lower;
    //left-blue
    higher = Wire.read();
    lower = Wire.read();
    tlb = higher << 8 | lower;
    //left-ir
    higher = Wire.read();
    lower = Wire.read();
    tlir = higher << 8 | lower;
  }
  Wire.endTransmission();

  //通信エラー検知
  if(trr > ERROR)rr = trr;
  //else restart_i2c();
  if(trg > ERROR)rg = trg;
  //else restart_i2c();
  if(trb > ERROR)rb = trb;
  //else restart_i2c();
  if(tlr > ERROR)lr = tlr;
  //else restart_i2c();
  if(tlg > ERROR)lg = tlg;
  //else restart_i2c();
  if(tlb > ERROR)lb = tlb;
  //else restart_i2c();
  rir = trir;
  lir = tlir;

  //銀検知読み取り
  silver = analogRead(P_SILVER);
  //TPR105読み取り
  tpr_r = analogRead(P_TPR_R);
  tpr_m = analogRead(P_TPR_M);
  tpr_l = analogRead(P_TPR_L);
}

void yabee_reset_sityattao(){
  motor_write(-48,-48);
  while(Line_Sensor[0] == WHITE && Line_Sensor[1] == WHITE
  && Line_Sensor[3] == WHITE && Line_Sensor[4] == WHITE){
    color_read();
    judge_color();
  }
  motor_write(0,0);
  delay(500);
}

void setup(){
  /*#ifdef COLOR_DEBUG
  //ほげー
  test_sensor_setup();
  return;
  #endif*/
  //ピンセット
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
  pinMode(BZ,OUTPUT);
  // シリアル開始
  Serial.begin(9600);
  //サーボ初期化・I2Cスタート
  i2c_init();
  color_read();
  judge_color();
  /*if(Line_Sensor[0] == WHITE && Line_Sensor[1] == WHITE
  && Line_Sensor[3] == WHITE && Line_Sensor[4] == WHITE){
    yabee_reset_sityattao();
  }*/
  motor_write(-48,-48);
  delay(500);
  motor_write(0,0);
  delay(250);
}

void restart_i2c(){
  Wire.end();
  motor_write(0,0);
  buzzer(470);
  i2c_init();
  buzzer(570);
}

void bmx_fixer(){
  xMag += bmx_x_os;
  yMag += bmx_y_os;
  //xMag = RC * before_x + (1 - RC) * xMag;
  //yMag = RC * before_y + (1 - RC) * yMag;
}

void bmx_maguro(){
  //鮪
  change_i2c_port(2);
  unsigned int data[8];
  for(int i = 0;i < 8;i++){
    Wire.beginTransmission(Addr_Mag);
    Wire.write(0x42 + i);
    Wire.endTransmission();
    Wire.requestFrom(Addr_Mag,1);
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
  bmx_fixer();
}

void bmx_accl(){
  //加速度
  unsigned int ac_data[6];
  change_i2c_port(2);
  for (int i = 0; i < 6; i++)
  {
    Wire.beginTransmission(Addr_Accl);
    Wire.write((2 + i));
    Wire.endTransmission();
    Wire.requestFrom(Addr_Accl, 1);
    if (Wire.available() == 1) ac_data[i] = Wire.read();
  }
  xAccl = ((ac_data[1] * 256) + (ac_data[0] & 0xF0)) / 16;
  if (xAccl > 2047)  xAccl -= 4096;
  yAccl = ((ac_data[3] * 256) + (ac_data[2] & 0xF0)) / 16;
  if (yAccl > 2047)  yAccl -= 4096;
  zAccl = ((ac_data[5] * 256) + (ac_data[4] & 0xF0)) / 16;
  if (zAccl > 2047)  zAccl -= 4096;
  xAccl = xAccl * 0.0098;
  yAccl = yAccl * 0.0098;
  zAccl = zAccl * 0.0098;
}

void test_sensor_setup(){
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(S11059_ADDR);
  Wire.write(CONTROL_MSB);
  Wire.write(CONTROL_1_LSB);
  Wire.endTransmission();
  Wire.beginTransmission(S11059_ADDR);
  Wire.write(CONTROL_MSB);
  Wire.write(CONTROL_2_LSB);
  Wire.endTransmission();
  bmx_init();
  Serial.println("uwu");
}

void servo_write(Bucket mode){
  change_i2c_port(3);
  if(mode == RAISE){
    buzzer(190);
    angle = 0;
    angle = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
    pwm.setPWM(14,0,SERVO_MAX);
    pwm.setPWM(15,0,SERVO_MIN);
    delay(1000);
  }else if(mode == DOWN){
    buzzer(290);
    angle = 180;
    angle = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
    pwm.setPWM(14,0,SERVO_MIN);
    pwm.setPWM(15,0,SERVO_MAX);   
    delay(1000);
  }else if(mode == RELEASE){
    pwm.setPWM(2,0,SERVO_MAX);
  }else if(mode == HOLD){
    pwm.setPWM(2,0,SERVO_MIN);
  }
}

void check_voltage(){
  int ahobakashine = analogRead(A11);
  if(ahobakashine < 510){
    motor_write(0,0);
    while(true){
      Serial.println("Battery is LOW");
      buzzer(440);
      delay(1000);
    }
  }
}

void p_trace(int BASE,float GAIN){
  #define INSIDE 0.7
  #define MOTOR_MAX 96
  #define MOTOR_MIN -96
  #define GREEN_VALUE 10
  #define OUTSIDE 0.38
  const int r_target = 450;
  const int l_target = 520;
  int r_v = (rr + rg + rb) / 3 - r_target;
  int l_v = (lr + lg + lb) / 3 - l_target;
  int in_val = r_v - l_v;
  in_val *= INSIDE;
  int out_val = tpr_l - tpr_r;
  out_val *= OUTSIDE;
  int ctrl = in_val + out_val;
  ctrl *= GAIN;
  int rm = BASE + ctrl;
  int lm = BASE - ctrl;
  if(rm > MOTOR_MAX)rm = MOTOR_MAX;
  if(lm > MOTOR_MAX)lm = MOTOR_MAX;
  if(rm < MOTOR_MIN)rm = MOTOR_MIN;
  if(lm < MOTOR_MIN)lm = MOTOR_MIN;
  motor_write(rm,lm);
  Serial.print(rm);
  Serial.print(",");
  Serial.println(out_val);
}

int tan2angle(int x,int y){
  return atan2(y,x) * 180 / PI;
}

void check_bmx(){
  int xmax,xmin,ymax,ymin;
  bmx_maguro();
  xmax = xMag;
  xmin = xMag;
  ymax = yMag;
  ymin = yMag;
  while(true){
    bmx_maguro();
    if(xMag > xmax)xmax = xMag;
    if(xMag < xmin)xmin = xMag;
    if(yMag > ymax)ymax = yMag;
    if(yMag < ymin)ymin = yMag;
    Serial.print(xmin);
    Serial.print(",");
    Serial.print(xmax);
    Serial.print(",");
    Serial.print(ymin);
    Serial.print(",");
    Serial.println(ymax);
  }
}

void rotate(int angle,int mode){
  #define ALLOW_DIF 5
  #define BEGIN_CHECK 65
  #define DELAY_TIME 250
  Serial.println("begin to rotate");
  motor_write(0,0);
  delay(500);
  bmx_maguro();
  int n_angle = tan2angle(xMag,yMag);
  int target = n_angle + angle;
  int origin = target;
  Serial.print(n_angle);
  Serial.print(",");
  Serial.println(target);
  int begin2check = 0;
  if(angle > 0){
    begin2check = target - BEGIN_CHECK;
  }else if(angle < 0){
    begin2check = target + BEGIN_CHECK;
  }
  if(angle > 0){
    //時計回り
    motor_write(-72,72);
    if(target > 180)target -= 360;
    while(true){
      bmx_maguro();
      n_angle = tan2angle(xMag,yMag);
      if(abs(target - n_angle) < ALLOW_DIF
      || abs(origin - n_angle) < ALLOW_DIF){
        Serial.println("finish:normal mode");
        break;
      }
      if((abs(target - n_angle) < BEGIN_CHECK
      || abs(origin - n_angle) < BEGIN_CHECK) && mode == 1){
        color_read();
        judge_color();
        if(Line_Sensor[2] == BLACK){
          Serial.println("finish:line mode");
          delay(DELAY_TIME);
          break;
        }
      }
    }
  }else if(angle < 0){
    //反時計回り
    motor_write(72,-72);
    if(target < -180)target += 360;
    while(true){
      bmx_maguro();
      n_angle = tan2angle(xMag,yMag);
      if(abs(target - n_angle) < ALLOW_DIF
      || abs(origin - n_angle) < ALLOW_DIF){
        Serial.println("finish:normal mode");
        break;
      }
      if((abs(target - n_angle) < BEGIN_CHECK
      || abs(origin - n_angle) < BEGIN_CHECK) && mode == 1){
        color_read();
        judge_color();
        if(Line_Sensor[2] == BLACK){
          Serial.println("finish:line mode");
          delay(DELAY_TIME);
          break;
        }
      }
    }
  }else return;
  motor_write(0,0);
  Serial.println("I've done it!");
}

void detect_green(){
  #define STRAIGHT 400
  #define GREEN_TIME 250
  bool isRightGreen = false;
  bool isLeftGreen = false;
  motor_write(0,0);
  buzzer(500);
  while(Line_Sensor[1] == GREEN || Line_Sensor[3] == GREEN){
    color_read();
    judge_color();
    p_trace(36,0.8);
    if(Line_Sensor[1] == GREEN)isRightGreen = true;
    if(Line_Sensor[3] == GREEN)isLeftGreen = true;
  Serial.print(isRightGreen);
  Serial.print(",");
  Serial.println(isLeftGreen);
  }
  motor_write(0,0);
  color_read();
  judge_color();
  if(Line_Sensor[0] == WHITE && Line_Sensor[4] == WHITE){
    //フェイク交差点だったとき
    Serial.println("An Fatal Error Has Occured.Program Will Exit.Stop Code:Black Line Not Found");
    buzzer(1000);
    return;
  }else{
    //THE☆交差点
    if(isRightGreen && isLeftGreen){
      //180ターン
      Serial.println("turning 180");
      buzzer(700);
      motor_write(48,48);
      delay(STRAIGHT);
      motor_write(0,0);
      rotate(180,1);
      delay(1000);
    }else if(isRightGreen && !isLeftGreen){
      //右に90ターン
      Serial.println("turning 90");
      buzzer(800);
      motor_write(48,48);
      delay(STRAIGHT);
      motor_write(0,0);
      rotate(90,1);
      delay(1000);
    }else if(!isRightGreen && isLeftGreen){
      //左に90ターン
      Serial.println("turning -90");
      buzzer(600);
      motor_write(48,48);
      delay(STRAIGHT);
      motor_write(0,0);
      rotate(-90,1);
      delay(1000);
    }
    motor_write(36,36);
    delay(250);
  }
}

bool touch_sensor(){
  bool isTouched = false;
  digitalRead(P_T_M) == HIGH ? isTouched = true : isTouched = isTouched;
  return isTouched;
}

float sonic_sensor_right(){
  float du,dis = 0;
  digitalWrite(P_S_RT,LOW);
  delayMicroseconds(2);
  digitalWrite(P_S_RT,HIGH);
  delayMicroseconds(10);
  digitalWrite(P_S_RT,LOW);
  du = pulseIn(P_S_RE,HIGH);
  if(du > 0){
    du = du / 2;
    dis = du * 340 * 100 / 1000000;
  }
  return dis;
}

float sonic_sensor_left(){
  float du,dis = 0;
  digitalWrite(P_S_LT,LOW);
  delayMicroseconds(2);
  digitalWrite(P_S_LT,HIGH);
  delayMicroseconds(10);
  digitalWrite(P_S_LT,LOW);
  du = pulseIn(P_S_LE,HIGH);
  if(du > 0){
    du = du / 2;
    dis = du * 340 * 100 / 1000000;
  }
  return dis;
}

void go_straight(float r_base,float l_base){
  #define VOL 36
  #define BAKA 60
  #define DIF 2.0
  float hoge;
  hoge = sonic_sensor_right();
  if(hoge > 500.0){
    motor_write(BAKA,BAKA);
  }else if(hoge > r_base + DIF){
    motor_write(60 - VOL,60 + VOL);
  }else if(hoge < r_base){
    motor_write(60 + VOL,60 - VOL);
  }else{
    motor_write(BAKA,BAKA);
  }
  Serial.println(hoge);
}

void avoid_object(){
  #define GAIN 5
  #define VEHCLE 18.0
  const float distance = 6.0;
  const float allow_miss = 2.0;
  #define MOTOR_VAL 24
  Serial.println("begin to avoid obj");
  motor_write(0,0);
  buzzer(300);
  delay(100);
  motor_write(-48,-48);
  delay(500);
  motor_write(0,0);
  delay(100);
  //90度回転
  //右を障害物に向けるよ
  rotate(-90,0);
  delay(250);
  motor_write(36,36);
  delay(100);
  while(Line_Sensor[0] == WHITE && Line_Sensor[4] == WHITE){
    if(sonic_sensor_right() > distance + allow_miss){
      motor_write(24,24 * (distance + VEHCLE) / distance);
    }else if(sonic_sensor_right() < distance){
      motor_write(24 * (distance + VEHCLE) / distance,24);
    }else{
      motor_write(36,36);
    }
    color_read();
    judge_color();
  }
  motor_write(0,0);
  //向きを戻すよ
  delay(100);
  motor_write(36,36);
  delay(200);
  rotate(-90,1);/*
  motor_write(-64,-64);
  while(digitalRead(P_T_B) == HIGH);
  motor_write(0,0);*/
  Serial.println("finish avoiding obj");
}

void check_color(){
  for(int i = 0;i < 5;i++){
    Serial.print(Line_Sensor[i]);
    Serial.print(",");
  }
  Serial.println("");
}

void looking_corner(){
  //三角コーナーコード
  servo_write(DOWN);
  motor_write(48,48);
  float r_ori,l_ori;
  r_ori = sonic_sensor_right();
  l_ori = sonic_sensor_left();
  while(digitalRead(P_T_M) == LOW/*digitalRead(P_T_R) == LOW && digitalRead(P_T_L) == LOW*/){
    //まっすぐ進む
    go_straight(r_ori,l_ori);
    color_read();
    judge_color();
    if(Line_Sensor[1] != WHITE || Line_Sensor[3] != WHITE || isSilver){
      motor_write(0,0);
      buzzer(850);
      motor_write(-48,-48);
      delay(BACK_LINE);
      motor_write(0,0);
      delay(500);
      rotate(90,0);
      delay(500);
    }
  }
  delay(250);
  motor_write(0,0);
  if(digitalRead(P_T_R) == LOW && digitalRead(P_T_L) == LOW){
    //違うらしいよ
    //右に回転
    rotate(90,0);
    return;
  }else{
    //三角コーナーじゃん！
    servo_write(RAISE);
    bmx_maguro();
    rotate(135,0);
    motor_write(-64,-64);
    while(digitalRead(P_T_B) == LOW);
    delay(250);
    motor_write(0,0);
    if(true/*analogRead(P_TPR_B) > B_TPR_BORDER*/){
      //黒の時
      servo_write(RELEASE);
      delay(500);
      servo_write(HOLD);
      for(int i = 0;i < 3;i++){
        motor_write(48,48);
        delay(1500);
        motor_write(0,0);
        delay(250);
        motor_write(-48,-48);
        while(digitalRead(P_T_B) == HIGH);
        motor_write(0,0);
        servo_write(RELEASE);
        delay(500);
        servo_write(HOLD);
        delay(250);
      }
      rotate(-45,0);
      buzzer(600);
      state = SENPAN;
      return;
    }else{
      //三角コーナーじゃなくて壁の時の処理
      //考えてないよ☆（ゝω・）vｷｬﾋﾟ
    }
  }
}

void famima(){
  int d;
  d=40;
  oto(Fs,7*d,d);
  oto(D,7*d,d);
  oto(A,7*d,d);
  oto(D,7*d,d);
  oto(E,7*d,d);
  oto(hiA,7*d,8*d);
  oto(A,7*d,d);
  oto(E,7*d,d);
  oto(Fs,7*d,d);
  oto(E,7*d,d);
  oto(A,7*d,d);
  oto(D,7*d,d);
}

void done_escape(){
  isSilver = false;
  motor_write(0,0);
  famima();
  motor_write(48,48);
  delay(1000);
  motor_write(0,0);
  state = TRACE;
  return;
}

bool isEvent(){
  if(Line_Sensor[1] != GREEN && Line_Sensor[3] != GREEN && !isSilver
    && digitalRead(P_T_M) == LOW){
      return false;
    }else{
      return true;
    }
}

void nakamura_is_senpan(){
  //基本は時計回りだよ
  //穴を見つけたら行ってみてダメなら180度ターン
  float r_ori,l_ori;
  r_ori = sonic_sensor_right();
  l_ori = sonic_sensor_left();
  while(true){
    color_read();
    judge_color();
    go_straight(r_ori,l_ori);
    if(isEvent()){
      motor_write(0,0);
      buzzer(1000);
      if(touch_sensor()){
        tone(BZ,1000);
        //タッチセンサー反応処理
        if(sonic_sensor_right() > ROTATE_BORDER){
          rotate(90,0);
          r_ori = sonic_sensor_right();
          l_ori = sonic_sensor_left();
          go_straight(r_ori,l_ori);
          while(!isEvent()){
            color_read();
            judge_color();
            go_straight(r_ori,l_ori);
          }
          if(isSilver){
            done_escape();
          }else{
            motor_write(0,0);
            rotate(180,0);
            r_ori = sonic_sensor_right();
            l_ori = sonic_sensor_left();
          }
        }else{
          rotate(-90,0);
          r_ori = sonic_sensor_right();
          l_ori = sonic_sensor_left();
        }
      }else{
        if(isSilver){
          done_escape();
          return;
        }else{
          motor_write(-48,-48);
          delay(BACK_LINE);
          motor_write(0,0);
          rotate(-90,0);
          r_ori = sonic_sensor_right();
          l_ori = sonic_sensor_left();
        }
      }
    }
  }
}

bool check_color_match(){
  if(Previous_Line_Sensor[0] == Line_Sensor[0])return true;
  if(Previous_Line_Sensor[1] == Line_Sensor[1])return true;
  if(Previous_Line_Sensor[2] == Line_Sensor[2])return true;
  if(Previous_Line_Sensor[3] == Line_Sensor[3])return true;
  if(Previous_Line_Sensor[4] == Line_Sensor[4])return true;
  return false;
}

void p_trace_v2(){
  //PIDみたいなことをやってみる
  //P:ライン位置により定数変更
  //I:時間積分、ライン位置により定数変更
  //D:時間微分
  #define P_V2_OUTSIDE 50
  #define P_V2_INSIDE 10
  #define INTEGRAL 1.2
  #define DELTA 2.0
  previous_volume = volume;
  int f_tpr_r,f_tpr_l;
  f_tpr_r = 1024 - tpr_r;
  f_tpr_l = 1024 - tpr_l;
  //めんどくせー！
  //右が正の操作量
  volume = P_V2_INSIDE * ((rr + rg + rb) / 3 - (lr + lg + lb) / 3) + P_V2_OUTSIDE * (f_tpr_r - f_tpr_l);
  Line_Sensor[2] == WHITE ? volume = volume : volume = volume * 2;
  if(volume > 192) volume = 192;
  else if(volume < -192) volume = -192;
  //I
  long now_time = millis();
  if(check_color_match()) timer = now_time;
  int op_integral = INTEGRAL * volume * (millis() - timer) / 1000;
  //D
  int op_delta = DELTA * (previous_volume - volume);
  int operation = (volume + op_integral + op_delta) / 3;
  if(operation > 192) operation = 192;
  else if(operation < -192) operation = -192;
  motor_write(128 + operation /2,128 - operation / 2);
}

void saua(){
  int d;
  d=20;
  //zensou
  oto(D,4*d,8*d);
  oto(D,4*d,8*d);
  oto(C,7*d,d);
  oto(D,4*d,8*d);
  oto(D,4*d,8*d);
  oto(C,7*d,d);
  oto(D,4*d,8*d);
  oto(D,4*d,8*d);
  oto(C,7*d,d);
  oto(D,15*d,d);
  oto(F,15*d,d);
  oto(D,4*d,8*d);
  oto(D,4*d,8*d);
  oto(C,7*d,d);
  oto(D,4*d,8*d);
  oto(D,4*d,8*d);
  oto(C,7*d,d);
  oto(D,15*d,d);
  oto(F,15*d,d);
  oto(G,15*d,d);
  oto(hiA,15*d,d);
  // Amero
  oto(G,7*d,d);
  oto(hiA,7*d,d);
  oto(D,4*d,0);
  oto(C,4*d,0);
  oto(D,4*d,0);
  oto(C,4*d,0);
  oto(G,7*d,d);
  oto(hiA,7*d,d);
  oto(D,4*d,0);
  oto(C,4*d,0);
  oto(D,4*d,0);
  oto(C,4*d,0);
  oto(G,7*d,d);
  oto(hiA,7*d,d);
  oto(D,4*d,0);
  oto(C,4*d,0);
  oto(D,4*d,0);
  oto(C,4*d,0);
  oto(F,8*d,0);
  oto(E,2.5*d,0);
  oto(F,2.5*d,0);
  oto(E,2.5*d,0.5);
  oto(D,8*d,0);
  oto(C,8*d,0);
  oto(G,7*d,d);
  oto(hiA,7*d,d);
  oto(D,4*d,0);
  oto(C,4*d,0);
  oto(D,4*d,0);
  oto(C,4*d,0);
  oto(G,7*d,d);
  oto(hiA,7*d,d);
  oto(D,4*d,0);
  oto(C,4*d,0);
  oto(D,4*d,0);
  oto(C,4*d,0);
  oto(G,8*d,0);
  oto(hiA,8*d,0);
  oto(hiC,8*d,0);
  oto(hiF,8*d,0);
  oto(hiE,4*d,0);
  oto(hiF,4*d,0);
  oto(hiE,4*d,0);
  oto(hiD,4*d,0);
  oto(hiC,8*d,0);
  oto(hiA,8*d,0);
  //Amero ushio
  oto(G,7*d,d);
  oto(hiA,7*d,d);
  oto(D,4*d,0);
  oto(C,4*d,0);
  oto(D,4*d,0);
  oto(C,4*d,0);
  oto(G,7*d,d);
  oto(hiA,7*d,d);
  oto(D,4*d,0);
  oto(C,4*d,0);
  oto(D,4*d,0);
  oto(C,4*d,0);
  oto(G,7*d,d);
  oto(hiA,7*d,d);
  oto(D,4*d,0);
  oto(C,4*d,0);
  oto(D,4*d,0);
  oto(C,4*d,0);
  oto(F,8*d,0);
  oto(E,2.5*d,0);
  oto(F,2.5*d,0);
  oto(E,2.5*d,0.5);
  oto(D,8*d,0);
  oto(C,8*d,0);
  oto(D,8*d,0);
  oto(C,4*d,0);
  oto(D,4*d,0);
  oto(F,8*d,0);
  oto(D,4*d,0);
  oto(G,4*d,0);
  oto(hiA,8*d,0);
  oto(G,4*d,0);
  oto(hiA,4*d,0);
  oto(hiC,4*d,0);
  oto(hiF,4*d,0);
  oto(hiA,4*d,0);
  oto(hiC,4*d,0);
  oto(hiF,8*d,0);
  oto(hiE,2*d,0);
  oto(hiF,4*d,0);
  oto(hiE,2*d,0);
  oto(hiD,8*d,0);
  oto(hiC,8*d,0);
  oto(hiD,12*d,0);
}

void test_sensor_loop(){/*
  float r = sonic_sensor_right();
  float l = sonic_sensor_left();
  while(true){
    go_straight(r,l);
  }
  /*color_read();
  Serial.print((rr + rg + rb) / 3);
  Serial.print(",");
  Serial.println((lr + lg + lb) / 3);
  //Serial.println(silver);
  return;
  //motor_write(72,0);
  /*servo_write(RAISE);
  delay(2000);
  servo_write(DOWN);
  delay(2000);*/
  /*rotate(90);
  delay(2000);
  rotate(-90);
  delay(2000);
  //servo_write(RAISE);
  //servo_write(DOWN);*//*
  Serial.print(sonic_sensor_right());
  Serial.print(",");
  Serial.println(sonic_sensor_left());
  return;*/
  color_read();
  judge_color();
  Serial.println(silver);
  return;
  /*Serial.print(rr);
  Serial.print(",");
  Serial.print(rg);
  Serial.print(",");
  Serial.print(rb);
  Serial.print(",");
  Serial.print(lr);
  Serial.print(",");
  Serial.print(lg);
  Serial.print(",");
  Serial.print(lb);
  Serial.print(",");
  Serial.print((float)rg/rr);
  Serial.print(",");
  Serial.println((float)lg/lr);*/
  for(int i = 0;i < 5;i++){
    Serial.print(Line_Sensor[i]);
    Serial.print(",");
  }/*
  Serial.println("");*//*
  check_bmx();
  bmx_maguro();
  Serial.print(xMag);
  Serial.print(",");/*
  Serial.print(yMag);
  Serial.print(",");
  Serial.println(atan2(yMag,xMag) * 180 / PI);//*/
  /*color_read();
  judge_color();
  p_trace(54,1.0);
  check_color();*/
  //Serial.println(sonic_sensor_right());*/
}

bool isLine(){
  if(Line_Sensor[0] != WHITE || Line_Sensor[1] != WHITE || Line_Sensor[2] != WHITE
    || Line_Sensor[3] != WHITE || Line_Sensor[4] != WHITE){
    return true;
  }else{
    return false;
  }
}

int isSlope(){
  #define ACCL_BORDER 2.35
  bmx_accl();
  Serial.println(xAccl);
  if(xAccl > ACCL_BORDER){
    tone(BZ,500);
    return 0;
  }else if(xAccl < -ACCL_BORDER){
    tone(BZ,500);
    return 1;
  }else{
    noTone(BZ);
    return 2;
  }
}

void loop() {
  //check_bmx();
  //test_sensor_loop();
  //return;
  //カラーセンサー読み取り
  #ifdef COLOR_DEBUG
  test_sensor_loop();
  return;
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
  check_voltage();
  if(state == TRACE){//ライントレースプログラム
  //check_color();
    color_read();
    judge_color();
    //Serial.println(silver);
    if(isSilver){
      motor_write(0,0);
      isSilver = false;
      state = SENPAN;
      Serial.println("SILVER DETECTED");
      famima();
      motor_write(48,48);
      delay(2000);
      return;
    }
    if(touch_sensor()){
      //回避動作開始
      avoid_object();
      return;
    }
    //特殊処理発生かどうか知りたい
    if(isSlope() == 0){
      p_trace(72,0.3);
    }else if(isSlope() == 1){
      p_trace(48,0.4);
    }else if((Line_Sensor[1] == WHITE || Line_Sensor[1] == BLACK) 
          && (Line_Sensor[3] == WHITE || Line_Sensor[3] == BLACK)){
      if(Line_Sensor[0] == WHITE && Line_Sensor[1] == WHITE && Line_Sensor[2] == WHITE
        && Line_Sensor[3] == WHITE && Line_Sensor[4] == WHITE
      ){
        motor_write(72,72);
        if(Previous_Line_Sensor[0] != WHITE || Previous_Line_Sensor[1] != WHITE || Previous_Line_Sensor[2] != WHITE
          || Previous_Line_Sensor[3] != WHITE || Previous_Line_Sensor[4] != WHITE){
            //はじめての　しろぉ
            white_time = millis();
          }else{
            if(millis() - white_time > 2000){
              motor_write(0,0);
              buzzer(1000);
              long hoge;
              #define ROTATION 1000
              motor_write(64,-64);
              hoge = millis();
              while(millis() - hoge < ROTATION){
                color_read();
                judge_color();
                if(isLine()){
                  motor_write(0,0);
                  white_time = millis();
                  return;
                }
              }
              motor_write(-64,64);
              hoge = millis();
              while(millis() - hoge < ROTATION * 2){
                color_read();
                judge_color();
                if(isLine()){
                  motor_write(0,0);
                  white_time = millis();
                  return;
                }
              }
              motor_write(64,-64);
              hoge = millis();
              while(millis() - hoge < ROTATION){
                color_read();
                judge_color();
                if(isLine()){
                  motor_write(0,0);
                  white_time = millis();
                  return;
                }
              }
              white_time = millis();
            }
          }
      }else{
        p_trace(50,0.95);
      }
    }else if(Line_Sensor[1] == RED || Line_Sensor[3] == RED){
      //赤処理
      motor_write(0,0);
      Serial.print((float)rg/rr);
      Serial.print(",");
      Serial.println((float)lg/lr);
      Serial.println("Mission Failed:Respcet +");
      saua();
      while(true);
    }else if(Line_Sensor[1] == GREEN || Line_Sensor[3] == GREEN){
      //緑処理
      Serial.println("Green Greens");
      detect_green();
    }else{
      Serial.println("Strange Error:No Color Found");
    }
  }else if(state == RESCUE){
    //救助コード
    if(sonic_sensor_right() < CORNER_BORDER || sonic_sensor_left() < CORNER_BORDER){
      //壁にぎりぎりなら
      if(count == 0);
      else{
        //三角コーナー探すモード切替
        state = CORNER;
        Serial.println("change to corner mode");
        motor_write(0,0);
        buzzer(334);
        return;
      }
    }
    motor_write(0,0);
    servo_write(DOWN);
    color_read();
    judge_color();
    float r_onigiri,l_onigiri;
    r_onigiri = sonic_sensor_right();
    l_onigiri = sonic_sensor_left();
    while(/*digitalRead(P_T_R) == HIGH && digitalRead(P_T_L) == HIGH*/digitalRead(P_T_M) == LOW
    && Line_Sensor[1] != GREEN && Line_Sensor[3] != GREEN && !isSilver){
      color_read();
      judge_color();
      go_straight(r_onigiri,l_onigiri);
    }
    motor_write(0,0);
    if(Line_Sensor[1] == GREEN || Line_Sensor[3] == GREEN || isSilver){
      isSilver = false;
      Serial.println("Other Line Detected");
      buzzer(1500);
      motor_write(-48,-48);
      delay(BACK_LINE);
      motor_write(0,0);
    }
    //この辺どうしよう、バケットの負荷大丈夫？
    //もしあれなら超音波使ったほうがいいかもね
    servo_write(RAISE);
    if(count % 2 == 0){
      //もし奇数回目なら左に回転？
      Serial.println("turning left");
      rotate(-90,0);
      motor_write(48,48);
      delay(1000);
      motor_write(0,0);
      rotate(-90,0);
      //その後左に回転
    }else{
      //もし偶数回目なら右に回転？
      Serial.println("turning right");
      rotate(90,0);
      motor_write(48,48);
      delay(1000);
      motor_write(0,0);
      rotate(90,0);
      //その後右に回転
    }
    delay(500);
    motor_write(-36,-36);
    while(digitalRead(P_T_B) == LOW);
    motor_write(0,0);
    count++;
  }else if(state == CORNER){
    //右に回転
    looking_corner();
  }else if(state == SENPAN){
    nakamura_is_senpan();
  }
}