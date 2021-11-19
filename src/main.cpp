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
#include <Adafruit_PWMServoDriver.h>

//ピン指定
#define P_TPR_R A12
#define P_TPR_M A13
#define P_TPR_L A14
#define P_SILVER A15
#define P_TPR_B A11
#define P_M_APWM 4
#define P_M_A1 2
#define P_M_A2 3
#define P_M_BPWM 5
#define P_M_B1 7
#define P_M_B2 6
#define P_S_RT 23
#define P_S_LT 24
#define P_S_RE 25
#define P_S_LE 26
#define P_T_R 52
#define P_T_M 50
#define P_T_L 48
#define P_T_B 46
#define BZ 9

//アドレス指定
#define S11059_ADDR 0x2A
#define CONTROL_MSB 0x00
#define CONTROL_1_LSB 0x89
#define CONTROL_2_LSB 0x09
#define SENSOR_REGISTER 0x03
#define BMX_MAG 0x13
#define PCA9685_ADDR 0x40

//debug
//#define COLOR_DEBUG

//Servo's Hz Min And Max
#define SERVO_MIN 150
#define SERVO_MAX 500

//三角コーナー探すモードかどうか
#define CORNER_BORDER 3

//RCフィルタ(0 < x < 1)
const float RC = 0.5;
int before_x,before_y;

//Enum
enum Color{BLACK,WHITE,GREEN,RED,SILVER};
enum Bucket{RAISE,DOWN,RELEASE,HOLD};
enum State{TRACE,RESCUE,CORNER};

//int
int lower, higher, rr, rg, rb, rir, lr, lg, lb, lir,silver,tpr_l,tpr_m,tpr_r,xMag,yMag,zMag,count = 0;

//bmxのオフセット
const int bmx_x_os = -55;
const int bmx_y_os = 101;

//右から左へ
Color Line_Sensor[5];
Color Previous_Line_Sensor[5];

//走行モード切替
State state = TRACE;

//PCA9685変数
Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver(0x41);

//silver
bool isSilver = false;

//Timer
long timer;

//前回の値保持(P操作量)
int volume;
int previous_volume;

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

void judge_color(){
  #define RR_BORDER 288
  #define RG_BORDER 260
  #define RB_BORDER 260
  #define RIR_BORDER 200
  #define LR_BORDER 268
  #define LG_BORDER 350
  #define LB_BORDER 200
  #define LIR_BORDER 200
  #define R_TPR_BORDER 350
  #define M_TPR_BORDER 28
  #define L_TPR_BORDER 350
  #define B_TPR_BORDER 100
  #define SILVER_BORDER 500
  #define R_GpR_MIN 1.0
  #define R_GpR_MAX 1.6
  #define L_GpR_MIN 1.0
  #define L_GpR_MAX 1.6

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
  if(silver < SILVER_BORDER){
    isSilver = true;
  }
}

void color_read(){
  //カラーセンサー読み取り
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
  change_i2c_port(0);
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

void bmx_init(){
  change_i2c_port(2);
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

void bmx_fixer(){
  xMag += bmx_x_os;
  yMag += bmx_y_os;
  //xMag = RC * before_x + (1 - RC) * xMag;
  //yMag = RC * before_y + (1 - RC) * yMag;
}

void bmx_read(){
  change_i2c_port(2);
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
  bmx_fixer();
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
    servo.setPWM(0,0,SERVO_MAX);
    servo.setPWM(1,0,SERVO_MAX);
  }else if(mode == DOWN){
    servo.setPWM(0,0,SERVO_MIN);
    servo.setPWM(1,0,SERVO_MIN);
  }else if(mode == RELEASE){
    servo.setPWM(2,0,SERVO_MAX);
  }else if(mode == HOLD){
    servo.setPWM(2,0,SERVO_MIN);
  }
  delay(500);
}

void setup(){
  #ifdef COLOR_DEBUG
  //ほげー
  test_sensor_setup();
  return;
  #endif
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
  //地磁気センサーリセット
  bmx_init();
  //サーボ初期化
  //servo.setPWMFreq(50);
  //servo_write(RAISE);
}

void check_voltage(){
  if(analogRead(A0) < 655){
    while(true){
      tone(BZ,440);
      delay(1000);
      noTone(BZ);
      delay(1000);
    }
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

void p_trace(){
  #define GAIN 0.1
  const int r_target = 425;
  const int l_target = 480;
  int r_v = (rr + rg + rb) / 3 - r_target;
  int l_v = (lr + lg + lb) / 3 - l_target;
  int val = r_v - l_v;
  val *= 0.4;
  int rm = 30 + val;
  int lm = 30 - val;
  if(rm > 80)rm = 80;
  if(lm > 80)lm = 80;
  motor_write(rm,lm);
  //Serial.println(val);
  /*Serial.print(r_v);
  Serial.print(",");
  Serial.println(l_v);*/
}

int tan2angle(int x,int y){
  return atan2(y,x) * 180 / PI;
}

void check_bmx(){
  int xmax,xmin,ymax,ymin;
  bmx_read();
  xmax = xMag;
  xmin = xMag;
  ymax = yMag;
  ymin = yMag;
  while(true){
    bmx_read();
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

void rotate(int angle){
  #define ALLOW_DIF 20
  Serial.println("begin to rotate");
  motor_write(0,0);
  delay(500);
  bmx_read();
  int n_angle = tan2angle(xMag,yMag);
  int target = n_angle + angle;
  Serial.print(n_angle);
  Serial.print(",");
  Serial.println(target);
  if(angle > 0){
    //時計回り
    motor_write(-64,64);
    if(target > 180){
      target -= 360;
      while(abs(target - n_angle) > ALLOW_DIF){
        bmx_read();
        n_angle = tan2angle(xMag,yMag);
      }
    }else{
      while(abs(target - n_angle) > ALLOW_DIF){
        bmx_read();
        n_angle = tan2angle(xMag,yMag);
        Serial.println(n_angle);
      }
    }
  }else if(angle < 0){
    //反時計回り
    motor_write(64,-64);
    if(target < -180){
      target += 360;
      while(abs(target - n_angle) > ALLOW_DIF){
        bmx_read();
        n_angle = tan2angle(xMag,yMag);
      }
    }else{
      while(abs(target - n_angle) > ALLOW_DIF){
        bmx_read();
        n_angle = tan2angle(xMag,yMag);
      }
    }
  }else return;
  motor_write(0,0);
  Serial.println("I've done it!");
}

void go_straight(int val){
  //必要になったら書こう
  //まっすぐ進むためのコード
}

void detect_green(){
  bool isRightGreen = false;
  bool isLeftGreen = false;
  motor_write(0,0);
  buzzer(500);
  while(Line_Sensor[1] == GREEN || Line_Sensor[3] == GREEN){
    color_read();
    judge_color();
    p_trace();
    if(Line_Sensor[1] == GREEN)isRightGreen = true;
    if(Line_Sensor[3] == GREEN)isLeftGreen = true;
  Serial.print(isRightGreen);
  Serial.print(",");
  Serial.println(isLeftGreen);
  }
  motor_write(0,0);
  color_read();
  judge_color();
  /*if(Line_Sensor[2] == WHITE){
    Serial.println("What's wrong??? There is no detecton on middle sensor.");
  }*/
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
      rotate(180);
    }else if(isRightGreen && !isLeftGreen){
      //右に90ターン
      Serial.println("turning 90");
      buzzer(800);
      rotate(90);
    }else if(!isRightGreen && isLeftGreen){
      //左に90ターン
      Serial.println("turning -90");
      buzzer(600);
      rotate(-90);
    }
  }
}

bool touch_sensor(){
  bool isTouched = false;
  digitalRead(P_T_M) == LOW ? isTouched = true : isTouched = isTouched;
  return isTouched;
}

double sonic_sensor_right(){
  double du,dis = 0;
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

double sonic_sensor_left(){
  double du,dis = 0;
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

void avoid_object(){
  const double distance = 5.0;
  const double allow_miss = 3.0;
  motor_write(0,0);
  delay(100);
  motor_write(-64,-64);
  delay(500);
  motor_write(0,0);
  delay(100);
  //90度回転
  //右を障害物に向けるよ
  rotate(-90);
  motor_write(64,64);
  delay(100);
  while(Line_Sensor[0] == WHITE && Line_Sensor[2] == WHITE && Line_Sensor[4] == WHITE){
    if(sonic_sensor_right() > distance + allow_miss){
      //遠くなってるなら
      motor_write(48,64);
    }else if(sonic_sensor_right() < distance){
      //近くなってるなら
      motor_write(64,48);
    }else{
      //範囲内なら
      motor_write(64,64);
    }
    color_read();
    judge_color();
  }
  motor_write(0,0);
  //向きを戻すよ
  rotate(-90);
  motor_write(-64,-64);
  while(digitalRead(P_T_B) == HIGH);
  motor_write(0,0);
}

void check_color(){
  for(int i = 0;i < 5;i++){
    Serial.print(Line_Sensor[i]);
    Serial.print(",");
  }
  Serial.println("");
}

void looking_corner(){
  //三角コーナーからの逃亡コード
  servo_write(DOWN);
  motor_write(128,128);
  while(digitalRead(P_T_R) == HIGH && digitalRead(P_T_L) == HIGH){
    //まっすぐ進む
    motor_write(128,128);
    color_read();
    judge_color();
    if(Line_Sensor[1] != WHITE || Line_Sensor[3] != WHITE || isSilver){
      motor_write(0,0);
      delay(500);
      return;
    }
  }
  delay(500);
  motor_write(0,0);
  if(digitalRead(P_T_R) == LOW && digitalRead(P_T_L) == LOW){
    //違うらしいよ
    //右に回転
    return;
  }else{
    bmx_read();
    rotate(135);
    motor_write(-64,-64);
    while(digitalRead(P_T_B) == HIGH);
    delay(500);
    motor_write(0,0);
    if(analogRead(P_TPR_B) > B_TPR_BORDER){
      //黒の時
      servo_write(RELEASE);
      delay(1500);
      servo_write(HOLD);
      for(int i = 0;i < 3;i++){
        motor_write(128,128);
        delay(500);
        motor_write(0,0);
        delay(250);
        motor_write(-128,-128);
        while(digitalRead(P_T_B) == HIGH);
        motor_write(0,0);
        servo_write(RELEASE);
        delay(1500);
        servo_write(HOLD);
        servo_write(RAISE);
        delay(250);
      }
      rotate(-45);
      color_read();
      judge_color();
      motor_write(192,192);
      while(Line_Sensor[1] == WHITE && Line_Sensor[3] == WHITE){
        color_read();
        judge_color();
        if(Line_Sensor[1] != WHITE || Line_Sensor[3] != WHITE || isSilver){
          if(isSilver){
            //銀なので間違い
            isSilver = false;
            motor_write(0,0);
            delay(250);
            motor_write(-64,-64);
            delay(250);
            motor_write(0,0);
            //右に回転
            rotate(90);
            motor_write(192,192);
          }else if(Line_Sensor[1] == GREEN || Line_Sensor[3] == GREEN){
            //脱出完了
            delay(100);
            state = TRACE;
            return;
          }
        }
        if(analogRead(P_T_M) == LOW){
          //壁にぶつかった時
          motor_write(0,0);
          delay(250);
          //右に回転
          rotate(90);
          motor_write(192,192);
        }
      }
    }else{
      //三角コーナーじゃなくて壁の時の処理
      //考えてないよ☆（ゝω・）vｷｬﾋﾟ
    }
  }
}

void p_trace_v3(){
  #define INSIDE 1
  #define OUTSIDE 20
  int r_value =INSIDE * (rr + rg + rb) / 40;
  int l_value =INSIDE * (lr + lg + lb) / 40;
  int dif = r_value - l_value;
  Serial.println(dif);
  int rm = 48 * (50 - abs(dif)) / 25 + dif;
  int lm = 48 * (50 - abs(dif)) / 25 - dif;
  if(rm > 112) rm = 112;
  motor_write(rm,lm);
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

void test_sensor_loop(){
  /*rotate(90);
  delay(2000);
  rotate(-90);
  delay(2000);
  //servo_write(RAISE);
  //servo_write(DOWN);
  //color_read();
  //judge_color();
  /*Serial.print(tpr_r);
  Serial.print(",");
  Serial.println(tpr_l);
  Serial.print(lr);
  Serial.print(",");
  Serial.print(lg);
  Serial.print(",");
  Serial.print(lb);
  Serial.print(",");
  Serial.print(rr);
  Serial.print(",");
  Serial.print(rg);
  Serial.print(",");
  Serial.println(rb);*//*
  Serial.println((float)lg/lr);*//*
  for(int i = 0;i < 5;i++){
    Serial.print(Line_Sensor[i]);
    Serial.print(",");
  }
  Serial.println("");//*/
  //check_bmx();
  /*bmx_read();
  Serial.print(xMag);
  Serial.print(",");
  Serial.print(yMag);
  Serial.print(",");
  Serial.println(atan2(yMag,xMag) * 180 / PI);//*/
  /*color_read();
  judge_color();
  p_trace();
  check_color();*/
}

void loop() {
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
  if(state == TRACE){//ライントレースプログラム
  //check_color();
    color_read();
    judge_color();
    Serial.println(silver);
    if(isSilver){
      isSilver = false;
      state = RESCUE;
      Serial.println("SILVER DETECTED");
      return;
    }
    /*if(touch_sensor()){
      //回避動作開始
      avoid_object();
      return;
    }*/
    //特殊処理発生かどうか知りたい
    if((Line_Sensor[1] == WHITE || Line_Sensor[1] == BLACK) 
        && (Line_Sensor[3] == WHITE || Line_Sensor[3] == BLACK)){
      p_trace();
    }else if(Line_Sensor[1] == RED || Line_Sensor[3] == RED){
      //赤処理
      motor_write(0,0);
      delay(1000000);
    }else if(Line_Sensor[1] == GREEN || Line_Sensor[3] == GREEN){
      //緑処理
      Serial.println("Green Greens");
      detect_green();
    }else{
      Serial.println("Strange Error:No Color Found");
    }
  }else if(state == RESCUE){
    //救助コード
    if((int)sonic_sensor_right() < CORNER_BORDER || (int)sonic_sensor_left < CORNER_BORDER){
      //壁にぎりぎりなら
      if(count == 0);
      else{
        //三角コーナー探すモード切替
        state = CORNER;
        return;
      }
    }
    motor_write(0,0);
    servo_write(DOWN);
    double r_soinc,l_sonic;
    r_soinc = sonic_sensor_right();
    l_sonic = sonic_sensor_left();
    motor_write(128,128);
    while(digitalRead(P_T_R) == HIGH && digitalRead(P_T_L) == HIGH){
      int r_er = sonic_sensor_right() - r_soinc;
      int l_er = sonic_sensor_left() - l_sonic;
      motor_write(128 + l_er,128 + r_er);
    }
    //この辺どうしよう、バケットの負荷大丈夫？
    //もしあれなら超音波使ったほうがいいかもね
    delay(500);
    motor_write(0,0);
    delay(100);
    servo_write(RAISE);
    if(count % 2 == 0){
      //もし奇数回目なら左に回転？
      rotate(-90);
      motor_write(64,64);
      delay(1000);
      motor_write(0,0);
      rotate(-90);
      //その後左に回転
    }else{
      //もし偶数回目なら右に回転？
      rotate(90);
      motor_write(64,64);
      delay(1000);
      motor_write(0,0);
      rotate(90);
      //その後右に回転
    }
    count++;
  }else if(state == CORNER){
    //右に回転
    rotate(90);
    looking_corner();
  }
}