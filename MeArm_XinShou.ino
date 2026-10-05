#include <Servo.h>                //！！！！待调试舵机方向以及实际笔和爪子长度

#define pin_x 9        // 底座
#define pin_y 8        // 大臂
#define pin_z 7        // 小臂
#define pin_w 6        // 爪子
#define pin_joy1_x A0  // 左右
#define pin_joy1_y A1  // 前后
#define pin_joy2_x A2  // 爪子
#define pin_joy2_y A3  // 高度
#define pin_record  4 
#define pin_start  5
#define pin_pause  A4
#define pin_continue A5
#define pin_cancel 13

const int basemin = 0; 
const int basemax = 180;
const int farmmin = 35;
const int farmmax = 135;
const int rarmmin = 40;
const int rarmmax = 170;
const int clawmin = 25;
const int clawmax = 100;

const float L1 = 80.0;    // 大臂：肩轴到肘轴
const float L2 = 80.0;    // 小臂：肘轴到腕部
const float L3 = 38.0;    // 腕部到笔尖??????
const float d0 = 15.7;    // 底座转轴到肩轴的水平距离
const float h0 = 56.9;    // 桌面到肩轴的高度

float zeroBase = 90.0;
float zeroFarm = 130.0;
float zeroRarm = 15.0; 

int delaytime = 15;
const int step = 2;
const float joySpeed = 70.0;
Servo servoX;   // 底座
Servo servoY;   // 大
Servo servoZ;   // 小
Servo servoW;   // 爪

int angleBase = 90, angleFarm = 90, angleRarm = 90, angleClaw = 55;
int targetBase = 90, targetFarm = 90, targetRarm = 90, targetClaw = 55;
float px = 100.0, py = 0.0, pz = 40.0;//以机械臂的指向为x轴建立坐标系
int mode = 0;//0=摇杆控制  1=自动绘制  2=回放

unsigned long lasttime = 0;
unsigned long lastJoyTime = 0;

const int recordMax = 5;
float recordX[recordMax];
float recordY[recordMax];
float recordZ[recordMax];
int recordCount = 0;

const int pathMax = 48;
float pathX[pathMax];
float pathY[pathMax];
float pathZ[pathMax];
int pathCount = 0;

bool drawCurve = false;
bool drawing = false;
bool paused  = false;
int drawSeg = 0;//段
int drawStep = 0;//步
const int stepsPerSeg = 20;
unsigned long drawLastTime = 0;

const int recordMax = 120;
byte recordAngles[recordMax][4];//?????byte
int recordCount = 0;
bool recording = false;
bool playing   = false;
bool lastrecord = HIGH, lastStart = HIGH, lastPause = HIGH;
bool lastContinue = HIGH, lastCancel = HIGH;
int playIndex = 0;
unsigned long recordLastTime = 0;

String command = "";
bool complete = false;

int   xianZhi(int v, int lo, int hi);
bool  niYunDong(float x, float y, float z, int &a1, int &a2, int &a3);
void  zhengYunDong(int a1, int a2, int a3, float &x, float &y, float &z);
void  moveOne(Servo &servo, int &current, int target);
void  moveAllServo();
void  moveAllServoNow();
void  moveToXYZ(float x, float y, float z);
void  readJoystick();
void  readButtons();
void  record();
void  buildPath();
void  startDraw();
void  updateDraw();
void  doPause();
void  docontinue();
void  doCancel();
void  updateRecord();
void  pickplace(int n);
void  goHome();
bool  parseXYZW(String cmd, int &x, int &y, int &z, int &w);
bool  parseGoto(String cmd, float &x, float &y, float &z);
void  readSerial();
void  printHelp();

int xianZhi(int v, int lo, int hi) {      //限制
  if (v < lo) v = lo;
  if (v > hi) v = hi;
  return v;
}

bool niYunDong(float x, float y, float z, int &a1, int &a2, int &a3) {
  float r = sqrt(x * x + y * y);
  float th1 = atan2(y, x);

  float zw = z + L3;
  float dr = r - d0;
  float dz = zw - h0;
  float D  = sqrt(dr * dr + dz * dz);

  if (D > L1 + L2 - 1) return false;
  if (D < 30) return false;

  float phi = atan2(dz, dr);
  float cA = (L1 * L1 + D * D - L2 * L2) / (2 * L1 * D);
  float cB = (L1 * L1 + L2 * L2 - D * D) / (2 * L1 * L2);

  if (cA > 1) cA = 1;
  if (cA < -1) cA = -1;
  if (cB > 1) cB = 1;
  if (cB < -1) cB = -1;

  float alpha = acos(cA);
  float beta  = acos(cB);

  float th2 = phi + alpha;          // 大臂弧度
  float th3 = -(PI - beta);         // 小臂弧度，负号是安装方向决定的

  int deg1 = th1 * 180.0 / PI;
  int deg2 = th2 * 180.0 / PI;
  int deg3 = th3 * 180.0 / PI;

  a1 = zeroBase + deg1;    // 底座：正装
  a2 = zeroFarm - deg2;    // 肩：装反了，减
  a3 = zeroRarm - deg3;    // 肘：装反了，减

  if (a1 < basemin || a1 > basemax) return false;
  if (a2 < farmmin || a2 > farmmax) return false;
  if (a3 < rarmmin || a3 > rarmmax) return false;
  return true;
}

void zhengYunDong(int a1, int a2, int a3, float &x, float &y, float &z) {
  float th1 = (a1 - zeroBase) * PI / 180.0;
  float th2 = (zeroFarm - a2) * PI / 180.0;
  float th3 = (zeroRarm - a3) * PI / 180.0;

  float r  = d0 + L1 * cos(th2) + L2 * cos(th2 + th3);
  float zw = h0 + L1 * sin(th2) + L2 * sin(th2 + th3);

  x = r * cos(th1);
  y = r * sin(th1);
  z = zw - L3;
}

void moveOne(Servo &servo, int &current, int target) {
  if (current < target) {
    current += step;
    if (current > target) current = target;
    servo.write(current);
  } else if (current > target) {
    current -= step;
    if (current < target) current = target;
    servo.write(current);
  }
}

void moveAllServo() {
  unsigned long now = millis();
  if (now - lasttime < (unsigned long)delaytime) return;
  lasttime = now;
  moveOne(servoX, angleBase, targetBase);
  moveOne(servoY, angleFarm, targetFarm);
  moveOne(servoZ, angleRarm, targetRarm);
  moveOne(servoW, angleClaw, targetClaw);
}

void moveAllServoNow() {
  moveOne(servoX, angleBase, targetBase);
  moveOne(servoY, angleFarm, targetFarm);
  moveOne(servoZ, angleRarm, targetRarm);
  moveOne(servoW, angleClaw, targetClaw);
}

void moveToXYZ(float x, float y, float z) {
  int a1, a2, a3;
  if (!niYunDong(x, y, z, a1, a2, a3)) {
    Serial.println(F("够不到"));     //print(F(""))可以节省内存
    return;
  }
  targetBase = a1;
  targetFarm = a2;
  targetRarm = a3;

  int count = 0;   //防止精度导致死循环
  while (count < 350) {
  if (abs(angleBase - targetBase) <= 1 &&
      abs(angleFarm - targetFarm) <= 1 &&
      abs(angleRarm - targetRarm) <= 1) break;
  moveAllServoNow();
  delay(15);
  count++;
  }
}

void readJoystick() {
  if (mode != 0) return;
  unsigned long now = millis();
  if (now - lastJoyTime < 20) return;
  float dt = (now - lastJoyTime) / 1000.0;//根据实际时间来决定速度，操作更丝滑???
  if (dt > 0.05) dt = 0.05; 
  lastJoyTime = now;

  int j1x = analogRead(pin_joy1_x);
  int j1y = analogRead(pin_joy1_y);
  int j2x = analogRead(pin_joy2_x);
  int j2y = analogRead(pin_joy2_y);

  float v1x = (j1x - 512) / 512.0;
  float v1y = (j1y - 512) / 512.0;
  float v2x = (j2x - 512) / 512.0;
  float v2y = (j2y - 512) / 512.0;

  if (v1x > -0.12 && v1x < 0.12) v1x = 0;//死区
  if (v1y > -0.12 && v1y < 0.12) v1y = 0;
  if (v2x > -0.12 && v2x < 0.12) v2x = 0;
  if (v2y > -0.12 && v2y < 0.12) v2y = 0;

  float oldx = px, oldy = py, oldz = pz;

  // 只有摇杆真的被推了才去改三个舵机的目标角。
  // 摇杆松手在中间时什么都不做不然每 20ms 重算一次，会把串口指令直接覆盖掉!!!!!
  bool tuiLe = (v1x != 0 || v1y != 0 || v2y != 0);
  if (tuiLe) {
    // 摇杆方向和机械臂方向不一定相同改符号
    px = px + v1y * joySpeed * dt;
    py = py + v1x * joySpeed * dt;
    pz = pz - v2y * joySpeed * dt;

    if (px < 50) px = 50;
    if (px > 150) px = 150;
    if (py < -60) py = -60;
    if (py > 60) py = 60;
    if (pz < 5) pz = 5;
    if (pz > 90) pz = 90;

    // 算舵机角度，够不到就退回去
    int a1, a2, a3;
    if (niYunDong(px, py, pz, a1, a2, a3)) {
      targetBase = a1;
      targetFarm = a2;
      targetRarm = a3;
    } else {
      px = oldx; py = oldy; pz = oldz;
      Serial.println(F("够不到"));
    }
  }

  if (v2x > 0.3) targetClaw = clawmax;
  if (v2x < -0.3) targetClaw = clawmin;
}

void readButtons() {
  int v;
  v = digitalRead(pin_record);
  if (v == LOW && lastrecord == HIGH) { 
    delay(30); 
    if (digitalRead(pin_record) == LOW) 
      record(); 
  }
  lastrecord = v;

  v = digitalRead(pin_start);
  if (v == LOW && lastStart == HIGH) { 
    delay(30); 
    if (digitalRead(pin_start) == LOW)
       startDraw();
  }
  lastStart = v;

  v = digitalRead(pin_pause);
  if (v == LOW && lastPause == HIGH) {
    delay(30);
    if (digitalRead(pin_pause) == LOW)
      doPause(); 
  }
  lastPause = v;

  v = digitalRead(pin_continue);
  if (v == LOW && lastcontinue == HIGH) {
    delay(30); 
    if (digitalRead(pin_continue) == LOW) 
      docontinue(); 
  }
  lastcontinue = v;

  v = digitalRead(pin_cancel);
  if (v == LOW && lastCancel == HIGH) { 
    delay(30); 
    if (digitalRead(pin_cancel) == LOW) 
      doCancel()
  }
  lastCancel = v;
}

void record() {
  if (recordCount >= recordMax) {
    Serial.println(F("示教点已满"));
    return;
  }
  recordX[recordCount] = px;
  recordY[recordCount] = py;
  recordZ[recordCount] = pz;
  recordCount++;
  Serial.print(F("记录第 "));
  Serial.print(recordCount);
  Serial.print(F(" 个点： x="));
  Serial.print(px);
  Serial.print(F(" y="));
  Serial.print(py);
  Serial.print(F(" z="));
  Serial.println(pz);
}

void buildPath() {
  pathCount = 0;
  if (drawCurve == false) {
    for (int i = 0; i < recordCount; i++) {
      pathX[i] = recordX[i];
      pathY[i] = recordY[i];
      pathZ[i] = recordZ[i];
    }
    pathCount = recordCount;
    return;
  }

  const int perSeg = 8;
  for (int i = 0; i < recordCount - 1; i++) {
    float p0x, p0y, p0z, p3x, p3y, p3z;
    if (i == 0) 
      { p0x = recordX[0]; p0y = recordY[0]; p0z = recordZ[0]; }
    else        
      { p0x = recordX[i - 1]; p0y = recordY[i - 1]; p0z = recordZ[i - 1]; }
    if (i == recordCount - 2) 
      { p3x = recordX[recordCount - 1]; p3y = recordY[recordCount - 1]; p3z = recordZ[recordCount - 1]; }
    else                    
      { p3x = recordX[i + 2]; p3y = recordY[i + 2]; p3z = recordZ[i + 2]; }

    float p1x = recordX[i],     p1y = recordY[i],     p1z = recordZ[i];
    float p2x = recordX[i + 1], p2y = recordY[i + 1], p2z = recordZ[i + 1];

    for (int k = 0; k < perSeg; k++) {
      float t = k / (float)perSeg;
      float t2 = t * t;
      float t3 = t2 * t;
      float x = 0.5 * ((2 * p1x) + (-p0x + p2x) * t +(2 * p0x - 5 * p1x + 4 * p2x - p3x) * t2 +(-p0x + 3 * p1x - 3 * p2x + p3x) * t3);
      float y = 0.5 * ((2 * p1y) + (-p0y + p2y) * t +(2 * p0y - 5 * p1y + 4 * p2y - p3y) * t2 +(-p0y + 3 * p1y - 3 * p2y + p3y) * t3);
      float z = 0.5 * ((2 * p1z) + (-p0z + p2z) * t +(2 * p0z - 5 * p1z + 4 * p2z - p3z) * t2 +(-p0z + 3 * p1z - 3 * p2z + p3z) * t3);
      if (pathCount < pathMax) {
        pathX[pathCount] = x;
        pathY[pathCount] = y;
        pathZ[pathCount] = z;
        pathCount++;
      }
    }
  }

  if (pathCount < pathMax) {
    pathX[pathCount] = recordX[recordCount - 1];
    pathY[pathCount] = recordY[recordCount - 1];
    pathZ[pathCount] = recordZ[recordCount - 1];
    pathCount++;
  }
}

void startDraw() {
  buildPath();
  mode = 1;
  drawing = true;
  paused = false;
  drawSeg = 0;
  drawStep = 0;
  drawLastTime = millis();

  moveToXYZ(pathX[0], pathY[0], pathZ[0] + 15);
  moveToXYZ(pathX[0], pathY[0], pathZ[0]);

  if (drawCurve) Serial.println(F("开始画平滑曲线"));
  else           Serial.println(F("开始画折线"));
}

void updateDraw() {
  if (drawing == false) return;
  if (paused == true) return;

  unsigned long now = millis();
  if (now - drawLastTime < 20) return;
  drawLastTime = now;

  if (drawSeg >= pathCount - 1) {
    drawing = false;
    mode = 0;
    Serial.println(F("绘制完成"));
    return;
  }

  float t = drawStep / (float)stepsPerSeg;
  float x = pathX[drawSeg] + (pathX[drawSeg + 1] - pathX[drawSeg]) * t;
  float y = pathY[drawSeg] + (pathY[drawSeg + 1] - pathY[drawSeg]) * t;
  float z = pathZ[drawSeg] + (pathZ[drawSeg + 1] - pathZ[drawSeg]) * t;

  int a1, a2, a3;
  if (niYunDong(x, y, z, a1, a2, a3)) {
    targetBase = a1; targetFarm = a2; targetRarm = a3;
    angleBase = a1;  angleFarm = a2;  angleRarm = a3;
    servoX.write(a1);
    servoY.write(a2);
    servoZ.write(a3);
  } else {
    Serial.println(F("够不到"));
  }

  drawStep++;
  if (drawStep > stepsPerSeg) {
    drawStep = 0;
    drawSeg++;
  }
}

void doPause() {
  if (drawing) { paused = true; Serial.println(F("已暂停")); }
}
void docontinue() {
  if (drawing) { paused = false; drawLastTime = millis(); Serial.println(F("继续")); }
}
void doCancel() {
  if (drawing || recordCount > 0) {
    drawing = false;
    paused = false;
    mode = 0;
    recordCount = 0;
    Serial.println(F("已取消，示教点清空，回待机"));
    moveToXYZ(100, 0, 60);
  }
}

void updateRecord() {
  unsigned long now = millis();
  if (recording) {
    if (now - recordLastTime >= 100) {
      recordLastTime = now;
      if (recordCount < recordMax) {
        recordAngles[recordCount][0] = angleBase;
        recordAngles[recordCount][1] = angleFarm;
        recordAngles[recordCount][2] = angleRarm;
        recordAngles[recordCount][3] = angleClaw;
        recordCount++;
      } else {
        recording = false;
        Serial.println(F("录满了"));
      }
    }
  }

  if (playing) {
    if (now - recordLastTime >= 100) {
      recordLastTime = now;
      if (playIndex >= recordCount) {
        playing = false;
        mode = 0;
        Serial.println(F("回放结束"));
        return;
      }
      targetBase = recordAngles[playIndex][0];
      targetFarm = recordAngles[playIndex][1];
      targetRarm = recordAngles[playIndex][2];
      targetClaw = recordAngles[playIndex][3];
      playIndex++;
    }
  }
}

void pickplace(int n) {
  float fromX, fromY, toX, toY;
  if (n == 1) { fromX = 120; fromY = -35; toX = 120; toY = 35; }
  else if (n == 2) { fromX = 100; fromY = -20; toX = 140; toY = 20; }
  else { fromX = 80; fromY = 0; toX = 130; toY = -30; }

  Serial.println(F("开始抓取放置"));
  targetClaw = clawmax;
  moveToXYZ(fromX, fromY, 45);
  moveToXYZ(fromX, fromY, 12);
  targetClaw = clawmin;
  delay(600);
  moveToXYZ(fromX, fromY, 45);
  moveToXYZ(toX, toY, 45);
  moveToXYZ(toX, toY, 12);
  targetClaw = clawmax;
  delay(600);
  moveToXYZ(toX, toY, 45);
  Serial.println(F("放置完成"));
}

void goHome() {
  moveToXYZ(100, 0, 60);
  targetClaw = clawmax;
  Serial.println(F("回到待机位置"));
}

bool parseXYZW(String cmd, int &x, int &y, int &z, int &w) {
  cmd.replace(" ", "");
  cmd.replace("=", "");
  int idxX = cmd.indexOf('x');
  int idxY = cmd.indexOf('y');
  int idxZ = cmd.indexOf('z');
  int idxW = cmd.indexOf('w');
  if (idxX == -1 || idxY == -1 || idxZ == -1 || idxW == -1) return false;

  String sx = cmd.substring(idxX + 1, idxY + 1);
  String sy = cmd.substring(idxY + 1, idxZ + 1);
  String sz = cmd.substring(idxZ + 1, idxW + 1);
  String sw = cmd.substring(idxW + 1, cmd.length() + 1);
  sx.replace(",", "");
  sy.replace(",", "");
  sz.replace(",", "");
  sw.replace(",", "");
  x = sx.toInt();
  y = sy.toInt();
  z = sz.toInt();
  w = sw.toInt();
  return true;
}

bool parseGoto(String cmd, float &x, float &y, float &z) {   //获取逆运动坐标
  cmd.replace(" ", "");
  cmd.replace("g", "");
  int p1 = cmd.indexOf(',');
  if (p1 == -1) return false;
  int p2 = cmd.indexOf(',', p1 + 1);
  if (p2 == -1) return false;
  x = cmd.substring(0, p1).toFloat();
  y = cmd.substring(p1 + 1, p2).toFloat();
  z = cmd.substring(p2 + 1).toFloat();
  return true;
}

void readSerial() {
  while (Serial.available()) {
    char input = (char)Serial.read();
    if (input == '\n' || input == '\r') {
      complete = true;
      break;
    } else {
      command += input;
    }
  }

  if (complete == false) return;

  command.trim();
  if (command.length() > 0) {
    command.toLowerCase();

    if (command == "h") {
      delaytime -= 5;
      if (delaytime < 3) delaytime = 3;
      Serial.print(F("加速,delaytime = "));
      Serial.println(delaytime);
    } else if (command == "l") {
      delaytime += 5;
      if (delaytime > 50) delaytime = 50;
      Serial.print(F("减慢,delaytime = "));
      Serial.println(delaytime);
    } else if (command == "o") {
      targetClaw = clawmax;
      Serial.println(F("爪子张开"));
    } else if (command == "s") {
      targetClaw = clawmin;
      Serial.println(F("爪子夹紧"));
    } else if (command == "t") {
      record();
    } else if (command == "run") {
      startDraw();
    } else if (command == "pause") {
      doPause();
    } else if (command == "go") {
      docontinue();
    } else if (command == "stop") {
      doCancel();
    } else if (command == "line") {
      drawCurve = false;
      Serial.println(F("切换到折线"));
    } else if (command == "curve") {
      drawCurve = true;
      Serial.println(F("切换到曲线"));
    } else if (command == "rec") {
      recording = true;
      recordCount = 0;
      recordLastTime = millis();
      Serial.println(F("开始录制"));
    } else if (command == "endrec") {
      recording = false;
      Serial.print(F("录完了，一共 "));
      Serial.print(recordCount);
      Serial.println(F(" 个点"));
    } else if (command == "play") {
      playing = true;
      playIndex = 0;
      mode = 2;
      recordLastTime = millis();
      Serial.println(F("开始回放"));
    } else if (command == "home") {
      goHome();
    } else if (command == "a") {
      pickplace(1);
    } else if (command == "b") {
      pickplace(2);
    } else if (command == "c") {
      pickplace(3);
    } else if (command == "?") {
      printHelp();
    } else if (command == "zero") {
      Serial.print(F("底座零位 = "));
      Serial.println(zeroBase);
      Serial.print(F("肩零位 = "));
      Serial.println(zeroFarm);
      Serial.print(F("肘零位 = "));
      Serial.println(zeroRarm);
    } else if (command.charAt(0) == 'z' && command.length() >= 4) {
      // 标定零位：zba90 / zsh130 / zeb15，前三个字母是哪一个，后面是数字
      String head = command.substring(0, 3);
      float  val  = command.substring(3).toFloat();
      if (head == "zba") {
        zeroBase = val;
        Serial.print(F("底座零位改成 "));
        Serial.println(val);
      } else if (head == "zsh") {
        zeroFarm = val;
        Serial.print(F("肩零位改成 "));
        Serial.println(val);
      } else if (head == "zeb") {
        zeroRarm = val;
        Serial.print(F("肘零位改成 "));
        Serial.println(val);
      } else {
        Serial.println(F("零位指令只有 zba / zsh / zeb 三个"));
      }
    } else if (command.charAt(0) == 'g') {
      float gx, gy, gz;
      if (parseGoto(command, gx, gy, gz)) {
        int a1, a2, a3;
        if (niYunDong(gx, gy, gz, a1, a2, a3)) {
          px = gx; py = gy; pz = gz;
          targetBase = a1; targetFarm = a2; targetRarm = a3;
          Serial.print(F("走到 x="));
          Serial.print(gx);
          Serial.print(F(" y="));
          Serial.print(gy);
          Serial.print(F(" z="));
          Serial.println(gz);
        } else {
          Serial.println(F("这个坐标够不到"));
        }
      } else {
        Serial.println(F("格式不对，应该写 g100,0,12 这样"));
      }
    } else {
      int x, y, z, w;
      if (parseXYZW(command, x, y, z, w)) {
        targetBase = xianZhi(x, basemin, basemax);
        targetFarm = xianZhi(y, farmmin, farmmax);
        targetRarm = xianZhi(z, rarmmin, rarmmax);
        targetClaw = xianZhi(w, clawmin, clawmax);
        // 手动改了角度，"以为笔尖在哪"也要跟着改（用正解算），
        // 不然一推摇杆机械臂就会跳回老位置
        zhengYunDong(targetBase, targetFarm, targetRarm, px, py, pz);
        Serial.print(F("X="));
        Serial.print(targetBase);
        Serial.print(F(" Y="));
        Serial.print(targetFarm);
        Serial.print(F(" Z="));
        Serial.print(targetRarm);
        Serial.print(F(" W="));
        Serial.println(targetClaw);
      } else {
        Serial.println(F("指令错误"));
      }
    }
  }
  command = "";
  complete = false;
}

void printHelp() {
  Serial.println(F("===== MeArm 指令表 ====="));
  Serial.println(F("x90,y60,z120,w30  直接给四个舵机角度"));        //正运动
  Serial.println(F("g100,0,12         笔尖走到坐标(100,0,12)"));   //逆运动
  Serial.println(F("h / l             变快 / 变慢"));
  Serial.println(F("o / s             爪子张开 / 夹紧"));
  Serial.println(F("t                 记录示教点"));
  Serial.println(F("run               开始绘制"));
  Serial.println(F("pause / go / stop 暂停 / 继续 / 取消"));
  Serial.println(F("line / curve      折线 / 平滑曲线"));
  Serial.println(F("rec / endrec / play 录制 / 结束 / 回放"));
  Serial.println(F("home              回待机  a / b / c 抓取放置"));
  Serial.println(F("zba90 / zsh130 / zeb15 改零位    zero 看零位"));
  Serial.println(F("按键 D4记点 D5开始 A4暂停 A5继续 D13取消"));
}

void setup() {
  Serial.begin(9600);

  servoX.attach(pin_x);
  servoY.attach(pin_y);
  servoZ.attach(pin_z);
  servoW.attach(pin_w);

  servoX.write(angleBase);
  servoY.write(angleFarm);
  servoZ.write(angleRarm);
  servoW.write(angleClaw);

  pinMode(pin_record, INPUT_PULLUP);
  pinMode(pin_start, INPUT_PULLUP);
  pinMode(pin_pause, INPUT_PULLUP);
  pinMode(pin_continue, INPUT_PULLUP);
  pinMode(pin_cancel, INPUT_PULLUP);

  Serial.println(F("Ready"));
  printHelp();
}

void loop() {
  readJoystick();
  readButtons();
  readSerial();
  updateDraw();
  updateRecord();
  moveAllServo();
}
