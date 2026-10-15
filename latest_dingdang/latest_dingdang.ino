/*底座舵机 D9  大臂舵机 D8  小臂舵机 D7  爪子舵机 D6
  摇杆：左摇杆 A0/A1，右摇杆 A2/A3
  TEACH=D4  START=D5  PAUSE=A4  RESUME=A5  CANCEL=D13  按键另一头都接 GND
  注意多个任务要运用非阻塞式并且同步时间，设定好变量！！！！！moveToXYZ,doPickPlace期间不需要响应没用非阻塞
 */
#include <Servo.h>
/*遥控板 TX -> 本板 D2   遥控板 RX -> 本板 D3   两板 GND 要接一起
  下面这行必须写在 include 之前，把接收缓冲区从默认 64 改小到 32
 */

#define _SS_MAX_RX_BUFF 32
#include <SoftwareSerial.h>
SoftwareSerial padSerial(2, 3);   // 2=RX(接遥控板TX)  3=TX(接遥控板RX)

#define pin_x 9        // 底座
#define pin_y 8        // 大臂
#define pin_z 7        // 小臂
#define pin_w 6        // 爪子
#define pin_joy1_x A0  // 左摇杆 左右
#define pin_joy1_y A1  // 左摇杆 前后
#define pin_joy2_x A2  // 右摇杆 爪子
#define pin_joy2_y A3  // 右摇杆 上下
#define pin_teach  4   // 记录示教点
#define pin_start  5   // 开始绘制
#define pin_pause  A4  // 暂停
#define pin_resume A5  // 继续
#define pin_cancel 13  // 取消

const int basemin = 0;    const int basemax = 180;
const int farmmin = 30;   const int farmmax = 140;
const int rarmmin = 40;   const int rarmmax = 170;
const int clawmin = 25;   const int clawmax = 150;
const float L1 = 80.0;    // 大臂：肩轴到肘轴
const float L2 = 80.0;    // 小臂：肘轴到腕部
const float L3 = 38.0;    // 腕部到笔尖  ？？？？？
const float d0 = 15.7;    // 底座转轴到肩轴的水平距离
const float h0 = 56.9;    // 桌面到肩轴的高度

float zeroBase = 90.0;    //！！！！！！待调试
float zeroFarm = 130.0;
float zeroRarm = 15.0;
/*zeroBase：底座舵机转到这个角度时，整条臂正好朝正前方
  zeroFarm：大臂正好水平时的舵机角度
  zeroRarm：小臂和大臂正好成一条直线时的舵机角度
  zba90    把底座零位设成 90
  zfa130   把肩零位设成 130
  zra15    把肘零位设成 15
  zero     查看零位
 */
int delaytime = 15;
const int step = 2;
const float joySpeed = 70.0;

int mid1x = 512, mid1y = 512;   //搭配死区一起减小误差，避免因为硬件导致的抽搐！！！！！待调试
int mid2x = 512, mid2y = 512;

Servo servoX;   // 底座
Servo servoY;   // 大
Servo servoZ;   // 小
Servo servoW;   // 爪

int angleBase = 90, angleFarm = 90, angleRarm = 90, angleClaw = 55;   //初始化！！！实际角度有误差待调试
int targetBase = 90, targetFarm = 90, targetRarm = 90, targetClaw = 55;

float px = 100.0, py = 0.0, pz = 40.0;

int mode = 0;
int padIndex = 0;

unsigned long lasttime = 0;
unsigned long lastJoyTime = 0;

const int teachMax = 5;
float teachX[teachMax];
float teachY[teachMax];
float teachZ[teachMax];
int teachCount = 0;

const int pathMax = 34;

int pathX[pathMax];
int pathY[pathMax];
int pathZ[pathMax];
int pathCount = 0;

bool drawCurve = false;

bool drawing = false;
bool paused  = false;
int drawSeg = 0;
int drawStep = 0;
const int stepsPerSeg = 10;
unsigned long drawLastTime = 0;

const int recordMax = 120;
byte recordAngles[recordMax][4];   //byt 省内存
int recordCount = 0;
bool recording = false;
bool playing   = false;
int playIndex = 0;
unsigned long recordLastTime = 0;
unsigned long playLastTime = 0;

bool lastTeach = HIGH, lastStart = HIGH, lastPause = HIGH;
bool lastResume = HIGH, lastCancel = HIGH;

String command = "";
bool complete = false;
String padCmd = "";

int   xianZhi(int v, int lo, int hi);
bool  niYunDongXue(float x, float y, float z, int &a1, int &a2, int &a3);
void  zhengYunDongXue(int a1, int a2, int a3, float &x, float &y, float &z);
void  moveOne(Servo &servo, int &current, int target);
void  moveAllServo();
void  moveAllServoNow();
void  moveToXYZ(float x, float y, float z);
void  chongZhiShiJian();
void  tongBuWeiZhi();
void  readJoystick();
void  readButtons();
void  doTeach();
void  addShapePoint(float x, float y);
void  loadShape(int n);
void  buildPath();
void  startDraw();
void  updateDraw();
void  doPause();
void  doResume();
void  doCancel();
void  updateRecord();
void  doPickPlace(int n);
void  goHome();
bool  parseXYZW(String cmd, int &x, int &y, int &z, int &w);
bool  parseGoto(String cmd, float &x, float &y, float &z);
void  readSerial();
void  readPad();
void  printHelp();

int xianZhi(int v, int low, int high) {
  if (v < low) v = low;
  if (v > high) v = high;
  return v;
}

bool niYunDongXue(float x, float y, float z, int &a1, int &a2, int &a3) {
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

  float th2 = phi + alpha;
  float th3 = -(PI - beta);

  int deg1 = th1 * 180.0 / PI;
  int deg2 = th2 * 180.0 / PI;
  int deg3 = th3 * 180.0 / PI;

  a1 = zeroBase + deg1;
  a2 = zeroFarm - deg2;
  a3 = zeroRarm - deg3;

  if (a1 < basemin || a1 > basemax) return false;
  if (a2 < farmmin || a2 > farmmax) return false;
  if (a3 < rarmmin || a3 > rarmmax) return false;
  return true;
}

void zhengYunDongXue(int a1, int a2, int a3, float &x, float &y, float &z) {

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
  if (now - lasttime < (unsigned long)delaytime) return;   //非阻塞式的关键
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
  if (!niYunDongXue(x, y, z, a1, a2, a3)) {
    Serial.println(F("这个点够不到，跳过"));
    return;
  }
  targetBase = a1;
  targetFarm = a2;
  targetRarm = a3;

  int count = 0;
  while (count < 400) {
    if (angleBase == targetBase && angleFarm == targetFarm && angleRarm == targetRarm) break;
    moveAllServoNow();
    delay(15);
    count++;
  }
  chongZhiShiJian();
}

void chongZhiShiJian() {
  unsigned long now = millis();
  lastJoyTime  = now;
  lasttime     = now;
  drawLastTime = now;
}

void tongBuWeiZhi() {
  zhengYunDongXue(angleBase, angleFarm, angleRarm, px, py, pz);
}

void readJoystick() {
  if (mode != 0) { lastJoyTime = millis(); return; }
  unsigned long now = millis();
  if (now - lastJoyTime < 20) return;
  float dt = (now - lastJoyTime) / 1000.0;
  lastJoyTime = now;

  if (dt > 0.05) dt = 0.05;     //动态时间让程序更平稳不会大跳

  int j1x = analogRead(pin_joy1_x);
  int j1y = analogRead(pin_joy1_y);
  int j2x = analogRead(pin_joy2_x);
  int j2y = analogRead(pin_joy2_y);

  float v1x = (j1x - mid1x) / 512.0;
  float v1y = (j1y - mid1y) / 512.0;
  float v2x = (j2x - mid2x) / 512.0;
  float v2y = (j2y - mid2y) / 512.0;

  if (v1x > -0.12 && v1x < 0.12) v1x = 0;
  if (v1y > -0.12 && v1y < 0.12) v1y = 0;
  if (v2x > -0.12 && v2x < 0.12) v2x = 0;
  if (v2y > -0.12 && v2y < 0.12) v2y = 0;

  float oldx = px, oldy = py, oldz = pz;

  bool tuiLe = (v1x != 0 || v1y != 0 || v2y != 0);

  if (tuiLe) {
    px = px + v1y * joySpeed * dt;
    py = py + v1x * joySpeed * dt;
    pz = pz - v2y * joySpeed * dt;

    if (px < 50) px = 50;
    if (px > 150) px = 150;
    if (py < -60) py = -60;
    if (py > 60) py = 60;
    if (pz < 5) pz = 5;
    if (pz > 90) pz = 90;

    int a1, a2, a3;
    if (niYunDongXue(px, py, pz, a1, a2, a3)) {
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

  v = digitalRead(pin_teach);
  if (v == LOW && lastTeach == HIGH) { delay(30); if (digitalRead(pin_teach) == LOW) doTeach(); }
  lastTeach = v;

  v = digitalRead(pin_start);
  if (v == LOW && lastStart == HIGH) { delay(30); if (digitalRead(pin_start) == LOW) startDraw(); }
  lastStart = v;

  v = digitalRead(pin_pause);
  if (v == LOW && lastPause == HIGH) { delay(30); if (digitalRead(pin_pause) == LOW) doPause(); }
  lastPause = v;

  v = digitalRead(pin_resume);
  if (v == LOW && lastResume == HIGH) { delay(30); if (digitalRead(pin_resume) == LOW) doResume(); }
  lastResume = v;

  v = digitalRead(pin_cancel);
  if (v == LOW && lastCancel == HIGH) { delay(30); if (digitalRead(pin_cancel) == LOW) doCancel(); }
  lastCancel = v;
}

void doTeach() {
  if (teachCount >= teachMax) {
    Serial.println(F("示教点已经记满了"));
    return;
  }
  teachX[teachCount] = px;
  teachY[teachCount] = py;
  teachZ[teachCount] = pz;
  teachCount++;
  Serial.print(F("记录第 "));
  Serial.print(teachCount);
  Serial.print(F(" 个点： x="));
  Serial.print(px);
  Serial.print(F(" y="));
  Serial.print(py);
  Serial.print(F(" z="));
  Serial.println(pz);
  if (teachCount == teachMax) Serial.println(F("5 个点记满了，可以按 run 开始画"));
}

void addShapePoint(float x, float y) {
  if (teachCount >= teachMax) return;
  teachX[teachCount] = x;
  teachY[teachCount] = y;
  teachZ[teachCount] = 0;
  teachCount++;
}

void loadShape(int n) {
  teachCount = 0;
  drawCurve = false; 
  if (n == 1) {
    addShapePoint(75, -40);
    addShapePoint(140, -40);
    Serial.println(F("画直线"));
  } else {
    addShapePoint(80, -35);
    addShapePoint(80, 35);
    addShapePoint(135, 0);
    addShapePoint(80, -35);
    Serial.println(F("画三角形"));
  }
  startDraw();
}

void buildPath() {
  pathCount = 0;
  if (drawCurve == false) {
    for (int i = 0; i < teachCount; i++) {
      pathX[i] = teachX[i];
      pathY[i] = teachY[i];
      pathZ[i] = teachZ[i];
    }
    pathCount = teachCount;
    return;
  }
  const int perSeg = 8;
  for (int i = 0; i < teachCount - 1; i++) {
    float p0x, p0y, p0z, p3x, p3y, p3z;
    if (i == 0) { p0x = teachX[0]; p0y = teachY[0]; p0z = teachZ[0]; }
    else        { p0x = teachX[i - 1]; p0y = teachY[i - 1]; p0z = teachZ[i - 1]; }
    if (i == teachCount - 2) { p3x = teachX[teachCount - 1]; p3y = teachY[teachCount - 1]; p3z = teachZ[teachCount - 1]; }
    else                     { p3x = teachX[i + 2]; p3y = teachY[i + 2]; p3z = teachZ[i + 2]; }

    float p1x = teachX[i],     p1y = teachY[i],     p1z = teachZ[i];
    float p2x = teachX[i + 1], p2y = teachY[i + 1], p2z = teachZ[i + 1];

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
    pathX[pathCount] = teachX[teachCount - 1];
    pathY[pathCount] = teachY[teachCount - 1];
    pathZ[pathCount] = teachZ[teachCount - 1];
    pathCount++;
  }
}

void startDraw() {
  if (teachCount < 2) {
    Serial.println(F("点数不够"));
    return;
  }
  buildPath();
  mode = 1;
  drawing = true;
  paused = false;
  drawSeg = 0;
  drawStep = 0;
  drawLastTime = millis();
  goHome();
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
    tongBuWeiZhi();
    Serial.println(F("绘制完成"));
    return;
  }

  float t = drawStep / (float)stepsPerSeg;
  float x = pathX[drawSeg] + (pathX[drawSeg + 1] - pathX[drawSeg]) * t;
  float y = pathY[drawSeg] + (pathY[drawSeg + 1] - pathY[drawSeg]) * t;
  float z = pathZ[drawSeg] + (pathZ[drawSeg + 1] - pathZ[drawSeg]) * t;

  int a1, a2, a3;
  if (niYunDongXue(x, y, z, a1, a2, a3)) {
    targetBase = a1; targetFarm = a2; targetRarm = a3;
    angleBase = a1;  angleFarm = a2;  angleRarm = a3;
    servoX.write(a1);
    servoY.write(a2);
    servoZ.write(a3);
  } else {
    Serial.println(F("路径够不到，跳过去了"));
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
void doResume() {
  if (drawing) { paused = false; drawLastTime = millis(); Serial.println(F("继续")); }
}
void doCancel() {
  if (drawing || teachCount > 0) {
    drawing = false;
    paused = false;
    mode = 0;
    teachCount = 0;
    Serial.println(F("已取消，示教点清空，回待机"));
    moveToXYZ(100, 0, 60);
    tongBuWeiZhi();
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
    if (now - playLastTime >= 100) {
      playLastTime = now;
      if (playIndex >= recordCount) {
        playing = false;
        mode = 0;
        tongBuWeiZhi();
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

void doPickPlace(int n) {
  /* 
   *   物体  取的位置      放的位置
   *    A   (110, -35)   (140,  20)
   *    B   ( 95, -10)   (120,  50)
   *    C   ( 75,  20)   (100, -50)
   */
  float fromX, fromY, toX, toY;
  if (n == 1)      { fromX = 110; fromY = -35; toX = 140; toY =  20; }
  else if (n == 2) { fromX =  95; fromY = -10; toX = 120; toY =  50; }
  else             { fromX =  75; fromY =  20; toX = 100; toY = -50; }

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
  tongBuWeiZhi();
  chongZhiShiJian();
}

void goHome() {
  moveToXYZ(100, 0, 60);
  targetClaw = clawmax;
  tongBuWeiZhi();
  Serial.println(F("回到待机位置"));
  chongZhiShiJian();
}

bool parseXYZW(String cmd, int &x, int &y, int &z, int &w) {
  int idxX = cmd.indexOf('x');
  int idxY = cmd.indexOf('y');
  int idxZ = cmd.indexOf('z');
  int idxW = cmd.indexOf('w');
  if (idxX == -1 || idxY == -1 || idxZ == -1) return false;

  String sx, sy, sz, sw;
  if (idxW == -1) {
    sx = cmd.substring(idxX + 1, idxY);
    sy = cmd.substring(idxY + 1, idxZ);
    sz = cmd.substring(idxZ + 1);
    w  = targetClaw;
  } else {
    sx = cmd.substring(idxX + 1, idxY);
    sy = cmd.substring(idxY + 1, idxZ);
    sz = cmd.substring(idxZ + 1, idxW);
    sw = cmd.substring(idxW + 1);
    sw.replace(",", "");
    w  = sw.toInt();
  }
  sx.replace(",", "");
  sy.replace(",", "");
  sz.replace(",", "");
  x = sx.toInt();
  y = sy.toInt();
  z = sz.toInt();
  return true;
}

bool parseGoto(String cmd, float &x, float &y, float &z) {
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
      Serial.print(F("变快了,delaytime = "));
      Serial.println(delaytime);
    } else if (command == "l") {
      delaytime += 5;
      if (delaytime > 50) delaytime = 50;
      Serial.print(F("变慢了,delaytime = "));
      Serial.println(delaytime);
    } else if (command == "o") {
      targetClaw = clawmax;
      Serial.println(F("爪子张开"));
    } else if (command == "s") {
      targetClaw = clawmin;
      Serial.println(F("爪子夹紧"));
    } else if (command == "cal") {
      mid1x = analogRead(pin_joy1_x);
      mid1y = analogRead(pin_joy1_y);
      mid2x = analogRead(pin_joy2_x);
      mid2y = analogRead(pin_joy2_y);
      Serial.print(F("中值已校准  左X="));
      Serial.print(mid1x);
      Serial.print(F(" Y="));
      Serial.print(mid1y);
      Serial.print(F("   右X="));
      Serial.print(mid2x);
      Serial.print(F(" Y="));
      Serial.println(mid2y);
    } else if (command == "joy") {
      Serial.print(F("左摇杆 X="));
      Serial.print(analogRead(pin_joy1_x));
      Serial.print(F(" Y="));
      Serial.print(analogRead(pin_joy1_y));
      Serial.print(F("   右摇杆 X="));
      Serial.print(analogRead(pin_joy2_x));
      Serial.print(F(" Y="));
      Serial.println(analogRead(pin_joy2_y));
    } else if (command == "t") {
      doTeach();
    } else if (command == "run") {
      startDraw();
    } else if (command == "pause") {
      doPause();
    } else if (command == "go") {
      doResume();
    } else if (command == "stop") {
      doCancel();
    } else if (command == "line") {
      drawCurve = false;
      Serial.println(F("切换到折线"));
    } else if (command == "curve") {
      drawCurve = true;
      Serial.println(F("切换到平滑曲线"));
    } else if (command == "rec") {
      recording = true;
      recordCount = 0;
      recordLastTime = millis();
      Serial.println(F("开始录制"));
    } else if (command == "endrec") {
      recording = false;
      Serial.print(F("录完了，一共 "));
      Serial.print(recordCount);
      Serial.print(F(" 个点，大约 "));
      Serial.print(recordCount / 10);
      Serial.println(F(" 秒"));
      if (recordCount < 100) Serial.println(F("不够 10 秒"));
    } else if (command == "play") {
      if (recordCount == 0) {
        Serial.println(F("还没录过，先发 rec 录一段"));
      } else {
        recording = false;
        playing = true;
        playIndex = 0;
        mode = 2;
        playLastTime = millis();
        Serial.println(F("开始回放"));
      }
    } else if (command == "home") {
      goHome();
    } else if (command == "dline") {
      loadShape(1);
    } else if (command == "dtri") {
      loadShape(2);
    } else if (command == "a") {
      doPickPlace(1);
    } else if (command == "b") {
      doPickPlace(2);
    } else if (command == "c") {
      doPickPlace(3);
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
      String head = command.substring(0, 3);
      float  val  = command.substring(3).toFloat();
      if (head == "zba") {
        zeroBase = val;
        Serial.print(F("底座零位改成 "));
        Serial.println(val);
      } else if (head == "zfa") {
        zeroFarm = val;
        Serial.print(F("肩零位改成 "));
        Serial.println(val);
      } else if (head == "zra") {
        zeroRarm = val;
        Serial.print(F("肘零位改成 "));
        Serial.println(val);
      } else {
        Serial.println(F("零位指令只有 zba / zfa / zra 三个"));
      }
    } else if (command.charAt(0) == 'g') {
      float gx, gy, gz;
      if (parseGoto(command, gx, gy, gz)) {
        int a1, a2, a3;
        if (niYunDongXue(gx, gy, gz, a1, a2, a3)) {
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
        zhengYunDongXue(targetBase, targetFarm, targetRarm, px, py, pz);
        Serial.print(F("X="));
        Serial.print(targetBase);
        Serial.print(F(" Y="));
        Serial.print(targetFarm);
        Serial.print(F(" Z="));
        Serial.print(targetRarm);
        Serial.print(F(" W="));
        Serial.println(targetClaw);
      } else {
        Serial.println(F("指令错误，发?查说明"));
      }
    }
  }
  command = "";
  complete = false;
}

void readPad() {
  while (padSerial.available()) {
    char c = (char)padSerial.read();
    if (c == '\n' || c == '\r') {
      if (padCmd.length() == 0) continue;
      padCmd.trim();
      padCmd.toLowerCase();

      Serial.print(F("[遥控板] 收到 "));
      Serial.println(padCmd);

      if (padCmd == "k1") {
        doPickPlace(padIndex % 3 + 1);
        padIndex++;
      } else if (padCmd == "k2") {
        if (recording == false) {
          recording = true;
          recordCount = 0;
          recordLastTime = millis();
          Serial.println(F("开始录制（用摇杆操控机械臂）"));
        } else {
          recording = false;
          Serial.print(F("录完了，一共 "));
          Serial.print(recordCount);
          Serial.print(F(" 个点，大约 "));
          Serial.print(recordCount / 10);
          Serial.println(F(" 秒"));
        }
      } else if (padCmd == "k3") {
        if (recordCount > 0) {
          recording = false;
          playing = true;
          playIndex = 0;
          mode = 2;
          playLastTime = millis();
          Serial.println(F("开始回放"));
        } else {
          Serial.println(F("还没录过按遥控板按键2录一段"));
        }
      } else if (padCmd == "k4") {
        goHome();
        Serial.println(F("回到待机位置"));
      }

      padCmd = "";
    } else {
     if (padCmd.length() < 8) padCmd += c;
    }
  }
}

void printHelp() {
  Serial.println(F("===== MeArm 指令表 ====="));
  Serial.println(F("x90,y60,z120,w30  直接给四个舵机角度"));
  Serial.println(F("g100,0,12         笔尖走到坐标(100,0,12)"));
  Serial.println(F("h / l             变快 / 变慢"));
  Serial.println(F("o / s             爪子张开 / 夹紧"));
  Serial.println(F("t                 记录示教点（共 5 个）"));
  Serial.println(F("run               开始绘制"));
  Serial.println(F("pause / go / stop 暂停 / 继续 / 取消"));
  Serial.println(F("line / curve      折线 / 平滑曲线"));
  Serial.println(F("dline / dtri      画直线 / 画三角形"));
  Serial.println(F("rec / endrec / play 录制 / 结束 / 回放"));
  Serial.println(F("home              回待机  a / b / c 抓取放置"));
  Serial.println(F("joy               看摇杆原始读数（排查抖动）"));
  Serial.println(F("cal               摇杆松开不动时发，校准中值"));
  Serial.println(F("zba90 / zfa130 / zra15 改零位    zero 看零位"));
  Serial.println(F(""));
  Serial.println(F("按键D4记点 D5开始 A4暂停 A5继续 D13取消"));
  Serial.println(F("遥控板K1循环夹取 K2录制 K3播放 K4回中"));
}

void setup() {
  Serial.begin(9600);
  padSerial.begin(9600);

  pinMode(2, INPUT_PULLUP);

  servoX.attach(pin_x);
  servoY.attach(pin_y);
  servoZ.attach(pin_z);
  servoW.attach(pin_w);

  servoX.write(angleBase);
  servoY.write(angleFarm);
  servoZ.write(angleRarm);
  servoW.write(angleClaw);

  pinMode(pin_teach, INPUT_PULLUP);
  pinMode(pin_start, INPUT_PULLUP);
  pinMode(pin_pause, INPUT_PULLUP);
  pinMode(pin_resume, INPUT_PULLUP);
  pinMode(pin_cancel, INPUT_PULLUP);

  goHome();

  Serial.println(F("Ready"));

  printHelp();
}

void loop() {
  readJoystick();
  readButtons();
  readSerial();
  readPad();
  updateDraw();
  updateRecord();
  moveAllServo();
}
