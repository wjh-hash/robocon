#include <Servo.h>

Servo base, farm, rarm, claw;

void setup() {
  base.attach(9);
  farm.attach(8);
  rarm.attach(7);
  claw.attach(6);
  Serial.begin(9600);
}

void loop() {
  // 读取摇杆
  int xVal = analogRead(A0);  // 摇杆1 X轴
  int yVal = analogRead(A1);  // 摇杆1 Y轴
  int x2Val = analogRead(A2); // 摇杆2 X轴
  int y2Val = analogRead(A3); // 摇杆2 Y轴

  // 映射到舵机角度
  int baseAngle = map(xVal, 0, 1023, 0, 180);
  int farmAngle = map(yVal, 0, 1023, 0, 180);
  int raemAngle = map(x2Val, 0, 1023, 0, 180);
  int clawAngle = map(y2Val, 0, 1023, 0, 180);

  // 驱动舵机
  base.write(baseAngle);
  farm.write(farmAngle);
  rarm.write(raemAngle);
  claw.write(clawAngle);

  delay(15); 
}
