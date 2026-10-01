void setup() {
  // put your setup code here, to run once:

}
void loop() {
  int rawX = analogRead(PIN_JOY1_X); // 读取摇杆值
  int angleX = getJoystickAngle(rawX, 512, 30); // 调用自定义函数
  baseServo.write(angleX);
}
int getJoystickAngle(int rawValue, int center, int deadzone) {
  int angle;
  if (rawValue > center + deadzone) {
    angle = map(rawValue, center + deadzone, 1023, 90, 180);
  } 
  else if (rawValue < center - deadzone) {
    angle = map(rawValue, 0, center - deadzone, 0, 90);
  } 
  else {
    angle = 90; 
  }
  eturn constrain(angle, 0, 180);
}
