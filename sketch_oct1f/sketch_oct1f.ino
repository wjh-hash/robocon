void setup() {                                          //非阻塞式
  // put your setup code here, to run once:

}

int basepost = 90;          
int baseTarget = 90;        
unsigned long lastBaseMs = 0;
const int baseStep = 2;     
const int baseInterval = 15; 

void loop() {
  int angle1X = getJoystickAngle(raw1X, 512, 30, basemin, basemax);
  baseTarget = constrain(angle1X, basemin, basemax);

  if (millis() - lastBaseMs >= baseInterval) {
    lastBaseMs = millis();

    if (basepost < baseTarget) {
      basepost += baseStep;
      if (basepost > baseTarget) basepost = baseTarget;
      base.write(basepost);
    } else if (basepost > baseTarget) {
      basepost -= baseStep;
      if (basepost < baseTarget) basepost = baseTarget;
      base.write(basepost);
    }
  }


}
}
