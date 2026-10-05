#define pin_joy1_x A0
#define pin_joy1_y A1
#define pin_joy2_x A2
#define pin_joy2_y A3

int joy1x_center = 512;
int joy1y_center = 512;
int joy2x_center = 512;
int joy2y_center = 512;
const int CALIB_SAMPLES = 50;
const float DEAD_ZONE = 0.05;

void calibrateJoysticks() {
  Serial.println(F("正在校准摇杆，请勿触碰..."));
  delay(1000);
  long sum1x = 0, sum1y = 0, sum2x = 0, sum2y = 0;
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    sum1x += analogRead(pin_joy1_x);
    sum1y += analogRead(pin_joy1_y);
    sum2x += analogRead(pin_joy2_x);
    sum2y += analogRead(pin_joy2_y);
    delay(5);
  }
  joy1x_center = sum1x / CALIB_SAMPLES;
  joy1y_center = sum1y / CALIB_SAMPLES;
  joy2x_center = sum2x / CALIB_SAMPLES;
  joy2y_center = sum2y / CALIB_SAMPLES;
  Serial.print(F("校准完成: "));
  Serial.print(joy1x_center); Serial.print(F(", "));
  Serial.print(joy1y_center); Serial.print(F(", "));
  Serial.print(joy2x_center); Serial.print(F(", "));
  Serial.println(joy2y_center);
}

void setup() {
  Serial.begin(9600);
  calibrateJoysticks();
}

void loop() {
  int j1x = analogRead(pin_joy1_x);
  int j1y = analogRead(pin_joy1_y);
  int j2x = analogRead(pin_joy2_x);
  int j2y = analogRead(pin_joy2_y);

  float v1x = (j1x - joy1x_center) / 512.0;
  float v1y = (j1y - joy1y_center) / 512.0;
  float v2x = (j2x - joy2x_center) / 512.0;
  float v2y = (j2y - joy2y_center) / 512.0;

  if (abs(v1x) < DEAD_ZONE) v1x = 0;
  if (abs(v1y) < DEAD_ZONE) v1y = 0;
  if (abs(v2x) < DEAD_ZONE) v2x = 0;
  if (abs(v2y) < DEAD_ZONE) v2y = 0;

  Serial.print(F("v1x=")); Serial.print(v1x, 3);
  Serial.print(F(" v1y=")); Serial.print(v1y, 3);
  Serial.print(F(" v2x=")); Serial.print(v2x, 3);
  Serial.print(F(" v2y=")); Serial.println(v2y, 3);

  delay(100);
}