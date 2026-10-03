#include <Servo.h>
#define pin_x 9    // x base
#define pin_y 8   // y farm
#define pin_z 7   // z rarm
#define pin_w 6   // w claw
Servo servoX;                         
Servo servoY;                     
Servo servoZ;                           
Servo servoW;
const int basemin=0;
const int basemax=180;
const int farmmin=35;
const int farmmax=120;
const int rarmmin=45;
const int rarmmax=180;
const int clawmin=25;
const int clawmax=100;
int currentX = 90, currentY = 90, currentZ = 90, currentW = 55; 
int targetX = 90, targetY = 90, targetZ = 90, targetW = 90;  
int delaytime= 15;        
const int shorttime= 3;   
const int longtime= 50;  
const int step= 2; 
unsigned long lasttime= 0;

String command = "";
bool complete = false;

void setup() {
  Serial.begin(9600);

  servoX.attach(pin_x);
  servoY.attach(pin_y);
  servoZ.attach(pin_z);
  servoW.attach(pin_w);
  servoX.write(currentX);
  servoY.write(currentY);
  servoZ.write(currentZ);
  servoW.write(currentW);
  Serial.println("Ready");
}

void loop() {
  while (Serial.available()) {     //String command = Serial.readStringUntil('\n');
    char input = (char)Serial.read();
    if (input == '\n' || input == '\r') {
      complete = true;
      break;
    } else {
      command += input;
    }
  }

  if (complete) {
    command.trim();    //修剪头尾空白
    if (command.length() > 0) {
      command.toLowerCase();
      if (command == "h") {
        delaytime -= 5;
        if (delaytime < shorttime) delaytime = shorttime;
        Serial.print("fast,delaytime = ");
        Serial.println(delaytime);
      } else if (command == "l") {
        delaytime += 5;
        if (delaytime > longtime) delaytime = longtime;
        Serial.print("low,delaytime = ");
        Serial.println(delaytime);
      } else if (command == "o") {
        servoW.write(95);
        currentW=95;
      } else if (command == "s") {
        servoW.write(30);
        currentW=30;
      } else {
        int x, y, z, w;
        if (parseXYZW(command, x, y, z, w)) {
          targetX = constrain(x, basemin, basemax);
          targetY = constrain(y, farmmin, farmmax);
          targetZ = constrain(z, rarmmin, rarmmax);
          targetW = constrain(w, clawmin, clawmax);          
          Serial.print("X=");
          Serial.print(targetX);
          Serial.print(" Y=");
          Serial.print(targetY);
          Serial.print(" Z=");
          Serial.print(targetZ);
          Serial.print(" W=");
          Serial.println(targetW);
        } else {
          Serial.println("Invalid command");
        }
      }
    }
    command ="";
    complete = false;
  }

  unsigned long now = millis();
  if (now - lasttime>= delaytime) {
    lasttime= now;
    moveservo(servoX, currentX, targetX);
    moveservo(servoY, currentY, targetY);    
    moveservo(servoZ, currentZ, targetZ);
    moveservo(servoW, currentW, targetW);   
  }
}

void moveservo(Servo &servo, int &current, int target) {   //?????????取地址符
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

bool parseXYZW(String cmd, int &x, int &y, int &z,int &w ) {
  cmd.replace(" ", "");
  int idxX = cmd.indexOf('x');
  int idxY = cmd.indexOf('y');
  int idxZ = cmd.indexOf('z');
  int idxW = cmd.indexOf('w');
  if (idxX == -1 || idxY == -1 || idxZ == -1|| idxW == -1) return false;

  String sx = cmd.substring(idxX + 1, idxY+1);
  String sy = cmd.substring(idxY + 1, idxZ+1);
  String sz = cmd.substring(idxZ + 1, idxW+1);
  String sw = cmd.substring(idxW + 1, (cmd.length()+1));
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
