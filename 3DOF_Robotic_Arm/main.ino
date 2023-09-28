/*
  main.ino
  ME134 HW #2
  Jacob Choi 09/28/23
*/

#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <math.h>

/* GPIO */
#define J1_DIR_PIN 2  // HIGH: CW; LOW: CCW
#define J1_STEP_PIN 15
#define J2_DIR_PIN 4
#define J2_STEP_PIN 0
#define J3_PWM_PIN 32

/* Global variables */
#define NEMA_STEP 200.0 // Nema 17HS4401 stepper motor spec.
float l1 = 1;
float l2 = 1;
float l3 = 1;

// wrapper class
class Arm {
  public:
    Arm() {
      AccelStepper J1(1, J1_STEP_PIN, J1_DIR_PIN);
      AccelStepper J2(1, J2_STEP_PIN, J2_DIR_PIN);
      Servo J3;
      J3.attach(J3_PWM_PIN);
    }

    void write(int x, int y, int z) {
      inverseKinematics(x, y, z);
      writeAngles();
      // call writeAngle() with results from above
    }

    void inverseKinematics(int x, int y, int z) {
      theta1 = atan(y/x);
      double a = 0;
      if(cos(theta1) <= 0.0001) a = y/sin(theta1) - l1;
      else a = x/cos(theta1) - l1;

      theta2 = 0;
      double temp = a*a + z*z;
      if(temp <= 0.0001) theta2 = acos((l2*l2-l3*l3)/(2*l2));
      else theta2 = acos((a*a+z*z+l2*l2-l3*l3)/(2*l2));
      if(z <= 0.0001) theta2 += 90; 
      else theta2 += atan(a/z);

      theta3 = 0;
      if(temp <= 0.0001) theta3 = acos((l3*l3-l2*l2)/(2*l3)) - theta2;
      else theta3 = acos((a*a+z*z+l3*l3-l2*l2)/(2*l3)) - theta2;
      if(z <= 0.0001) theta3 += 90;
      else theta3 += atan(a/z);

      this->x = x;
      this->y = y;
      this->z = z;
    }

    void writeAngles() {
      int theta1_pos = int((theta1/360.0) * NEMA_STEP) + 1; // + 1 to account for float rounding truncation
      int theta2_pos = int((theta2/360.0) * NEMA_STEP) + 1;
      int theta3_pos = int(theta3) + 1;
      /*
      J1.moveTo(theta1_pos);
      J2.moveTo(theta2_pos);
      while (J1.distanceToGo > 0 && J2.distanceToGo > 0) {
        J1.run();
        J2.run();
      }
      J3.write(theta3_pos);
      */

      theta1_true = J1.currentPosition();
      theta2_true = J2.currentPosition();
      theta3_true = J3.read();
    }

    void print() {
      Serial.println("ANGLES:");
      Serial.println(String(theta1) + " " + String(theta2) + " " + String(theta3));

      Serial.println("COORDINATES:");
      Serial.println(String(x) + " " + String(y) + " " + String(z));

      Serial.println("TRUE ANGLES:");
      Serial.println(String(theta1_true) + " " + String(theta2_true) + " " + String(theta3_true));
    }

  private:
    /* Actuators */
    AccelStepper J1;
    AccelStepper J2;
    Servo J3;
    
    /* Angles */
    float theta1;
    float theta2;
    float theta3;

    int theta1_true;
    int theta2_true;
    int theta3_true;

    /* XYZ Coordinates */
    float x;
    float y;
    float z;
};

/* */

AccelStepper joint1(1, J1_STEP_PIN, J1_DIR_PIN);
AccelStepper joint2(1, J2_STEP_PIN, J2_DIR_PIN);
Servo joint3;

void setup() {
  Serial.begin(115200);

  joint1.setMaxSpeed(1000);
  joint1.setAcceleration(30);
  joint2.setMaxSpeed(1000);
  joint2.setAcceleration(30);
  joint3.attach(J3_PWM_PIN);
}
void loop() {
  while (true) {
    joint1.moveTo(25);
    joint2.moveTo(33);
    joint1.runToPosition();
    delay(1000);
    joint2.runToPosition();
    delay(1000);
    joint3.write(60);
    delay(1000);

    joint1.moveTo(0);
    joint2.moveTo(0);
    joint1.runToPosition();
    delay(1000);
    joint2.runToPosition();
    delay(1000);
    joint3.write(0);
    delay(10000);
  }
}
