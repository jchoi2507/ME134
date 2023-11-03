/*
motorController.ino
ME134 HW #3
Jacob Choi 10/19/2023
*/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

/* GPIO */
#define PWM 35 //27
#define CW 32 //26
#define CCW 33 //25

//#define ENCODER_1 33
//#define ENCODER_2 32

/* PWM */
#define MAX_DUTY 225
#define MIN_DUTY 0

/* PID */
float kp = 70;
float kd = 5000; // 5000
float ki = 0;
float deltaTime = 5000;

int targetAngle = 0;
float e_integral = 0;
float e_derivative;
float u;
int PID_signal;

const float ANGLE_THRESHOLD = 15.00; // operating range (degrees)
long currTime, prevTime = 0;
float currError, prevError;

/* Encoder */
//int encoder_1;
//int encoder_2;
//volatile int encoderCounter = 0; // encoder value

/* MPU6050 IMU */
float currAngle;
const float MPU6050_OFFSET = 3.8;

void setup() {
  Serial.begin(115200);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (true) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);

  pinMode(PWM, OUTPUT);
  pinMode(CW, OUTPUT);
  pinMode(CCW, OUTPUT);
  //pinMode(ENCODER_1, INPUT);
  //pinMode(ENCODER_2, INPUT);

  reactionWheel();
}

void loop() { }

void reactionWheel() {
  while (true) {
    currTime = micros();
    
    if (currTime - prevTime > deltaTime) { // PID loop frequency: 200Hz
      readSensor();
      if (abs(currAngle) < ANGLE_THRESHOLD) { // start PID loop
          // Proportional
          currError = targetAngle - currAngle;

          // Integral
          //e_integral = e_integral + currError*deltaTime;
          //e_integral = constrain(e_integral, -127, 127); // accounting for integral windup

          //if (currError < 0) { targetAngle -= 0.0001; } // dithering
          //else { targetAngle += 0.0001; }

          // Derivative
          e_derivative = (currError - prevError)/deltaTime;
          prevError = currError;

          // Control signal
          u = kp * currError + kd * e_derivative + ki * e_integral;
          PID_signal = constrain(u, -255, 255);

          if (PID_signal > 0) { // turn motor CCW
            motorController(true, abs(PID_signal));
            //Serial.println("TURNING CW: ");
            //Serial.println(PID_signal);
          }

          else if (PID_signal < 0) { // turn motor CW
            motorController(false, abs(PID_signal));
            //Serial.println("TURNING CCW: ");
            //Serial.println(PID_signal);
          }
      }
      else { analogWrite(PWM, MIN_DUTY); } // turn off motor if outside range
      prevTime = currTime;
    }
  }
}

void motorController(bool cw, int PID_signal) {
  if (cw) { // CW rotation
    digitalWrite(CW, HIGH);
    digitalWrite(CCW, LOW);
    //delay(pause_ms);
  }

  else if (!cw) { // CCW rotation
    digitalWrite(CCW, HIGH);
    digitalWrite(CW, LOW);
    //delay(pause_ms);
  }
  analogWrite(PWM, PID_signal);
}

void readSensor() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    currAngle = MPU6050_OFFSET + atan2(-a.acceleration.x , sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
}
