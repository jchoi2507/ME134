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
#define PWM 27
#define CW 26
#define CCW 25

#define ENCODER_1 33
#define ENCODER_2 32

/* PWM */
#define MAX_DUTY 225
#define MIN_DUTY 0

/* Encoder */
int encoder_1;
int encoder_2;
volatile int encoderCounter = 0; // encoder value

/* MPU6050 IMU */
float initial_pitch;
float final_pitch;
float delta_pitch;
const float MPU6050_OFFSET = 9.0;
const float MPU6050_THRESHOLD = 2.00;

void setup() {
  Serial.begin(115200);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (true) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  delay(100);

  pinMode(PWM, OUTPUT);
  pinMode(CW, OUTPUT);
  pinMode(CCW, OUTPUT);
  pinMode(ENCODER_1, INPUT);
  pinMode(ENCODER_2, INPUT);

  reactionWheel();
}

void loop() { }

void motorController(bool cw, int duty_percentage) {
  int duty_cycle = MAX_DUTY * duty_percentage/100; // converting percentage -> 0 ~ 225 range
  analogWrite(PWM, duty_cycle);

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
}

void reactionWheel() {
  //sensors_event_t a, g, temp;
  //mpu.getEvent(&a, &g, &temp);
  //initial_pitch = MPU6050_OFFSET + atan2(-a.acceleration.x , sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

  while (true) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
  
    final_pitch = MPU6050_OFFSET + atan2(-a.acceleration.x , sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
    //delta_pitch = final_pitch - initial_pitch;

    /*
    Serial.print("INITIAL: ");
    Serial.println(initial_pitch);
    Serial.print("FINAL: ");
    Serial.println(final_pitch);
    delay(500); */

    if (final_pitch > MPU6050_THRESHOLD) {
      motorController(true, 45); // turn CW

      while (final_pitch > MPU6050_THRESHOLD) { // turn motor until balanced
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        final_pitch = MPU6050_OFFSET + atan2(-a.acceleration.x , sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
      }
      analogWrite(PWM, MIN_DUTY);
      //initial_pitch = final_pitch; // update vars
    }

    else if (final_pitch < -MPU6050_THRESHOLD) { 
      motorController(false, 45); // turn CCW

      while (final_pitch < -MPU6050_THRESHOLD) { // turn motor until balanced
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        final_pitch = MPU6050_OFFSET + atan2(-a.acceleration.x , sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
      }
      analogWrite(PWM, MIN_DUTY);
      //initial_pitch = final_pitch; // update vars
    }
  }
}
