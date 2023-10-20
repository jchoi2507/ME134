/*
readMPU6050.ino
ME134 HW #3
Jacob Choi 10/19/2023
*/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

int minVal=265;
int maxVal=402;
float initialAngle = 0;

void setup(void) {
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
}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  /*
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");
  */

  /*
  int xAng = map(a.acceleration.x, minVal, maxVal, -90, 90);
  int yAng = map(a.acceleration.y, minVal, maxVal, -90, 90);
  double z = RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
  Serial.println(z);
  */

  float roll = atan2(a.acceleration.y , a.acceleration.z) * 180.0 / PI;
  float pitch = atan2(-a.acceleration.x , sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI; //account for roll already applied


  Serial.println(pitch + 7);

  Serial.println("");
  delay(500);
}
