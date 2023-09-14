#include "MotorControl.h"

void readEncoder() {
  encoder_1 = digitalRead(ENCODER_1_PIN);
  encoder_2 = digitalRead(ENCODER_2_PIN);
  if (encoder_1 != encoder_2) {encoderCounter += 5;} // CW rotation
  else if (encoder_1 == encoder_2) {encoderCounter -= 5;} // CCW rotation
}

void actuate() {
  int morseArrSize = morseCode.length();

  for (int i = 0; i < morseArrSize; i++) { 
    if (morseCode[i] == '.') { // CW for dots
      motorController(true, 50, 150); // initial clockwise rotation
      delay(1000); // hold "signal" for 1 second
      motorController(false, 50, 150); // reset back to origin
      delay(1000);
      checkPos();
    }
  
    else if (morseCode[i] == '-') { // CCW for dashes
      motorController(false, 50, 150); // initial clockwise rotation
      delay(1000); // hold "signal" for 1 second
      motorController(true, 50, 150); // reset back to origin
      delay(1000);
      checkPos();
    }
    Serial.println("COMPLETED MORSE ACTUATION FOR: " + String(morseCode[i]));
  }

  morseCode = ""; // reset overall morse code string
  delay(1000);
}

void motorController(bool cw, int duty_percentage, int pause_ms) {
  int duty_cycle = MAX_DUTY * duty_percentage/100; // converting percentage -> 0 ~ 225 range
  analogWrite(PWM_PIN, duty_cycle);

  if (cw) {
    digitalWrite(CW_PIN, HIGH);
    digitalWrite(CCW_PIN, LOW);
    delay(pause_ms);
  }

  else if (!cw) {
    digitalWrite(CCW_PIN, HIGH);
    digitalWrite(CW_PIN, LOW);
    delay(pause_ms);
  }
  analogWrite(PWM_PIN, MIN_DUTY);
}

void checkPos() {
  while (encoderCounter > ENCODER_THRESHOLD) { // rotated too far clockwise, need CCW offset
    Serial.println("ROTATING CCW TO RESET!!");
    motorController(false, 50, 50);
  }

  while (encoderCounter < -1 * ENCODER_THRESHOLD) {
    Serial.println("ROTATING CW TO RESET!!");
    motorController(true, 50, 50);
  }
  delay(500);
}

void translateToMorse() {
  String morseHour = "";
  String morseMinute = "";
  String morseColon = "---...";

  unsigned int hourArrSize = currHour.length(); // hours

  for (int i = 0; i < hourArrSize; i++) {
    String digit_str = String(currHour[i]); // converting char -> string
    int digit = digit_str.toInt(); // converting string -> int
    morseHour.concat(morseMap[digit]); // appending
  }

  unsigned int minuteArrSize = currMinute.length(); // minutes

  for (int i = 0; i < minuteArrSize; i++) {
    String digit_str = String(currMinute[i]); // converting char -> string
    int digit = digit_str.toInt(); // converting string -> int
    morseMinute.concat(morseMap[digit]); // appending
  }

  morseCode = morseHour + morseColon + morseMinute;
}
