/* 
  MotorControl.ino
  ME134 HW #1
  Jacob Choi 9/14/2023
*/

// readEncoder() increments encoder values based on phase shifts
void readEncoder() {
  encoder_1 = digitalRead(ENCODER_1_PIN);
  encoder_2 = digitalRead(ENCODER_2_PIN);
  if (encoder_1 != encoder_2) {encoderCounter += 5;} // CW rotation
  else if (encoder_1 == encoder_2) {encoderCounter -= 5;} // CCW rotation
}

// actuate() converts morse code -> motor rotation
void actuate() {
  int morseArrSize = morseCode.length();

  for (int i = 0; i < morseArrSize; i++) { 
    if (morseCode[i] == '.') { // CW for dots
      motorController(true, 45, 150); // initial CW rotation
      delay(500);
      motorController(false, 45, 150); // reset back to origin
      delay(500);
      checkPos(); // positional feedback
    }

    else if (morseCode[i] == '-') { // CCW for dashes
      motorController(false, 45, 150); // initial CCW rotation
      delay(500);
      motorController(true, 45, 150); // reset back to origin
      delay(500);
      checkPos(); // positional feedback
    }
    Serial.println("COMPLETED MORSE ACTUATION FOR: " + String(morseCode[i]));
  }
  morseCode = ""; // reset overall morse code string at the end
}

// motorController() sets the direction, PWM duty cycle, and pause intervals for motor rotation
void motorController(bool cw, int duty_percentage, int pause_ms) {
  int duty_cycle = MAX_DUTY * duty_percentage/100; // converting percentage -> 0 ~ 225 range
  analogWrite(PWM_PIN, duty_cycle);

  if (cw) { // CW rotation
    digitalWrite(CW_PIN, HIGH);
    digitalWrite(CCW_PIN, LOW);
    delay(pause_ms);
  }

  else if (!cw) { // CCW rotation
    digitalWrite(CCW_PIN, HIGH);
    digitalWrite(CW_PIN, LOW);
    delay(pause_ms);
  }
  analogWrite(PWM_PIN, MIN_DUTY);
}

// checkPos() compares encoder delta with pre-defined threshold and attempts
// to correct error
void checkPos() {
  if (encoderCounter > ENCODER_THRESHOLD) {
    Serial.println("ROTATING CCW TO RESET!!");
    motorController(false, 45, 125);
  }

  else if (encoderCounter < -1 * ENCODER_THRESHOLD) {
    Serial.println("ROTATING CW TO RESET!!");
    motorController(true, 45, 125);
  }
}

// translateToMorse() converts the current time -> morse code
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

  morseCode = morseHour + morseMinute; // NOTE: colon was removed for class demo
  Serial.println("FINAL MORSE CODE IS: " + morseCode);
}
