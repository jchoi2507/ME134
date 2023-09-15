/* 
  TimeControl.ino
  ME134 HW #1
  Jacob Choi 9/14/2023
*/

// setInitialTime() initializes the current time based on user serial input
void setInitialTime() {
  Serial.println("Current hour: ");
  while (Serial.available() == 0) {continue;}
  currHour = Serial.readStringUntil('\n');

  Serial.println("Current minute: ");
  while (Serial.available() == 0) {continue;}
  currMinute = Serial.readStringUntil('\n');

  currHour.trim(); // removing trailing NL CR chars
  currMinute.trim();

  initialTime = millis();
}

// updateTime() checks if desired time interval has passed, then updates the clock
void updateTime() {
  checkTime();
  updateMinute();
}

// checkTime() is a blocking function that completes when desired time interval has passed
void checkTime() {
  finalTime = millis();

  while (finalTime - initialTime < TIME_INTERVAL_MS) { // blocking
    finalTime = millis();
  }
  initialTime = finalTime;
}

// updateMinute() updates the minute value after the desired time interval has passed (usually 1 minute)
void updateMinute() {
  int currMinute_int = currMinute.toInt();
  currMinute_int++;

  if (currMinute_int >= 60) { // check if past 60
    currMinute = "00"; // reset minutes back to 00
    updateHour(); // update hours
  }

  else {
    currMinute = String(currMinute_int); 
  }
}

// updateHour() updates the hour value
void updateHour() {
  int currHour_int = currHour.toInt();
  currHour_int++;

  if (currHour_int > 12) { // check if past 12
    currHour = "1"; // reset hours back to 1
  }

  else {
    currHour = String(currHour_int);
  }
}

// printTime() is a debugging function that displays current time in serial monitor
void printTime() {
  Serial.println("CURRENT TIME: " + currHour + ":" + currMinute);
}
