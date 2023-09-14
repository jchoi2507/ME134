#include "TimeControl.h"

void setInitialTime() {
  Serial.println("Current time input (FORMAT: HR:MIN)");
  while (Serial.available() == 0) {continue;}

  currHour = Serial.readStringUntil(':'); // colon is terminated from serial buffer
  currMinute = Serial.readString();
  currHour.trim(); // remove trailing NL/CR chars
  currMinute.trim();
}

void updateTime() {
  checkTime();
  updateMinute();
}

// blocking function that completes when desired time has passed
void checkTime() {
  unsigned long initialTime = millis();
  unsigned long finalTime;

  while (finalTime - initialTime < 30000) { // desired time: every minute
    finalTime = millis();
    delay(500); // check for desired time every 0.5 seconds
  }
}

void updateMinute() {
  int currMinute_int = currMinute.toInt();
  currMinute_int++; // update new minute

  if (currMinute_int >= 60) { // check if past 60
    currMinute = "00"; // reset minutes back to 00
    updateHour(); // update hours
  }

  else {
    currMinute = String(currMinute_int); 
  }
}

void updateHour() {
  int currHour_int = currHour.toInt();
  currHour_int++; // update new hour

  if (currHour_int > 12) { // check if past 12
    currHour = "1"; // reset hours
  }

  else {
    currHour = String(currHour_int);
  }
}

// test function
void printTime() {
  //Serial.println("currHour: " + currHour);
  //Serial.println("currMinute: " + currMinute);
  Serial.println("CURRENT TIME: " + currHour + ":" + currMinute);

  //Serial.println("morseHour: " + morseHour);
  //Serial.println("morseMinute: " + morseMinute);
  //Serial.println("FULL MORSE CODE: " + morseCode);
}
