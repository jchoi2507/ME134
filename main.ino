/* 
TODO
- test checkPos() with intentionally over-rotated shafts
*/

/* 



*/


/* GPIO */
#define PWM_PIN 12
#define CW_PIN 27
#define CCW_PIN 14
#define ENCODER_1_PIN 33
#define ENCODER_2_PIN 32

/* PWM */
#define MAX_DUTY 225
#define MIN_DUTY 0

/* Encoder */
// 360 degree rotation corresponds to ~1100 delta encoder reading
// thus, 1/3 rotation corresponds to ~367 delta encoder reading **TODO: confirm the actuate PWM signals match up with these
// 

#define ENCODER_THRESHOLD 150 // arbitarily chosen
int encoder_1;
int encoder_2;
volatile int encoderCounter = 0;

/* Morse */
String morseMap[11] = {"-----", // 0
                    ".----", // 1
                    "..---", // 2
                    "...--", // 3
                    "....-", // 4
                    ".....", // 5
                    "-....", // 6
                    "--...", // 7
                    "---..", // 8
                    "----.", // 9
                    "---..."}; // colon
String morseCode = "";

/* Time */
String currHour = "";
String currMinute = "";

#include "MotorControl.h"
#include "TimeControl.h"

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  pinMode(CW_PIN, OUTPUT);
  pinMode(CCW_PIN, OUTPUT);
  pinMode(ENCODER_1_PIN, INPUT);
  pinMode(ENCODER_2_PIN, INPUT);

  Serial.begin(115200);
  attachInterrupt(ENCODER_1_PIN, readEncoder, CHANGE);
  setInitialTime();
}

void loop() {
  updateTime(); // updates time on clock per a desired time intervals
  printTime();
  translateToMorse(); // translate current time to morse code
  actuate();
}
