/* 
  main.ino
  ME134 HW #1
  Jacob Choi 9/14/2023
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
#define ENCODER_THRESHOLD 150 // max. allowable encoder delta
int encoder_1;
int encoder_2;
volatile int encoderCounter = 0; // encoder value

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
#define TIME_INTERVAL_MS 20000 // desired time (60000 for normal operation,
                               // lower to accelerate)
String currHour = "";
String currMinute = "";
unsigned long initialTime;
unsigned long finalTime;

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  pinMode(CW_PIN, OUTPUT);
  pinMode(CCW_PIN, OUTPUT);
  pinMode(ENCODER_1_PIN, INPUT);
  pinMode(ENCODER_2_PIN, INPUT);

  Serial.begin(115200);
  attachInterrupt(ENCODER_1_PIN, readEncoder, CHANGE); // calls readEncoder() when hall sensor changes HIGH/LOW
  setInitialTime(); // initializes current time
}

void loop() {
  printTime();
  updateTime(); // update time based on desired time interval
  printTime();
  translateToMorse(); // translate current time to morse code
  actuate();
}
