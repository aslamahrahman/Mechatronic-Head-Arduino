#include "ESP32_Servo.h"
#include <HardwareSerial.h>

/*BEGIN MISCELLANEOUS---------------------------*/
#define LED_BUILTIN 2
/*END MISCELLANEOUS-----------------------------*/

/*BEGIN STEPPER STUFF--------------------------*/
const int stepperDir = 22;
const int stepperSig = 23;
const int xres = 640;
const int thresholdX = 100;
const int delayMicros = 10;
volatile int absX = 0;
const int maxAbsX = 100; 
const int stepperStep = 1;
const long freqMax = 600; //frequency of PWM
const long freqMin = 500; //frequency of PWM
const int stepperChannel = 2;
const int resolution = 8;
const int pwm = 255/2;
const long timeout = 500;
/*END STEPPER STUFF---------------------------*/

/*BEGIN SERVO STUFF---------------------------*/
const int servoSig = 19;
const int yres = 480;
const int yres_lower = 50;
const int yres_upper = 400;
const int thresholdY = 15;
const int servoStepAngle = 1;
volatile int absY = 0;
const int minAbsY = 130;
const int maxAbsY = 180;
const int midY = 160;
Servo servoHead;
int y = yres/2;
/*END SERVO STUFF-----------------------------*/

/*BEGIN MULTITHREADING STUFF------------------*/
TaskHandle_t moveStepperHandle;
TaskHandle_t moveServoHandle;
TaskHandle_t audioHandle;
/*END MULTITHREADING STUFF-------------------*/

/*BEGIN AUDIO STUFF---------------------------*/
volatile bool isAudio = LOW;
volatile long period = 0; // in milliseconds
const long danceDelay = 100; // in milliseconds
const int danceWaitTime1 = 100;
const int danceWaitTime2 = danceWaitTime1*2;
volatile int toggleDirection = -1;
const int danceAngle = 10;
volatile long lastMoveTime = millis();
HardwareSerial audioSerial(2);
const long audioTimeout = 5000;
/*END AUDIO STUFF-----------------------------*/

void setup() {
  /*BEGIN INITIALIZE--------------------------*/
  Serial.begin(115200);
  audioSerial.begin(115200, SERIAL_8N1, 16, 17);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(stepperDir, OUTPUT);
  pinMode(stepperSig, OUTPUT);
  pinMode(servoSig, OUTPUT);

  //initialize spi pins
  /*END INITIALIZE---------------------------*/

  /*BEGIN STEPPER STUFF--------------------------*/
  ledcSetup(stepperChannel, freqMin, resolution);
  ledcAttachPin(stepperSig, stepperChannel);
  /*END STEPPER STUFF----------------------------*/

  /*BEGIN SERVO STUFF-----------------------------*/
  servoHead.attach(servoSig, 500, 2500);
  absY = midY;
  servoHead.write(absY);
  xTaskCreatePinnedToCore(
   moveServo,     /* Task function. */
   "moveServo",   /* name of task. */
   10000,         /* Stack size of task */
   NULL,          /* parameter of the task */
   1,             /* priority of the task */
   &moveServoHandle,    /* Task handle to keep track of created task */
   1);            /* pin task to core 1 */
  /*END SERVO STUFF-------------------------------*/

  /*BEGIN AUDIO STUFF---------------------------*/
  xTaskCreatePinnedToCore(
   receiveAudio,   /* Task function. */
   "receiveAudio", /* name of task. */
   10000,         /* Stack size of task */
   NULL,          /* parameter of the task */
   1,             /* priority of the task */
   &audioHandle,  /* Task handle to keep track of created task */
   1);            /* pin task to core 0 */
  /*END AUDIO STUFF-----------------------------*/
}

void loop() {
  /*SERIAL*/
  static long timeNow = millis();
  if(Serial.available()) {
    digitalWrite(LED_BUILTIN, HIGH);
    String str = Serial.readStringUntil('\n');
    int x = str.substring(0,3).toInt();
    y = str.substring(3,6).toInt();
    moveStepper(x);
  }
  else {
    digitalWrite(LED_BUILTIN, LOW);
    if(millis() - timeNow > timeout) {
      stopStepper();
      timeNow = millis();
    }
  }
  /*SERIAL END*/
}

void moveStepper(int param) {
  int x = param;
  // Change speed of motion based on distance from target
  int freq = map(abs(x-xres/2), 0, xres/2, freqMin, freqMax);
  ledcSetup(stepperChannel, freq, resolution);
  
  if(x > (xres/2 + thresholdX)) {
    if(absX > -maxAbsX) {
      digitalWrite(stepperDir, HIGH);
      ledcWrite(stepperChannel, pwm);
      absX -= stepperStep;
    }
  }
  else if(x < (xres/2 - thresholdX)) {
    if(absX < maxAbsX) {
      digitalWrite(stepperDir, LOW);
      ledcWrite(stepperChannel, pwm);
      absX += stepperStep;
    }
  }
  else {
    stopStepper();
  }
  return;
}

void stopStepper(void) {
  ledcWrite(stepperChannel, 0);
  return;
}

void moveServo(void *param) {
  while(1) {
    int angle = map(y, yres_lower, yres_upper, minAbsY, maxAbsY);
    
    if(isAudio && ((millis() - lastMoveTime)) > (period - danceDelay)) {
      // Dance and track
      if(absY < (angle - thresholdY)) { 
        absY = absY + servoStepAngle + toggleDirection*danceAngle;
        servoHead.write(absY);
        toggleDirection *= -1;
        delay(danceWaitTime1);
        absY += toggleDirection*danceAngle;
        servoHead.write(absY);
        toggleDirection *= -1;
        delay(danceWaitTime2);
      }
      else if(absY > (angle + thresholdY)) { 
        absY = absY - servoStepAngle + toggleDirection*danceAngle;
        servoHead.write(absY);
        toggleDirection *= -1;
        delay(danceWaitTime1);
        absY += toggleDirection*danceAngle;
        servoHead.write(absY);
        toggleDirection *= -1;
        delay(danceWaitTime2);
      }
      else {
        absY += toggleDirection*danceAngle;
        servoHead.write(absY);
        toggleDirection *= -1;
        delay(danceWaitTime1);
        absY += toggleDirection*danceAngle;
        servoHead.write(absY);
        toggleDirection *= -1;
        delay(danceWaitTime2);
      }
      lastMoveTime = millis();
    }
    else {
      // Only track
      if(absY < (angle - thresholdY)) { 
        absY += servoStepAngle;
        servoHead.write(absY);
      }
      else if(absY > (angle + thresholdY)) {
        absY -= servoStepAngle;
        servoHead.write(absY);
      }
    }
//    vTaskDelay(10/portTICK_PERIOD_MS);
  }
}

void receiveAudio(void *param) {
  while(1) {
    static long audioTimeNow = millis();
    if(audioSerial.available()) {
      String str = audioSerial.readStringUntil('\n');
      int p1 = str.substring(0,1).toInt();
      if(p1 == 1) {
        isAudio = true;
        int bpm = str.substring(1,4).toInt();
        if(bpm != 0) {
          period = 60000/bpm;
        }
        else {
          isAudio = false; 
        }
      }
      else {
        isAudio = false;
      }
    }
    else {
      if(millis() - audioTimeNow > audioTimeout) {
        isAudio = false;
        audioTimeNow = millis();
      }
    }
    vTaskDelay(10/portTICK_PERIOD_MS);
  }
}
