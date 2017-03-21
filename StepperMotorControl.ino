


#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>


LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Connect two steppers with 200 steps per revolution (1.8 degree)
Adafruit_StepperMotor *myStepper1 = AFMS.getStepper(200, 2);
Adafruit_StepperMotor *myStepper2 = AFMS.getStepper(200, 1);

Servo servo;

void forwardstep1() {
  myStepper1->onestep(FORWARD, MICROSTEP);
}
void backwardstep1() {
  myStepper1->onestep(BACKWARD, MICROSTEP);
}
// wrappers for the second motor!
void forwardstep2() {
  myStepper2->onestep(FORWARD, MICROSTEP);
}
void backwardstep2() {
  myStepper2->onestep(BACKWARD, MICROSTEP);
}

AccelStepper stepper1(forwardstep1, backwardstep1);
AccelStepper stepper2(forwardstep2, backwardstep2);

MultiStepper steppers;

//short steps1 [] = {64, 64, 65, 63, 62, 62, 61, 60, 60, 59, 58, 58, 57, 57, 56, 56, 55, 55, 54, 54, 53, 53, 52, 52, 51, 51, 51, 50, 50, 49, 49, 49, 49, 48, 48, 48, 47, 47, 47, 47, 47, 46, 46, 46, 46, 46, 46, 46, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 46, 46, 46, 46, 46, 46, 46, 47, 47, 47, 47, 47, 48, 48, 48, 49, 49, 49, 50, 50, 51, 51, 53, 54, 55, 56, 58, 59, 60, 61, 63, 64, 65, 66, 68, 69, 70, 72, 73, 75, 76, 77, 77, 77, 77, 77, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 76, 77, 77, 77, 77, 77, 78, 78, 78, 78, 79, 79, 79, 80, 80, 80, 81, 79, 78, 77, 76, 76, 75, 74, 73, 72, 71, 70, 70, 69, 68, 67, 67, 66, 65, 64};
//short steps2 [] = {333, 331, 330, 330, 329, 328, 327, 326, 324, 323, 322, 321, 320, 318, 317, 316, 314, 313, 312, 310, 309, 308, 306, 305, 303, 302, 300, 299, 297, 296, 294, 293, 291, 290, 288, 287, 285, 283, 282, 280, 279, 277, 275, 273, 272, 270, 268, 266, 265, 263, 261, 259, 257, 255, 253, 251, 250, 248, 246, 244, 241, 239, 237, 235, 233, 231, 229, 227, 224, 222, 220, 217, 215, 213, 210, 208, 205, 203, 200, 198, 195, 193, 193, 194, 194, 194, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 194, 197, 199, 201, 203, 205, 208, 210, 212, 214, 216, 218, 220, 222, 224, 226, 228, 229, 231, 233, 235, 237, 238, 240, 242, 243, 245, 247, 248, 250, 252, 253, 255, 256, 258, 259, 261, 262, 264, 265, 267, 270, 274, 277, 281, 284, 288, 291, 294, 298, 301, 304, 308, 311, 314, 317, 320, 323, 326, 333};
//short servoangle [] = {170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 180};

int steps1;
int steps2;
int servoangle;

char steps1_string[5];
char steps2_string[5];
char servoangle_char[2];

float angle1;
float angle2;
int angleInt1;
int angleInt2;


long positions[2] = {0, 0};

int i ;
boolean flag = 0;
boolean servoflag = 0;
#define HIGHSPEED 1000.0
#define LOWSPEED 200.0
#define SERVOUP 180
#define SERVODOWN 166
#define BUTTONPIN 23

const byte interruptPin1 = 3;
const byte interruptPin2 = 2;


void setup() {

  AFMS.begin();
  servo.attach(10);

  pinMode(BUTTONPIN, INPUT_PULLUP);

  pinMode(interruptPin1, INPUT_PULLUP);
  pinMode(interruptPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin1), Stop1, FALLING);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), Stop2, FALLING);

  servo.write(180);
  stepper1.setMaxSpeed(HIGHSPEED);
  stepper2.setMaxSpeed(HIGHSPEED);
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
  Serial.begin(9600);
  Serial2.begin(9600);
  Serial2.write("F");
  lcd.init();
  lcd.backlight();
}

void loop() {
  Serial.println("Press the Black Button to acquire the picture");
  lcd.clear();
  lcd.print("Press the black");
  lcd.setCursor(0, 1);
  lcd.print("button");

  while (Serial2.available() == 0) {}

  if (Serial2.read() == 'r') {


    Serial.println("Press 'y' OR the Red Button to start the system ");
    Serial.println("Press 's' to skip the links setup ");
    lcd.clear();
    lcd.print("Press the red");
    lcd.setCursor(0, 1);
    lcd.print("button");
    while (Serial.available() == 0) {
      if (digitalRead(BUTTONPIN) == 0) {
        flag = 1;
        break;
      }
    }

    char ch = Serial.read();
    if (Serial.read() == 10) {}

    if (ch != 's') {
      Serial.println("Going to start position");
      lcd.clear();
      lcd.print("Going to start");
      lcd.setCursor(0, 1);
      lcd.print("position");

      positions[0] = 18800;
      positions[1] = 0;
      steppers.moveTo(positions);
      steppers.runSpeedToPosition();

      Serial.println("First Link set up");
      lcd.clear();
      lcd.print("First Link ");
      lcd.setCursor(0, 1);
      lcd.print("set up");


      stepper1.setCurrentPosition(0);

      positions[0] = 0;
      positions[1] = 18800;

      steppers.moveTo(positions);
      steppers.runSpeedToPosition();

      Serial.println("Second Link set up ");
      lcd.clear();
      lcd.print("Second Link");
      lcd.setCursor(0, 1);
      lcd.print("set up");


      stepper2.setCurrentPosition(0);

      positions[0] = 0;
      positions[1] = 0;

      steppers.moveTo(positions);
      steppers.runSpeedToPosition();
    }

    if (ch == 'y' || ch == 's' || flag == 1  )
    {
      Serial.println("Waiting for serial comunication");
      flag = 0;
      Serial2.end();
      Serial2.begin(9600);
      Serial2.write("S");
      delay(100);
      while (Serial2.available() == 0) {
        Serial.println("Data not available");
        delay(3000);
      }
      Serial.println("Data received");
      lcd.clear();
      lcd.print("Data received");
      lcd.setCursor(0, 1);
      lcd.print("Printing Start");
      delay(2000);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("MOVING TO ANGLES:");
      lcd.setCursor(0, 1);
      lcd.print("T1=");
      lcd.setCursor(9, 1);
      lcd.print("T2=");

      while (Serial2.available() != 0) {

        for (i = 0; i < 4; i++) {
          steps1_string [i] = Serial2.read();
          delay(10);
        }

        for (i = 0; i < 4; i++) {
          steps2_string [i] = Serial2.read();
        }

        servoangle_char[0] = Serial2.read();

        steps1 = atoi(steps1_string) - 1000;
        steps2 = atoi(steps2_string) - 1000;
        servoangle = atoi(servoangle_char);

        Serial2.write("S");
        //for (i = 0; i < 4; i++) {
        //          Serial.println(steps1_string[i]);
        //          delay(10);
        //        }
        //
        //        for (i = 0; i < 4; i++) {
        //          Serial.println(steps2_string[i]);
        //        }


        angle1 = steps1 / 47 * 0.9;
        angle2 = steps2 / 47 * 0.9;

        angleInt1 = (int)angle1;
        angleInt2 = (int)angle2;

        Serial.print("Moving to Step1 = ");
        Serial.print(steps1);
        Serial.print("      Theta1 = ");
        Serial.print(angle1);
        Serial.println((char)223);
        Serial.print("Moving to Step2 = ");
        Serial.print(steps2);
        Serial.print("      Theta2 = ");
        Serial.print(angle2);
        Serial.println((char)223);
        Serial.print("Servo position = ");
        if (servoangle == 0) {
          Serial.println("DOWN");
        }
        else {
          Serial.println("UP");
        }


        Serial.println(angleInt1);
        Serial.println(angleInt2);
        lcd.setCursor(3, 1);
        lcd.print(angleInt1);
        lcd.print((char)223);
        lcd.print(" ");
        lcd.setCursor(12, 1);
        lcd.print(angleInt2);
        lcd.print((char)223);
        lcd.print(" ");

        if (flag == 0) {
          Serial.println("Going to initial point");
          stepper1.setMaxSpeed(HIGHSPEED);
          stepper2.setMaxSpeed(HIGHSPEED);
        }

        positions[0] = -steps1;
        positions[1] = -steps2;

        steppers.moveTo(positions);
        steppers.runSpeedToPosition();

        if (servoangle == 0 && servoflag == 0) {
          for (i = SERVOUP; i > SERVODOWN; i--) {
            servo.write(i);
            delay(500);
          }
          servoflag = 1;
        }
        if (servoangle == 1) {
          for (i = SERVODOWN; i < SERVOUP; i++) {
            servo.write(i);
            delay(500);
          }
          servoflag = 0;
        }
        delay(1000);
        if (flag == 0) {
          Serial.println("Printing start");
          stepper1.setMaxSpeed(LOWSPEED);
          stepper2.setMaxSpeed(LOWSPEED);
          flag = 1;
        }
      }
      
      Serial.println("Printing finished");
      lcd.clear();
      lcd.print("Printing");
      lcd.setCursor(0, 1);
      lcd.print("finished");
    }
  }
}

void Stop1() {
  Serial.println("STOP1 Pressed");
  stepper1.stop();
  stepper2.stop();
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  positions[0] = -150;
  positions[1] = 0;
  steppers.moveTo(positions);
}
void Stop2() {
  Serial.println("STOP2 Pressed");
  stepper1.stop();
  stepper2.stop();
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  positions[0] = 0;
  positions[1] = -300;
  steppers.moveTo(positions);
}



