//**********************Arduino code for making a go-baby-go car with a joystick, two motors, an acceleration control knob, and datalogger !!!! vroom! vroom! :) ***************************
//Go to https://www.arduino.cc/en/Reference/HomePage for more information on the commands used in this program.
//////////////////////////////////////CHANGE THESE NUMBERS TO CALIBRATE AND MODIFY THE CAR'S DRIVING///////////////////
#define CONTROL_RIGHT 642  //use to calibrate joystick (value from the X axis of the joystick when all the way to the left)
#define CONTROL_CENTER_X 512 //use to calibrate joystick (value from the X axis of the joystick when it is centered)
#define CONTROL_LEFT 379  //use to calibrate joystick (value from the X axis of the joystick when all the way to the right)
#define CONTROL_UP 649  //use to calibrate joystick (value from the Y axis of the joystick when all the way to the bottom)
#define CONTROL_CENTER_Y 522 //use to calibrate joystick (value from the Y axis of the joystick when it is centered)
#define CONTROL_DOWN 388  //use to calibrate joystick (value from the Y axis of the joystick when all the way to the top)
#define ACCELERATION_FORWARD  .25    //change # to change the amount of acceleration when going forward (fast acceleration could be uncomfortable) bigger number=quicker acceleration (slowly change the numbers)
#define DECELERATION_FORWARD  .25   //change # to change the amount of deceleration when going forward (fast acceleration could be uncomfortable) bigger number=quicker acceleration (slowly change the numbers)
#define ACCELERATION_BACKWARD .2    //change # to change the amount of acceleration when going backward (fast acceleration could be uncomfortable) bigger number=quicker acceleration (slowly change the numbers)
#define DECELERATION_BACKWARD .2   //change # to change the amount of deceleration when going backward (fast acceleration could be uncomfortable) bigger number=quicker acceleration (slowly change the numbers)
#define USE_ACCELERATION_KNOB false //true= you are using an optional knob to control the acceleration of the car, false=you aren't using a knob to change acceleration
#define USE_SPEED_KNOB true //true= you are using an optional knob to control the speed of the car, false=you aren't using a knob to change the speed
#define USE_SHUTOFF_SWITCH false //true= you are using an extra switch to be able to stop the truck from moving
#define USE_BALL_SHOOTER false //true= you are using an optional ball shooter for extra fun
#define ACCELERATION_KNOB_DECELERATION_RATIO 3 // how much faster to decelerate than accelerate when using the knob  / based on ACCELERATION_FORWARD
#define ACCELERATION_KNOB_BACKWARD_RATIO .5 // how much faster to accelerate and decelerate when going backward      / based on ACCELERATION_FORWARD
#define FORWARD_BACKWARD_SPEED_RATIO .9 //how much faster to go backward than forward for speed knob
#define TURN_SPEED_RATIO 1 //max turning speed as fraction of fastest_forward, for speed knob
#define FASTEST_FORWARD 1725  //change # to limit the forward speed 2000=max speed  1500=not at all
#define FASTEST_BACKWARD 1275 //change # to limit the backward speed 1000=max speed 1500=not at all
float TURN_SPEED = .5; //change # to limit the turning speed 0=none 1=full power (the truck might turn too quickly)
#define USE_DATALOGGER false // true= you are using an optional datalogger, false= you aren't using an optional datalogger
#define LOG_DATA_BY_TIME false  //true=use a timer to log new data
#define LOG_DATA_INTERVAL 1000 //milliseconds
#define SLOW_TURNING_WHEN_MOVING .6 // 0-1 what amount of TURN_SPEED to use when moving forward or backward (to be able to veer to a side more smoothly when moving forward)
///////////////////////////////////pins on the arduino//////////////////////////////////////////////////////////////////////
#define joyXPin 15//------------pin connected to the "X" axis of the joystick
#define joyYPin 17//------------pin connected to the "Y" axis of the joystick
#define leftMotorControllerPin  9//--pin connected to the signal pin of the left motor controller
#define rightMotorControllerPin 12//--pin connected to the signal pin of the right motor controller
#define accelerationKnobPin A0 //pin connected to the optional acceleration setting knob
#define speedKnobPin A6 //pin connected to the optional acceleration setting knob
#define shutoffSwitchPin 7 //pin connected to an optional switch which makes the truck stop moving
#define shutoffSwitchLight 5 //pin connected to an optional light which shows if the truck is not allowed to move
#define SDSelect 4  //a pin connected to the sd card
#define ballshooterServoPin 8
#define ballshooterSwitchPin 4
#define ballshooterLightPin 13
/////////////////////////////////motor values/////////////
#define startGoingLargeVal 1513//when the motorcontroller starts moving
#define startGoingSmallVal 1487//when the motorcontroller starts moving
#define accelerationKnobLowVal 5 //analogRead when accelKnob is in lowest position
#define accelerationKnobHighVal 1018 //analogRead when accelKnob is in highest position
#define speedKnobLowVal 1020 //analogRead when speedKnob is turned to slowest setting
#define speedKnobHighVal 3 //analogRead when speedKnob is turned to highest setting
#define ballshooterServoForwards 160
#define ballshooterServoBackwards 20
#define ballshooterTimeBetweenLaunching 5000
#define ballshooterTimeForServo 1300
///////////////////////////////////////////////
#include <Servo.h> //servo library
#if USE_DATALOGGER==true // run the following lines if a datalogger is connected
#include <SPI.h>    //      -
#include <SD.h>     //      -
#include <Wire.h>   //      -
#include "RTClib.h" //      -
DateTime now;       //      -
File dataFile;      //      -
File dataFile2;
RTC_DS3231 rtc;    //       -
#endif      //          //////////
/////////////////////////////////////variables
Servo rightMotorController;  //for motor controller (uses servo signals)
Servo leftMotorController;  //for motor controller (uses servo signals)
Servo ballshooterServo;  //for ball shooter
float leftMotorWriteVal = 1500.00; //variable to give to the motor controller
float rightMotorWriteVal = 1500.00; //variable to give to the motor controller
float rightMotorTryVal = 1500.00; //variable to end up giving to the motor controller (this is before acceleration is calculated)
float leftMotorTryVal = 1500.00; //variable to end up giving to the motor controller (this is before acceleration is calculated)
float joyXVal = 0000.000; //variable read from the "X" axis of the joystick
float joyYVal = 0000.000; //variable read from the "Y" axis of the joystick
unsigned int timezie = 0; //(cute) variable to help with the timing of printing values for debugging and calibrating
float speedVal = 1500.000; //forward speed
float turnVal = 0000.000; // how much to turn
float acceleration_forward = ACCELERATION_FORWARD; //value set by optional acceleration knob or by ACCELERATION_FORWARD
float deceleration_forward = DECELERATION_FORWARD; //value set by optional acceleration knob or by DECELERATION_FORWARD
float acceleration_backward = ACCELERATION_BACKWARD; //value set by optional acceleration knob or by ACCELERATION_BACKWARD
float deceleration_backward = DECELERATION_BACKWARD; //value set by optional acceleration knob or by DECELERATION_BACKWARD
float fastest_forward = FASTEST_FORWARD; //value set by optional speed knob or FASTEST_FORWARD
float fastest_backward = FASTEST_BACKWARD; //value set by optional speed knob or FASTEST_BACKWARD
int ballshooterServoVal = ballshooterServoBackwards;
unsigned int ballshooterTimeSinceLaunch = ballshooterTimeForServo;
boolean ballshooterLastSwitchVal = LOW;
#if USE_DATALOGGER == true////////////////////////////set up the following variables if a datalogger is connected
float howLongDriving = -00002.00;             //
unsigned long millisHowLongDriving = 0;       //
int maxPercentForward = 1000;                 //
int minPercentForward = 2000;                 //
int averagePercentForward = 0;                //
int maxPercentTurning = 0;                    //
int averagePercentTurn = 0;                   //
unsigned int forwardArrayCounter = 0;         //
unsigned int turnArrayCounter = 0;            //
unsigned int averageForwardArray[250];        //
unsigned int averageTurnArray[250];           //
unsigned int averageForwardCount;             //
unsigned int averageTurnCount;                //
unsigned int timeSinceDataLog = 0;            //
boolean justTurnedOn = true;                  //
#endif///////////////////////////////////////////////
//////////////////////////////////////////////////////////
void setup() {  //run once when the car is turned on to set up stuff (pin modes)
  Serial.begin(19200); //start serial port  (for finding problems and calibrating)
  delay(100);
  if (USE_ACCELERATION_KNOB == true) {
    pinMode(accelerationKnobPin, INPUT);
  }
  if (USE_SPEED_KNOB == true) {
    pinMode(speedKnobPin, INPUT);
  }
  if (USE_SHUTOFF_SWITCH == true) {
    pinMode(shutoffSwitchPin, INPUT_PULLUP);
    pinMode(shutoffSwitchLight, OUTPUT);
  }
  pinMode(joyXPin, INPUT); //set joystick pin as an input
  pinMode(joyYPin, INPUT); //set joystick pin as an input
  if (USE_BALL_SHOOTER == true) {
    ballshooterServo.attach(ballshooterServoPin);
    ballshooterServo.write(ballshooterServoBackwards);
    pinMode(ballshooterSwitchPin, INPUT_PULLUP);
    pinMode(ballshooterLightPin, OUTPUT);
    digitalWrite(ballshooterLightPin, LOW);
  }
  leftMotorController.attach(leftMotorControllerPin);//use pin for the motor controller
  rightMotorController.attach(rightMotorControllerPin);//use pin for the motor controller
  leftMotorController.writeMicroseconds(1500);//tell the motor controller to not move
  rightMotorController.writeMicroseconds(1500);//tell the motor controller to not move
#if USE_DATALOGGER == true
  while (!SD.begin(SDSelect)) {
    Serial.println("SD card not put in correctly, or broken.");
    delay(100);
  }
  Serial.println("SD card working.");
  if (! rtc.begin()) {
    Serial.println("Couldn't find clock. Program stopped. Do you need to disable the datalogger?");
    while (1);
  }
  else {
    Serial.println("Clock connected.");
  }
  Serial.print("Checking clock... ");
  if (rtc.lostPower()) {
    Serial.println("The clock wasn't running. Setting the clock's time.");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  else {
    Serial.println("clock running.");
  }
  Serial.println("Data-logger ready.");
#endif
  Serial.println();
  Serial.println("  wait 5 seconds for the ESC to calibrate itself");//send message over serial port
  Serial.println("current values:");//print preset values...
  Serial.println("---------");
  Serial.print("CONTROL_RIGHT: ");
  Serial.println(CONTROL_RIGHT);
  Serial.print("CONTROL_CENTER_X: ");
  Serial.println(CONTROL_CENTER_X);
  Serial.print("CONTROL_LEFT: ");
  Serial.println(CONTROL_LEFT);
  Serial.print("CONTROL_UP: ");
  Serial.println(CONTROL_UP);
  Serial.print("CONTROL_CENTER_Y: ");
  Serial.println(CONTROL_CENTER_Y);
  Serial.print("CONTROL_DOWN: ");
  Serial.println(CONTROL_DOWN);
  Serial.print("ACCELERATION_FORWARD: ");
  Serial.println(ACCELERATION_FORWARD);
  Serial.print("DECELERATION_FORWARD: ");
  Serial.println(DECELERATION_FORWARD);
  Serial.print("ACCELERATION_BACKWARD: ");
  Serial.println(ACCELERATION_BACKWARD);
  Serial.print("DECELERATION_BACKWARD: ");
  Serial.println(DECELERATION_BACKWARD);
  Serial.print("USE_ACCELERATION_KNOB: ");
  Serial.println(USE_ACCELERATION_KNOB);
  Serial.print("ACCELERATION_KNOB_DECELERATION_RATIO: ");
  Serial.println(ACCELERATION_KNOB_DECELERATION_RATIO);
  Serial.print("ACCELERATION_KNOB_BACKWARD_RATIO: ");
  Serial.println(ACCELERATION_KNOB_BACKWARD_RATIO);
  Serial.print("FORWARD_BACKWARD_SPEED_RATIO: ");
  Serial.println(FORWARD_BACKWARD_SPEED_RATIO);
  Serial.print("TURN_SPEED_RATIO: ");
  Serial.println(TURN_SPEED_RATIO);
  Serial.print("FASTEST_FORWARD: ");
  Serial.println(FASTEST_FORWARD);
  Serial.print("FASTEST_BACKWARD: ");
  Serial.println(FASTEST_BACKWARD);
  Serial.print("TURN_SPEED: ");
  Serial.println(TURN_SPEED);
  Serial.print("USE_DATALOGGER: ");
  Serial.println(USE_DATALOGGER);
  Serial.print("joyXPin: ");
  Serial.println(joyXPin);
  Serial.print("joyYPin: ");
  Serial.println(joyYPin);
  Serial.print("leftMotorControllerPin: ");
  Serial.println(leftMotorControllerPin);
  Serial.print("rightMotorControllerPin: ");
  Serial.println(rightMotorControllerPin);
  Serial.print("accelerationKnobPin: ");
  Serial.println(accelerationKnobPin);
  Serial.print("speedKnobPin: ");
  Serial.println(speedKnobPin);
  Serial.print("shutoffSwitchPin: ");
  Serial.println(shutoffSwitchPin);
  Serial.print("SDSelect: ");
  Serial.println(SDSelect);
  Serial.print("startGoingLargeVal: ");
  Serial.println(startGoingLargeVal);
  Serial.print("startGoingSmallVal: ");
  Serial.println(startGoingSmallVal);
  Serial.print("slow turn when moving: ");
  Serial.println(SLOW_TURNING_WHEN_MOVING);
  Serial.print("accelerationKnobLowVal: ");
  Serial.println(accelerationKnobLowVal);
  Serial.print("accelerationKnobHighVal: ");
  Serial.println(accelerationKnobHighVal);
  Serial.print("speedKnobLowVal: ");
  Serial.println(speedKnobLowVal);
  Serial.print("speedKnobHighVal: ");
  Serial.println(speedKnobHighVal);
  Serial.println();
  delay(3000);//wait for the motor controller to calibrate itself
  leftMotorController.writeMicroseconds(1540);//pulse motors slightly
  rightMotorController.writeMicroseconds(1540);//pulse motors slightly
  delay(250);//pause while motors are pulsing
  leftMotorController.writeMicroseconds(1500);//stop the motors
  rightMotorController.writeMicroseconds(1500);//stop the motors
  Serial.println("READY TO DRIVE!!!!!!!!!!!!!!!!!!!! (go_baby_go_car_program_for_double_motors)");
  Serial.println();
}
/////////////////////////////////////////////////////////
void loop() { //runs over and over (main stuff)
  ///////////////////////////////////////////////////////////input code
  joyXVal = analogRead(joyXPin); //read joystick input and save it to a variable
  joyYVal = analogRead(joyYPin); //read joystick input and save it to a variable
  ////////////////////////////////////ball shooter code
#if USE_BALL_SHOOTER == true
  if (ballshooterTimeSinceLaunch <= ballshooterTimeBetweenLaunching) {
    ballshooterTimeSinceLaunch++;
    digitalWrite(ballshooterLightPin, LOW);
    if (ballshooterTimeSinceLaunch < ballshooterTimeForServo) {
      ballshooterServoVal = ballshooterServoForwards;
    }
    else {
      ballshooterServoVal = ballshooterServoBackwards;
    }
  }
  else {
    if (digitalRead(ballshooterSwitchPin) == HIGH) {
      ballshooterLastSwitchVal = HIGH;
      digitalWrite(ballshooterLightPin, HIGH);
    }
    if (digitalRead(ballshooterSwitchPin) == LOW && ballshooterLastSwitchVal == HIGH) {
      ballshooterLastSwitchVal = LOW;
      ballshooterTimeSinceLaunch = 0;
    }
    ballshooterServoVal = ballshooterServoBackwards;
  }
  ballshooterServo.write(ballshooterServoVal);
#endif
  /////////////////////////////////////acceleration knob code
  if (USE_ACCELERATION_KNOB == true) { //do the following if the acceleration knob is being used
    acceleration_forward = constrain(float(map(float(analogRead(accelerationKnobPin)), accelerationKnobLowVal, accelerationKnobHighVal, 0.00, 1000.00)) / 1000.000, 0, ACCELERATION_FORWARD); //set acceleration_forward from the acceleration knob
    acceleration_backward = ACCELERATION_KNOB_BACKWARD_RATIO * acceleration_forward; //set acceleration_backward from acceleration_forward using preset ratio
    deceleration_forward = ACCELERATION_KNOB_BACKWARD_RATIO * acceleration_forward; //set deceleration_forward from acceleration_forward using preset ratio
    deceleration_backward = ACCELERATION_KNOB_BACKWARD_RATIO * acceleration_backward; //set deceleration_backward from acceleration_backward using preset ratio
  }
  if (USE_SPEED_KNOB == true) { //do the following if the speed knob is being used
    fastest_forward = constrain(map(analogRead(speedKnobPin), speedKnobLowVal, speedKnobHighVal, 1500, 2000), 1505, 2000);
    fastest_backward = 1500 + (FORWARD_BACKWARD_SPEED_RATIO * (-fastest_forward + 1500));
    TURN_SPEED = (float)TURN_SPEED_RATIO * (fastest_forward - 1500.0) / 500.0;
  }
  ////////////////////////////////////////////joystick code//////////////////
  if (CONTROL_LEFT < CONTROL_RIGHT) {
    if (joyXVal < CONTROL_CENTER_X) {
      turnVal = map(joyXVal, CONTROL_LEFT, CONTROL_CENTER_X, TURN_SPEED * -constrain(map(analogRead(speedKnobPin), 3, 1020, 70, 500), 100, 500), 0);
    }
    if (joyXVal > CONTROL_CENTER_X) {
      turnVal = map(joyXVal, CONTROL_RIGHT, CONTROL_CENTER_X, TURN_SPEED * constrain(map(analogRead(speedKnobPin), 3, 1020, 70, 500), 100, 500), 0);
    }
  }
  if (CONTROL_LEFT > CONTROL_RIGHT) {
    if (joyXVal > CONTROL_CENTER_X) {
      turnVal = map(joyXVal, CONTROL_LEFT, CONTROL_CENTER_X, TURN_SPEED * -constrain(map(analogRead(speedKnobPin), 3, 1020, 70, 500), 100, 500), 0);
    }
    if (joyXVal < CONTROL_CENTER_X) {
      turnVal = map(joyXVal, CONTROL_RIGHT, CONTROL_CENTER_X, TURN_SPEED * constrain(map(analogRead(speedKnobPin), 3, 1020, 70, 500), 100, 500), 0);
    }
  }
  if (CONTROL_UP < CONTROL_DOWN) {
    if (joyYVal < CONTROL_CENTER_Y) {
      speedVal = map(joyYVal, CONTROL_UP, CONTROL_CENTER_Y, fastest_forward, 1500);
    }
    if (joyYVal > CONTROL_CENTER_Y) {
      speedVal = map(joyYVal, CONTROL_DOWN, CONTROL_CENTER_Y, fastest_backward, 1500);
    }
  }
  if (CONTROL_UP > CONTROL_DOWN) {
    if (joyYVal > CONTROL_CENTER_Y) {
      speedVal = map(joyYVal, CONTROL_UP, CONTROL_CENTER_Y, fastest_forward, 1500);
    }
    if (joyYVal < CONTROL_CENTER_Y) {
      speedVal = map(joyYVal, CONTROL_DOWN, CONTROL_CENTER_Y, fastest_backward, 1500);
    }
  }
  speedVal = constrain(speedVal, min(fastest_backward, fastest_forward), max(fastest_backward, fastest_forward));
  ///////slow turning proportionally to speed of car
  if (speedVal >= 1500) {
    turnVal = map(turnVal, TURN_SPEED * -500, TURN_SPEED * 500, map(abs(speedVal - 1500), 0, fastest_forward - 1500, TURN_SPEED * -500, TURN_SPEED * -SLOW_TURNING_WHEN_MOVING * 500), map(abs(speedVal - 1500), 0, fastest_forward - 1500, TURN_SPEED * 500, TURN_SPEED * SLOW_TURNING_WHEN_MOVING * 500));
  }
  else {
    turnVal = map(turnVal, TURN_SPEED * -500, TURN_SPEED * 500, map(abs(1500 - speedVal), 0, 1500 - fastest_backward, TURN_SPEED * -500, TURN_SPEED * -SLOW_TURNING_WHEN_MOVING * 500), map(abs(1500 - speedVal), 0, 1500 - fastest_backward, TURN_SPEED * 500, TURN_SPEED * SLOW_TURNING_WHEN_MOVING * 500));
  }
  //////
  leftMotorTryVal = speedVal + turnVal; //use the forward-backward speed value and the turn value to find the left motor value
  rightMotorTryVal = speedVal - turnVal; //use the forward-backward speed value and the turn value to find the right motor value
  ////////////////////////////////////code to make the car accelerate slowly
  ////for left motor
  if (leftMotorTryVal < startGoingLargeVal && leftMotorTryVal > startGoingSmallVal) { //if the motor wouldn't be moving enough anyway...
    leftMotorTryVal = 1500; //...don't turn on the motor.
  }
  if (leftMotorTryVal < 1500) { //if going backwards...
    if (leftMotorTryVal <= leftMotorWriteVal - acceleration_backward) { //...if trying to go faster backwards...
      leftMotorWriteVal = leftMotorWriteVal - acceleration_backward; //...accelerate by acceleration_backward
    }
    if (leftMotorTryVal > leftMotorWriteVal + deceleration_backward) { //if trying to go slower backwards...
      leftMotorWriteVal = leftMotorWriteVal + deceleration_backward; //...decelerate by deceleration_backward
    }
  }
  if (leftMotorTryVal >= 1500) { //if going forwards...
    if (leftMotorTryVal >= leftMotorWriteVal + acceleration_forward) { //...if trying to go faster forwards...
      leftMotorWriteVal = leftMotorWriteVal + acceleration_forward; //...accelerate by acceleration_forward
    }
    if (leftMotorTryVal < leftMotorWriteVal - deceleration_forward) { // if trying to go slower forwards...
      leftMotorWriteVal = leftMotorWriteVal - deceleration_forward; //...decelerate by deceleration_forward
    }
  }
  ////////for right motor
  if (rightMotorTryVal < startGoingLargeVal && rightMotorTryVal > startGoingSmallVal) { //if the motor wouldn't be moving enough anyway...
    rightMotorTryVal = 1500; //...don't turn on the motor.
  }
  if (rightMotorTryVal < 1500) { //if going backwards...
    if (rightMotorTryVal <= rightMotorWriteVal - acceleration_backward) { //...if trying to go faster backwards...
      rightMotorWriteVal = rightMotorWriteVal - acceleration_backward; //...accelerate by acceleration_backward
    }
    if (rightMotorTryVal > rightMotorWriteVal + deceleration_backward) { //if trying to go slower backwards...
      rightMotorWriteVal = rightMotorWriteVal + deceleration_backward; //...decelerate by deceleration_backward
    }
  }
  if (rightMotorTryVal >= 1500) { //if going forwards...
    if (rightMotorTryVal >= rightMotorWriteVal + acceleration_forward) { //...if trying to go faster forwards...
      rightMotorWriteVal = rightMotorWriteVal + acceleration_forward; //...accelerate by acceleration_forward
    }
    if (rightMotorTryVal < rightMotorWriteVal - deceleration_forward) { // if trying to go slower forwards...
      rightMotorWriteVal = rightMotorWriteVal - deceleration_forward; //...decelerate by deceleration_forward
    }
  }
  if (USE_SHUTOFF_SWITCH == true) {
    if (digitalRead(shutoffSwitchPin) == LOW) {
      digitalWrite(shutoffSwitchLight, HIGH);
      rightMotorWriteVal = 1500;
      leftMotorWriteVal = 1500;
    }
    else {
      digitalWrite(shutoffSwitchLight, LOW);
    }
  }
  //  //////////////////sd code/////////////////////////////////////////////////////////////////
#if USE_DATALOGGER == true
  if (abs(speedVal - 1500) > 20 || abs(turnVal) > 13) {
    if (millis() - millisHowLongDriving >= 50) {
      millisHowLongDriving = millis();
      howLongDriving += .05;
      if (forwardArrayCounter >= 249) {
        forwardArrayCounter = 0;
      }
      if (turnArrayCounter >= 249) {
        turnArrayCounter = 0;
      }
      averageTurnArray[turnArrayCounter] = abs(turnVal);
      averageForwardArray[forwardArrayCounter] = speedVal;
      turnArrayCounter++;
      forwardArrayCounter++;
      if (speedVal > maxPercentForward) {
        maxPercentForward = speedVal;
      }
      if (speedVal < minPercentForward) {
        minPercentForward = speedVal;
      }
      if (abs(turnVal) > maxPercentTurning) {
        maxPercentTurning = abs(turnVal);
      }

    }
  }
  else {
    if (howLongDriving > .20) {
      //calculate averages/////
      for (int i = 0; i <= 249; i++) {
        if (averageForwardArray[i] != 0) {
          averageForwardCount++;
          averagePercentForward += averageForwardArray[i];
        }
        if (averageTurnArray[i] != 0) {
          averageTurnCount++;
          averagePercentTurn += averageTurnArray[i];
        }
        averageForwardArray[i] = 0;
        averageTurnArray[i] = 0;
      }
      averagePercentForward = averagePercentForward / averageForwardCount;
      averagePercentTurn = averagePercentTurn / averageTurnCount;
      //save data to sd card//////////
      dataFile = SD.open("datalog.txt", FILE_WRITE);
      if (dataFile) {
        Serial.println("writing data to SD card: ");
        now = rtc.now();
        dataFile.print(now.month());
        dataFile.print('/');
        dataFile.print(now.day());
        dataFile.print('/');
        dataFile.print(now.year());
        dataFile.print(",");
        dataFile.print(now.hour(), DEC);
        dataFile.print(':');
        dataFile.print(now.minute(), DEC);
        dataFile.print(':');
        dataFile.print(now.second(), DEC);
        dataFile.print(",");
        dataFile.print(maxPercentTurning);
        dataFile.print(",");
        dataFile.print(maxPercentForward);
        dataFile.print(",");
        dataFile.print(minPercentForward);
        dataFile.print(",");
        dataFile.print(averagePercentForward);
        dataFile.print(",");
        dataFile.print(averagePercentTurn);
        dataFile.print(",");
        dataFile.print(howLongDriving);
        dataFile.print(",");
        dataFile.println(justTurnedOn);
        dataFile.close();
      }
      else {
        Serial.println("Error opening datalog.txt!"); // if the file isn't open, print an error
      }

      Serial.print(now.month());
      Serial.print('/');
      Serial.print(now.day());
      Serial.print('/');
      Serial.print(now.year());
      Serial.print(",");
      Serial.print(now.hour(), DEC);
      Serial.print(':');
      Serial.print(now.minute(), DEC);
      Serial.print(':');
      Serial.print(now.second(), DEC);
      Serial.print(", ");
      Serial.print(maxPercentTurning);
      Serial.print(", ");
      Serial.print(maxPercentForward);
      Serial.print(", ");
      Serial.print(minPercentForward);
      Serial.print(", ");
      Serial.print(averagePercentForward, 10);
      Serial.print(", ");
      Serial.print(averagePercentTurn, 10);
      Serial.print(", ");
      Serial.print(howLongDriving);
      Serial.print(", ");
      Serial.println(justTurnedOn);
      Serial.println();
      justTurnedOn = false;
    }

    //clear variables//////
    howLongDriving = 00000.00;
    millisHowLongDriving = 0;
    maxPercentForward = 1500;
    minPercentForward = 1500;
    averagePercentForward = 1500;
    maxPercentTurning = 0;
    averagePercentTurn = 0;
    forwardArrayCounter = 0;
    turnArrayCounter = 0;
    averageForwardCount = 1;
    averageTurnCount = 1;
  }
  if (LOG_DATA_BY_TIME) {
    if (millis() - timeSinceDataLog >= LOG_DATA_INTERVAL) {
      timeSinceDataLog = millis();
      dataFile2 = SD.open("datime.txt", FILE_WRITE);
      if (dataFile2) {
        Serial.println("writing data to SD card: ");
        now = rtc.now();
        dataFile2.print(now.month());
        dataFile2.print('/');
        dataFile2.print(now.day());
        dataFile2.print('/');
        dataFile2.print(now.year());
        dataFile2.print(",");
        dataFile2.print(now.hour(), DEC);
        dataFile2.print(':');
        dataFile2.print(now.minute(), DEC);
        dataFile2.print(':');
        dataFile2.print(now.second(), DEC);
        dataFile2.print(",");
        dataFile2.print(speedVal);
        dataFile2.print(",");
        dataFile2.print(turnVal);
        dataFile2.print(",");
        dataFile2.print(leftMotorWriteVal);
        dataFile2.print(",");
        dataFile2.print(rightMotorWriteVal);
        dataFile2.print(",");
        dataFile2.print(howLongDriving);
        dataFile2.print(",");
        dataFile2.println(justTurnedOn);
        dataFile2.close();
      }
      else {
        Serial.println("Error opening datime.txt!"); // if the file isn't open, print an error
      }

      Serial.print(now.month());
      Serial.print('/');
      Serial.print(now.day());
      Serial.print('/');
      Serial.print(now.year());
      Serial.print(",");
      Serial.print(now.hour(), DEC);
      Serial.print(':');
      Serial.print(now.minute(), DEC);
      Serial.print(':');
      Serial.print(now.second(), DEC);
      Serial.print(", ");
      Serial.print(turnVal);
      Serial.print(",");
      Serial.print(leftMotorWriteVal);
      Serial.print(",");
      Serial.print(rightMotorWriteVal);
      Serial.print(",");
      Serial.print(howLongDriving);
      Serial.print(",");
      Serial.println(justTurnedOn);
    }
  }
#endif
  ///////////////////////////////////////////////send values to servo and motor controller////////////////////////
  leftMotorWriteVal = constrain(leftMotorWriteVal, 1000, 2000); //make sure the value is in the right range
  rightMotorWriteVal = constrain(rightMotorWriteVal, 1000, 2000); //make sure the value is in the right range
  rightMotorController.writeMicroseconds(int(rightMotorWriteVal)); //send a servo signal to the motor controller for the Motor value (after acceleration)
  leftMotorController.writeMicroseconds(int(leftMotorWriteVal)); //send a servo signal to the motor controller for the Motor value (after acceleration)
  /////////////////////////////////////print values to help figure out problems
  if (timezie > 500) { //around twice a second
    timezie = 0;
    Serial.print("X joystick value: ");
    Serial.print(joyXVal); Serial.print(","); //joystickX value
    Serial.print("Y joystick value: ");
    Serial.print(joyYVal); Serial.print(","); //joystickY value
    Serial.println();
    Serial.print("speedVal: ");
    Serial.println(speedVal);
    Serial.print("turnVal: ");
    Serial.println(turnVal);
    Serial.println();
    Serial.println("motor values: ");
    Serial.print(leftMotorTryVal); Serial.print(","); //value before slow acceleration is calculated in
    Serial.print(rightMotorTryVal); Serial.print(","); //value sent to the motor controller
    Serial.println();
    Serial.print(leftMotorWriteVal); Serial.print(","); //value before slow acceleration is calculated in
    Serial.print(rightMotorWriteVal); Serial.print(","); //value sent to the motor controller
    Serial.println();
    if (USE_ACCELERATION_KNOB) {
      Serial.println();
      Serial.print("analogRead accelKnobPin: ");
      Serial.println(analogRead(accelerationKnobPin));
      Serial.print("acceleration_forward: ");
      Serial.println(acceleration_forward);
    }
    if (USE_SPEED_KNOB) {
      Serial.println();
      Serial.print("analogRead speedKnobPin: ");
      Serial.println(analogRead(speedKnobPin));
      Serial.print("fastest_forward: ");
      Serial.println(fastest_forward);
      Serial.print("TURN_SPEED: ");
      Serial.println(TURN_SPEED);
    }
    Serial.println();
    Serial.println();
  }
  timezie++;
}
