#include <hardwareSerial.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include "ApplicationFunctionSet_xxx0.h"
#include "DeviceDriverSet_xxx0.h"

// When enabled the robot will output values but the motors will not activate
#define freeze_mode_enabled false
#define forward_speed 50
#define turn_speed 75
#define init_servo_pos 10
#define second_servo_pos 50

ApplicationFunctionSet Application_FunctionSet;

DeviceDriverSet_Motor AppMotor;
DeviceDriverSet_ULTRASONIC AppULTRASONIC_OBS_L;
DeviceDriverSet_ULTRASONIC AppULTRASONIC_OBS_M;
DeviceDriverSet_ULTRASONIC AppULTRASONIC_OBS_R;
DeviceDriverSet_ULTRASONIC AppULTRASONIC_BACK;
DeviceDriverSet_LINE_TRACKER AppLINE_TRACKER;
DeviceDriverSet_FLAME_IR AppLINE_FLAME_IR;
DeviceDriverSet_Servo AppServo;

static boolean function_xxx(long x, long s, long e)
{
  if (s <= x && x <= e)
    return true;
  else
    return false;
}

enum SmartRobotCarMotionControl
{
  Forward,       //(1)
  Backward,      //(2)
  Left,          //(3)
  Right,         //(4)
  LeftForward,   //(5)
  LeftBackward,  //(6)
  RightForward,  //(7)
  RightBackward, //(8)
  stop_it        //(9)
};

void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed);

void ApplicationFunctionSet::ApplicationFunctionSet_Init(void)
{
  Serial.begin(9600);
  AppMotor.DeviceDriverSet_Motor_Init();
  AppULTRASONIC_OBS_M.DeviceDriverSet_ULTRASONIC_Init_OBS_M();
  AppULTRASONIC_OBS_L.DeviceDriverSet_ULTRASONIC_Init_OBS_L();
  AppULTRASONIC_OBS_R.DeviceDriverSet_ULTRASONIC_Init_OBS_R();
  AppULTRASONIC_BACK.DeviceDriverSet_ULTRASONIC_Init_BACK();
  AppLINE_TRACKER.DeviceDriverSet_LINE_TRACKER_Init();
  AppLINE_FLAME_IR.DeviceDriverSet_FLAME_IR_Init();
  AppServo.DeviceDriverSet_Servo_Init(init_servo_pos);

  while (Serial.read() >= 0)
  {
    /*Clear serial port cache...*/
  }
  delay(1000);
}

static void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed)
{
  ApplicationFunctionSet Application_FunctionSet;
  uint8_t speed = is_speed;
  uint8_t Kp, UpperLimit;
  Kp = 2;
  UpperLimit = 180;

  if (freeze_mode_enabled)
  {
    direction = stop_it;
  }

  switch (direction) {
  case Forward:
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                            /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case Backward:
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                            /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case  Left:
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case Right:
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case LeftForward:
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed / 2, /*controlED*/ control_enable); //Motor control
    break;
  case LeftBackward:
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed / 2, /*controlED*/ control_enable); //Motor control
    break;
  case RightForward:
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed / 2,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case RightBackward:
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed / 2,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case stop_it:
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ 0,
                                           /*direction_B*/ direction_void, /*speed_B*/ 0, /*controlED*/ control_enable); //Motor control
    break;
  default:
    Serial.println("Invalid Direction");
    break;
  }
}

char lastPrintStatement[100]; 
static void printOnce(const char* tryingToPrint) {
  if (strcmp(tryingToPrint, lastPrintStatement) != 0) {
    Serial.println(tryingToPrint);
    strcpy(lastPrintStatement, tryingToPrint);
  }
}

int randomDirectionForLineTracking = random(0, 2);
long lastTimeLineWasDetected = millis();

void ApplicationFunctionSet::ApplicationFunctionSet_Line_Tracking(void)
{ 
  float get_LT_L = AppLINE_TRACKER.DeviceDriverSet_LINE_TRACKER_get_LT_L();
  float get_LT_M = AppLINE_TRACKER.DeviceDriverSet_LINE_TRACKER_get_LT_M();
  float get_LT_R = AppLINE_TRACKER.DeviceDriverSet_LINE_TRACKER_get_LT_R();
  
  if (function_xxx(get_LT_M, TrackingDetection_S, TrackingDetection_E)) {
    lastTimeLineWasDetected = millis(); // Start the timer
    ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, forward_speed);
    printOnce("LT: Mid");
  } else if (function_xxx(get_LT_R, TrackingDetection_S, TrackingDetection_E)) {
    lastTimeLineWasDetected = millis();
    ApplicationFunctionSet_SmartRobotCarMotionControl(Right, turn_speed);
    printOnce("LT: Right");
  }
  else if (function_xxx(get_LT_L, TrackingDetection_S, TrackingDetection_E)) {
    lastTimeLineWasDetected = millis();
    ApplicationFunctionSet_SmartRobotCarMotionControl(Left, turn_speed);
    printOnce("LT: Left");
  } else if (millis() - lastTimeLineWasDetected <= 1000) {
    if (randomDirectionForLineTracking == -1) {
      randomDirectionForLineTracking = random(0, 2);
    }
    
    if (randomDirectionForLineTracking == 0) {
      ApplicationFunctionSet_SmartRobotCarMotionControl(Right, turn_speed);
      printOnce("Looking for Line: Right");
    } else {
      ApplicationFunctionSet_SmartRobotCarMotionControl(Left, turn_speed);
      printOnce("Looking for Line: Left");
    }
  } else {
    printOnce("ELSE: Forward");
    ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, forward_speed);
    randomDirectionForLineTracking = -1;
    lastTimeLineWasDetected = 0; // Reset the timer when the line is not detected
  }
}

bool flame_visible_state = false;

void ApplicationFunctionSet::ApplicationFunctionSet_Main(void) {
  uint16_t get_Distance_OBS_L;
  uint16_t get_Distance_OBS_M;
  uint16_t get_Distance_OBS_R;
  uint16_t get_Distance_BACK;

  AppULTRASONIC_OBS_L.DeviceDriverSet_ULTRASONIC_Get_OBS_L(&get_Distance_OBS_L /*out*/);
  // Serial.print("ULTRASONIC_OBS_L=");
  // Serial.println(get_Distance_OBS_L);
  AppULTRASONIC_OBS_M.DeviceDriverSet_ULTRASONIC_Get_OBS_M(&get_Distance_OBS_M /*out*/);
  // Serial.print("ULTRASONIC_OBS_M=");
  // Serial.println(get_Distance_OBS_M);
  AppULTRASONIC_OBS_R.DeviceDriverSet_ULTRASONIC_Get_OBS_R(&get_Distance_OBS_R /*out*/);
  // Serial.print("ULTRASONIC_OBS_R=");
  // Serial.println(get_Distance_OBS_R);
  AppULTRASONIC_BACK.DeviceDriverSet_ULTRASONIC_Get_BACK(&get_Distance_BACK /*out*/);
  // Serial.print("ULTRASONIC_BACK=");
  // Serial.println(get_Distance_BACK);

  float get_FLAME_L = AppLINE_FLAME_IR.DeviceDriverSet_get_FLAME_IR_L();
  float get_FLAME_M = AppLINE_FLAME_IR.DeviceDriverSet_get_FLAME_IR_M();
  float get_FLAME_R = AppLINE_FLAME_IR.DeviceDriverSet_get_FLAME_IR_R();

  int randomTime = random(50, 501);
  int randomDirection = random(0, 2);

  if (get_FLAME_M == 0 || get_FLAME_R == 0 || get_FLAME_L == 0) {
    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    if (function_xxx(get_FLAME_M, 0, 0)) {
      printOnce("Not Plugged In FLAME M");}
    if (function_xxx(get_FLAME_R, 0, 0)) {
      printOnce("Not Plugged In FLAME R");}
    if (function_xxx(get_FLAME_L, 0, 0)) {
      printOnce("Not Plugged In FLAME L");}
  } else if (function_xxx(get_Distance_OBS_L, 0, 0) || function_xxx(get_Distance_OBS_M, 0, 0) || function_xxx(get_Distance_OBS_R, 0, 0) || function_xxx(get_Distance_BACK, 0, 0)) {
    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    if (function_xxx(get_Distance_OBS_L, 0, 0)){
      printOnce("Not Plugged In OBS L");}
    if (function_xxx(get_Distance_OBS_M, 0, 0)){
      printOnce("Not Plugged In OBS M");}
    if (function_xxx(get_Distance_OBS_R, 0, 0)){
      printOnce("Not Plugged In OBS R");}
    if (function_xxx(get_Distance_BACK, 0, 0)){
      printOnce("Not Plugged In BACK");}
  // } else if (function_xxx(get_Distance_BACK, 1, 25)) {
  //   ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  //   printOnce("Ultra: BACK");
  } else if (get_FLAME_L < 500) {
    ApplicationFunctionSet_SmartRobotCarMotionControl(Left, turn_speed);
    printOnce("IR FLAME L");
  } else if (get_FLAME_R < 500) {
    ApplicationFunctionSet_SmartRobotCarMotionControl(Right, turn_speed);
    printOnce("IR FLAME R");
  } else if (get_FLAME_M < 500) {
    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    printOnce("IR FLAME M");
    AppServo.DeviceDriverSet_Servo_control(second_servo_pos);
    flame_visible_state = true;
  } else if (function_xxx(get_Distance_OBS_M, 1, 10)) {
    printOnce("Ultra: OBS Mid");
    if (get_Distance_OBS_L < get_Distance_OBS_R) {
      ApplicationFunctionSet_SmartRobotCarMotionControl(Right, turn_speed);
      delay(50);
    } else if (get_Distance_OBS_L > get_Distance_OBS_R) {
      ApplicationFunctionSet_SmartRobotCarMotionControl(Left, turn_speed);
      delay(50);
    } else {
      if (randomDirection == 0) {
        ApplicationFunctionSet_SmartRobotCarMotionControl(Right, turn_speed);
        delay(50);
      } else {
        ApplicationFunctionSet_SmartRobotCarMotionControl(Left, turn_speed);
        delay(50);
      }
    }
    lastTimeLineWasDetected = lastTimeLineWasDetected - 5000;
  } else if (function_xxx(get_Distance_OBS_L, 1, 12)) {
    printOnce("Ultra: OBS Left");
    ApplicationFunctionSet_SmartRobotCarMotionControl(Right, turn_speed);
    delay(25);
    lastTimeLineWasDetected = lastTimeLineWasDetected - 5000;
  } else if (function_xxx(get_Distance_OBS_R, 1, 12)) {
    printOnce("Ultra: OBS Right");
    ApplicationFunctionSet_SmartRobotCarMotionControl(Left, turn_speed);
    delay(25);
    lastTimeLineWasDetected = lastTimeLineWasDetected - 5000;
  } else {
    ApplicationFunctionSet_Line_Tracking();
  }

  if (get_FLAME_M >= 500 && flame_visible_state) {
    flame_visible_state = false;
    AppServo.DeviceDriverSet_Servo_control(init_servo_pos);
  }
}

