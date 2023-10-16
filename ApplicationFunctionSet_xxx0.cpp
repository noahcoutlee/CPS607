/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-28 14:10:45
 * @LastEditors: Changhua
 * @Description: SmartRobot robot tank
 * @FilePath: 
 */
#include <hardwareSerial.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include "ApplicationFunctionSet_xxx0.h"
#include "DeviceDriverSet_xxx0.h"

#define _is_print 1
#define _Test_print 0

ApplicationFunctionSet Application_FunctionSet;

/*硬件设备成员对象序列*/
DeviceDriverSet_Motor AppMotor;
DeviceDriverSet_ULTRASONIC AppULTRASONIC_L;
DeviceDriverSet_ULTRASONIC AppULTRASONIC_R;
DeviceDriverSet_ULTRASONIC AppULTRASONIC_OBS_L;
DeviceDriverSet_ULTRASONIC AppULTRASONIC_OBS_M;
DeviceDriverSet_ULTRASONIC AppULTRASONIC_OBS_R;

/*f(x) int */
static boolean
function_xxx(long x, long s, long e) //f(x)
{
  if (s <= x && x <= e)
    return true;
  else
    return false;
}

/*运动方向控制序列*/
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
};               //direction方向:（1）、（2）、 （3）、（4）、（5）、（6）

/*模式控制序列*/
enum SmartRobotCarFunctionalModel
{
  Standby_mode,           /*空闲模式*/
  TraceBased_mode,        /*循迹模式*/
  ObstacleAvoidance_mode, /*避障模式*/
  Follow_mode,            /*跟随模式*/
  Rocker_mode,            /*摇杆模式*/
};

/*控制管理成员*/
struct Application_xxx
{
  SmartRobotCarMotionControl Motion_Control;
  SmartRobotCarFunctionalModel Functional_Mode;
  unsigned long CMD_CarControl_Millis;
  unsigned long CMD_LightingControl_Millis;
};
Application_xxx Application_SmartRobotCarxxx0;;

void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed);

void ApplicationFunctionSet::ApplicationFunctionSet_Init(void)
{
  bool res_error = true;
  Serial.begin(9600);
  AppMotor.DeviceDriverSet_Motor_Init();
  AppULTRASONIC_L.DeviceDriverSet_ULTRASONIC_Init_L();
  AppULTRASONIC_R.DeviceDriverSet_ULTRASONIC_Init_R();
  AppULTRASONIC_OBS_M.DeviceDriverSet_ULTRASONIC_Init_OBS_M();
  AppULTRASONIC_OBS_L.DeviceDriverSet_ULTRASONIC_Init_OBS_L();
  AppULTRASONIC_OBS_R.DeviceDriverSet_ULTRASONIC_Init_OBS_R();

  while (Serial.read() >= 0)
  {
    /*Clear serial port cache...*/
  }
  delay(2000);
  Application_SmartRobotCarxxx0.Functional_Mode = ObstacleAvoidance_mode;
}


/*
  运动控制:
  1# direction方向:前行（1）、后退（2）、 左前（3）、右前（4）、后左（5）、后右（6）
  2# speed速度(0--255)
*/
static void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed)
{
  ApplicationFunctionSet Application_FunctionSet;
  uint8_t Kp, UpperLimit;
  uint8_t speed = is_speed;


    Kp = 2;
    UpperLimit = 180;

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

static void delay_xxx(uint16_t _ms)
{
  for (unsigned long i = 0; i < _ms; i++)
  {
    delay(1);
  }
}

/*
  Obstacle avoidance function
*/
void ApplicationFunctionSet::ApplicationFunctionSet_Obstacle(void) {

  if (Application_SmartRobotCarxxx0.Functional_Mode == ObstacleAvoidance_mode) {
    uint16_t get_Distance_L;
    uint16_t get_Distance_R;
    uint16_t get_Distance_OBS_L;
    uint16_t get_Distance_OBS_M;
    uint16_t get_Distance_OBS_R;

    AppULTRASONIC_L.DeviceDriverSet_ULTRASONIC_Get_L(&get_Distance_L /*out*/);
    // Serial.print("ULTRASONIC_L=");
    // Serial.println(get_Distance_L);
    AppULTRASONIC_R.DeviceDriverSet_ULTRASONIC_Get_R(&get_Distance_R /*out*/);
    // Serial.print("ULTRASONIC_R=");
    // Serial.println(get_Distance_R);
    AppULTRASONIC_OBS_L.DeviceDriverSet_ULTRASONIC_Get_OBS_L(&get_Distance_OBS_L /*out*/);
    Serial.print("ULTRASONIC_OBS_L=");
    Serial.println(get_Distance_OBS_L);
    AppULTRASONIC_OBS_M.DeviceDriverSet_ULTRASONIC_Get_OBS_M(&get_Distance_OBS_M /*out*/);
    Serial.print("ULTRASONIC_OBS_M=");
    Serial.println(get_Distance_OBS_M);
    AppULTRASONIC_OBS_R.DeviceDriverSet_ULTRASONIC_Get_OBS_R(&get_Distance_OBS_R /*out*/);
    Serial.print("ULTRASONIC_OBS_R=");
    Serial.println(get_Distance_OBS_R);



    int randomTime = (rand() % (2000 - 1000 + 1)) + 1000;
    int randomDirection = (rand() % 2);
    
    if (function_xxx(get_Distance_L, 0, 0) || function_xxx(get_Distance_R, 0, 0) || function_xxx(get_Distance_OBS_L, 0, 0) || function_xxx(get_Distance_OBS_M, 0, 0) || function_xxx(get_Distance_OBS_R, 0, 0)) {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 50);
      
      if (function_xxx(get_Distance_L, 0, 0)){
        Serial.println("Not Plugged In Top L");}
      else if (function_xxx(get_Distance_R, 0, 0)){
        Serial.println("Not Plugged In Top R");}
      delay_xxx(50);


    
    } else if (function_xxx(get_Distance_OBS_L, 1, 15)) {
      ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 50);
      delay_xxx(randomTime);
    } else if (function_xxx(get_Distance_OBS_R, 1, 15)) {
      ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 50);
      delay_xxx(randomTime);
    } else if (function_xxx(get_Distance_OBS_M, 1, 15)) {
      if (randomDirection == 0) {
        ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 50);
        delay_xxx(randomTime);
      } else {
        ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 50);
        delay_xxx(randomTime);
      }
    } else if (function_xxx(get_Distance_L, 1, 10) && function_xxx(get_Distance_R, 1, 10)) {
      ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 50);
      // Serial.println("Forward");
      delay_xxx(50);
    } else if (function_xxx(get_Distance_L, 1, 10)) {
      ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 50);
      // Serial.println("Left");
      delay_xxx(randomTime);
    } else if (function_xxx(get_Distance_R, 1, 10)) {
      ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 50);
      // Serial.println("Right");
      delay_xxx(randomTime);
    } else {
      // Seed the random number generator with the current time
      // srand(time(NULL));
      // Generate a random integer between 0 and RAND_MAX
      
      // Serial.println(randomTime);
      // Serial.println(randomDirection);

      if (randomDirection == 0) {
        ApplicationFunctionSet_SmartRobotCarMotionControl(Backward, 50);
        delay_xxx(500);
        ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 50);
        // Serial.println("Back Right");
        delay_xxx(randomTime);
      } else {
        ApplicationFunctionSet_SmartRobotCarMotionControl(Backward, 50);
        delay_xxx(500);
        ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 50);
        // Serial.println("Back Left");
        delay_xxx(randomTime);
      }

    }

    


  } else {
    Serial.println("Error: Not in Obstacle Avoidance Mode?");
  }
}
