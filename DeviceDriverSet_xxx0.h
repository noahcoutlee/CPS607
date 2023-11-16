/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-12 14:45:27
 * @LastEditors: Changhua
 * @Description: SmartRobot robot tank
 * @FilePath: 
 */
#ifndef _DeviceDriverSet_xxx0_H_
#define _DeviceDriverSet_xxx0_H_

#define _Test_DeviceDriverSet 0
#include <arduino.h>


class DeviceDriverSet_ITR20001
{
public:
  bool DeviceDriverSet_ITR20001_Init(void);
  float DeviceDriverSet_ITR20001_getAnaloguexxx_L(void);
  float DeviceDriverSet_ITR20001_getAnaloguexxx_M(void);
  float DeviceDriverSet_ITR20001_getAnaloguexxx_R(void);
#if _Test_DeviceDriverSet
  void DeviceDriverSet_ITR20001_Test(void);
#endif

private:
#define PIN_ITR20001xxxL A2
#define PIN_ITR20001xxxM A1
#define PIN_ITR20001xxxR A0
};

/*Motor*/
class DeviceDriverSet_Motor
{
public:
  void DeviceDriverSet_Motor_Init(void);
#if _Test_DeviceDriverSet
  void DeviceDriverSet_Motor_Test(void);
#endif
  void DeviceDriverSet_Motor_control(boolean direction_A, uint8_t speed_A, //A组电机参数
                                     boolean direction_B, uint8_t speed_B, //B组电机参数
                                     boolean controlED                     //AB使能允许 true
  );                                                                       //电机控制
private:
#define PIN_Motor_PWMA 5
#define PIN_Motor_PWMB 6
#define PIN_Motor_BIN_1 8
#define PIN_Motor_AIN_1 7
#define PIN_Motor_STBY 3

public:
#define speed_Max 255
#define direction_just true
#define direction_back false
#define direction_void 3

#define Duration_enable true
#define Duration_disable false
#define control_enable true
#define control_disable false
};
/*ULTRASONIC*/

//#include <NewPing.h>
class DeviceDriverSet_ULTRASONIC
{
public:
  void DeviceDriverSet_ULTRASONIC_Init_R(void);
  void DeviceDriverSet_ULTRASONIC_Init_L(void);
  void DeviceDriverSet_ULTRASONIC_Init_OBS_L(void);
  void DeviceDriverSet_ULTRASONIC_Init_OBS_M(void);
  void DeviceDriverSet_ULTRASONIC_Init_OBS_R(void);

  void DeviceDriverSet_IR_Init(void);
#if _Test_DeviceDriverSet
  void DeviceDriverSet_ULTRASONIC_Test_R(void);
  void DeviceDriverSet_ULTRASONIC_Test_L(void);
  void DeviceDriverSet_ULTRASONIC_Init_OBS_L(void);
  void DeviceDriverSet_ULTRASONIC_Init_OBS_M(void);
  void DeviceDriverSet_ULTRASONIC_Init_OBS_R(void);
#endif
  void DeviceDriverSet_ULTRASONIC_Get_R(uint16_t *ULTRASONIC_Get_R  /*out*/);
  void DeviceDriverSet_ULTRASONIC_Get_L(uint16_t *ULTRASONIC_Get_L  /*out*/);
  void DeviceDriverSet_ULTRASONIC_Get_OBS_L(uint16_t *ULTRASONIC_Get_OBS_L  /*out*/);
  void DeviceDriverSet_ULTRASONIC_Get_OBS_M(uint16_t *ULTRASONIC_Get_OBS_M  /*out*/);
  void DeviceDriverSet_ULTRASONIC_Get_OBS_R(uint16_t *ULTRASONIC_Get_OBS_R  /*out*/);

  void DeviceDriverSet_Get_IR(uint16_t *Get_IR  /*out*/);

private:
#define TRIG_PIN_R 11     // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN_R 10      // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIG_PIN_L 13     // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN_L 12
#define TRIG_PIN_OBS_L 27
#define ECHO_PIN_OBS_L 25
#define TRIG_PIN_OBS_M 26
#define ECHO_PIN_OBS_M 24
#define TRIG_PIN_OBS_R 35
#define ECHO_PIN_OBS_R 33
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define IN_IR A15
};

#endif
