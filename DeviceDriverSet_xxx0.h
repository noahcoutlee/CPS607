#ifndef _DeviceDriverSet_xxx0_H_
#define _DeviceDriverSet_xxx0_H_

#include <arduino.h>

class DeviceDriverSet_Motor
{
  public:
    #define direction_just true
    #define direction_back false
    #define direction_void 3

    #define control_enable true

    void DeviceDriverSet_Motor_Init(void);
    void DeviceDriverSet_Motor_control(boolean direction_A, uint8_t speed_A, boolean direction_B, uint8_t speed_B, boolean controlED);
  private:
    #define PIN_Motor_PWMA 5
    #define PIN_Motor_PWMB 6
    #define PIN_Motor_BIN_1 8
    #define PIN_Motor_AIN_1 7
    #define PIN_Motor_STBY 3
};

class DeviceDriverSet_ULTRASONIC
{
  public:
    void DeviceDriverSet_ULTRASONIC_Init_OBS_L(void);
    void DeviceDriverSet_ULTRASONIC_Init_OBS_M(void);
    void DeviceDriverSet_ULTRASONIC_Init_OBS_R(void);

    void DeviceDriverSet_ULTRASONIC_Get_OBS_L(uint16_t *ULTRASONIC_Get_OBS_L  /*out*/);
    void DeviceDriverSet_ULTRASONIC_Get_OBS_M(uint16_t *ULTRASONIC_Get_OBS_M  /*out*/);
    void DeviceDriverSet_ULTRASONIC_Get_OBS_R(uint16_t *ULTRASONIC_Get_OBS_R  /*out*/);

    void DeviceDriverSet_IR_Init(void);
    void DeviceDriverSet_Get_IR(uint16_t *Get_IR  /*out*/);

  private:
    #define TRIG_PIN_OBS_L 27
    #define ECHO_PIN_OBS_L 25
    #define TRIG_PIN_OBS_M 26
    #define ECHO_PIN_OBS_M 24
    #define TRIG_PIN_OBS_R 35
    #define ECHO_PIN_OBS_R 33
};

class DeviceDriverSet_LINE_TRACKER
{
  public:
    bool DeviceDriverSet_LINE_TRACKER_Init(void);
    float DeviceDriverSet_LINE_TRACKER_get_LT_L(void);
    float DeviceDriverSet_LINE_TRACKER_get_LT_M(void);
    float DeviceDriverSet_LINE_TRACKER_get_LT_R(void);

  private:
    #define PIN_LT_L A2
    #define PIN_LT_M A1
    #define PIN_LT_R A0
};


class DeviceDriverSet_FLAME_IR
{
  public:
    bool DeviceDriverSet_FLAME_IR_Init(void);
    float DeviceDriverSet_get_FLAME_IR_L(void);
    float DeviceDriverSet_get_FLAME_IR_M(void);
    float DeviceDriverSet_get_FLAME_IR_R(void);

  private:
    #define FLAME_IR_R A15
    #define FLAME_IR_M A14
    #define FLAME_IR_L A13
};

#endif
