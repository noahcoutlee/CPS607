#include "DeviceDriverSet_xxx0.h"

/*Motor control*/
void DeviceDriverSet_Motor::DeviceDriverSet_Motor_Init(void)
{
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);
}

void DeviceDriverSet_Motor::DeviceDriverSet_Motor_control(boolean direction_A, uint8_t speed_A, boolean direction_B, uint8_t speed_B, boolean controlED)
{
  if (controlED == control_enable)
  {
    digitalWrite(PIN_Motor_STBY, HIGH);
    {
      switch (direction_A)
      {
      case direction_just:
        digitalWrite(PIN_Motor_AIN_1, HIGH);
        analogWrite(PIN_Motor_PWMA, speed_A);
        break;
      case direction_back:
        digitalWrite(PIN_Motor_AIN_1, LOW);
        analogWrite(PIN_Motor_PWMA, speed_A);
        break;
      case direction_void:
        analogWrite(PIN_Motor_PWMA, 0);
        digitalWrite(PIN_Motor_STBY, LOW);
        break;
      default:
        analogWrite(PIN_Motor_PWMA, 0);
        digitalWrite(PIN_Motor_STBY, LOW);
        break;
      }
    }
    {
      switch (direction_B)
      {
      case direction_just:
        digitalWrite(PIN_Motor_BIN_1, HIGH);
        analogWrite(PIN_Motor_PWMB, speed_B);
        break;
      case direction_back:
        digitalWrite(PIN_Motor_BIN_1, LOW);
        analogWrite(PIN_Motor_PWMB, speed_B);
        break;
      case direction_void:
        analogWrite(PIN_Motor_PWMB, 0);
        digitalWrite(PIN_Motor_STBY, LOW);
        break;
      default:
        analogWrite(PIN_Motor_PWMB, 0);
        digitalWrite(PIN_Motor_STBY, LOW);
        break;
      }
    }
  }
  else
  {
    digitalWrite(PIN_Motor_STBY, LOW);
    return;
  }
}

void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_ULTRASONIC_Init_OBS_L(void)
{
  pinMode(ECHO_PIN_OBS_L, INPUT);
  pinMode(TRIG_PIN_OBS_L, OUTPUT);
  pinMode(29, OUTPUT);
  pinMode(23, INPUT);
}

void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_ULTRASONIC_Init_OBS_M(void)
{
  pinMode(ECHO_PIN_OBS_M, INPUT);
  pinMode(TRIG_PIN_OBS_M, OUTPUT);
  pinMode(28, OUTPUT);
  pinMode(22, INPUT);
}

void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_ULTRASONIC_Init_OBS_R(void)
{
  pinMode(ECHO_PIN_OBS_R, INPUT);
  pinMode(TRIG_PIN_OBS_R, OUTPUT);
  pinMode(37, OUTPUT);
  pinMode(31, INPUT);
}

void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_ULTRASONIC_Init_BACK(void)
{
  pinMode(ECHO_PIN_BACK, INPUT);
  pinMode(TRIG_PIN_BACK, OUTPUT);
  pinMode(36, OUTPUT);
  pinMode(30, INPUT);
}

void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_ULTRASONIC_Get_OBS_L(uint16_t *ULTRASONIC_Get_OBS_L /*out*/)
{
  unsigned int tempda_x_L = 0;
  digitalWrite(29, HIGH);
  digitalWrite(TRIG_PIN_OBS_L, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_OBS_L, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_OBS_L, LOW);
  tempda_x_L = ((unsigned int)pulseIn(ECHO_PIN_OBS_L, HIGH) / 58);
  *ULTRASONIC_Get_OBS_L = tempda_x_L;
}

void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_ULTRASONIC_Get_OBS_M(uint16_t *ULTRASONIC_Get_OBS_M /*out*/)
{
  unsigned int tempda_x_M = 0;
  digitalWrite(28, HIGH);
  digitalWrite(TRIG_PIN_OBS_M, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_OBS_M, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_OBS_M, LOW);
  tempda_x_M = ((unsigned int)pulseIn(ECHO_PIN_OBS_M, HIGH) / 58);
  *ULTRASONIC_Get_OBS_M = tempda_x_M;
}

void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_ULTRASONIC_Get_OBS_R(uint16_t *ULTRASONIC_Get_OBS_R /*out*/)
{
  unsigned int tempda_x_R = 0;
  digitalWrite(37, HIGH);
  digitalWrite(TRIG_PIN_OBS_R, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_OBS_R, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_OBS_R, LOW);
  tempda_x_R = ((unsigned int)pulseIn(ECHO_PIN_OBS_R, HIGH) / 58);
  *ULTRASONIC_Get_OBS_R = tempda_x_R;
}

void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_ULTRASONIC_Get_BACK(uint16_t *ULTRASONIC_Get_BACK /*out*/)
{
  unsigned int tempda_x = 0;
  digitalWrite(36, HIGH);
  digitalWrite(TRIG_PIN_BACK, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_BACK, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_BACK, LOW);
  tempda_x = ((unsigned int)pulseIn(ECHO_PIN_BACK, HIGH) / 58);
  *ULTRASONIC_Get_BACK = tempda_x;
}

bool DeviceDriverSet_LINE_TRACKER::DeviceDriverSet_LINE_TRACKER_Init(void)
{
  pinMode(PIN_LT_L, INPUT);
  pinMode(PIN_LT_M, INPUT);
  pinMode(PIN_LT_R, INPUT);
  return false;
}
float DeviceDriverSet_LINE_TRACKER::DeviceDriverSet_LINE_TRACKER_get_LT_L(void)
{
  return analogRead(PIN_LT_L);
}
float DeviceDriverSet_LINE_TRACKER::DeviceDriverSet_LINE_TRACKER_get_LT_M(void)
{
  return analogRead(PIN_LT_M);
}
float DeviceDriverSet_LINE_TRACKER::DeviceDriverSet_LINE_TRACKER_get_LT_R(void)
{
  return analogRead(PIN_LT_R);
}

// IR sensor
bool DeviceDriverSet_FLAME_IR::DeviceDriverSet_FLAME_IR_Init(void)
{
  pinMode(FLAME_IR_R, INPUT);
  pinMode(53, OUTPUT);
  pinMode(FLAME_IR_M, INPUT);
  pinMode(52, OUTPUT);
  pinMode(FLAME_IR_L, INPUT);
  pinMode(51, OUTPUT);
  pinMode(50, OUTPUT);
  digitalWrite(53, HIGH);
  digitalWrite(52, HIGH);
  digitalWrite(51, HIGH);
  digitalWrite(50, LOW);
  return false;
}

float DeviceDriverSet_FLAME_IR::DeviceDriverSet_get_FLAME_IR_R(void)
{
  return analogRead(FLAME_IR_R);
}

float DeviceDriverSet_FLAME_IR::DeviceDriverSet_get_FLAME_IR_L(void)
{
  return analogRead(FLAME_IR_L);
}

float DeviceDriverSet_FLAME_IR::DeviceDriverSet_get_FLAME_IR_M(void)
{
  return analogRead(FLAME_IR_M);
}

Servo myservo;
void DeviceDriverSet_Servo::DeviceDriverSet_Servo_Init(unsigned int Position_angle)
{
  myservo.attach(PIN_Servo_z);
  myservo.write(Position_angle);
}

void DeviceDriverSet_Servo::DeviceDriverSet_Servo_control(unsigned int Position_angle)
{
  myservo.write(Position_angle);
}