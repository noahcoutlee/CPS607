/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-12 16:36:20
 * @LastEditors: Changhua
 * @Description: SmartRobot robot tank
 * @FilePath: 
 */
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

/*ITR20001 Detection*/
bool DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_Init(void)
{
  pinMode(PIN_ITR20001xxxL, INPUT);
  pinMode(PIN_ITR20001xxxM, INPUT);
  pinMode(PIN_ITR20001xxxR, INPUT);
  return false;
}
float DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_getAnaloguexxx_L(void)
{
  return analogRead(PIN_ITR20001xxxL);
}
float DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_getAnaloguexxx_M(void)
{
  return analogRead(PIN_ITR20001xxxM);
}
float DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_getAnaloguexxx_R(void)
{
  return analogRead(PIN_ITR20001xxxR);
}
#if _Test_DeviceDriverSet
void DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_Test(void)
{
  Serial.print("\tL=");
  Serial.print(analogRead(PIN_ITR20001xxxL));

  Serial.print("\tM=");
  Serial.print(analogRead(PIN_ITR20001xxxM));

  Serial.print("\tR=");
  Serial.println(analogRead(PIN_ITR20001xxxR));
}
#endif

#if _Test_DeviceDriverSet
void DeviceDriverSet_Motor::DeviceDriverSet_Motor_Test(void)
{
  digitalWrite(PIN_Motor_AIN_1, LOW);
  analogWrite(PIN_Motor_PWMA, 100);
  digitalWrite(PIN_Motor_BIN_1, HIGH);
  analogWrite(PIN_Motor_PWMB, 100);

  delay(3000);
  digitalWrite(PIN_Motor_AIN_1, HIGH);
  analogWrite(PIN_Motor_PWMA, 100);
  digitalWrite(PIN_Motor_BIN_1, LOW);
  analogWrite(PIN_Motor_PWMB, 100);
  delay(3000);
}
#endif

/*
 Motor_control：AB / 方向、速度
*/
/*
 Motor_control：AB / 方向、速度
*/
void DeviceDriverSet_Motor::DeviceDriverSet_Motor_control(boolean direction_A, uint8_t speed_A, //A组电机参数
                                                          boolean direction_B, uint8_t speed_B, //B组电机参数
                                                          boolean controlED                     //AB使能允许 true
                                                          )                                     //电机控制
{

  if (controlED == control_enable) //使能允许？
  {
    digitalWrite(PIN_Motor_STBY, HIGH);
    { //A...Right

      switch (direction_A) //方向控制
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

    { //B...Left
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

// IR sensor

void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_IR_Init(void)
{
  pinMode(IN_IR, INPUT);
}


/*ULTRASONIC*/
//#include <NewPing.h>
// NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_ULTRASONIC_Init_R(void)
{
  pinMode(ECHO_PIN_R, INPUT); //Ultrasonic module initialization
  pinMode(TRIG_PIN_R, OUTPUT);
}

void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_ULTRASONIC_Init_L(void)
{
  pinMode(ECHO_PIN_L, INPUT); //Ultrasonic module initialization
  pinMode(TRIG_PIN_L, OUTPUT);
}

void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_ULTRASONIC_Init_OBS_L(void)
{
  pinMode(ECHO_PIN_OBS_L, INPUT); //Ultrasonic module initialization
  pinMode(TRIG_PIN_OBS_L, OUTPUT);
  pinMode(29, OUTPUT);
  pinMode(23, INPUT);
}

void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_ULTRASONIC_Init_OBS_M(void)
{
  pinMode(ECHO_PIN_OBS_M, INPUT); //Ultrasonic module initialization
  pinMode(TRIG_PIN_OBS_M, OUTPUT);
  pinMode(28, OUTPUT);
  pinMode(22, INPUT);
}

void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_ULTRASONIC_Init_OBS_R(void)
{
  pinMode(ECHO_PIN_OBS_R, INPUT); //Ultrasonic module initialization
  pinMode(TRIG_PIN_OBS_R, OUTPUT);
  pinMode(37, OUTPUT);
  pinMode(31, INPUT);
}

void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_Get_IR(uint16_t *Get_IR /*out*/)
{
  *Get_IR = analogRead(IN_IR);
}

void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_ULTRASONIC_Get_L(uint16_t *ULTRASONIC_Get_L /*out*/)
{
  unsigned int tempda_x_L = 0;
  digitalWrite(TRIG_PIN_L, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_L, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_L, LOW);
  tempda_x_L = ((unsigned int)pulseIn(ECHO_PIN_L, HIGH) / 58);
  *ULTRASONIC_Get_L = tempda_x_L;
}


void DeviceDriverSet_ULTRASONIC::DeviceDriverSet_ULTRASONIC_Get_R(uint16_t *ULTRASONIC_Get_R)
{
  unsigned int tempda_x_R = 0;
  digitalWrite(TRIG_PIN_R, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_R, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_R, LOW);
  tempda_x_R = ((unsigned int)pulseIn(ECHO_PIN_R, HIGH) / 58);
  *ULTRASONIC_Get_R = tempda_x_R;
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