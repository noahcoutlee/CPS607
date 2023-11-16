#ifndef _ApplicationFunctionSet_xxx0_H_
#define _ApplicationFunctionSet_xxx0_H_

#include <arduino.h>

class ApplicationFunctionSet
{
public:
  void ApplicationFunctionSet_Init(void);
  void ApplicationFunctionSet_Main(void);
  void ApplicationFunctionSet_Tracking(void);         

public:
  uint16_t TrackingDetection_S = 250;
  uint16_t TrackingDetection_E = 850;
};
extern ApplicationFunctionSet Application_FunctionSet;
#endif
