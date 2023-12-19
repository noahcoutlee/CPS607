#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.h"

void setup()
{
  Application_FunctionSet.ApplicationFunctionSet_Init();
}

void loop()
{
  Application_FunctionSet.ApplicationFunctionSet_Main();
}
