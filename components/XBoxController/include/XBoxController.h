#ifndef __XBOXCONTROLLER_H__
#define __XBOXCONTROLLER_H__

#include "Arduino.h"
#include "XboxSeriesXControllerESP32_asukiaaa.hpp"

#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"

#include "struct_component.h"
//#include "Remote_Motion.h"


#define XBOX_DEFAULT_NAMESPACE                  ("mac_address")
#define XBOX_DEFAULT_MAC                        ("68:6c:e6:42:57:89")

void xboxController_task(void *pt);
//void xboxController_mode_select(void *pt);

#endif
