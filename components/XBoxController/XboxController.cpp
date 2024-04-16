#include "include/XBoxController.h"


void xboxController_task(void *pt)
{
    memset(&xbox_info, 0, sizeof(xbox_info_t));
    
    XboxSeriesXControllerESP32_asukiaaa::Core xboxController(XBOX_DEFAULT_MAC);
    XboxSeriesXHIDReportBuilder_asukiaaa::ReportBase repo;

    TickType_t xLastWake = xTaskGetTickCount();

    xboxController.begin();
  
    while(1)
    {
        vTaskDelayUntil(&xLastWake, 250);

        xboxController.onLoop();

        if(xboxController.isConnected()) 
        {
            if(!xboxController.isWaitingForFirstNotification()) 
            {
                xbox_info.btnA = (uint8_t) xboxController.xboxNotif.btnA;
                xbox_info.btnB = (uint8_t) xboxController.xboxNotif.btnB;
                xbox_info.btnX = (uint8_t) xboxController.xboxNotif.btnX;
                xbox_info.btnY = (uint8_t) xboxController.xboxNotif.btnY;
                
                
                xbox_info.btnDirUp = (uint8_t) xboxController.xboxNotif.btnDirUp;
                xbox_info.btnDirLeft = (uint8_t) xboxController.xboxNotif.btnDirLeft;
                xbox_info.btnDirRight = (uint8_t) xboxController.xboxNotif.btnDirRight;
                xbox_info.btnDirDown = (uint8_t) xboxController.xboxNotif.btnDirDown;
                
                
                xbox_info.btnShare = (uint8_t) xboxController.xboxNotif.btnShare;
                xbox_info.btnStart = (uint8_t) xboxController.xboxNotif.btnStart;
                xbox_info.btnSelect = (uint8_t) xboxController.xboxNotif.btnSelect;
                xbox_info.btnXbox = (uint8_t) xboxController.xboxNotif.btnXbox;
                

                xbox_info.btnLB = (uint8_t) xboxController.xboxNotif.btnLB;
                xbox_info.btnRB = (uint8_t) xboxController.xboxNotif.btnRB;
                xbox_info.btnLS = (uint8_t) xboxController.xboxNotif.btnLS;
                xbox_info.btnRS = (uint8_t) xboxController.xboxNotif.btnRS;

                
                xbox_info.joyLHori = xboxController.xboxNotif.joyLHori;
                xbox_info.joyLVert = xboxController.xboxNotif.joyLVert;
                xbox_info.joyRHori = xboxController.xboxNotif.joyRHori;
                xbox_info.joyRVert = xboxController.xboxNotif.joyRVert;

                xbox_info.trigLT = xboxController.xboxNotif.trigLT;
                xbox_info.trigRT = xboxController.xboxNotif.trigRT;

                xbox_status = 1;
            }
        } 
        else
        {
            if(xboxController.getCountFailedConnection() > 2)
            {
                // ESP.restart();
                
                esp_bluedroid_disable();
                esp_bluedroid_deinit();
                esp_bt_controller_disable();
                esp_bt_controller_deinit();

                printf("BLE Closed\n");

                xbox_status = 0;
                break;
            }
        }
    }

    vTaskDelete(NULL);
}


/*
void xboxController_mode_select(void *pt)
{
    TickType_t tick = xTaskGetTickCount();

    while(xbox_status != 0)
    {
        vTaskDelayUntil(&tick, 1000);

        if(xbox_info.btnX == 1)
        {
            vTaskDelayUntil(&tick, 65);
            if(xbox_info.btnX == 1)
            {
                vTaskDelayUntil(&tick, 45);
                if(xbox_info.btnX == 1) xbox_control_mode = !xbox_control_mode;
            }
        }
    }

    vTaskDelete(NULL);
}
*/
