#include "INS.h"
#include "Motor_DAMIAO_Driver.h"
#include "Motor_DrEmpower.h"
#include "Motor_Driver.h"
#include "Remote_Control.h"
#include "RoboControl.h"
#include "main.h"

extern uint16_t CAN1_Err_Ticker;
extern uint16_t CAN2_Err_Ticker;
extern uint16_t CAN3_Err_Ticker;

void ErrorHandle_Task(void const* argument) {
    for (;;) {
        /*===| 遥控器断连处理 |===*/
        Remote_Control_Struct.Remote_GetData_Ticker++;
        Remote_Control_Struct.ImageLink_GetData_Ticker++;

        if (Remote_Control_Struct.Remote_GetData_Ticker >= 500) {
            Remote_Control_Struct.If_Remote_Connect = 0;
            Remote_Control_Struct.Remote_GetData_Ticker = 200;
        }
        if (Remote_Control_Struct.ImageLink_GetData_Ticker >= 500) {
            Remote_Control_Struct.If_ImageLink_Connect = 0;
            Remote_Control_Struct.ImageLink_GetData_Ticker = 500;
        }

        if (Remote_Control_Struct.If_ImageLink_Connect == 0
            && Remote_Control_Struct.If_Remote_Connect == 0) {
            Robo_Stop();
            memset(&Remote_Control_Last_Struct.Mouse_Speed_X, 0, 30);
        }

        /*===| CAN错误处理 |===*/
        CAN1_Err_Ticker++;
        CAN2_Err_Ticker++;
        CAN3_Err_Ticker++;
        if (CAN1_Err_Ticker > 100) {
            MX_FDCAN1_Init();
            HAL_FDCAN_Start(&hfdcan1);
            CAN_Filter_Init(&hfdcan1);
            CAN1_Err_Ticker = 0;
        }
        if (CAN2_Err_Ticker > 100) {
            MX_FDCAN2_Init();
            HAL_FDCAN_Start(&hfdcan2);
            CAN_Filter_Init(&hfdcan2);
            CAN2_Err_Ticker = 0;
        }
        if (CAN3_Err_Ticker > 100) {
            MX_FDCAN3_Init();
            HAL_FDCAN_Start(&hfdcan3);
            CAN_Filter_Init(&hfdcan3);
            CAN3_Err_Ticker = 0;
        }

        /*===| 电机离线断连处理 |===*/
        Motor.Chassis1.Ticker++;
        if (Motor.Chassis1.Ticker >= 100)
            Motor.Chassis1.If_Online = 0;
        if (Motor.Chassis1.Ticker > 110)
            Motor.Chassis1.Ticker = 100;
        Motor.Chassis2.Ticker++;
        if (Motor.Chassis2.Ticker >= 100)
            Motor.Chassis2.If_Online = 0;
        if (Motor.Chassis2.Ticker > 110)
            Motor.Chassis2.Ticker = 100;
        Motor.Chassis3.Ticker++;
        if (Motor.Chassis3.Ticker >= 100)
            Motor.Chassis3.If_Online = 0;
        if (Motor.Chassis3.Ticker > 110)
            Motor.Chassis3.Ticker = 100;
        Motor.Chassis4.Ticker++;
        if (Motor.Chassis4.Ticker >= 100)
            Motor.Chassis4.If_Online = 0;
        if (Motor.Chassis4.Ticker > 110)
            Motor.Chassis4.Ticker = 100;
        Motor.Fric1.Ticker++;
        if (Motor.Fric1.Ticker >= 100)
            Motor.Fric1.If_Online = 0;
        if (Motor.Fric1.Ticker > 110)
            Motor.Fric1.Ticker = 100;
        Motor.Fric2.Ticker++;
        if (Motor.Fric2.Ticker >= 100)
            Motor.Fric2.If_Online = 0;
        if (Motor.Fric2.Ticker > 110)
            Motor.Fric2.Ticker = 100;
        Motor.Pitch.Ticker++;
        if (Motor.Pitch.Ticker >= 100)
            Motor.Pitch.If_Online = 0;
        if (Motor.Pitch.Ticker > 110)
            Motor.Pitch.Ticker = 100;
        Motor.Trigger.Ticker++;
        if (Motor.Trigger.Ticker >= 100)
            Motor.Trigger.If_Online = 0;
        if (Motor.Trigger.Ticker > 110)
            Motor.Trigger.Ticker = 100;
        Motor.Yaw.Ticker++;
        if (Motor.Yaw.Ticker >= 100)
            Motor.Yaw.If_Online = 0;
        if (Motor.Yaw.Ticker > 110)
            Motor.Yaw.Ticker = 100;

        //        /*===| Yaw电机离线断连处理 |===*/
        //        if(Motor_Data_Struct[Gimbal_Yaw_ID].Ticker == 110)
        //        {
        //            Motor_DM_CMD_ClearErr(&Gimbal_Yaw_CAN, Gimbal_Yaw_Send_CAN_ID);
        //            osDelay(1);
        //            Motor_DM_CMD_Enable(&Gimbal_Yaw_CAN, Gimbal_Yaw_Send_CAN_ID);
        //        }

        osDelay(1);
    }
}