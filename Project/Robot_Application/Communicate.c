//========================================================
//                      Commnuicate[双板通信]
//      双板通信的发送和解包
//      当前有；云台Vx,Vy,Wz,YawErr，以及一些状态标志位
//========================================================

#include "Communicate.h"
#include "INS.h"
#include "RoboControl.h"
#include "Shoot.h"
#include "SuperCap_Driver.h"
#include "Vofa.h"

/*===| 通信错误看门狗---如果10ms内没有收到双板通信数据，则重置底盘状态 |===*/
uint8_t Board_Commnuicate_Error_Ticker;

/**
 * @brief 通信任务
 */
void Communicate_Task(void* argument) {
    osDelay(50);

    for (;;) {
        /*===| 发送超电控制帧 |===*/
        SuperCap_Send_Control_Data();
        //        USB_Printf("%f,%f,%f,%f,%f,%f\n",
        //                                         Motor_Data_Struct[Shoot_Fric_First_Left_ID].Speed_RPM * Fric_RPM_to_m_s,
        //                                        -Motor_Data_Struct[Shoot_Fric_First_Right_ID].Speed_RPM * Fric_RPM_to_m_s,
        //                                         Motor_Data_Struct[Shoot_Fric_First_Middle_ID].Speed_RPM * Fric_RPM_to_m_s,
        //                                         Motor_Data_Struct[Shoot_Fric_Second_Left_ID].Speed_RPM * Fric_RPM_to_m_s,
        //                                         Motor_Data_Struct[Shoot_Fric_Second_Right_ID].Speed_RPM * Fric_RPM_to_m_s,
        //                                        -Motor_Data_Struct[Shoot_Fric_Second_Middle_ID].Speed_RPM * Fric_RPM_to_m_s
        //                );
        //        USB_Printf("%f,%f,%f,%f,%f,%f\n",
        //        Gimbal_Control_Struct.Yaw_Target,
        //        Gimbal_Control_Struct.Yaw_Feedback,
        //        Gimbal_Control_Struct.Yaw_Angle_PID_Struct.Error,
        //        Gimbal_Control_Struct.Yaw_Angle_PID_Struct.Output,
        //        Gimbal_Control_Struct.Yaw_Speed_PID_Struct.Feedback,
        //        Gimbal_Control_Struct.Yaw_Speed_PID_Struct.Output/100.0f
        //        );

        osDelay(1);
    }
}
