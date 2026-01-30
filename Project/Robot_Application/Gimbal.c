//======================================================================
//                      Gimbal[云台控制]
//      驱动相应电机使云台陀螺仪达到RoboControl的Pitch，Yaw目标角度
//======================================================================

#include "Gimbal.h"
#include "Aim.h"
#include "INS.h"
#include "Motor_DAMIAO_Driver.h"
#include "Motor_DJI_Driver.h"
#include "Motor_Unitree_Driver.h"
#include "RoboControl.h"

/*===| 云台控制数据结构体 |===*/
Gimbal_Control_StructTypeDef Gimbal_Control_Struct;

void Gimbal_Task(void* argument) {
    /*===| 云台PID参数初始化 |===*/
    PID_Init(&Gimbal_Control_Struct.Yaw_Angle_PID_Struct, 15, 0, 300, 0, 0, 200);
    PID_Init(&Gimbal_Control_Struct.Yaw_Speed_PID_Struct, 150, 3, 0, 20, 1200, 16384);
    PID_Init(&Gimbal_Control_Struct.Pitch_Angle_PID_Struct, 10, 0, 200, 0, 0, 100);
    PID_Init(&Gimbal_Control_Struct.Pitch_Speed_PID_Struct, 400, 1, 0, 0, 3500, 16384);

    for (;;) {
        /*===| 得到当前Pitch和Yaw数据 |===*/
        Gimbal_Control_Struct.Pitch_Feedback = INS_Data_Gimbal.Pitch;
        Gimbal_Control_Struct.Yaw_Feedback = INS_Data_Gimbal.YawTotalAngle;

        /*===| 得到云台控制参数 ===*/
        Gimbal_Control_Struct.Gimbal_State = RoboControl_Struct.Gimbal_State;

        /*===| 得到自瞄位置与当前位置的偏移角度[-180 ~ +180度] |===*/
        Gimbal_Control_Struct.Vision_Yaw_Err = INS_Data_Gimbal.Yaw - Aim_Receive_Struct.yaw;
        if (Gimbal_Control_Struct.Vision_Yaw_Err > 180.0f)
            Gimbal_Control_Struct.Vision_Yaw_Err = Gimbal_Control_Struct.Vision_Yaw_Err - 360;
        else if (Gimbal_Control_Struct.Vision_Yaw_Err < -180.0f)
            Gimbal_Control_Struct.Vision_Yaw_Err = 360 + Gimbal_Control_Struct.Vision_Yaw_Err;

        if (Gimbal_Control_Struct.Gimbal_State == Gimbal_State_Aim && Aim_Receive_Struct.appear) {
            RoboControl_Struct.Robo_Target_Yaw =
                Gimbal_Control_Struct.Yaw_Feedback - Gimbal_Control_Struct.Vision_Yaw_Err;
            RoboControl_Struct.Robo_Target_Pitch = Aim_Receive_Struct.pitch;
        }

        Gimbal_Control_Struct.Pitch_Target = RoboControl_Struct.Robo_Target_Pitch;
        Gimbal_Control_Struct.Yaw_Target = RoboControl_Struct.Robo_Target_Yaw;

        /*===| 得到Yaw与底盘正方向的偏移角度[-180 ~ +180度] |===*/
        Gimbal_Control_Struct.Yaw_Err = Motor.Yaw.Angle - Gimbal_Median_Yaw;
        if (Gimbal_Control_Struct.Yaw_Err >= 180)
            Gimbal_Control_Struct.Yaw_Err = 360 - Gimbal_Control_Struct.Yaw_Err;
        if (Gimbal_Control_Struct.Yaw_Err <= -180)
            Gimbal_Control_Struct.Yaw_Err = 360 + Gimbal_Control_Struct.Yaw_Err;
        RoboControl_Struct.Yaw_Err = Gimbal_Control_Struct.Yaw_Err;

        /*===| 串级PID串级计算得到Yaw电机的电流大小 |===*/
        PID_Position_Calculate(
            &Gimbal_Control_Struct.Yaw_Angle_PID_Struct,
            Gimbal_Control_Struct.Yaw_Target,
            Gimbal_Control_Struct.Yaw_Feedback
        );
        PID_Position_Calculate(
            &Gimbal_Control_Struct.Yaw_Speed_PID_Struct,
            Gimbal_Control_Struct.Yaw_Angle_PID_Struct.Output,
            Motor.Yaw.Speed_RPM - Chassis_Control_Struct.Odometer_Chassis_Wz / PI * 30
        );

        /*===| 串级PID串级计算得到Pitch电机的电流大小 |===*/
        PID_Position_Calculate(
            &Gimbal_Control_Struct.Pitch_Angle_PID_Struct,
            Gimbal_Control_Struct.Pitch_Target,
            Gimbal_Control_Struct.Pitch_Feedback
        );
        PID_Position_Calculate(
            &Gimbal_Control_Struct.Pitch_Speed_PID_Struct,
            Gimbal_Control_Struct.Pitch_Angle_PID_Struct.Output,
            -Motor.Pitch.Total_Angle_Speed_RPM
        );

        //        USB_Printf("%f,%f,%f,%f,%f\n",
        //        Gimbal_Control_Struct.Yaw_Target,
        //        Gimbal_Control_Struct.Yaw_Feedback,
        //        Gimbal_Control_Struct.Yaw_Angle_PID_Struct.Output,
        //        Motor_Data_Struct[Gimbal_Yaw_ID].Total_Angle_Speed_RPM + Chassis_Control_Struct.Odometer_Wz * PI_to_Degree/60.0f,
        //        Gimbal_Control_Struct.Yaw_Speed_PID_Struct.Output / 10000.0f);

        /*===| 发送电流参数 |===*/
        if (RoboControl_Struct.Robo_Enable && INS_Data_Gimbal.If_INS_Init) {
            if (Gimbal_Control_Struct.If_Setting_Zero) {
                Motor_DM_CMD_SetZero(&Gimbal_Yaw_CAN, Gimbal_Yaw_Send_CAN_ID);
            } else {
                Motor_DM1to4_SendCurrent(
                    &Gimbal_Yaw_CAN,
                    Gimbal_Yaw_Send_CAN_ID,
                    (int16_t
                    )(RoboControl_Struct.Smooth_Start_K
                      * Gimbal_Control_Struct.Yaw_Speed_PID_Struct.Output),
                    0,
                    0,
                    0
                );
            }

            Motor_DJI_SendCurrent(
                &Gimbal_Pitch_CAN,
                Gimbal_Pitch_Send_CAN_ID,
                0,
                RoboControl_Struct.Smooth_Start_K
                    * -Gimbal_Control_Struct.Pitch_Speed_PID_Struct.Output,
                0,
                0
            );

        } else {
            Motor_DJI_SendCurrent(&Gimbal_Pitch_CAN, Gimbal_Pitch_Send_CAN_ID, 0, 0, 0, 0);
            Motor_DM1to4_SendCurrent(&Gimbal_Yaw_CAN, Gimbal_Yaw_Send_CAN_ID, 0, 0, 0, 0);
        }
        osDelay(1);
    }
}
