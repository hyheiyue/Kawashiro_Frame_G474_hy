//========================================================
//                      Aim[自瞄]
//      自瞄的通信与处理
//========================================================

#include "Aim.h"
#include "Chassis.h"
#include "Gimbal.h"
#include "INS.h"
#include "Referee_Unpack.h"
#include "RoboControl.h"

#define Bullet_Speed_Average_Count_Number 5
float Bullet_Speed_Average = 15.0f;
float Bullet_Speed[Bullet_Speed_Average_Count_Number];
float Bullet_Speed_Last;

uint32_t Aim_DWT_Count;

Aim_Receive_StructTypedef Aim_Receive_Struct = { .Shoot_Frequency = 15 };
Aim_Send_StructTypedef Aim_Send_Struct = { 0 };

void Aim_Task(void const* argument) {
    for (uint8_t i = 0; i < Bullet_Speed_Average_Count_Number; i++) {
        Bullet_Speed[i] = Bullet_Speed_Average;
    }

    for (;;) {
        /*===| 统计平均弹速 |===*/
        Get_Average_Bullet_Speed();

        Aim_Send_Struct.bullet_speed = Bullet_Speed_Average;
        if (Robo_State.robot_id < 100)
            Aim_Send_Struct.detect_color = 1;
        else
            Aim_Send_Struct.detect_color = 0;
        Aim_Send_Struct.yaw = INS_Data_Gimbal.Yaw;
        Aim_Send_Struct.roll = INS_Data_Gimbal.Roll;
        Aim_Send_Struct.pitch = INS_Data_Gimbal.Pitch;
        Aim_Send_Struct.CheckSum =
            Aim_Send_Struct.yaw + Aim_Send_Struct.roll + Aim_Send_Struct.pitch;
        Aim_Send_Struct.yaw_vel = INS_Data_Gimbal.Yaw_Speed;
        Aim_Send_Struct.Vx = Chassis_Control_Struct.Odometer_Gimbal_Vy;
        Aim_Send_Struct.Vy = -Chassis_Control_Struct.Odometer_Gimbal_Vx;

        Aim_Send_Struct.time_stamp += DWT_GetDeltaT(&Aim_DWT_Count) * 1000000;

        USB_Send((uint8_t*)&Aim_Send_Struct, sizeof(Aim_Send_StructTypedef));

        osDelay(1);
    }
}

void Get_Average_Bullet_Speed(void) {
    if (Robo_ShootData.initial_speed != Bullet_Speed_Last && Robo_ShootData.initial_speed > 12.0f
        && Robo_ShootData.initial_speed < 16.0f)
    {
        for (uint8_t i = Bullet_Speed_Average_Count_Number - 1; i > 0; i--) {
            Bullet_Speed[i] = Bullet_Speed[i - 1];
        }
        Bullet_Speed[0] = Robo_ShootData.initial_speed;

        float Speed_Sum = 0;
        for (uint8_t i = 0; i < Bullet_Speed_Average_Count_Number; i++) {
            Speed_Sum += Bullet_Speed[i];
        }

        Bullet_Speed_Average = Speed_Sum / (float)Bullet_Speed_Average_Count_Number;
    }
    Bullet_Speed_Last = Robo_ShootData.initial_speed;
}

void Aim_Get_Data_CallBack(uint8_t* Data) {
    memcpy((uint8_t*)&Aim_Receive_Struct, Data, sizeof(Aim_Receive_StructTypedef));

    if (Aim_Receive_Struct.Shoot_Frequency > 0.1 && Aim_Receive_Struct.Shoot_Frequency < 25)
        Shoot_Control_Struct.Shoot_Frequency = Aim_Receive_Struct.Shoot_Frequency;
}

uint8_t Aim_If_Allow_Shoot(void) {
    if ((fabs(Aim_Receive_Struct.target_yaw - INS_Data_Gimbal.Yaw)
         < Aim_Receive_Struct.enable_yaw_diff)
        && (fabs(Aim_Receive_Struct.target_pitch - INS_Data_Gimbal.Pitch)
            < Aim_Receive_Struct.enable_pitch_diff))
        return 1;
    else
        return 0;
}
