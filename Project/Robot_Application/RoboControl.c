//====================================================================================
//                           RoboControl[机器状态控制]
//      根据操控意愿把遥控器数据处理成机器人
//      整体[RoboEnable,Vx,Vy,Wz,Yaw,Pitch等],各个模块[Gimbal,Shoot,Chassis等]的状态,
//      各个模块再根据状态执行相应的功能
//      其中包括状态设置，冲突处理，错误检测与处理等
//
//      机器人状态结构体：RoboControl_Struct
//====================================================================================

#include "RoboControl.h"
#include "Aim.h"
#include "Buzzer.h"
#include "INS.h"
#include "Motor_DAMIAO_Driver.h"
#include "Motor_DJI_Driver.h"
#include "Motor_DrEmpower.h"
#include "Motor_Unitree_Driver.h"
#include "Remote_Control.h"

/*===| 机器人整体状态数据结构体 |===*/
RoboControl_StructTypeDef RoboControl_Struct;

/*===| 计算Wz的PID结构体 |===*/
PID_Struct_TypeDef RoboGimbal_Wz_PID_Struct;

void Robo_Task(void* argument) {
    /*===| Yaw参数初始化 |===*/
    RoboControl_Struct.Robo_Target_Yaw = 0;

    /*===| Pitch参数初始化 |===*/
    RoboControl_Struct.Robo_Target_Pitch = 0;

    /*===| Wz_PID初始化 |===*/
    PID_Init(&RoboGimbal_Wz_PID_Struct, 0.15, 0, 10, 0, 0, Robo_Wz_MaxSpeed);

    /*===| 默认模块状态 |===*/
    RoboControl_Struct.Chassis_State = Chassis_STATIC;
    RoboControl_Struct.Shoot_State = Shoot_State_Off;
    RoboControl_Struct.Gimbal_State = Gimbal_State_Normal;
    RoboControl_Struct.Chassis_Speed_Level = 3;
    RoboControl_Struct.SuperCap_State = 0;
    Referee_Data_Init();

    for (;;) {
        /*===| 遥控方式选择 |===*/
        /*===| 如果遥控器连接 |===*/
        if (Remote_Control_Struct.If_Remote_Connect) {
            /*===| 右拨杆拨到最下面开启键盘控制，否则是遥控器控制 |===*/
            if (Remote_Control_Struct.S2 != SW_Down)
                RoboControl_Struct.Controler = Remote;
            else if (Remote_Control_Struct.S2 == SW_Down) {
                /*===| 左拨杆拨中选择使用遥控器的键鼠数据，拨到上选择图传链路的键鼠数据 |===*/
                if (Remote_Control_Struct.S1 == SW_Up) {
                    RoboControl_Struct.Controler = ImageLink_Key;
                } else if (Remote_Control_Struct.S1 == SW_Mid) {
                    RoboControl_Struct.Controler = Remote_Key;
                }
            }
        } else if (Remote_Control_Struct.If_ImageLink_Connect) {
            RoboControl_Struct.Controler = ImageLink_Key;
        } else {
            RoboControl_Struct.Controler = none;
        }

        /*===| 遥控方式 |===*/
        /*===| 遥控器摇杆控制 |===*/
        if (RoboControl_Struct.Controler == Remote) {
            RemoteControl_Float();
            if (Remote_Control_Struct.If_Remote_Data_New == 1) {
                RemoteControl_Bool();
                Remote_Control_Struct.If_Remote_Data_New = 0;
            }
        }
        /*===| 遥控器键鼠控制 |===*/
        else if (RoboControl_Struct.Controler == Remote_Key)
        {
            Remote_Control_Struct.IF_USE_ImageLink = 0;
            KeyControl_Float();
            if (Remote_Control_Struct.If_Remote_Data_New == 1) {
                KeyControl_Bool();
                Remote_Control_Struct.If_Remote_Data_New = 0;
            }
        }
        /*===| 图传链路键鼠控制 |===*/
        else if (RoboControl_Struct.Controler == ImageLink_Key)
        {
            Remote_Control_Struct.IF_USE_ImageLink = 1;
            KeyControl_Float();
            if (Remote_Control_Struct.If_ImageLink_Data_New == 1) {
                KeyControl_Bool();
                Remote_Control_Struct.If_ImageLink_Data_New = 0;
            }
        } else {
            RoboControl_Struct.Robo_Enable = 0;
        }

        /*===| Pitch控制参数限幅 |===*/
        Limit_float(
            &RoboControl_Struct.Robo_Target_Pitch,
            Gimbal_Pitch_Limit_Up,
            Gimbal_Pitch_Limit_Down
        );

        /*===| 根据底盘运动状态得到Wz |===*/
        Get_Chassis_Wz();

        /*===| 控制参数低通滤波 |===*/
        Control_Filter();

        /*===| 平滑启动(3s) |===*/
        RoboControl_Struct.Smooth_Start_K += 0.000333;
        if (RoboControl_Struct.Smooth_Start_K >= 1)
            RoboControl_Struct.Smooth_Start_K = 1;

        osDelay(1);
    }
}

/**
 * @brief 根据底盘运动状态得到Wz
 */
void Get_Chassis_Wz(void) {
    /*===| 底盘关闭 |===*/
    if (RoboControl_Struct.Chassis_State == Chassis_OFF) {
        RoboControl_Struct.Robo_Target_Wz = 0;
    }
    /*===| 底盘静止 |===*/
    else if (RoboControl_Struct.Chassis_State == Chassis_STATIC)
    {
        RoboControl_Struct.Robo_Target_Wz = 0;
    }
    /*===| 底盘跟随模式 |===*/
    else if (RoboControl_Struct.Chassis_State == Chassis_FOLLOW)
    {
        PID_Position_Calculate(&RoboGimbal_Wz_PID_Struct, 0, RoboControl_Struct.Yaw_Err);
        RoboControl_Struct.Robo_Target_Wz = RoboGimbal_Wz_PID_Struct.Output;
    }
    /*===| 底盘侧向跟随模式 |===*/
    else if (RoboControl_Struct.Chassis_State == Chassis_F_SIDE)
    {
        PID_Position_Calculate(
            &RoboGimbal_Wz_PID_Struct,
            2.0f * (RoboControl_Struct.Chassis_Follow_45_Direction_Flag - 0.5f) * 45,
            RoboControl_Struct.Yaw_Err
        );
        RoboControl_Struct.Robo_Target_Wz = RoboGimbal_Wz_PID_Struct.Output;
    }
    /*===| 底盘小陀螺 |===*/
    else if (RoboControl_Struct.Chassis_State == Chassis_SPIN)
    {
        /*===| 小陀螺得到Wz,每次切换正反转 |===*/
        if (RoboControl_Struct.SPIN_Direction_Flag) {
            RoboControl_Struct.Robo_Target_Wz = Robo_Wz_MaxSpeed;
        } else {
            RoboControl_Struct.Robo_Target_Wz = -Robo_Wz_MaxSpeed;
        }
        /*===| 根据档位限制旋转速度，遥控器控制状态不限制 |===*/
        RoboControl_Struct.Robo_Target_Wz = RoboControl_Struct.Robo_Target_Wz
            * (1.0f + RoboControl_Struct.Chassis_Speed_Level) / 6.0f;

        Motor.Yaw.Round = 0;
    }
    /*===| 冲刺模式 |===*/
    else if (RoboControl_Struct.Chassis_State == Chassis_DASH)
    {
        /*===| 如果底盘没有回正，则等待底盘回到正方向 |===*/
        if (RoboControl_Struct.Yaw_Err > 5.0f || RoboControl_Struct.Yaw_Err < -5.0f) {
            RoboControl_Struct.Chassis_State = Chassis_FOLLOW;
            PID_Position_Calculate(&RoboGimbal_Wz_PID_Struct, 0, RoboControl_Struct.Yaw_Err);
            RoboControl_Struct.Robo_Target_Wz = RoboGimbal_Wz_PID_Struct.Output;
        } else {
            RoboControl_Struct.Chassis_State = Chassis_DASH;
            RoboControl_Struct.Robo_Target_Wz = 0;
        }
    }
}

/**
 * @brief 输入参数低通滤波
 */
void Control_Filter(void) {
    RoboControl_Struct.Robo_Target_Vx = 0.4f * RoboControl_Struct.Robo_Target_Vx
        + (1 - 0.4f) * RoboControl_Struct.Robo_Target_Vx_Last;
    RoboControl_Struct.Robo_Target_Vy = 0.4f * RoboControl_Struct.Robo_Target_Vy
        + (1 - 0.4f) * RoboControl_Struct.Robo_Target_Vy_Last;
    RoboControl_Struct.Robo_Target_Wz = 0.9f * RoboControl_Struct.Robo_Target_Wz
        + (1 - 0.9f) * RoboControl_Struct.Robo_Target_Wz_Last;
    RoboControl_Struct.Robo_Target_Yaw = 0.8f * RoboControl_Struct.Robo_Target_Yaw
        + (1 - 0.8f) * RoboControl_Struct.Robo_Target_Yaw_Last;
    RoboControl_Struct.Robo_Target_Pitch = 0.8f * RoboControl_Struct.Robo_Target_Pitch
        + (1 - 0.8f) * RoboControl_Struct.Robo_Target_Pitch_Last;

    RoboControl_Struct.Robo_Target_Vx_Last = RoboControl_Struct.Robo_Target_Vx;
    RoboControl_Struct.Robo_Target_Vy_Last = RoboControl_Struct.Robo_Target_Vy;
    RoboControl_Struct.Robo_Target_Wz_Last = RoboControl_Struct.Robo_Target_Wz;
    RoboControl_Struct.Robo_Target_Yaw_Last = RoboControl_Struct.Robo_Target_Yaw;
    RoboControl_Struct.Robo_Target_Pitch_Last = RoboControl_Struct.Robo_Target_Pitch;
}

/**
 * @brief 遥控器控制
 */
void RemoteControl_Float(void) {
    /*===| 底盘和云台运动控制 |===*/
    static uint32_t Remote_DWT_Count;
    float Dt = DWT_GetDeltaT(&Remote_DWT_Count);

    if (Remote_Control_Struct.RC_Side < 0.7f && Remote_Control_Struct.RC_Side > -0.7f) {
        RoboControl_Struct.Robo_Target_Vx = -Robo_Vx_MaxSpeed
            * RoboControl_Struct.Chassis_Speed_Level / 5.0f * Remote_Control_Struct.RC_Left_X;
        RoboControl_Struct.Robo_Target_Vy = -Robo_Vy_MaxSpeed
            * RoboControl_Struct.Chassis_Speed_Level / 5.0f * Remote_Control_Struct.RC_Left_Y;
        RoboControl_Struct.Robo_Target_Yaw -= Dt * 300.0f * Remote_Control_Struct.RC_Right_X;
        RoboControl_Struct.Robo_Target_Pitch += Dt * 80.0f * Remote_Control_Struct.RC_Right_Y;
    }
}
void RemoteControl_Bool(void) {
    /*===| 左拨杆拨到最下面关闭所有设备 |===*/
    if (Remote_Control_Struct.S1 == SW_Down)
        Robo_Stop();

    /*===| 重启设备 |===*/
    else if (Remote_Control_Struct.S1 != SW_Down && Remote_Control_Last_Struct.S1 == SW_Down)
        Robo_Restart();
    /*===| 侧滑轮往上进入自定义功能选项1，往下是选项2，取消摇杆控制移动，通过摇杆选择触发一次定义的八个功能 |===*/
    if (Remote_Control_Struct.RC_Side > 0.7f) {
        /*===| 自定义功能-左摇杆向右 |===*/
        if (Remote_Control_Struct.RC_Left_X > 0.8f && Remote_Control_Last_Struct.RC_Left_X < 0.8f) {
            /*===| 底盘跟随模式 |===*/
            RoboControl_Struct.Chassis_State = Chassis_FOLLOW;
        }
        /*===| 自定义功能-左摇杆向左 |===*/
        else if (Remote_Control_Struct.RC_Left_X < -0.8f && Remote_Control_Last_Struct.RC_Left_X > -0.8f)
        {
            /*===| 底盘静止模式 |===*/
            RoboControl_Struct.Chassis_State = Chassis_STATIC;
        }
        /*===| 自定义功能-左摇杆向上 |===*/
        if (Remote_Control_Struct.RC_Left_Y > 0.8f && Remote_Control_Last_Struct.RC_Left_Y < 0.8f) {
            /*===| 改后驱冲刺 |===*/
            RoboControl_Struct.Chassis_State = Chassis_DASH;
        }
        /*===| 自定义功能-左摇杆向下 |===*/
        else if (Remote_Control_Struct.RC_Left_Y < -0.8f && Remote_Control_Last_Struct.RC_Left_Y > -0.8f)
        {}
        /*===| 自定义功能-右摇杆向右 |===*/
        if (Remote_Control_Struct.RC_Right_X > 0.8f && Remote_Control_Last_Struct.RC_Right_X < 0.8f)
        {
            /*===| 速度等级+1 |===*/
            RoboControl_Struct.Chassis_Speed_Level += 1;
            if (RoboControl_Struct.Chassis_Speed_Level > 5)
                RoboControl_Struct.Chassis_Speed_Level = 5;
        }
        /*===| 自定义功能-右摇杆向左 |===*/
        else if (Remote_Control_Struct.RC_Right_X < -0.8f && Remote_Control_Last_Struct.RC_Right_X > -0.8f)
        {
            /*===| 速度等级-1 |===*/
            RoboControl_Struct.Chassis_Speed_Level -= 1;
            if (RoboControl_Struct.Chassis_Speed_Level < 1)
                RoboControl_Struct.Chassis_Speed_Level = 1;
        }
        /*===| 自定义功能-右摇杆向上 |===*/
        if (Remote_Control_Struct.RC_Right_Y > 0.8f && Remote_Control_Last_Struct.RC_Right_Y < 0.8f)
        {
            /*===| 开超电 |===*/
            RoboControl_Struct.SuperCap_State = 1;
            Buzzer_Set_SoundEffect(Buzzer_SoundEffect_SuperCap_ON);
        }
        /*===| 自定义功能-右摇杆向下 |===*/
        else if (Remote_Control_Struct.RC_Right_Y < -0.8f && Remote_Control_Last_Struct.RC_Right_Y > -0.8f)
        {
            /*===| 关超电 |===*/
            RoboControl_Struct.SuperCap_State = 0;
            Buzzer_Set_SoundEffect(Buzzer_SoundEffect_SuperCap_OFF);
        }
    } else if (Remote_Control_Struct.RC_Side < -0.7f && Remote_Control_Struct.S2 == SW_Mid) {
        /*===| 自定义功能-左摇杆向右 |===*/
        if (Remote_Control_Struct.RC_Left_X > 0.8f && Remote_Control_Last_Struct.RC_Left_X < 0.8f) {
        }
        /*===| 自定义功能-左摇杆向左 |===*/
        else if (Remote_Control_Struct.RC_Left_X < -0.8f && Remote_Control_Last_Struct.RC_Left_X > -0.8f)
        {}
        /*===| 自定义功能-左摇杆向上 |===*/
        if (Remote_Control_Struct.RC_Left_Y > 0.8f && Remote_Control_Last_Struct.RC_Left_Y < 0.8f) {
        }
        /*===| 自定义功能-左摇杆向下 |===*/
        else if (Remote_Control_Struct.RC_Left_Y < -0.8f && Remote_Control_Last_Struct.RC_Left_Y > -0.8f)
        {}
        /*===| 自定义功能-右摇杆向右 |===*/
        if (Remote_Control_Struct.RC_Right_X > 0.8f && Remote_Control_Last_Struct.RC_Right_X < 0.8f)
        {
            Shoot_Control_Struct.Shoot_Frequency += 3;
            if (Shoot_Control_Struct.Shoot_Frequency < 3)
                Shoot_Control_Struct.Shoot_Frequency = 3;
            else if (Shoot_Control_Struct.Shoot_Frequency > 24)
                Shoot_Control_Struct.Shoot_Frequency = 24;
            if (Shoot_Control_Struct.Shoot_Frequency / 3 == 1)
                Buzzer_Set_SoundEffect(Buzzer_SoundEffect_Shoot1);
            else if (Shoot_Control_Struct.Shoot_Frequency / 3 == 2)
                Buzzer_Set_SoundEffect(Buzzer_SoundEffect_Shoot2);
            else if (Shoot_Control_Struct.Shoot_Frequency / 3 == 3)
                Buzzer_Set_SoundEffect(Buzzer_SoundEffect_Shoot3);
            else if (Shoot_Control_Struct.Shoot_Frequency / 3 == 4)
                Buzzer_Set_SoundEffect(Buzzer_SoundEffect_Shoot4);
            else if (Shoot_Control_Struct.Shoot_Frequency / 3 == 5)
                Buzzer_Set_SoundEffect(Buzzer_SoundEffect_Shoot5);
            else if (Shoot_Control_Struct.Shoot_Frequency / 3 == 6)
                Buzzer_Set_SoundEffect(Buzzer_SoundEffect_Shoot6);
            else if (Shoot_Control_Struct.Shoot_Frequency / 3 == 7)
                Buzzer_Set_SoundEffect(Buzzer_SoundEffect_Shoot7);
            else if (Shoot_Control_Struct.Shoot_Frequency / 3 == 8)
                Buzzer_Set_SoundEffect(Buzzer_SoundEffect_Shoot8);
        }
        /*===| 自定义功能-右摇杆向左 |===*/
        else if (Remote_Control_Struct.RC_Right_X < -0.8f && Remote_Control_Last_Struct.RC_Right_X > -0.8f)
        {
            Shoot_Control_Struct.Shoot_Frequency -= 3;
            if (Shoot_Control_Struct.Shoot_Frequency < 3)
                Shoot_Control_Struct.Shoot_Frequency = 3;
            else if (Shoot_Control_Struct.Shoot_Frequency > 24)
                Shoot_Control_Struct.Shoot_Frequency = 24;
            if (Shoot_Control_Struct.Shoot_Frequency / 3 == 1)
                Buzzer_Set_SoundEffect(Buzzer_SoundEffect_Shoot1);
            else if (Shoot_Control_Struct.Shoot_Frequency / 3 == 2)
                Buzzer_Set_SoundEffect(Buzzer_SoundEffect_Shoot2);
            else if (Shoot_Control_Struct.Shoot_Frequency / 3 == 3)
                Buzzer_Set_SoundEffect(Buzzer_SoundEffect_Shoot3);
            else if (Shoot_Control_Struct.Shoot_Frequency / 3 == 4)
                Buzzer_Set_SoundEffect(Buzzer_SoundEffect_Shoot4);
            else if (Shoot_Control_Struct.Shoot_Frequency / 3 == 5)
                Buzzer_Set_SoundEffect(Buzzer_SoundEffect_Shoot5);
            else if (Shoot_Control_Struct.Shoot_Frequency / 3 == 6)
                Buzzer_Set_SoundEffect(Buzzer_SoundEffect_Shoot6);
            else if (Shoot_Control_Struct.Shoot_Frequency / 3 == 7)
                Buzzer_Set_SoundEffect(Buzzer_SoundEffect_Shoot7);
            else if (Shoot_Control_Struct.Shoot_Frequency / 3 == 8)
                Buzzer_Set_SoundEffect(Buzzer_SoundEffect_Shoot8);
        }
        /*===| 自定义功能-右摇杆向上 |===*/
        if (Remote_Control_Struct.RC_Right_Y > 0.8f && Remote_Control_Last_Struct.RC_Right_Y < 0.8f)
        {
            /*===| 开自瞄 |===*/
            RoboControl_Struct.Gimbal_State = Gimbal_State_Aim;
            Buzzer_Set_SoundEffect(Buzzer_SoundEffect_Aim_ON);
        }
        /*===| 自定义功能-右摇杆向下 |===*/
        else if (Remote_Control_Struct.RC_Right_Y < -0.8f && Remote_Control_Last_Struct.RC_Right_Y > -0.8f)
        {
            /*===| 关自瞄 |===*/
            RoboControl_Struct.Gimbal_State = Gimbal_State_Normal;
            RoboControl_Struct.Robo_Target_Yaw = Gimbal_Control_Struct.Yaw_Feedback;
            RoboControl_Struct.Robo_Target_Pitch = Gimbal_Control_Struct.Pitch_Feedback;
            Buzzer_Set_SoundEffect(Buzzer_SoundEffect_Aim_OFF);
        }
    }

    //        if(Remote_Control_Struct.RC_Side < -0.7f && RoboControl_Struct.Shoot_State == Shoot_State_Off)
    //        {
    //            RoboControl_Struct.Chassis_State = Chassis_DASH;
    //        }
    //        else if(RoboControl_Struct.Chassis_State == Chassis_DASH)
    //        {
    //            RoboControl_Struct.Chassis_State = Chassis_FOLLOW;
    //        }

    /*===| 左拨杆拨到中间开启小陀螺 |===*/
    if (Remote_Control_Struct.S1 == SW_Up) {
        /*===| 每次开启小陀螺反向 |===*/
        if (Remote_Control_Last_Struct.S1 != SW_Up)
            RoboControl_Struct.SPIN_Direction_Flag = !RoboControl_Struct.SPIN_Direction_Flag;

        RoboControl_Struct.Chassis_State = Chassis_SPIN;
    } else {
        if (RoboControl_Struct.Chassis_State == Chassis_SPIN) {
            RoboControl_Struct.Chassis_State = Chassis_FOLLOW;
        }
    }

    /*===| 发射状态控制 |===*/
    /*===| 右摇杆控制发射状态 上[关闭摩擦轮] 中[打开摩擦轮]|===*/
    if (Remote_Control_Struct.S2 == SW_Mid)
        RoboControl_Struct.Shoot_State = Shoot_State_Off;
    else if (Remote_Control_Struct.S2 == SW_Up)
        RoboControl_Struct.Shoot_State = Shoot_State_Ready;

    if (RoboControl_Struct.Shoot_State != Shoot_State_Off) {
        /*===| 侧滑轮往下拨一次单发 |===*/
        if (Remote_Control_Struct.RC_Side < -0.1f && Remote_Control_Last_Struct.RC_Side > -0.1f) {
            if (RoboControl_Struct.Gimbal_State == Gimbal_State_Aim) {
                if (Aim_If_Allow_Shoot())
                    Shoot_Single(1);
            } else {
                Shoot_Single(1);
            }
        }
        /*===| 侧滑轮往下拨到底连发 |===*/
        else if (Remote_Control_Struct.RC_Side < -0.95f)
        {
            if (RoboControl_Struct.Gimbal_State == Gimbal_State_Aim) {
                if (Aim_If_Allow_Shoot())
                    RoboControl_Struct.Shoot_State = Shoot_State_Continue;
                else
                    RoboControl_Struct.Shoot_State = Shoot_State_Ready;
            } else {
                RoboControl_Struct.Shoot_State = Shoot_State_Continue;
            }
        }
        /*===| 侧滑轮松开则回到准备发射状态 |===*/
        else if (Remote_Control_Struct.RC_Side == 0)
        {
            RoboControl_Struct.Shoot_State = Shoot_State_Ready;
        }
    }
}

/**
 * @brief 键盘控制
 */
void KeyControl_Float(void) {
    static uint32_t Key_DWT_Count;
    float Dt = DWT_GetDeltaT(&Key_DWT_Count);

    /*===| 键盘控制Pitch错误值过滤 |===*/
    if (fabs(Remote_Control_Struct.Mouse_Speed_Y - Remote_Control_Last_Struct.Mouse_Speed_Y) > 3.6f)
    {
        Remote_Control_Struct.Mouse_Speed_Y = 0;
    }
    /*===| 底盘和云台运动控制 |===*/
    RoboControl_Struct.Robo_Target_Vx = Robo_Vx_MaxSpeed * RoboControl_Struct.Chassis_Speed_Level
        / 5.0f * (Remote_Control_Struct.Keyboard_D - Remote_Control_Struct.Keyboard_A);
    RoboControl_Struct.Robo_Target_Vy = Robo_Vy_MaxSpeed * RoboControl_Struct.Chassis_Speed_Level
        / 5.0f * (Remote_Control_Struct.Keyboard_W - Remote_Control_Struct.Keyboard_S);
    if (RoboControl_Struct.Gimbal_State == Gimbal_State_Slow) {
        RoboControl_Struct.Robo_Target_Yaw -= Dt * 30.0f * Remote_Control_Struct.Mouse_Speed_X;
        RoboControl_Struct.Robo_Target_Pitch -= Dt * 30.0f * Remote_Control_Struct.Mouse_Speed_Y;
    } else {
        RoboControl_Struct.Robo_Target_Yaw -= Dt * 150.0f * Remote_Control_Struct.Mouse_Speed_X;
        RoboControl_Struct.Robo_Target_Pitch -= Dt * 100.0f * Remote_Control_Struct.Mouse_Speed_Y;
    }
}
void KeyControl_Bool(void) {
    /*===| 按住Ctrl进入瞄准模式 |===*/
    if (KeyCtrl_Single_Press) {
        RoboControl_Struct.Gimbal_State = Gimbal_State_Slow;
        RoboControl_Struct.Chassis_State = Chassis_STATIC;
    } else if (KeyCtrl_Single_Release) {
        RoboControl_Struct.Gimbal_State = Gimbal_State_Normal;
        RoboControl_Struct.Chassis_State = Chassis_FOLLOW;
    }

    /*===| 按住Shift改后驱冲刺 |===*/
    if (KeyShift_Press) {
        RoboControl_Struct.Chassis_State = Chassis_DASH;
    } else if (KeyShift_Single_Release) {
        RoboControl_Struct.Chassis_State = Chassis_FOLLOW;
    }
    /*===| 按X改底盘跟随模式 |===*/
    if (KeyX_Single_Press) {
        if (RoboControl_Struct.Chassis_State != Chassis_STATIC)
            RoboControl_Struct.Chassis_State = Chassis_STATIC;
        else
            RoboControl_Struct.Chassis_State = Chassis_FOLLOW;
    }
    /*===| 按住右键自瞄 |===*/
    if (MouseR_Single_Press) {
        RoboControl_Struct.Gimbal_State = Gimbal_State_Aim;
        Buzzer_Set_SoundEffect(Buzzer_SoundEffect_Aim_ON);
    }
    if (MouseR_Single_Release) {
        RoboControl_Struct.Gimbal_State = Gimbal_State_Normal;
        RoboControl_Struct.Robo_Target_Yaw = Gimbal_Control_Struct.Yaw_Feedback;
        RoboControl_Struct.Robo_Target_Pitch = Gimbal_Control_Struct.Pitch_Feedback;
        Buzzer_Set_SoundEffect(Buzzer_SoundEffect_Aim_OFF);
    }
    /*===| 按C开启小陀螺 |===*/
    if (KeyC_Single_Press) {
        if (RoboControl_Struct.Chassis_State != Chassis_SPIN) {
            /*===| 每次开启小陀螺反向 |===*/
            RoboControl_Struct.SPIN_Direction_Flag = !RoboControl_Struct.SPIN_Direction_Flag;
            RoboControl_Struct.Chassis_State = Chassis_SPIN;
        } else {
            RoboControl_Struct.Chassis_State = Chassis_FOLLOW;
        }
    }

    /*===| 按Q/E向左/右转90度 |===*/
    if (KeyQ_Single_Press)
        RoboControl_Struct.Robo_Target_Yaw += 90;
    if (KeyE_Single_Press)
        RoboControl_Struct.Robo_Target_Yaw -= 90;

    /*===| 按R开关摩擦轮 |===*/
    if (KeyR_Single_Press) {
        if (RoboControl_Struct.Shoot_State != Shoot_State_Off) {
            RoboControl_Struct.Shoot_State = Shoot_State_Off;
        } else {
            RoboControl_Struct.Shoot_State = Shoot_State_Ready;
        }
    }

    if (RoboControl_Struct.Shoot_State != Shoot_State_Off) {
        /*===| 左键连发，如果有自瞄就有火控连发 |===*/
        if (MouseL_Press) {
            if (RoboControl_Struct.Gimbal_State == Gimbal_State_Aim) {
                if (Aim_If_Allow_Shoot())
                    RoboControl_Struct.Shoot_State = Shoot_State_Continue;
                else
                    RoboControl_Struct.Shoot_State = Shoot_State_Ready;
            } else {
                RoboControl_Struct.Shoot_State = Shoot_State_Continue;
            }
        } else {
            RoboControl_Struct.Shoot_State = Shoot_State_Ready;
        }
    }

    /*===| 按V开关超电 |===*/
    if (KeyV_Single_Press) {
        RoboControl_Struct.SuperCap_State = !RoboControl_Struct.SuperCap_State;
        if (RoboControl_Struct.SuperCap_State)
            Buzzer_Set_SoundEffect(Buzzer_SoundEffect_SuperCap_ON);
        else
            Buzzer_Set_SoundEffect(Buzzer_SoundEffect_SuperCap_OFF);
    }

    /*===| 按F切换速度档位 |===*/
    if (KeyF_Single_Press) {
        RoboControl_Struct.Chassis_Speed_Level++;
        if (RoboControl_Struct.Chassis_Speed_Level == 6)
            RoboControl_Struct.Chassis_Speed_Level = 1;
    }

    /*===| 按B重刷UI |===*/
    if (KeyB_Press) {
        RoboControl_Struct.Refresh_UI_Flag = 1;
    } else {
        RoboControl_Struct.Refresh_UI_Flag = 0;
    }

    /*===| 按Z拨弹轮减1/6弹位 |===*/
    if (KeyZ_Single_Press) {
        Shoot_Getout(1.0f / 6.0f);
    }

    /*===| 按G重启 |===*/
    if (KeyG_Single_Press) {
        Robo_Restart();
    }
}

/**
 * @brief 关闭机器人
 */
void Robo_Stop(void) {
    RoboControl_Struct.Robo_Enable = 0;

    RoboControl_Struct.Robo_Target_Vx = 0;
    RoboControl_Struct.Robo_Target_Vy = 0;
    RoboControl_Struct.Robo_Target_Wz = 0;
    RoboControl_Struct.Shoot_State = Shoot_State_Off;
    RoboControl_Struct.SuperCap_State = 0;
    RoboControl_Struct.Chassis_State = Chassis_OFF;
}

/**
 * @brief 重启机器人
 */
void Robo_Restart(void) {
    /*===| 重启后Pitch回到0，Yaw更新到当前位置 |===*/
    RoboControl_Struct.Robo_Target_Yaw = INS_Data_Gimbal.YawTotalAngle;
    RoboControl_Struct.Robo_Target_Pitch = 0;

    /*===| 默认模块状态 |===*/
    RoboControl_Struct.Chassis_State = Chassis_STATIC;
    RoboControl_Struct.Shoot_State = Shoot_State_Off;
    RoboControl_Struct.Gimbal_State = Gimbal_State_Normal;
    RoboControl_Struct.Smooth_Start_K = 0;

    RoboControl_Struct.Robo_Enable = 1;
}
