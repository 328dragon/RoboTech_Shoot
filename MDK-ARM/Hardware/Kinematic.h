/*
 * @Author: Elaina
 * @Date: 2024-07-07 17:06:10
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-10-18 01:02:15
 * @FilePath: \MDK-ARM\Hardware\Kinematic.h
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#ifndef __KINEMATIC_H
#define __KINEMATIC_H
#include "stdint.h"
#include "math.h"
#include "pid_template.h"
#define PI 3.1415926535
namespace Kinematic
{
    // 左上为0，右上为1，左下为2，右下为3
    struct cmd_vel_t
    {
        float linear_x;
        float linear_y;
        float angular_z;
    };
    struct odom_t
    {
        float x;
        float y;
        float yaw;
    };
    enum class_t
    {
        O_shape,
        X_shape
    };
    class Kinematic_t
    {
    public:
        Kinematic_t() 
        {
            current_odom={0,0,0};
        }
        void forward(cmd_vel_t &cmd_vel_in, float *current_speed);

        void inv(cmd_vel_t const &cmd_vel_in, float *speed_control);

        void inv(cmd_vel_t const &cmd_vel_in, float *speed_control, odom_t &odom_in); // 传入当前的odom_t,表示基于大地坐标系推算电机速度
        
        void ClearOdometry(); // 清除里程计

        void CalculationUpdate(uint16_t dt, cmd_vel_t &cmd_vel_in, odom_t &odom_in); // 从底盘速度更新里程计
        void CalculationUpdate(uint16_t dt, cmd_vel_t &cmd_vel_in, odom_t &odom_in,float yaw); 
        void update_odom(odom_t &odom_in); // 更新里程计
        odom_t target_odom;
        odom_t current_odom;
        cmd_vel_t current_vel;
        cmd_vel_t target_val;
        odom_t _odom_error;

    private:
        float a = 0.1;
        float b = 0.1;
        class_t diclass = X_shape;
        // 三个轴的位置环pid控制,采用增量式pid
    };
    // 可以沿着x,y前进与绕着z轴旋转，现在定义小车前进方向为x，左边为y的右手系
}
#endif
