/*
 * @Author: Elaina
 * @Date: 2024-07-07 17:06:10
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-10-18 00:56:07
 * @FilePath: \MDK-ARM\Hardware\Kinematic.cpp
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include "Kinematic.h"
#define PI 3.1415926
/**
 * @brief 根据自身坐标系解算运动学逆解(只算一次)
 * @param {cmd_vel_t} *cmd_vel_in
 * @param {float} *speed_control
 * @return {*}
 */
using namespace Kinematic;
void Kinematic_t::inv(cmd_vel_t const &cmd_vel_in, float *speed_control)
{
    // o型
    switch (diclass)
    {
    case class_t::O_shape:
        speed_control[0] = cmd_vel_in.linear_y - cmd_vel_in.linear_x + cmd_vel_in.angular_z * (a + b);
        speed_control[1] = cmd_vel_in.linear_y + cmd_vel_in.linear_x - cmd_vel_in.angular_z * (a + b);
        speed_control[2] = cmd_vel_in.linear_y - cmd_vel_in.linear_x - cmd_vel_in.angular_z * (a + b);
        speed_control[3] = cmd_vel_in.linear_y + cmd_vel_in.linear_x + cmd_vel_in.angular_z * (a + b);
        break;

        // x型
    case class_t::X_shape:
        // 左上 (0) 轮：受 linear_x、linear_y、angular_z 的影响
        speed_control[0] = cmd_vel_in.linear_x - cmd_vel_in.linear_y - cmd_vel_in.angular_z * (a + b);

        // 右上 (1) 轮：受 linear_x、linear_y、angular_z 的影响
        speed_control[1] = cmd_vel_in.linear_x + cmd_vel_in.linear_y + cmd_vel_in.angular_z * (a + b);

        // 左下 (2) 轮：受 linear_x、linear_y、angular_z 的影响
        speed_control[2] = cmd_vel_in.linear_x + cmd_vel_in.linear_y - cmd_vel_in.angular_z * (a + b);

        // 右下 (3) 轮：受 linear_x、linear_y、angular_z 的影响
        speed_control[3] = cmd_vel_in.linear_x - cmd_vel_in.linear_y + cmd_vel_in.angular_z * (a + b);
        break;

    default:
        break;
    }
}

/**
 * @brief 根据大地坐标系解算运动学逆解(需要根据当前姿态实时计算)
 * @param {cmd_vel_t} *cmd_vel_in
 * @param {float} *speed_control
 * @param {odom_t} *odom_in
 * @return {*}
 */
void Kinematic_t::inv(cmd_vel_t const &cmd_vel_in, float *speed_control, odom_t &odom_in)
{
    float yaw = odom_in.yaw;
    cmd_vel_t cmd_vel_body;
    cmd_vel_body.angular_z = cmd_vel_in.angular_z;
    // 将大地坐标系下的目标速度转换为车身坐标系下的目标速度
    float target_vx = cmd_vel_in.linear_x;
    float target_vy = cmd_vel_in.linear_y;
    float target_omega = cmd_vel_in.angular_z;

    cmd_vel_body.linear_x = target_vx * cos(yaw) + target_vy * sin(yaw);
    cmd_vel_body.linear_y = -target_vx * sin(yaw) + target_vy * cos(yaw);
    inv(cmd_vel_body, speed_control);
}
/**
 * @brief 从当前车身的速度推算底盘的速度
 * @param {cmd_vel_t} *cmd_vel_in
 * @param {float} *current_speed
 * @return {*}
 */
void Kinematic_t::forward(cmd_vel_t &cmd_vel_in, float *current_speed)
{
    float v0 = current_speed[0]; // 左上轮子速度
    float v1 = current_speed[1]; // 右上轮子速度
    float v2 = current_speed[2]; // 左下轮子速度
    float v3 = current_speed[3]; // 右下轮子速度

    // 修正后的正解算公式
    cmd_vel_in.linear_x = (v0 + v1 + v2 + v3) / 4.0;               // X轴方向速度
    cmd_vel_in.linear_y = (-v0 + v1 + v2 - v3) / 4.0;              // Y轴方向速度
    cmd_vel_in.angular_z = (-v0 + v1 - v2 + v3) / (4.0 * (a + b)); // 角速度
}



/**
 * @brief 里程计更新函数，需要传递进dt(单位为ms)
 * @param {uint16_t} dt 传入的时间间隔
 * @param {cmd_vel_t} *cmd_vel_in 传入当前的速度
 * @param {odom_t} *odom_in 传入被跟新里程计
 * @return {*}
 */

void Kinematic_t::CalculationUpdate(uint16_t dt, cmd_vel_t &cmd_vel_in, odom_t &odom_in)
{
    float delta_t = (float)dt / 1000;
    float dyaw = cmd_vel_in.angular_z * delta_t;
    float dx = cmd_vel_in.linear_x * delta_t;
    float dy = cmd_vel_in.linear_y * delta_t;
    odom_in.yaw += dyaw;
    odom_in.y += dx * sinf(odom_in.yaw) + dy * cosf(odom_in.yaw);
    odom_in.x += dx * cosf(odom_in.yaw) - dy * sinf(odom_in.yaw);
}

void Kinematic_t::CalculationUpdate(uint16_t dt, cmd_vel_t &cmd_vel_in, odom_t &odom_in,float yaw)
{
    float delta_t = (float)dt / 1000;
    float dyaw = cmd_vel_in.angular_z * delta_t;
    float dx = cmd_vel_in.linear_x * delta_t;
    float dy = cmd_vel_in.linear_y * delta_t;
    odom_in.yaw = yaw;
    odom_in.y += dx * sinf(odom_in.yaw) + dy * cosf(odom_in.yaw);
    odom_in.x += dx * cosf(odom_in.yaw) - dy * sinf(odom_in.yaw);
}
void Kinematic_t::ClearOdometry()
{
    current_odom.x = 0;
    current_odom.y = 0;
    current_odom.yaw = 0;
} 
/**
 * @brief 更新里程计
 * 
 * @param odom_in 
 */
void Kinematic_t::update_odom(odom_t &odom_in)
{
    current_odom = odom_in;
}
