/*
 * @Author: Elaina
 * @Date: 2024-10-17 23:26:43
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-10-19 01:00:30
 * @FilePath: \MDK-ARM\Hardware\controller.cpp
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include "controller.h"
using namespace Controller;
void Controller_t::setMotorTargetSpeed(float *target_speed)
{
    MotorList->Foreach([this, target_speed](Motor::IMotorSpeed_t *motor)
                       { motor->set_speed_target(target_speed[motor->_id]); });
}
void Controller_t::MotorUpdate(uint16_t dt)
{
    MotorList->Foreach([this, dt](Motor::IMotorSpeed_t *motor)
                       { motor->update((void *)&dt);
												current_speed[motor->_id] = motor->get_linear_speed(); });
}
void Controller_t::StatusUpdate(odom_t &odom_in)
{
    if (_ControlMode == ControlMode_t::location_contorl)
    {
        if (_status.isResolved() == false)
        {
            odom_t &target_odom = kinematic->target_odom;
            odom_t &_odom_error = kinematic->_odom_error;
            if (fabs(odom_in.x - target_odom.x) < _odom_error.x && fabs(odom_in.y - target_odom.y) < _odom_error.y && fabs(odom_in.yaw - target_odom.yaw) < _odom_error.yaw)
            {
                // _ControlStatus = finish;
                _status.resolve();
            }
        }
    }
}
/**
 * @brief 设置速度目标(不传入bool默认为自身坐标系)
 * @param {float} linear_x
 * @param {float} linear_y
 * @param {float} angular_z
 * @param {bool} use_ground_control
 * @return {*}
 */
void Controller_t::set_vel_target(cmd_vel_t cmd_vel_in, bool use_ground_control)
{
    kinematic->target_val = cmd_vel_in;
    if (use_ground_control)
    {
        _ControlMode = ControlMode_t::speed_control_groud;
        }
    else
    {
        _ControlMode = ControlMode_t::speed_control_self;
        kinematic->inv(kinematic->target_val, target_speed);
    }
}
/**
 * @brief 控制速度更新,根据当前的里程计与运动状态更新控制量
 * @param {odom_t} *odom_in
 * @return {*}
 * @note:
 */
void Controller_t::control_update(odom_t &odom_in)
{
    switch (_ControlMode)
    {
    case ControlMode_t::location_contorl:
    {
        if(_status.isResolved())
        {
            _ControlMode=ControlMode_t::speed_control_self;
            kinematic->inv({0,0,0}, target_speed);
            return;
        }
        odom_t &target_odom = kinematic->target_odom;
        float vx = pid_x.cal(target_odom.x, odom_in.x)+kinematic->target_val.linear_x;
        float vy = pid_y.cal(target_odom.y, odom_in.y)+kinematic->target_val.linear_y;
        float v_yaw = pid_yaw.cal(target_odom.yaw, odom_in.yaw)+kinematic->target_val.angular_z;

        /* code */
        kinematic->inv({vx, vy, v_yaw}, target_speed, odom_in);
        break;
    }
    // 自身坐标下的速度控制什么都不用弄
    case ControlMode_t::speed_control_self:
    {
        break;
    }
    // 大地坐标下的速度控制
    case ControlMode_t::speed_control_groud:
    {
        cmd_vel_t &target_val = kinematic->target_val;
        kinematic->inv(target_val, target_speed, odom_in);
        break;
    }
    }
}
/**
 * @brief 设置位置闭环目标
 * @param {odom_t} target_odom 目标位置
 * @param {odom_t} target_error 误差
 * @param {bool} privilege 是否清除里程计
 * @return {KinematicState_t} 返回当前的状态
 * @note
 */
SimpleStatus_t &Controller_t::SetClosePosition(const odom_t &target_odom, const odom_t &target_error, bool clearodom)
{
    kinematic->target_odom = target_odom;
    kinematic->_odom_error = target_error;

    if (clearodom)
    {
        kinematic->ClearOdometry();
    }
    _ControlMode = ControlMode_t::location_contorl;
    _status.start();
    return _status;
}

void Controller_t::KinematicAndControlUpdate(uint16_t dt)
{
    kinematic->forward(kinematic->current_vel, current_speed);
    // 先更新里程计
    kinematic->CalculationUpdate(dt, kinematic->current_vel, kinematic->current_odom);
    // 再更新控制量
    control_update(kinematic->current_odom);
    // 更新状态
    StatusUpdate(kinematic->current_odom);
    // 应用控制量
    setMotorTargetSpeed(target_speed);
}
void Controller_t::KinematicAndControlUpdate(uint16_t dt, float yaw)
{
    kinematic->forward(kinematic->current_vel, current_speed);
    // 先更新里程计
    kinematic->CalculationUpdate(dt, kinematic->current_vel, kinematic->current_odom,yaw);
   
    // 再更新控制量
    control_update(kinematic->current_odom);
    // 更新状态
    StatusUpdate(kinematic->current_odom);
    // 应用控制量
    setMotorTargetSpeed(target_speed);
}