/*
 * @Author: Elaina
 * @Date: 2024-07-07 17:06:10
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-10-18 00:13:01
 * @FilePath: \MDK-ARM\Hardware\Motor.cpp
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include "Motor.h"
using namespace Motor;
#if USE_CAN_Motor
void MotorCanBase_t::CanSend(uint8_t *data, uint8_t len, uint32_t id)
{
    CAN_TxHeaderTypeDef Can_Tx;
    Can_Tx.DLC = len;
    Can_Tx.ExtId = 0x0000;
    Can_Tx.StdId = id;
    Can_Tx.IDE = CAN_ID_STD;
    Can_Tx.RTR = CAN_RTR_DATA;
    Can_Tx.TransmitGlobalTime = DISABLE; // 不传输时间戳
    uint32_t TxMailbox;
    while (HAL_CAN_GetTxMailboxesFreeLevel(_can) == 0)
    {
        /* code */
    }
    HAL_CAN_AddTxMessage(_can, &Can_Tx, data, &TxMailbox);
}
#if USE_3508
void Motor3508_t::set_speed_target(float target)
{
    _vel_target = target * rev_fator * forward;
    pid.target_update(_vel_target);
}
void Motor3508_t::update()
{
    int16_t control = pid.update(_vel_raw.data_int);
    _common_buffer[(_id - 1) % 4 * 2] = (control >> 8) & 0xFF;
    _common_buffer[(_id - 1) % 4 * 2] = (control) & 0xFF;
    if (have_tx_permission)
    {
        uint32_t Canid = 0x200;
        if (_id / 4 == 1)
        {
            Canid = 0x1FF;
        }
        CanSend(_common_buffer, 8, Canid);
    }
}
#endif
#if USE_SteeringWheelModel
// void MotorModule_t::set_target(int16_t vel_target, int16_t angle_target)
// {
//     _vel_target_union.data_int = vel_target;
//     _angle_target_union.data_int = angle_target;
//     uint8_t data[8];
//     data[0] = ((int16_t)_angle_target_union.data_int) >> 8;
//     data[1] = (int16_t)_angle_target_union.data_int;
//     data[2] = ((int16_t)_vel_target_union.data_int) >> 8;
//     data[3] = (int16_t)_vel_target_union.data_int;
//     CanSend(data, 8, _id);
// }
float MotorModule_t::normalize_angle(float angle)
{
    while (angle > PI)
    {
        angle -= 2 * PI;
    }
    while (angle < -PI)
    {
        angle += 2 * PI;
    }
    return angle;
}
void MotorModule_t::set_target(float vel_target, float angle_target, bool use_youhua)
{
    _vel_target = vel_target;
    _angle_target = angle_target - angle_zero;

    if (use_youhua)
    {
        // 算出轮子不反向角度与轮子反向角度
        float delta_theta_forward = normalize_angle(_angle_target - angle_last);
        float delta_theta_reverse = normalize_angle(normalize_angle(_angle_target + PI) - angle_last);
        if (fabs(delta_theta_forward) > fabs(delta_theta_reverse))
        {
            _vel_target = -_vel_target;
            _angle_target = normalize_angle(_angle_target + PI);
        }
        else
        {

            _angle_target = normalize_angle(_angle_target);
        }
    }

    vel_int = _vel_target * vel_factor * forward;
    angle_int = _angle_target * angle_factor;

    uint8_t data[8];
    data[0] = ((int16_t)angle_int) >> 8;
    data[1] = (int16_t)angle_int;
    data[2] = ((int16_t)vel_int) >> 8;
    data[3] = (int16_t)vel_int;
    CanSend(data, 8, _id);
    angle_last = _angle_target;
}
#endif
#if USE_CAN_AbsoluteMotor
/**
 * @brief 用SPI来获得电机的角度数据
 * @return {*}
 * @note:
 */
void Motor2006_Interface_t::angle_update(int16_t relative_angle)
{

    //     int16_t relative_angle_data = *((int16_t *)relative_angle);
}
void Motor2006_Interface_t::angle_update()
{
    uint16_t data;
    SPI_ReadWriteByte(0xFFFF);
    data = SPI_ReadWriteByte(0xFFFF);
    data &= 0x3FFF;
    //     if(data>=absolute_angle_zero)
    //     {
    //         absolute_angle_raw=data-absolute_angle_zero;
    //     }
    //     else
    //     {
    //         absolute_angle_raw=
    //     }
    (data >= absolute_angle_zero) ? absolute_angle_raw = data - absolute_angle_zero : absolute_angle_raw = data - absolute_angle_zero + absolute_angle_max;
}
uint16_t Motor2006_Interface_t::SPI_ReadWriteByte(uint16_t TxData)
{
    uint16_t rx_data;
    HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive(_spi, (uint8_t *)&TxData, (uint8_t *)&rx_data, 1, 1000) != HAL_OK)
    {
        rx_data = 0;
    }
    HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
    return rx_data;
}

void Motor2006_t::set_angle_target(float target)
{
    angle_target = target * angle_fator;
    angle_pid.target_update(angle_target);
}
void Motor2006_t::AngleControlUpdate()
{
    update();
    // 算出当前最近的角度
    int16_t angle_close = (_angle_raw - (int16_t)angle_target) > angle_raw_max / 2 ? _angle_raw - angle_raw_max : _angle_raw;
    int16_t control = angle_pid.update(angle_close);
    Motor_t::set_speed_target(control);

    Motor_t::ControlUpdate();
}

#endif
#endif

#if USE_COMMON_Motor
void MotorCommon_t::set_speed_target(float target)
{
    _vel_target = target * _Factor * _forward / (_WheelDiameter);
    pid.target_update(_vel_target);
}
float MotorCommon_t::get_linear_speed()
{
    return _vel_raw.data_float * (_WheelDiameter) / (_Factor * _forward);
}
void MotorCommon_t::update(void *param)
{
    // 获得编码器的值
    uint16_t dt = *(uint16_t*)(param);
    int delat_tick = _Encoder->Instance->CNT;
    if (delat_tick > 0xefff)
    {
        delat_tick -= 0xffff;
    }
    _Encoder->Instance->CNT = 0;
    _vel_raw.data_float = delat_tick / dt;
    // 更新速度
    int pwm = pid.update((int16_t)_vel_raw.data_float);
    pwm_out(pwm);
}

void MotorCommon_t::pwm_out(int pwm)
{
    int out_speed = pwm;
    if (out_speed > 0)
    {
        __HAL_TIM_SetCompare(_TIM, _channel, out_speed);
        HAL_GPIO_WritePin(_PH_Port, _PH_Pin, GPIO_PIN_SET);
    }
    else
    {
        __HAL_TIM_SetCompare(_TIM, _channel, -out_speed);
        HAL_GPIO_WritePin(_PH_Port, _PH_Pin, GPIO_PIN_RESET);
    }
}
#endif