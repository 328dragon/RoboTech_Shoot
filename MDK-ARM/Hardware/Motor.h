/*
 * @Author: Elaina
 * @Date: 2024-07-07 17:06:10
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-10-18 18:35:15
 * @FilePath: \MDK-ARM\Hardware\Motor.h
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#ifndef __MOTOR_H
#define __MOTOR_H
#define USE_CAN_Motor 0 // 使用CAN电机
#define USE_3508 1
#define USE_CAN_AbsoluteMotor 0  // 使用CAN绝对编码电机
#define USE_SteeringWheelModel 1 // 使用舵轮模块
#define USE_Debug 1              // 使用调试
#define USE_COMMON_Motor 1       // 使用正常电机
#include "pid_template.h"
#include "main.h"
namespace Motor
{
    union data_t
    {
        uint8_t data_raw[4];
        float data_float;
        int32_t data_int;
        // int16_t data_int16;
    };

    enum MotorType_t
    {
        Steering_Motor,
        Drive_Motor,
        Motor3508,
        Motor2006

    };
    /*接口*/
    class IMotorSpeed_t
    {
    public:
        IMotorSpeed_t() = default;
        IMotorSpeed_t(uint8_t id) : _id(id)
        {
        }
        virtual void set_speed_target(float target) = 0;
        virtual void update(void *param) = 0;
        virtual void set_angle_target(float target)
        {
        }
        virtual float get_linear_speed() = 0;

        uint8_t _id;
    };

    class MotorBase_t
    {
    public:
        MotorBase_t()
        {
        }
        MotorBase_t(uint8_t id) : _id(id)
        {
        }
        uint8_t _id;
        float _vel_target;
        float _angle_target;
        data_t _vel_raw;
        data_t _angle_raw;

#if USE_Debug
        float debug = 0; // 给调试用的
#endif
    };
#if USE_CAN_Motor
    // 接口类
    class MotorCanBase_t : public MotorBase_t
    {
    public:
        MotorCanBase_t()
        {
        }
        MotorCanBase_t(CAN_HandleTypeDef *hcan, uint8_t id) : MotorBase_t(id), _can(hcan)
        {
        }
        void bind_pin(CAN_HandleTypeDef *hcan, uint8_t id)
        {
            _can = hcan;
            _id = id;
        }

    protected:
        void CanSend(uint8_t *data, uint8_t len, uint32_t id);

    private:
        CAN_HandleTypeDef *_can;
        // uint8_t id;
    };
#if USE_3508
    class Motor3508_t : public MotorCanBase_t, public IMotorSpeed_t
    {
    public:
        Motor3508_t(uint8_t id, bool have_tx_permission = false) : MotorCanBase_t()
        {
            _id = id;
            this->have_tx_permission = have_tx_permission;
        }
        void set_speed_target(float target) override;
        void update() override;
        // float debug;
        int forward = 1; // 正反转
    private:
        uint8_t _common_buffer[8];

        float rev_fator = 33; // 转速系数 从原始数据转化到真实数据的倒数
        bool have_tx_permission = false;

        pid_base_template_t<int16_t, float> pid = pid_base_template_t<int16_t, float>({5, 2, 0, -5000, 5000, 2000});
    };

#endif
#if USE_SteeringWheelModel
    /*封装成模块的舵轮分块*/
    class MotorModule_t : public MotorCanBase_t
    {
    public:
        MotorModule_t()
        {
        }
        MotorModule_t(CAN_HandleTypeDef *hcan, uint8_t id) : MotorCanBase_t(hcan, id)
        {
            // _type = Steering_Motor;
        }

        // void set_target(int16_t vel_target, int16_t angle_target);
        void set_target(float vel_target, float angle_target, bool use_youhua = false);
        static float normalize_angle(float angle);

    private:
        float angle_last;
        int8_t forward = 1;
        int16_t vel_int;
        int16_t angle_int;
        int16_t angle_factor = 2608; // 45.51/PI*180
        int16_t vel_factor = 2000;
        float angle_zero = PI / 2;
    };
#endif
#if USE_CAN_AbsoluteMotor
    /*与2006绝对式编码器相关部分*/
    // 2006的基础派生类
    /*与绝对编码相关的部分没写完*/

    class Motor2006_Interface_t
    {
    public:
        Motor2006_Interface_t()
        {
        }
        Motor2006_Interface_t(SPI_HandleTypeDef *spi, GPIO_TypeDef *cs_port, uint16_t cs_pin, int16_t zero_data)
        {
            _spi = spi;
            _cs_port = cs_port;
            _cs_pin = cs_pin;
            absolute_angle_zero = zero_data;
        }
        void angle_update(int16_t relative_angle);
        void angle_update();

    protected:
        int16_t absolute_angle_max = 16383; // 绝对编码器的最大值
        int16_t absolute_angle_raw;         // 绝对编码器的原始值
        int16_t absolute_angle_zero;        // 绝对式编码器原点

        int16_t Turns_num;                 // 转动的圈数
        int16_t Last_relative_angle;       // 上一次的相对角度
        int16_t relative_angle_max = 8192; // 相对编码器的最大值
        float Turns_factor;                // 从圈数转化到角度的系数
        float real_angle;                  // 最后的当前真实角度

    private:
        uint16_t SPI_ReadWriteByte(uint16_t TxData);
        SPI_HandleTypeDef *_spi;
        GPIO_TypeDef *_cs_port;
        uint16_t _cs_pin;
    };
    class Motor2006_t : public Motor_t, public Motor2006_Interface_t
    {
    public:
        Motor2006_t(SPI_HandleTypeDef *spi, GPIO_TypeDef *cs_port, uint16_t cs_pin, int16_t zero_data) : Motor2006_Interface_t(spi, cs_port, cs_pin, zero_data)
        {
            Motor2006_t();
        }
        Motor2006_t()
        {
            _type = Motor2006;
            pid.pid_update(5, 1.4, 0.5); // 6002 pid参数
        }
        void set_angle_target(float target);
        void AngleControlUpdate();

    private:
        float zero_angle;
        float angle_target;
        pid_base_template_t<int16_t, float> angle_pid = pid_base_template_t<int16_t, float>({0.1, 0, 0, -60, 60, 2000});
    };

#endif
#endif
#if USE_COMMON_Motor
    class MotorCommon_t : public MotorBase_t, public IMotorSpeed_t
    {
    public:
        MotorCommon_t()
        {
        }
        MotorCommon_t(TIM_HandleTypeDef *TIM, uint32_t channel, GPIO_TypeDef *PH_Port, uint16_t PH_Pin, TIM_HandleTypeDef *Encoder, int forward = 1, uint8_t id = 0) : MotorBase_t(id), IMotorSpeed_t(id)
        {
            _TIM = TIM;
            _channel = channel;
            _PH_Port = PH_Port;
            _PH_Pin = PH_Pin;
            // _encoder.bindpin(Encoder);
            _Encoder = Encoder;
            _forward = forward;
            HAL_TIM_PWM_Start(_TIM, _channel);
            HAL_TIM_Encoder_Start(_Encoder, TIM_CHANNEL_ALL);
        }
        void update(void *param) override;
        void set_speed_target(float target) override;
        void pwm_out(int pwm);
        float get_linear_speed() override;
        float _Factor = 19;
        float _WheelDiameter = 0.08;
        int _forward = 1;

    private:
        TIM_HandleTypeDef *_TIM;
        uint32_t _channel;
        GPIO_TypeDef *_PH_Port;
        uint16_t _PH_Pin;
        TIM_HandleTypeDef *_Encoder;
        pid_base_template_t<float, int16_t> pid = pid_base_template_t<float, int16_t>({20, 5, 1, -800, 800, 600});
    };
#endif
}
class PwmOut_t
{
public:
    PwmOut_t() = default;
    PwmOut_t(TIM_HandleTypeDef *TIM, uint32_t channel) : _TIM(TIM), _channel(channel)
    {
        HAL_TIM_PWM_Start(_TIM, _channel);
        _ARR = __HAL_TIM_GET_AUTORELOAD(_TIM);
    }
    /**
     * @brief 设置占空比
     *
     */
    void set_duty_cycle(float duty_cycle)
    {
        __HAL_TIM_SET_COMPARE(_TIM, _channel, 1.0*_ARR * duty_cycle/100);
    }
    float debug=0;

private:
    uint32_t _ARR;
    TIM_HandleTypeDef *_TIM;

    uint32_t _channel;
};
#endif
