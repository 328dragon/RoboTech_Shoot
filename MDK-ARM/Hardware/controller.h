/**
 * @file controller.h
 * @author Elaina (1463967532@qq.com)
 * @brief 
 * @version 0.1
 * @date 2024-10-21
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef __CONTROLLER_H
#define __CONTROLLER_H
#include "Lib_pormise.h"
#include "Motor.h"
#include "Kinematic.h"
#include "Lib_List.h"
namespace Controller
{

    using namespace Kinematic;
    enum ControlMode_t
    {

        location_contorl,    // 位置闭环
        speed_control_self,  // 自身坐标系速度开环
        speed_control_groud, // 大地坐标系速度开环
    };
    class Controller_t
    {
    public:
        Controller_t() = default;
        Controller_t(List::List_t<Motor::IMotorSpeed_t *> *MotorList, Kinematic::Kinematic_t *kinematic)
        {
            this->MotorList = MotorList;
            this->kinematic = kinematic;
        }
        void KinematicAndControlUpdate(uint16_t dt);
        void KinematicAndControlUpdate(uint16_t dt, float yaw);
        void setMotorTargetSpeed(float *target_speed);
        void MotorUpdate(uint16_t dt);
        void StatusUpdate(odom_t &odom_in);                                         // 更新状态
        void set_vel_target(cmd_vel_t cmd_vel_in, bool use_ground_control = false); // 设置目标速度
        void control_update(odom_t &odom_in);
        SimpleStatus_t &SetClosePosition(const odom_t &target_odom, const odom_t &target_error = {0.1, 0.1, 0.2}, bool clearodom = false);
        Kinematic::Kinematic_t *kinematic;

    private:
        List::List_t<Motor::IMotorSpeed_t *> *MotorList;

        float target_speed[4];
        float current_speed[4];
        ControlMode_t _ControlMode = ControlMode_t::speed_control_self;
        SimpleStatus_t _status = SimpleStatus_t();
        pid_Increment_template_t<float, float> pid_x = pid_Increment_template_t<float, float>({0.3, 1, 0.2, -0.4, 0.4});
        pid_Increment_template_t<float, float> pid_y = pid_Increment_template_t<float, float>({0.3, 1, 0.2, -0.4, 0.4});
        pid_Increment_template_t<float, float> pid_yaw = pid_Increment_template_t<float, float>({0.6, 2, 0.2, -0.6, 0.6});
    };

}
#endif