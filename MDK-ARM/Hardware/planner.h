#ifndef __PLANNER_H
#define __PLANNER_H
#include "Lib_Math.h"
#include "Lib_pormise.h"
#include "controller.h"
namespace Planner
{
    using namespace Controller;
    enum ControlMode_t
    {
        OpenControl,
        CloseControl,
    };
    class Planner_t
    {
    public:
        Planner_t() = default;
        Planner_t(Controller_t *controller) : _controller(controller)
        {
            _target_odom={0,0,0};
            _spline[0] = Math::CubicSpline();
            _spline[1] = Math::CubicSpline();
            _spline[2] = Math::CubicSpline();

        }
        SimpleStatus_t &LoactaionOpenControl(const odom_t &target_odom, float max_v, const cmd_vel_t &target_vel = {0, 0, 0}, bool clearodom = false);
        SimpleStatus_t &LoactaionCloseControl(const odom_t &target_odom, float max_v, const odom_t &target_error = {0.1, 0.1, 0.2}, bool clearodom = false);
        void update(uint16_t dt);

    private:
        float _target_t;
        uint16_t _current_t;
        Kinematic::odom_t start_odom;
        Kinematic::odom_t _target_odom;
        Math::CubicSpline _spline[3];
        Controller_t *_controller;
        SimpleStatus_t _promise = SimpleStatus_t();
        ControlMode_t _controlmode = ControlMode_t::OpenControl;
    };
}
#endif
