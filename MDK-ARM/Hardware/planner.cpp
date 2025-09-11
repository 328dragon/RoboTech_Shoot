#include "planner.h"
using namespace Planner;
void Planner_t::update(uint16_t dt)
{
    _current_t += dt;
    float t = (float)_current_t / 1000.0;
    if (_promise.isResolved() == false)
    {
        switch (_controlmode)
        {
        case ControlMode_t::OpenControl:
        { // _current_t += dt;
            // float t = (float)_current_t / 1000.0;
            _controller->set_vel_target({_spline[0].dx(t), _spline[1].dx(t), _spline[2].dx(t)}, true);
            if (t >= _target_t)
            {
                _promise.resolve();
            }
            break;
        }
        case ControlMode_t::CloseControl:
        {
            t > _target_t ? t = _target_t : t = t;
            // 求的是增量要加初始值
            _controller->SetClosePosition({_spline[0](t) + start_odom.x, _spline[1](t) + start_odom.y, _spline[2](t) + start_odom.yaw}, _controller->kinematic->_odom_error, false);
            Kinematic::odom_t &error = _controller->kinematic->_odom_error;
            Kinematic::odom_t &current = _controller->kinematic->current_odom;
            if (abs(_target_odom.x - current.x) < error.x && abs(_target_odom.y - current.y) < error.y && abs(_target_odom.yaw - current.yaw) < error.yaw)
            {
                _promise.resolve();
            }
            break;
        }
        default:
            break;
        }
    }
}
SimpleStatus_t &Planner_t::LoactaionCloseControl(const odom_t &target_odom, float max_v, const odom_t &target_error, bool clearodom)
{
    LoactaionOpenControl(target_odom, max_v, {0, 0, 0}, clearodom);
    _controller->kinematic->_odom_error = target_error;
    _controlmode = ControlMode_t::CloseControl;
    _target_odom = target_odom;
    start_odom = _controller->kinematic->current_odom;
    return _promise;
}
SimpleStatus_t &Planner_t::LoactaionOpenControl(const odom_t &target_odom, float max_v, const cmd_vel_t &target_vel, bool clearodom)
{
    _promise.start();
    float targetx, targety, targetyaw;

    if (!clearodom)
    {
        targetx = target_odom.x - _controller->kinematic->current_odom.x;
        targety = target_odom.y - _controller->kinematic->current_odom.y;
        targetyaw = target_odom.yaw - _controller->kinematic->current_odom.yaw;
    }
    else
    {
        targetx = target_odom.x;
        targety = target_odom.y;
        targetyaw = target_odom.yaw;
        _controller->kinematic->ClearOdometry();
    }
    float target_t = Math::Sqrt(targetx * targetx + targety * targety) / (max_v * 0.5);
    _spline[0] = Math::CubicSpline({0, 0}, {target_t, targetx}, {0, target_vel.linear_x});
    _spline[1] = Math::CubicSpline({0, 0}, {target_t, targety}, {0, target_vel.linear_y});
    _spline[2] = Math::CubicSpline({0, 0}, {target_t, targetyaw}, {0, target_vel.angular_z});
    _target_t = target_t;
    _current_t = 0;

    _controlmode = ControlMode_t::OpenControl;
    return _promise;
}