#ifndef __GW_GRASYCALSE_H
#define  __GW_GRASYCALSE_H

#include "main.h"
#include "pid_template.h"
 namespace GW_grasycalse
{
//////////中心：3、4////////
//0||1||2||3||4||5||6||7
enum Gw_GrayscaleMode_t
{
    GrasyCross,
    GrasyOnLine,
    OutLine,
};
class Gw_Grayscale_t
{
public:

    void read_data(void)
    {


    }
    float ReturnXControl(void)
    {

        return control;
    }
    float ReturnCotorl(void)
    {

        return control;
    }
    bool IsCurrentMode(Gw_GrayscaleMode_t mode)
    {
        read_data();
        switch (mode)
        {
        case GrasyCross:
            return data[2] && data[3] && data[4] && data[5];
            break;
        case GrasyOnLine:
            return data[3] || data[4];
            break;
        case OutLine:
            return data[1] && data[2] && data[3] && data[4] && (data[5]&&data[0])&&data[6];
            break;
        default:
            return false;
            break;
        }
    }

protected:
    float control;
    uint8_t data[8];//有八个光电管
		I2C_HandleTypeDef * _I2c;
    pid_base_template_t<float,float> pid = pid_base_template_t<float,float>({0.1, 0, 0.1, -3, 3});
    pid_base_template_t<float,float> pidx = pid_base_template_t<float,float>({0.025, 0, 0.01, -0.3, 0.3});
};




};

#endif

