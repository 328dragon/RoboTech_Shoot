#ifndef __IMU_H
#define __IMU_H
#include "stdint.h"
namespace IMU
{
    union imu_data_t
    {
        uint8_t data[2];
    };
    struct ImuEular_t
    {
        imu_data_t roll;
        imu_data_t pitch;
        imu_data_t yaw;
    };

    class IMU_t
    {
    public:
        uint8_t buffer[100];
        float getralval(uint8_t *data)
        {
            int16_t yaw = (data[1] << 8) | data[0];

            // 将16位整数转换为角度，范围是-180到180度
            float yawAngle = (yaw / 32768.0) * 3.1415926;
            return yawAngle;
        }
        void setzeroyaw()
        {
            zero_yaw=getralval(Eular.yaw.data);
        }
        float getyaw()
        {
            return getralval(Eular.yaw.data)-zero_yaw;
        }
        void update()
        {
            uint8_t *data = buffer;
            if (data[0] == 0x55)
            {
                switch (data[1])
                {
                case 0x53:
                    Eular.roll.data[0] = data[2];
                    Eular.roll.data[1] = data[3];
                    Eular.pitch.data[0] = data[4];
                    Eular.pitch.data[1] = data[5];
                    Eular.yaw.data[0] = data[6];
                    Eular.yaw.data[1] = data[7];
                    break;
                default:
                    break;
                }
            }
        }
        ImuEular_t Eular;
        float zero_yaw;
    private:
    };

}
#endif