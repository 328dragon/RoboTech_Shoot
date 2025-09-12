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
        void setzeroyaw()
        {
            yaw_zero = yaw_raw; // 设置当前yaw为零点
        }
        float getyaw()
        {
                  yaw= yaw_raw - yaw_zero; // 减去零点偏移
            if (yaw < -3.1415926f)
            {
                yaw += 2 * 3.1415926f;
            }
            else if (yaw > 3.1415926f)
            {
                yaw -= 2 * 3.1415926f;
            }
            return yaw;
        }
        void update()
        {
            uint8_t *data = buffer;
             uint8_t data_length = data[2];
            if (data_length == 76) // HI91浮点型数据输入
            {
                memcpy(acc, &data[18], 12);
                memcpy(gyr, &data[30], 12);
                memcpy(mag, &data[42], 12);
                memcpy(imu, &data[54], 12);
                memcpy(quat, &data[66], 16);
            }
            // 处理yaw线转化成弧度
            yaw_raw= imu[2] * 3.1415926 / 180.0f;
            // 进行归一化
            if(yaw_raw<-3.1415926f)
            {
                yaw_raw += 2 * 3.1415926f;
            }
            else if(yaw_raw>3.1415926f)
            {
                yaw_raw -= 2 * 3.1415926f;
            }
            getYaw();
        }
        ImuEular_t Eular;
        float zero_yaw;
    private:
    };

}
#endif