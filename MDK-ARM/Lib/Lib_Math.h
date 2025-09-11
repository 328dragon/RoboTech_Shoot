#ifndef __LIB_MATH_H
#define __LIB_MATH_H
#include "stdint.h"
namespace Math
{
    float Sqrt(float x);
    float pow(float x, uint8_t n);
    struct Point
    {
        double x;
        double y;
    };
    class Polynomial3_t
    {
    public:
        Polynomial3_t() = default;
        Polynomial3_t(float a, float b, float c, float d)
        {
            data.a = a;
            data.b = b;
            data.c = c;
            data.d = d;
        }
        Polynomial3_t(float *data)
        {
            this->data.a = data[0];
            this->data.b = data[1];
            this->data.c = data[2];
            this->data.d = data[3];
        }
        float operator()(float x)
        {
            return data.a * x * x * x + data.b * x * x + data.c * x + data.d;
        }
        float d1_x(float x)
        {
            return 3 * data.a * x * x + 2 * data.b * x + data.c;
        }
        float d2_x(float x)
        {
            return 6 * data.a * x + 2 * data.b;
        }
        float d3_x(float x)
        {
            return 6 * data.a;
        }

        union Lib_Math_Polynomial3_t_Data_t
        {
            float data[4];
            struct
            {
                float a;
                float b;
                float c;
                float d;
            };
        };

    private:
        Lib_Math_Polynomial3_t_Data_t data;
    };

    class CubicSpline
    {
    public:
        CubicSpline() = default;
        CubicSpline(const Point &p0, const Point &p1, const Point &slope)
        {
            // 两个点
            x0 = p0.x;
            y0 = p0.y;
            x1 = p1.x;
            y1 = p1.y;

            // 斜率
            m0 = slope.x;
            m1 = slope.y;

            // 计算参数
            float h = x1 - x0;
            a = y0;
            b = m0;
            c = (3 * (y1 - y0) / (h * h)) - (m0 + 2 * m1) / h;
            d = (2 * (y0 - y1) / (h * h * h)) + (m0 + m1) / (h * h);
        }

        float operator()(float x)
        {
            if (x < x0 || x > x1)
            {
                return 0;
            }
            float h = x - x0;
            return a + b * h + c * h * h + d * h * h * h;
        }
        float dx(float x)
        {
            if (x < x0 || x > x1)
            {
                return 0;
            }
            float h = x - x0;
             return b + 2 * c * h + 3 * d * h * h;
        }

    private:
        float x0, y0, x1, y1;
        float m0, m1;
        float a, b, c, d;
    };

    float LinearInterpolation(float x, float x1, float x2, float y1, float y2);
    float TernaryInterpolation();
    float CubicSplineInterpolation(float x, float x1, float x2, float y1, float y2, float y1_, float y2_);

}
#endif
