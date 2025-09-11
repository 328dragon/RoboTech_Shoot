/*
 * @Author: Elaina
 * @Date: 2024-10-17 19:03:01
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-10-17 19:43:30
 * @FilePath: \MDK-ARM\Lib\Lib_Common.h
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#ifndef __LIB_COMMON_H
#define __LIB_COMMON_H
#include <new>
#include "FreeRTOS.h"
extern "C"
{
    void Error_Handler();
}
void *operator new(std::size_t size) throw(std::bad_alloc);
void operator delete(void *pointer) throw();
template <typename T>
class CustomAllocator
{
public:
    using value_type = T;
    CustomAllocator() = default;
    template <typename U>
    CustomAllocator(const CustomAllocator<U> &) noexcept {}
    T *allocate(std::size_t n)
    {

        if (n > std::size_t(-1) / sizeof(T))
        {
            Error_Handler();
        }
        if (auto p = (T *)(pvPortMalloc(n * sizeof(T))))
        {
            return p;
        }
        else
        {
            Error_Handler();
        }
    }
    void deallocate(T *p, std::size_t)
    {
        vPortFree(p);
    }
};
#endif