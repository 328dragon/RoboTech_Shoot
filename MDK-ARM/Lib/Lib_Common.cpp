#include "Lib_Common.h"
void *operator new(size_t size) throw(std::bad_alloc)
{
    return pvPortMalloc(size);
}

void operator delete(void *pointer) throw()
{
    vPortFree(pointer);
}