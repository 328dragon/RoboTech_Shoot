#ifndef __LIB_PROMISE_H
#define __LIB_PROMISE_H
class Promise_t
{
public:
};
/*只有返回的Promise*/
class SimpleStatus_t
{
public:
    SimpleStatus_t() = default;
    bool isResolved()
    {
        return _isResolved;
    }
    void resolve()
    {
        _isResolved = true;
    }
    void start()
    {
        _isResolved = false;
    }
private:
    bool _isResolved = false;
};
#endif