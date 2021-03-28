#pragma once

#include <exception>

template<class T>
class DebugException : public std::exception
{
public:
    DebugException(T value) : value(value)
    {

    }

    T&& get()
    {
        return value;
    }


private:
    T value;
};
