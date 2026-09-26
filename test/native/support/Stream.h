#pragma once

// Нативная замена Stream из Arduino core: Print + чтение.

#include "Print.h"

class Stream : public Print
{
public:
    virtual int available() = 0;
    virtual int read() = 0;
    virtual int peek() = 0;
};
