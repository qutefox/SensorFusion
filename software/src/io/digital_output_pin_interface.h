#pragma once

namespace io
{

class DigitalOutputPinInterface
{
public:
    DigitalOutputPinInterface() { }
    virtual ~DigitalOutputPinInterface() { }

    virtual int write(bool value) = 0;
    virtual int read(bool& value) = 0;
    virtual int toggle() = 0;
};


} // namespace io
