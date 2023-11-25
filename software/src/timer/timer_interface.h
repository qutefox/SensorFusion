#pragma once

#include <stdint.h>
#include <functional>

namespace timer
{

enum class Timers : uint8_t
{
    TIMER0 = 0,
    TIMER1 = 1,
    TIMER2 = 2
};

class TimerInterface
{
public:
    virtual int start_timer(Timers timer, uint32_t frequency, std::function<void()> callback) = 0;
    virtual int stop_timer(Timers timer) = 0;
    virtual bool is_started(Timers timer) const = 0;
};

} // namespace timer
