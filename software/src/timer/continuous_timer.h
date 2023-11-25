#pragma once

#include <stdint.h>
#include "tmr.h"
#include "timer_interface.h"

namespace timer
{

class ContinuousTimer : public TimerInterface
{
    static ContinuousTimer* instance;
    static uint32_t lock;

    bool started[3];
    std::function<void()> callbacks[3];

protected:
    ContinuousTimer();
    virtual ~ContinuousTimer();

public:
    ContinuousTimer(ContinuousTimer& other) = delete;
    void operator=(const ContinuousTimer& other) = delete;

    static ContinuousTimer* get_instance();

    virtual int start_timer(Timers timer, uint32_t frequency, std::function<void()> callback) override;
    virtual int stop_timer(Timers timer) override;
    virtual bool is_started(Timers timer) const override;

    std::function<void()> get_callback(Timers timer) const;
};

} // namespace timer
