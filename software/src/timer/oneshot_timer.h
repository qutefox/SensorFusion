#pragma once

#include <stdint.h>
#include "tmr.h"
#include "timer_interface.h"

namespace timer
{

class OneshotTimer : public TimerInterface
{
    static OneshotTimer* instance;
    static uint32_t lock;

    bool started[3];
    std::function<void()> callbacks[3];

protected:
    OneshotTimer();
    virtual ~OneshotTimer();

public:
    OneshotTimer(OneshotTimer& other) = delete;
    void operator=(const OneshotTimer& other) = delete;

    static OneshotTimer* get_instance();

    virtual int start_timer(Timers timer, uint32_t frequency, std::function<void()> callback) override;
    virtual int stop_timer(Timers timer) override;
    virtual bool is_started(Timers timer) const override;

    std::function<void()> get_callback(Timers timer) const;
    void timer_finished(Timers timer);
};

} // namespace timer
