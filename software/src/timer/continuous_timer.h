#pragma once

#include <stdint.h>
#include "tmr.h"

namespace timer
{

class ContinuousTimer
{
    static ContinuousTimer* instance;
    static uint32_t lock;

    void (*callbacks[MXC_IRQ_COUNT])(void);

protected:
    ContinuousTimer();
    virtual ~ContinuousTimer();

public:
    ContinuousTimer(ContinuousTimer& other) = delete;
    void operator=(const ContinuousTimer& other) = delete;

    static ContinuousTimer* get_instance();

    int set_timer0_interrupt_callback(uint32_t frequency, void (*irq_handler)(void));
    int set_timer1_interrupt_callback(uint32_t frequency, void (*irq_handler)(void));
    int set_timer2_interrupt_callback(uint32_t frequency, void (*irq_handler)(void));

    void run_callback(uint8_t timer_index);
};

} // namespace timer
