#pragma once

#include <stdint.h>
#include "continuous_timer.h"

namespace timer
{

class Counter
{
    static Counter* instance;
    static uint32_t lock;

    timer::ContinuousTimer* continuous_timer;

    volatile uint32_t counter0 = 0;
    volatile uint32_t counter1 = 0;
    volatile uint32_t counter2 = 0;

protected:
    Counter();
    virtual ~Counter();

public:
    Counter(Counter& other) = delete;
    void operator=(const Counter& other) = delete;

    static Counter* get_instance();

    int start_counter0(uint32_t frequency);
    void increment_counter0();
    void stop_counter0();
    uint32_t reset_counter0();

    int start_counter1(uint32_t frequency);
    void increment_counter1();
    void stop_counter1();
    uint32_t reset_counter1();

    int start_counter2(uint32_t frequency);
    void increment_counter2();
    void stop_counter2();
    uint32_t reset_counter2();
};

} // namespace timer
