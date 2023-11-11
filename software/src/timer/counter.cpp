#include "src/timer/counter.h"

#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_lock.h"

#include "src/timer/continuous_timer.h"

using namespace timer;

Counter* Counter::instance = nullptr;
uint32_t Counter::lock = 0;

Counter::Counter()
    : continuous_timer{ timer::ContinuousTimer::get_instance() }
{
    counter0 = counter1 = counter2 = 0;
}

Counter::~Counter()
{
    counter0 = counter1 = counter2 = 0;
}

Counter* Counter::get_instance()
{
    MXC_GetLock(&lock, 1);
    if (instance == nullptr)
    {
        instance = new Counter();
    }
    MXC_FreeLock(&lock);
    return instance;
}

int Counter::start_counter0(uint32_t frequency)
{
    return continuous_timer->set_timer0_interrupt_callback(frequency,
        []()
        {
            Counter* counter = Counter::get_instance();
            counter->increment_counter0();
        });
}

void Counter::increment_counter0()
{
    ++counter0;
}

void Counter::stop_counter0()
{
    continuous_timer->set_timer0_interrupt_callback(0, nullptr);
}

uint32_t Counter::reset_counter0()
{
    uint32_t ret_val = counter0;
    counter0 = 0;
    return ret_val;
}

int Counter::start_counter1(uint32_t frequency)
{
    return continuous_timer->set_timer1_interrupt_callback(frequency,
        []()
        {
            Counter* counter = Counter::get_instance();
            counter->increment_counter1();
        });
}

void Counter::increment_counter1()
{
    ++counter1;
}

void Counter::stop_counter1()
{
    continuous_timer->set_timer1_interrupt_callback(0, nullptr);
}

uint32_t Counter::reset_counter1()
{
    uint32_t ret_val = counter1;
    counter1 = 0;
    return ret_val;
}

int Counter::start_counter2(uint32_t frequency)
{
    return continuous_timer->set_timer2_interrupt_callback(frequency,
        []()
        {
            Counter* counter = Counter::get_instance();
            counter->increment_counter2();
        });
}

void Counter::increment_counter2()
{
    ++counter2;
}

void Counter::stop_counter2()
{
    continuous_timer->set_timer2_interrupt_callback(0, nullptr);
}

uint32_t Counter::reset_counter2()
{
    uint32_t ret_val = counter2;
    counter2 = 0;
    return ret_val;
}
