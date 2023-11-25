#include "src/timer/oneshot_timer.h"

#include "mxc_device.h"
#include "mxc_errors.h"
#include "nvic_table.h"
#include "mxc_lock.h"
#include "tmr.h"

using namespace timer;

OneshotTimer* OneshotTimer::instance = nullptr;
uint32_t OneshotTimer::lock = 0;

OneshotTimer::OneshotTimer()
{
    for (uint8_t i = 0 ; i < 3 ; ++i)
    {
        started[i] = false;
        callbacks[i] = [](void) -> void { };
    }
}

OneshotTimer::~OneshotTimer()
{

}

OneshotTimer* OneshotTimer::get_instance()
{
    MXC_GetLock(&lock, 1);
    if (instance == nullptr)
    {
        instance = new OneshotTimer();
    }
    MXC_FreeLock(&lock);
    return instance;
}

std::function<void()> OneshotTimer::get_callback(Timers timer) const
{
    uint8_t timer_index = static_cast<uint8_t>(timer);
    return callbacks[timer_index];
}

int OneshotTimer::start_timer(Timers timer, uint32_t frequency, std::function<void()> callback)
{
    uint8_t timer_index = static_cast<uint8_t>(timer);
    callbacks[timer_index] = callback;
    if (timer == Timers::TIMER0)
    {
        MXC_NVIC_SetVector(MXC_TMR_GET_IRQ(timer_index),
            []() -> void
            {
                auto instance = OneshotTimer::get_instance();
                instance->get_callback(Timers::TIMER0)();
                instance->timer_finished(Timers::TIMER0);
                MXC_TMR_ClearFlags(MXC_TMR_GET_TMR(0));
            });
    }
    else if (timer == Timers::TIMER1)
    {
        MXC_NVIC_SetVector(MXC_TMR_GET_IRQ(timer_index),
            []() -> void
            {
                auto instance = OneshotTimer::get_instance();
                instance->get_callback(Timers::TIMER1)();
                instance->timer_finished(Timers::TIMER1);
                MXC_TMR_ClearFlags(MXC_TMR_GET_TMR(1));
            });
    }
    else if (timer == Timers::TIMER2)
    {
        MXC_NVIC_SetVector(MXC_TMR_GET_IRQ(timer_index),
            []() -> void
            {
                auto instance = OneshotTimer::get_instance();
                instance->get_callback(Timers::TIMER2)();
                instance->timer_finished(Timers::TIMER2);
                MXC_TMR_ClearFlags(MXC_TMR_GET_TMR(2));
            });
    }

    NVIC_EnableIRQ(MXC_TMR_GET_IRQ(timer_index));
    uint32_t periodTicks = MXC_TMR_GetPeriod(MXC_TMR_GET_TMR(timer_index), 1, frequency);
    MXC_TMR_Shutdown(MXC_TMR_GET_TMR(timer_index));
    mxc_tmr_cfg_t tmr;
    tmr.pres = TMR_PRES_1;
    tmr.mode = TMR_MODE_ONESHOT;
    tmr.cmp_cnt = periodTicks;
    tmr.pol = 0;
    int err = MXC_TMR_Init(MXC_TMR_GET_TMR(timer_index), &tmr);
    MXC_TMR_Start(MXC_TMR_GET_TMR(timer_index));
    if (err == E_NO_ERROR) started[timer_index] = true;
    return err;
}

int OneshotTimer::stop_timer(Timers timer)
{
    uint8_t timer_index = static_cast<uint8_t>(timer);
    MXC_TMR_Shutdown(MXC_TMR_GET_TMR(timer_index));
    MXC_NVIC_SetVector(MXC_TMR_GET_IRQ(timer_index), nullptr);
    NVIC_DisableIRQ(MXC_TMR_GET_IRQ(timer_index));
    callbacks[timer_index] = [](void) -> void { };
    started[timer_index] = false;
    return E_NO_ERROR;
}

bool OneshotTimer::is_started(Timers timer) const
{
    uint8_t timer_index = static_cast<uint8_t>(timer);
    return started[timer_index];
}

void OneshotTimer::timer_finished(Timers timer)
{
    uint8_t timer_index = static_cast<uint8_t>(timer);
    started[timer_index] = false;
}