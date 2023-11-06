#include "src/timer/continuous_timer.h"

#include "mxc_device.h"
#include "mxc_errors.h"
#include "nvic_table.h"
#include "mxc_lock.h"
#include "tmr.h"

using namespace timer;

ContinuousTimer* ContinuousTimer::instance = nullptr;
uint32_t ContinuousTimer::lock = 0;

ContinuousTimer::ContinuousTimer()
{
    callbacks[0] = nullptr;
    callbacks[1] = nullptr;
    callbacks[2] = nullptr;
}

ContinuousTimer::~ContinuousTimer()
{
    callbacks[0] = nullptr;
    callbacks[1] = nullptr;
    callbacks[2] = nullptr;
}

ContinuousTimer* ContinuousTimer::get_instance()
{
    MXC_GetLock(&lock, 1);
    if (instance == nullptr)
    {
        instance = new ContinuousTimer();
    }
    MXC_FreeLock(&lock);
    return instance;
}

void ContinuousTimer::run_callback(uint8_t timer_index)
{
    if (timer_index > 2) return;
    if (callbacks[timer_index] == nullptr) return;
    callbacks[timer_index]();
}

int ContinuousTimer::set_timer0_interrupt_callback(uint32_t frequency, void (*irq_handler)(void))
{
    callbacks[0] = irq_handler;

    if (irq_handler == nullptr)
    {
        MXC_TMR_Shutdown(MXC_TMR0);
        MXC_NVIC_SetVector(TMR0_IRQn, nullptr);
        NVIC_DisableIRQ(TMR0_IRQn);
        return E_NO_ERROR;
    }

    MXC_NVIC_SetVector(TMR0_IRQn,
        []() -> void
        {
            MXC_TMR_ClearFlags(MXC_TMR0);
            ContinuousTimer* continuous_timer = ContinuousTimer::get_instance();
            continuous_timer->run_callback(0);
        });
    NVIC_EnableIRQ(TMR0_IRQn);
    uint32_t periodTicks = MXC_TMR_GetPeriod(MXC_TMR0, 1, frequency);
    MXC_TMR_Shutdown(MXC_TMR0);
    mxc_tmr_cfg_t tmr;
    tmr.pres = TMR_PRES_1;
    tmr.mode = TMR_MODE_CONTINUOUS;
    tmr.cmp_cnt = periodTicks;
    tmr.pol = 0;
    int err = MXC_TMR_Init(MXC_TMR0, &tmr);
    MXC_TMR_Start(MXC_TMR0);
    return err;
}

int ContinuousTimer::set_timer1_interrupt_callback(uint32_t frequency, void (*irq_handler)(void))
{
    callbacks[1] = irq_handler;

    if (irq_handler == nullptr)
    {
        MXC_TMR_Shutdown(MXC_TMR1);
        MXC_NVIC_SetVector(TMR1_IRQn, nullptr);
        NVIC_DisableIRQ(TMR1_IRQn);
        return E_NO_ERROR;
    }

    MXC_NVIC_SetVector(TMR1_IRQn,
        []() -> void
        {
            MXC_TMR_ClearFlags(MXC_TMR1);
            ContinuousTimer* continuous_timer = ContinuousTimer::get_instance();
            continuous_timer->run_callback(1);
        });
    NVIC_EnableIRQ(TMR1_IRQn);
    uint32_t periodTicks = MXC_TMR_GetPeriod(MXC_TMR1, 1, frequency);
    MXC_TMR_Shutdown(MXC_TMR1);
    mxc_tmr_cfg_t tmr;
    tmr.pres = TMR_PRES_1;
    tmr.mode = TMR_MODE_CONTINUOUS;
    tmr.cmp_cnt = periodTicks;
    tmr.pol = 0;
    int err = MXC_TMR_Init(MXC_TMR1, &tmr);
    MXC_TMR_Start(MXC_TMR1);
    return err;
}

int ContinuousTimer::set_timer2_interrupt_callback(uint32_t frequency, void (*irq_handler)(void))
{
    callbacks[2] = irq_handler;

    if (irq_handler == nullptr)
    {
        MXC_TMR_Shutdown(MXC_TMR2);
        MXC_NVIC_SetVector(TMR2_IRQn, nullptr);
        NVIC_DisableIRQ(TMR2_IRQn);
        return E_NO_ERROR;
    }

    MXC_NVIC_SetVector(TMR2_IRQn,
        []() -> void
        {
            MXC_TMR_ClearFlags(MXC_TMR2);
            ContinuousTimer* continuous_timer = ContinuousTimer::get_instance();
            continuous_timer->run_callback(2);
        });
    NVIC_EnableIRQ(TMR2_IRQn);
    uint32_t periodTicks = MXC_TMR_GetPeriod(MXC_TMR2, 1, frequency);
    MXC_TMR_Shutdown(MXC_TMR2);
    mxc_tmr_cfg_t tmr;
    tmr.pres = TMR_PRES_1;
    tmr.mode = TMR_MODE_CONTINUOUS;
    tmr.cmp_cnt = periodTicks;
    tmr.pol = 0;
    int err = MXC_TMR_Init(MXC_TMR2, &tmr);
    MXC_TMR_Start(MXC_TMR2);
    return err;
}
