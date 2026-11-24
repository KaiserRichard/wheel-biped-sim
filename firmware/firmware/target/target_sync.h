#ifndef WBR_TARGET_SYNC_H
#define WBR_TARGET_SYNC_H

#include "target_hw.h"

/* Short, nestable sections only: never mask interrupts around a control tick. */
static inline uint32_t target_irq_save(void)
{
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return saved;
}

static inline void target_irq_restore(uint32_t saved)
{
    __DMB();
    if (saved == 0u) {
        __enable_irq();
    }
}

#endif
