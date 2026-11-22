#include <assert.h>

#include "s8_stop_position.h"

int main(void)
{
    float error = 0.5f;
    bool latched = false;
    const float period = 0.25f, limit = 1.0f;

    assert(s8_stop_position(&error, &latched, true, 2.0f, period, limit) == 0.0f);
    assert(latched);
    assert(s8_stop_position(&error, &latched, true, 2.0f, period, limit) == 0.5f);
    assert(s8_stop_position(&error, &latched, true, 4.0f, period, limit) == limit);
    assert(s8_stop_position(&error, &latched, true, -12.0f, period, limit) == -limit);
    assert(s8_stop_position(&error, &latched, false, 4.0f, period, limit) == 0.0f);
    assert(!latched);
    assert(s8_stop_position(&error, &latched, true, -4.0f, period, limit) == 0.0f);
    assert(latched);
    return 0;
}
