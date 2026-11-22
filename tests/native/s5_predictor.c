#include <assert.h>

#include "s5_predictor.h"

int main(void)
{
    s5_predictor_t state = {.mass_kg = 2.0f, .length_ref_m = 1.0f};
    const float observation[S5_OBS] = {4.0f};
    const float first[S5_EFFORTS] = {1.0f, 2.0f};
    const float second[S5_EFFORTS] = {3.0f, 4.0f};
    float trim[3], effort[S5_EFFORTS];

    s5_trim(&state, trim);
    assert(trim[0] == 3.0f && trim[1] == 3.0f && trim[2] == 1.0f);

    s5_commit(&state, first, true);
    s5_commit(&state, second, true);
    s5_commit(&state, second, true);
    s5_feedback(&state, 1.0f, observation, effort);
    assert(effort[0] == 18.0f && effort[1] == -2.0f);

    s5_commit(&state, first, false);
    s5_feedback(&state, 2.0f, observation, effort);
    assert(effort[0] == 16.0f && effort[1] == 0.0f);
    return 0;
}
