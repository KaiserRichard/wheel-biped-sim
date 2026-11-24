/* Synthetic schedule for the C contract. The entries are arithmetic sentinels. */
#define S5_MASS_COUNT 2
#define S5_LENGTH_COUNT 2
#define S5_COEFFICIENTS 12

static const float s5_masses[] = {1.0f, 3.0f};
static const float s5_lengths[] = {0.5f, 1.5f};
static const float s5_standing_trim[2][2][3] = {
    {{0.0f, 3.0f, 0.0f}, {2.0f, 3.0f, 0.0f}},
    {{4.0f, 3.0f, 0.0f}, {6.0f, 3.0f, 4.0f}},
};
static const float s5_gain[2][2][2][12] = {
    {{{[0] = 0.0f, [6] = 2.0f}, {[0] = 2.0f, [6] = 2.0f}},
     {{[0] = 4.0f, [6] = 2.0f}, {[0] = 6.0f, [6] = 2.0f}}},
    {{{[11] = -1.0f}, {[11] = -1.0f}}, {{[11] = -1.0f}, {[11] = -1.0f}}},
};
