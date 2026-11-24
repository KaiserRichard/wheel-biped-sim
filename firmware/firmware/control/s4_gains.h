/*
 * S4 / H5 Stage 5 Controller Gain Defaults
 *
 * Ported from source-reference
 * and validated S11 runtime defaults (run_g2.py:49-50).
 *
 * Pure C11, zero HAL/RTOS dependencies.
 */

#ifndef WBR_S4_GAINS_H
#define WBR_S4_GAINS_H

#ifndef S4_DT
#define S4_DT 0.002f
#endif
#ifndef S4_SYNC_KP
#define S4_SYNC_KP 15.0f
#endif
#ifndef S4_SYNC_KD
#define S4_SYNC_KD 0.35f
#endif
#ifndef S4_YAW_KP
#define S4_YAW_KP 0.3f
#endif
#ifndef S4_YAW_KI
#define S4_YAW_KI 1.0f
#endif
#ifndef S4_ROLL_KP
#define S4_ROLL_KP 40.0f
#endif
#ifndef S4_ROLL_KD
#define S4_ROLL_KD 5.0f
#endif
#ifndef S4_HEIGHT_KP
#define S4_HEIGHT_KP 800.0f
#endif
#ifndef S4_HEIGHT_KD
#define S4_HEIGHT_KD 50.0f
#endif
#ifndef S4_SAG_GAIN
#define S4_SAG_GAIN 1.0f
#endif

#endif /* WBR_S4_GAINS_H */
