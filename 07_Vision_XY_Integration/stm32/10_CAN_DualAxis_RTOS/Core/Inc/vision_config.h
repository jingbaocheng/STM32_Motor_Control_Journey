#ifndef __VISION_CONFIG_H
#define __VISION_CONFIG_H

/*
 * Vision MVP configuration
 * ------------------------
 * This file is the ONLY place that should be edited while finding the
 * mechanical camera working position.  Do not tune Home/CAN/RTOS at the
 * same time.
 */

/* Driver enable settling time after boot. */
#define CONTROL_ENABLE_SETTLE_MS          3000U

/*
 * KEY0 start policy:
 * one physical press held for this long starts the one-shot Home sequence.
 * We intentionally use a stable level here instead of an edge/event queue,
 * because KEY0 is only a laboratory start authorization for the Vision MVP.
 */
#define KEY0_START_HOLD_MS                50U

/*
 * Frozen camera-ready machine coordinates for the current optical setup.
 * These values are no longer exploratory tuning candidates.
 */
#define CAMERA_READY_X_DEFAULT            96304LL
#define CAMERA_READY_Y_DEFAULT            36864LL
#define CAMERA_READY_Y_ENABLE_DEFAULT     1U

#define CAMERA_READY_SPEED                150U
#define CAMERA_READY_ACCEL                1U
#define CAMERA_READY_SKIP_TOLERANCE        8LL
#define CAMERA_READY_VERIFY_TOLERANCE      64LL
/* Camera hardware trigger: PB0 idle HIGH, active LOW. */
#define CAMERA_SETTLE_MS                  500U

/* Camera hardware trigger: PB0 idle HIGH, active LOW. */
#define CAMERA_TRIGGER_PULSE_MS            10U

#endif
