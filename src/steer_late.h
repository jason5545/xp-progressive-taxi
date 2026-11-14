/*
 * CDDL HEADER START
 *
 * This file and its contents are supplied under the terms of the
 * Common Development and Distribution License ("CDDL"), version 1.0.
 * You may only use this file in accordance with the terms of version
 * 1.0 of the CDDL.
 *
 * A full copy of the text of the CDDL should have accompanied this
 * source.  A copy of the CDDL is also available via the Internet at
 * http://www.illumos.org/license/CDDL.
 *
 * CDDL HEADER END
 */
/*
 * Copyright 2026 jason5545. All rights reserved.
 */

#ifndef _STEER_LATE_H_
#define _STEER_LATE_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Re-asserting our nosewheel angle against aircraft plugins that keep
 * writing it themselves (FlightFactor 777 v2: tire_steer_cmd read back 0
 * every frame during the 11/14 pushbacks, bp/connected did not help).
 *
 * steer_late_start()/_stop(): pushback begins / ends. Start creates a
 * flight loop that runs every frame before the flight model; it is created
 * at pushback time, long after the aircraft plugin loaded.
 * steer_late_note(deg): bp.c just wrote tire_steer_cmd = deg for the nose
 * gear this frame. We also write the "actual" angle right away, and the
 * late flight loop writes both again in the same frame.
 *
 * Every pushback logs how often the value we wrote had been changed by
 * someone else, and at which point, so a single test push tells which of
 * the two writes (if any) the aircraft honours.
 */
void steer_late_start(void);
void steer_late_stop(void);
void steer_late_note(double deg);

#ifdef __cplusplus
}
#endif

#endif /* _STEER_LATE_H_ */
