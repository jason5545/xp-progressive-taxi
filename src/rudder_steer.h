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

#ifndef _RUDDER_STEER_H_
#define _RUDDER_STEER_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Steering the nosewheel through the rudder input for aircraft that drive
 * their own nosewheel and ignore tire_steer_cmd (FlightFactor 777 v2; see
 * design/ff777-nosewheel.md).
 *
 * During a pushback the tug steers and the nosewheel just follows it:
 * turn_nosewheel() in bp.c computes the nosewheel angle from the tug's
 * heading relative to the aircraft and writes it to tire_steer_cmd. For
 * these aircraft we instead hand that same angle to the aircraft as a
 * rudder input (override_joystick_heading + yoke_heading_ratio), and the
 * aircraft's own steering puts the nosewheel there.
 *
 * rudder_steer_start()/_stop(): pushback begins / ends (bp_start(),
 * bp_complete(), bp_fini()). The rudder is only taken while the tug is
 * connected (bp_connected) and given back as soon as it isn't.
 * rudder_steer_note(deg): bp.c just commanded nosewheel angle deg.
 */
void rudder_steer_start(void);
void rudder_steer_stop(void);
void rudder_steer_note(double deg);

#ifdef __cplusplus
}
#endif

#endif /* _RUDDER_STEER_H_ */
