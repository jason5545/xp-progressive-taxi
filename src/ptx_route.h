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

/*
 * The taxi route datarefs (ptaxi/route/...) that Safety Pilot follows:
 * registration, publishing and withdrawing, and the per-frame progress
 * values. See design/taxi-route-interface.md.
 */

#ifndef	_PTX_ROUTE_H_
#define	_PTX_ROUTE_H_

#include <stdbool.h>

#include "planner/ptx_path.h"

#ifdef	__cplusplus
extern "C" {
#endif

typedef struct {
	double	nose_fwd;	/* nose wheel ahead of the aircraft origin, m */
	double	main_fwd;	/* main gear ahead of the origin (negative) */
	double	wheelbase;
	double	max_steer;	/* deg, BetterPushback's rule */
} ptx_acf_gear_t;

/* Reads the gear geometry from X-Plane's datarefs like bp.c does. */
bool ptx_acf_gear(ptx_acf_gear_t *gear);

void ptx_route_init(void);	/* registers the datarefs */
void ptx_route_fini(void);

/* Takes ownership of path and publishes it now. */
void ptx_route_publish(ptx_path_t *path, const char *why);
/*
 * Takes ownership of path; withdraws the current route and publishes path
 * once the pushback that's being started has finished and the tug has
 * gone, if the aircraft ended up at the path's start.
 */
void ptx_route_publish_after_pushback(ptx_path_t *path, const char *why);
void ptx_route_clear(const char *why);

#ifdef	__cplusplus
}
#endif

#endif	/* _PTX_ROUTE_H_ */
