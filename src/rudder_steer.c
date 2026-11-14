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

#include <math.h>
#include <stddef.h>
#include <string.h>

#include <XPLMDataAccess.h>
#include <XPLMProcessing.h>

#include <acfutils/dr.h>
#include <acfutils/helpers.h>
#include <acfutils/log.h>

#include "bp.h"
#include "rudder_steer.h"
#include "xplane.h"

/*
 * FlightFactor 777 v2, measured 11/14 at RCSS (design/ff777-nosewheel.md):
 * yoke_heading_ratio 0.1 -> 7.5 deg, 0.25 -> 19.0, 0.5 -> 38.0,
 * 0.75 -> 57.4, saturating at about +-68 deg; the nosewheel moves at about
 * 35 deg/s. Positive = right, same sign as tire_steer_cmd.
 */
#define	FF777_DEG_PER_RATIO	76.0
/* nosewheel more than this away from what we asked = lagging */
#define	LAG_DEG			10.0

static struct {
	bool_t		inited;
	dr_t		ov, yaw, act, author, icao;

	bool_t		enabled;	/* this pushback steers by rudder */
	bool_t		holding;	/* we currently own the rudder */
	int		ov0;		/* override_joystick_heading before */
	double		deg;		/* last commanded nosewheel angle */
	XPLMFlightLoopID floop;

	/* per-pushback statistics, logged when the rudder is given back */
	long		frames, lag_frames;
	double		max_lag, max_deg;
} rs;

static bool_t
drs_init(void)
{
	if (rs.inited)
		return (B_TRUE);
	if (!dr_find(&rs.ov, "sim/operation/override/override_joystick_heading") ||
	    !dr_find(&rs.yaw, "sim/joystick/yoke_heading_ratio") ||
	    !dr_find(&rs.act, "sim/flightmodel2/gear/tire_steer_actual_deg") ||
	    !dr_find(&rs.author, "sim/aircraft/view/acf_author") ||
	    !dr_find(&rs.icao, "sim/aircraft/view/acf_ICAO"))
		return (B_FALSE);
	rs.inited = B_TRUE;
	return (B_TRUE);
}

/* FlightFactor 777 (all variants: B772, B77L, B77W, ...) */
static bool_t
is_ff777(void)
{
	char author[256] = { 0 }, icao[16] = { 0 };

	dr_gets(&rs.author, author, sizeof (author));
	dr_gets(&rs.icao, icao, sizeof (icao));
	return (strstr(author, "FlightFactor") != NULL &&
	    strncmp(icao, "B77", 3) == 0);
}

static void
take(void)
{
	if (rs.holding)
		return;
	rs.ov0 = dr_geti(&rs.ov);
	dr_seti(&rs.ov, 1);
	rs.holding = B_TRUE;
	logMsg(BP_INFO_LOG "rudder_steer: tug connected, steering the "
	    "nosewheel through the rudder input (override_joystick_heading "
	    "was %d)", rs.ov0);
}

static void
give_back(void)
{
	if (!rs.holding)
		return;
	dr_setf(&rs.yaw, 0);
	dr_seti(&rs.ov, rs.ov0);
	rs.holding = B_FALSE;
	logMsg(BP_INFO_LOG "rudder_steer: rudder given back; %ld frames, "
	    "max nosewheel command %.1f deg, nosewheel more than %.0f deg off "
	    "in %ld frames (max %.1f deg off)", rs.frames, rs.max_deg, LAG_DEG,
	    rs.lag_frames, rs.max_lag);
}

static float
rudder_floop(float elapsed1, float elapsed2, int counter, void *refcon)
{
	double ratio, act = NAN;

	UNUSED(elapsed1);
	UNUSED(elapsed2);
	UNUSED(counter);
	UNUSED(refcon);

	if (!rs.enabled || slave_mode)
		return (-1);
	if (!bp_started || !bp_connected) {
		give_back();
		return (-1);
	}
	take();

	ratio = rs.deg / FF777_DEG_PER_RATIO;
	ratio = MIN(MAX(ratio, -1.0), 1.0);
	dr_setf(&rs.yaw, ratio);

	if (dr_getvf(&rs.act, &act, bp.acf.nw_i, 1) == 1 && isfinite(act)) {
		double lag = fabs(act - rs.deg);
		rs.frames++;
		rs.max_deg = MAX(rs.max_deg, fabs(rs.deg));
		if (lag > LAG_DEG)
			rs.lag_frames++;
		rs.max_lag = MAX(rs.max_lag, lag);
	}
	return (-1);
}

void
rudder_steer_note(double deg)
{
	if (isfinite(deg))
		rs.deg = deg;
}

void
rudder_steer_start(void)
{
	XPLMCreateFlightLoop_t floop = {
		.structSize = sizeof (XPLMCreateFlightLoop_t),
		.phase = xplm_FlightLoop_Phase_BeforeFlightModel,
		.callbackFunc = rudder_floop,
		.refcon = NULL
	};

	rudder_steer_stop();
	if (slave_mode || !drs_init() || !is_ff777())
		return;
	rs.enabled = B_TRUE;
	rs.holding = B_FALSE;
	rs.deg = 0;
	rs.frames = rs.lag_frames = 0;
	rs.max_lag = rs.max_deg = 0;
	rs.floop = XPLMCreateFlightLoop(&floop);
	XPLMScheduleFlightLoop(rs.floop, -1, 1);
	logMsg(BP_INFO_LOG "rudder_steer: FlightFactor 777, the nosewheel "
	    "will be steered through the rudder input while the tug is "
	    "connected (%.0f deg per 1.0)", FF777_DEG_PER_RATIO);
}

void
rudder_steer_stop(void)
{
	give_back();
	rs.enabled = B_FALSE;
	if (rs.floop != NULL) {
		XPLMDestroyFlightLoop(rs.floop);
		rs.floop = NULL;
	}
}
