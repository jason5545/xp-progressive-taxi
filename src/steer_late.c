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

#include <XPLMDataAccess.h>
#include <XPLMProcessing.h>

#include <acfutils/dr.h>
#include <acfutils/helpers.h>
#include <acfutils/log.h>

#include "bp.h"
#include "steer_late.h"
#include "xplane.h"

/* readback differing from what we wrote by more than this = overwritten */
#define	OVERWRITE_TOL	0.5	/* degrees */
/* nosewheel commands smaller than this aren't interesting for the stats */
#define	STATS_MIN_DEG	3	/* degrees */

static struct {
	bool_t		inited;
	dr_t		cmd, act;
	XPLMFlightLoopID floop;

	double		deg;		/* what bp.c wrote this frame */
	int		cycle;		/* XPLMGetCycleNumber() of that write */
	int		nw_i;

	/* per-pushback statistics, logged by steer_late_stop() */
	long		frames;		/* frames with |deg| >= STATS_MIN_DEG */
	long		lost_act_prev;	/* act changed since last frame's write */
	long		lost_cmd_mid;	/* cmd changed between note and floop */
	long		lost_act_mid;	/* act changed between note and floop */
	bool_t		logged_prev, logged_mid;
	double		last_deg;
	bool_t		have_last;
} sl;

static bool_t
drs_init(void)
{
	if (sl.inited)
		return (B_TRUE);
	if (!dr_find(&sl.cmd, "sim/flightmodel/parts/tire_steer_cmd") ||
	    !dr_find(&sl.act, "sim/flightmodel/parts/tire_steer_act"))
		return (B_FALSE);
	sl.inited = B_TRUE;
	return (B_TRUE);
}

static double
read_gear(dr_t *dr, int i)
{
	double v = NAN;

	if (dr_getvf(dr, &v, i, 1) != 1)
		return (NAN);
	return (v);
}

static bool_t
differs(double got, double want)
{
	return (!isfinite(got) || fabs(got - want) > OVERWRITE_TOL);
}

static void
write_both(int i, double deg)
{
	dr_setvf(&sl.cmd, &deg, i, 1);
	dr_setvf(&sl.act, &deg, i, 1);
}

static float
late_floop(float elapsed1, float elapsed2, int counter, void *refcon)
{
	UNUSED(elapsed1);
	UNUSED(elapsed2);
	UNUSED(counter);
	UNUSED(refcon);

	if (slave_mode || !bp_started || sl.cycle != XPLMGetCycleNumber())
		return (-1);

	if (fabs(sl.deg) >= STATS_MIN_DEG) {
		double cmd = read_gear(&sl.cmd, sl.nw_i);
		double act = read_gear(&sl.act, sl.nw_i);
		bool_t lc = differs(cmd, sl.deg), la = differs(act, sl.deg);

		sl.lost_cmd_mid += lc;
		sl.lost_act_mid += la;
		if ((lc || la) && !sl.logged_mid) {
			logMsg(BP_INFO_LOG "steer_late: nosewheel overwritten "
			    "between our write and our late loop (wrote %.1f, "
			    "cmd now %.1f, act now %.1f); re-asserting every "
			    "frame", sl.deg, cmd, act);
			sl.logged_mid = B_TRUE;
		}
	}
	write_both(sl.nw_i, sl.deg);

	return (-1);
}

void
steer_late_note(double deg)
{
	int nw_i = bp.acf.nw_i;

	if (slave_mode || !drs_init() || !isfinite(deg))
		return;

	if (sl.have_last && fabs(sl.last_deg) >= STATS_MIN_DEG &&
	    nw_i == sl.nw_i) {
		/*
		 * The cmd we wrote at the start of this frame already went
		 * in, so compare the act only for "since last frame", and
		 * the cmd against what our late loop left last frame.
		 */
		double act = read_gear(&sl.act, nw_i);
		bool_t la = differs(act, sl.last_deg);

		sl.frames++;
		sl.lost_act_prev += la;
		if (la && !sl.logged_prev) {
			logMsg(BP_INFO_LOG "steer_late: nosewheel actual angle "
			    "changed after our last write (wrote %.1f last "
			    "frame, act now %.1f): written after the flight "
			    "model or by the flight model itself",
			    sl.last_deg, act);
			sl.logged_prev = B_TRUE;
		}
	}

	sl.deg = deg;
	sl.cycle = XPLMGetCycleNumber();
	sl.nw_i = nw_i;
	sl.last_deg = deg;
	sl.have_last = B_TRUE;
	/* bp.c already wrote the cmd; also write the actual angle now */
	dr_setvf(&sl.act, &deg, nw_i, 1);
}

void
steer_late_start(void)
{
	XPLMCreateFlightLoop_t floop = {
		.structSize = sizeof (XPLMCreateFlightLoop_t),
		.phase = xplm_FlightLoop_Phase_BeforeFlightModel,
		.callbackFunc = late_floop,
		.refcon = NULL
	};

	steer_late_stop();
	if (slave_mode || !drs_init())
		return;
	sl.frames = sl.lost_act_prev = 0;
	sl.lost_cmd_mid = sl.lost_act_mid = 0;
	sl.logged_prev = sl.logged_mid = B_FALSE;
	sl.have_last = B_FALSE;
	sl.cycle = -1;
	/*
	 * Created now, at pushback time, so it is the newest flight loop in
	 * the sim (the aircraft plugin's were created at aircraft load).
	 */
	sl.floop = XPLMCreateFlightLoop(&floop);
	XPLMScheduleFlightLoop(sl.floop, -1, 1);
}

void
steer_late_stop(void)
{
	if (sl.floop == NULL)
		return;
	XPLMDestroyFlightLoop(sl.floop);
	sl.floop = NULL;
	logMsg(BP_INFO_LOG "steer_late: %ld steering frames (|nw| >= %d deg); "
	    "act changed since our previous frame %ld; between our two writes: "
	    "cmd %ld, act %ld", sl.frames, STATS_MIN_DEG, sl.lost_act_prev,
	    sl.lost_cmd_mid, sl.lost_act_mid);
}
