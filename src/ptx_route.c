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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <XPLMDataAccess.h>
#include <XPLMPlugin.h>
#include <XPLMProcessing.h>

#include <acfutils/dr.h>
#include <acfutils/helpers.h>
#include <acfutils/log.h>

#include "ptx_route.h"
#include "xplane.h"

#define	OFF_ROUTE_DIST	50.0	/* nose this far off the path ... */
#define	OFF_ROUTE_TIME	10.0	/* ... for this long withdraws it */
#define	TAKEOFF_GS	(40.0 / 1.943844)	/* m/s on RWY_DEP */
#define	PB_END_TOL	15.0	/* main gear vs path start after a pushback */
#define	PB_START_WAIT	10.0	/* the pushback must start within this */
#define	MIN_STEER	35.0	/* bp.c MIN_STEER_ANGLE */
#define	NAME_LEN	(PTX_NAME_LEN + 8)

static struct {
	bool		inited;

	/* published values */
	int		version, id, active, n, hs_n;
	double		ref_lat, ref_lon;
	float		x[PTX_PATH_MAX_PTS], y[PTX_PATH_MAX_PTS];
	float		s[PTX_PATH_MAX_PTS];
	int		flags[PTX_PATH_MAX_PTS];
	float		hs_s[PTX_PATH_MAX_HS];
	char		hs_names[256];
	char		rwy[8];
	float		rwy_hdg, prog_s, next_turn_s, next_turn_deg;
	char		cur_name[NAME_LEN], next_name[NAME_LEN];

	dr_t		dr_version, dr_id, dr_active, dr_n, dr_x, dr_y, dr_s;
	dr_t		dr_flags, dr_hs_n, dr_hs_s, dr_hs_names, dr_rwy;
	dr_t		dr_rwy_hdg, dr_prog_s, dr_next_turn_s, dr_next_turn_deg;
	dr_t		dr_cur_name, dr_next_name;
	XPLMDataRef	dr_ref_lat, dr_ref_lon;

	/* the route behind them */
	ptx_path_t	path;
	bool		have_path;
	double		prog;
	double		off_t;

	/* waiting for a pushback to finish */
	ptx_path_t	pending;
	bool		have_pending, pending_started;
	char		pending_why[128];
	double		pending_t;

	ptx_acf_gear_t	gear;
	XPLMFlightLoopID floop;

	/* X-Plane */
	dr_t		lat, lon, psi, gs, onground;
	dr_t		gear_type, gear_deploy, gear_steers, tire_z;
	dr_t		steer1, steer2;
} rt;

/* ---------------------------------------------------------------------- */

bool
ptx_acf_gear(ptx_acf_gear_t *g)
{
	int types[10] = { 0 }, steers[10] = { 0 };
	double deploy[10] = { 0 }, tz[10] = { 0 };
	int n = dr_getvi(&rt.gear_type, types, 0, 10);
	int nw = -1, nm = 0;
	double nw_z = 1e10, main_z = 0, ms;

	(void) dr_getvi(&rt.gear_steers, steers, 0, 10);
	(void) dr_getvf(&rt.gear_deploy, deploy, 0, 10);
	(void) dr_getvf(&rt.tire_z, tz, 0, 10);
	/* bp.c read_gear_info(): wheels only, the most forward steering one */
	for (int i = 0; i < n && i < 10; i++) {
		if (types[i] < 2 || deploy[i] == 0)
			continue;
		if (steers[i] == 1 && tz[i] < nw_z) {
			nw = i;
			nw_z = tz[i];
		}
	}
	if (nw < 0)
		return (false);
	for (int i = 0; i < n && i < 10; i++) {
		if (types[i] < 2 || deploy[i] == 0 || i == nw)
			continue;
		main_z += tz[i];
		nm++;
	}
	if (nm == 0)
		return (false);
	main_z /= nm;
	if (main_z - nw_z <= 0)
		return (false);
	ms = MIN(MAX(dr_getf(&rt.steer1), dr_getf(&rt.steer2)), 75.0);
	if (ms < MIN_STEER)
		ms = (75.0 + MIN_STEER) / 2;
	g->nose_fwd = -nw_z;
	g->main_fwd = -main_z;
	g->wheelbase = main_z - nw_z;
	g->max_steer = ms;
	return (true);
}

/* the aircraft's nose and main gear in the published path's frame */
static void
acf_points(const ptx_path_t *p, vect2_t *nose, vect2_t *main_g)
{
	double k = M_PI / 180 * 6371008.8;
	double lat = dr_getf(&rt.lat), lon = dr_getf(&rt.lon);
	double h = dr_getf(&rt.psi) * (M_PI / 180.0);
	vect2_t o = VECT2((lon - p->ref_lon) * cos(p->ref_lat * (M_PI / 180.0)) *
	    k, (lat - p->ref_lat) * k);
	vect2_t d = VECT2(sin(h), cos(h));

	if (nose != NULL)
		*nose = vect2_add(o, vect2_scmul(d, rt.gear.nose_fwd));
	if (main_g != NULL)
		*main_g = vect2_add(o, vect2_scmul(d, rt.gear.main_fwd));
}

/* ---------------------------------------------------------------------- */

static double
read_ref(void *refcon)
{
	return (*(double *)refcon);
}

static float
read_ref_f(void *refcon)
{
	return (*(double *)refcon);
}

static XPLMDataRef
reg_double(const char *name, double *v)
{
	XPLMDataRef dr = XPLMRegisterDataAccessor(name,
	    xplmType_Double | xplmType_Float, 0, NULL, NULL, read_ref_f, NULL,
	    read_ref, NULL, NULL, NULL, NULL, NULL, NULL, NULL, v, NULL);
	/* list it in DataRefTool / DataRefEditor like libacfutils' dr.c */
	static const char *sig[] = { "com.leecbaker.datareftool",
	    "xplanesdk.examples.DataRefEditor" };
	for (size_t i = 0; i < ARRAY_NUM_ELEM(sig); i++) {
		XPLMPluginID id = XPLMFindPluginBySignature(sig[i]);
		if (id != XPLM_NO_PLUGIN_ID)
			XPLMSendMessageToPlugin(id, 0x01000000, (void *)name);
	}
	return (dr);
}

static void
set_str(char *dst, size_t cap, const char *src)
{
	memset(dst, 0, cap);
	strlcpy(dst, src, cap);
}

static void
update_progress(void)
{
	double nts, ntd;
	const char *cur, *nxt;

	ptx_path_progress(&rt.path, rt.prog, &nts, &ntd, &cur, &nxt);
	rt.prog_s = rt.prog;
	rt.next_turn_s = nts;
	rt.next_turn_deg = ntd;
	if (strcmp(rt.cur_name, cur) != 0)
		set_str(rt.cur_name, sizeof (rt.cur_name), cur);
	if (strcmp(rt.next_name, nxt) != 0)
		set_str(rt.next_name, sizeof (rt.next_name), nxt);
}

static void
log_path(const ptx_path_t *p, const char *what, const char *why)
{
	char names[512], holds[256];
	size_t l = 0;

	names[0] = '\0';
	for (int i = 0; i < p->n_spans && l < sizeof (names); i++) {
		if (p->spans[i].name[0] == '\0')
			continue;
		l += snprintf(names + l, sizeof (names) - l, "%s%s",
		    l ? " > " : "", p->spans[i].name);
	}
	l = 0;
	holds[0] = '\0';
	for (int i = 0; i < p->hs_n && l < sizeof (holds); i++) {
		l += snprintf(holds + l, sizeof (holds) - l, "%s%s at %.0f m",
		    l ? ", " : "", p->hs_name[i], p->hs_s[i]);
	}
	logMsg("taxi route %s #%d (%s): rwy %s hdg %.1f, %d points, %.0f m, "
	    "%d turns (%d MANUAL, %d shrunk, tightest %.1f m); holds: %s; "
	    "via %s", what, rt.id, why, p->rwy, p->rwy_hdg, p->n, p->len,
	    p->n_turns, p->n_manual, p->n_shrunk, p->r_min_used, holds,
	    names);
}

void
ptx_route_publish(ptx_path_t *p, const char *why)
{
	vect2_t nose;

	if (!rt.inited) {
		ptx_path_free(p);
		return;
	}
	if (rt.have_path)
		ptx_path_free(&rt.path);
	rt.path = *p;
	memset(p, 0, sizeof (*p));
	rt.have_path = true;
	(void) ptx_acf_gear(&rt.gear);

	rt.active = 0;
	rt.n = MIN(rt.path.n, PTX_PATH_MAX_PTS);
	for (int i = 0; i < rt.n; i++) {
		rt.x[i] = rt.path.x[i];
		rt.y[i] = rt.path.y[i];
		rt.s[i] = rt.path.s[i];
		rt.flags[i] = rt.path.flags[i];
	}
	rt.hs_n = rt.path.hs_n;
	for (int i = 0; i < PTX_PATH_MAX_HS; i++)
		rt.hs_s[i] = (i < rt.hs_n ? rt.path.hs_s[i] : 0);
	set_str(rt.hs_names, sizeof (rt.hs_names), rt.path.hs_names);
	set_str(rt.rwy, sizeof (rt.rwy), rt.path.rwy);
	rt.rwy_hdg = rt.path.rwy_hdg;
	rt.ref_lat = rt.path.ref_lat;
	rt.ref_lon = rt.path.ref_lon;
	acf_points(&rt.path, &nose, NULL);
	rt.prog = ptx_path_track(&rt.path, nose, 0, NULL);
	rt.off_t = 0;
	update_progress();
	rt.id++;
	rt.active = 1;
	log_path(&rt.path, "published", why);
	XPLMScheduleFlightLoop(rt.floop, -1, 1);
}

void
ptx_route_clear(const char *why)
{
	if (rt.active) {
		rt.active = 0;
		logMsg("taxi route #%d withdrawn: %s", rt.id, why);
	}
}

void
ptx_route_publish_after_pushback(ptx_path_t *p, const char *why)
{
	ptx_route_clear("new pushback");
	if (rt.have_pending)
		ptx_path_free(&rt.pending);
	rt.pending = *p;
	memset(p, 0, sizeof (*p));
	rt.have_pending = true;
	rt.pending_started = false;
	rt.pending_t = 0;
	strlcpy(rt.pending_why, why, sizeof (rt.pending_why));
	XPLMScheduleFlightLoop(rt.floop, -1, 1);
}

static void
pending_check(float dt)
{
	vect2_t main_g;
	double d;

	if (bp_started || bp_connected) {
		rt.pending_started = true;
		return;
	}
	if (!rt.pending_started) {
		rt.pending_t += dt;
		if (rt.pending_t > PB_START_WAIT) {
			logMsg("taxi route: the pushback didn't start; route "
			    "for %s dropped", rt.pending.rwy);
			ptx_path_free(&rt.pending);
			rt.have_pending = false;
		}
		return;
	}
	/* pushback done and the tug gone */
	(void) ptx_acf_gear(&rt.gear);
	acf_points(&rt.pending, NULL, &main_g);
	d = hypot(main_g.x - rt.pending.x[0], main_g.y - rt.pending.y[0]);
	if (d > PB_END_TOL) {
		logMsg("taxi route: pushback ended %.0f m from the planned end "
		    "point (stopped early?); route for %s not published", d,
		    rt.pending.rwy);
		ptx_path_free(&rt.pending);
	} else {
		ptx_route_publish(&rt.pending, rt.pending_why);
	}
	rt.have_pending = false;
}

static float
route_floop(float elapsed1, float elapsed2, int counter, void *refcon)
{
	vect2_t nose;
	double off;
	int i;

	UNUSED(elapsed2);
	UNUSED(counter);
	UNUSED(refcon);
	if (rt.have_pending)
		pending_check(elapsed1);
	if (!rt.active)
		return (rt.have_pending ? -1 : 1.0);

	if (bp_started || bp_connected) {
		ptx_route_clear("pushback in progress");
		return (-1);
	}
	if (dr_geti(&rt.onground) != 1) {
		ptx_route_clear("airborne");
		return (1.0);
	}
	acf_points(&rt.path, &nose, NULL);
	rt.prog = ptx_path_track(&rt.path, nose, rt.prog, &off);
	if (off > OFF_ROUTE_DIST) {
		rt.off_t += elapsed1;
		if (rt.off_t > OFF_ROUTE_TIME) {
			char why[64];
			snprintf(why, sizeof (why), "nose %.0f m off the route "
			    "for %.0f s", off, rt.off_t);
			ptx_route_clear(why);
			return (1.0);
		}
	} else {
		rt.off_t = 0;
	}
	for (i = 0; i + 1 < rt.path.n && rt.path.s[i + 1] <= rt.prog; i++)
		;
	if ((rt.path.flags[i] & PTX_PF_RWY_DEP) &&
	    dr_getf(&rt.gs) > TAKEOFF_GS) {
		ptx_route_clear("take-off roll");
		return (1.0);
	}
	update_progress();
	return (-1);
}

/* ---------------------------------------------------------------------- */

void
ptx_route_init(void)
{
	XPLMCreateFlightLoop_t fl = {
	    .structSize = sizeof (fl),
	    .phase = xplm_FlightLoop_Phase_AfterFlightModel,
	    .callbackFunc = route_floop,
	    .refcon = NULL
	};

	if (rt.inited)
		return;
	memset(&rt, 0, sizeof (rt));
	fdr_find(&rt.lat, "sim/flightmodel/position/latitude");
	fdr_find(&rt.lon, "sim/flightmodel/position/longitude");
	fdr_find(&rt.psi, "sim/flightmodel/position/psi");
	fdr_find(&rt.gs, "sim/flightmodel/position/groundspeed");
	fdr_find(&rt.onground, "sim/flightmodel/failures/onground_any");
	fdr_find(&rt.gear_type, "sim/aircraft/parts/acf_gear_type");
	fdr_find(&rt.gear_deploy, "sim/aircraft/parts/acf_gear_deploy");
	fdr_find(&rt.gear_steers, "sim/aircraft/overflow/acf_gear_steers");
	fdr_find(&rt.tire_z, "sim/flightmodel/parts/tire_z_no_deflection");
	fdr_find(&rt.steer1, "sim/aircraft/gear/acf_nw_steerdeg1");
	fdr_find(&rt.steer2, "sim/aircraft/gear/acf_nw_steerdeg2");

	rt.version = PTX_PATH_VERSION;
	dr_create_i(&rt.dr_version, &rt.version, B_FALSE,
	    "ptaxi/route/version");
	dr_create_i(&rt.dr_id, &rt.id, B_FALSE, "ptaxi/route/id");
	dr_create_i(&rt.dr_active, &rt.active, B_FALSE, "ptaxi/route/active");
	rt.dr_ref_lat = reg_double("ptaxi/route/ref_lat", &rt.ref_lat);
	rt.dr_ref_lon = reg_double("ptaxi/route/ref_lon", &rt.ref_lon);
	dr_create_i(&rt.dr_n, &rt.n, B_FALSE, "ptaxi/route/n");
	dr_create_vf(&rt.dr_x, rt.x, PTX_PATH_MAX_PTS, B_FALSE,
	    "ptaxi/route/x");
	dr_create_vf(&rt.dr_y, rt.y, PTX_PATH_MAX_PTS, B_FALSE,
	    "ptaxi/route/y");
	dr_create_vf(&rt.dr_s, rt.s, PTX_PATH_MAX_PTS, B_FALSE,
	    "ptaxi/route/s");
	dr_create_vi(&rt.dr_flags, rt.flags, PTX_PATH_MAX_PTS, B_FALSE,
	    "ptaxi/route/flags");
	dr_create_i(&rt.dr_hs_n, &rt.hs_n, B_FALSE, "ptaxi/route/hs_n");
	dr_create_vf(&rt.dr_hs_s, rt.hs_s, PTX_PATH_MAX_HS, B_FALSE,
	    "ptaxi/route/hs_s");
	dr_create_b(&rt.dr_hs_names, rt.hs_names, sizeof (rt.hs_names),
	    B_FALSE, "ptaxi/route/hs_names");
	dr_create_b(&rt.dr_rwy, rt.rwy, sizeof (rt.rwy), B_FALSE,
	    "ptaxi/route/rwy");
	dr_create_f(&rt.dr_rwy_hdg, &rt.rwy_hdg, B_FALSE,
	    "ptaxi/route/rwy_hdg");
	dr_create_f(&rt.dr_prog_s, &rt.prog_s, B_FALSE, "ptaxi/route/prog_s");
	rt.next_turn_s = -1;
	dr_create_f(&rt.dr_next_turn_s, &rt.next_turn_s, B_FALSE,
	    "ptaxi/route/next_turn_s");
	dr_create_f(&rt.dr_next_turn_deg, &rt.next_turn_deg, B_FALSE,
	    "ptaxi/route/next_turn_deg");
	dr_create_b(&rt.dr_cur_name, rt.cur_name, sizeof (rt.cur_name),
	    B_FALSE, "ptaxi/route/cur_name");
	dr_create_b(&rt.dr_next_name, rt.next_name, sizeof (rt.next_name),
	    B_FALSE, "ptaxi/route/next_name");

	rt.floop = XPLMCreateFlightLoop(&fl);
	XPLMScheduleFlightLoop(rt.floop, 1.0, 1);
	rt.inited = true;
}

void
ptx_route_fini(void)
{
	if (!rt.inited)
		return;
	ptx_route_clear("plugin disabled");
	XPLMDestroyFlightLoop(rt.floop);
	dr_delete(&rt.dr_version);
	dr_delete(&rt.dr_id);
	dr_delete(&rt.dr_active);
	XPLMUnregisterDataAccessor(rt.dr_ref_lat);
	XPLMUnregisterDataAccessor(rt.dr_ref_lon);
	dr_delete(&rt.dr_n);
	dr_delete(&rt.dr_x);
	dr_delete(&rt.dr_y);
	dr_delete(&rt.dr_s);
	dr_delete(&rt.dr_flags);
	dr_delete(&rt.dr_hs_n);
	dr_delete(&rt.dr_hs_s);
	dr_delete(&rt.dr_hs_names);
	dr_delete(&rt.dr_rwy);
	dr_delete(&rt.dr_rwy_hdg);
	dr_delete(&rt.dr_prog_s);
	dr_delete(&rt.dr_next_turn_s);
	dr_delete(&rt.dr_next_turn_deg);
	dr_delete(&rt.dr_cur_name);
	dr_delete(&rt.dr_next_name);
	if (rt.have_path)
		ptx_path_free(&rt.path);
	if (rt.have_pending)
		ptx_path_free(&rt.pending);
	memset(&rt, 0, sizeof (rt));
}
