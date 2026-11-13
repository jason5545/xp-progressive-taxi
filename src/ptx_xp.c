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
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <XPLMGraphics.h>
#include <XPLMMenus.h>
#include <XPLMNavigation.h>
#include <XPLMPlanes.h>
#include <XPLMProcessing.h>
#include <XPLMUtilities.h>

#include <acfutils/assert.h>
#include <acfutils/dr.h>
#include <acfutils/helpers.h>
#include <acfutils/log.h>
#include <acfutils/math.h>
#include <acfutils/thread.h>

#include "acf_outline.h"
#include "bp.h"
#include "cfg.h"
#include "driving.h"
#include "planner/ptx_path.h"
#include "planner/ptx_plan.h"
#include "planner/ptx_scenery.h"
#include "ptx_menu.h"
#include "ptx_route.h"
#include "ptx_ui.h"
#include "ptx_xp.h"
#include "xplane.h"

#define	MAX_RENDS	64
/*
 * A ramp start within this distance of the aircraft's main-gear-to-nose
 * axis is the stand we're parked at. (Which point of the aircraft X-Plane
 * puts on the ramp start isn't documented, so we don't assume one.)
 */
#define	OWN_STAND_DIST	12.0
#define	MS2KT		1.943844

static struct {
	bool		started, enabled;

	/* commands */
	XPLMCommandRef	cmd_open, cmd_next, cmd_prev, cmd_best, cmd_close;
	XPLMCommandRef	cmd_xptug;
	XPLMCommandRef	cmd_taxi, cmd_taxi_clear;

	/* plugins sub-menu */
	XPLMMenuID	root;
	int		sub_item;
	XPLMMenuID	sub;

	/* airport loader (worker thread) */
	mutex_t		lock;
	thread_t	thr;
	bool		thr_active;
	bool		load_done;
	char		load_icao[16];
	ptx_apt_t	*load_apt;
	ptx_scenery_info_t load_info;
	XPLMFlightLoopID floop;

	/* the airport we have */
	char		icao[16];
	ptx_apt_t	*apt;
	ptx_graph_t	*g;

	/* the plan */
	bool		have_plan;
	ptx_result_t	res[MAX_RENDS];
	int		n_res;
	int		order[MAX_RENDS];
	ptx_ui_model_t	model;
	ptx_menu_ctx_t	ctx;
	char		stand_name[64];
	double		plan_hdg;
	bool		full_length, full_length_set;
	bool		start_best_when_ready;
	bool		route_when_ready;

	/* wingspan for the width class without bp_init() */
	char		outline_path[1024];
	acf_outline_t	*outline;

	dr_t		local_x, local_y, local_z, psi, elev, lat, lon;
	dr_t		wind_dir, wind_spd, onground;
} px;

static void plan_and_show(void);
static void start_row(int row);
static void taxi_route(void);

/* ---------------------------------------------------------------------- */

static char
span_class(double span)
{
	if (span <= 0)
		return (0);
	return (span < 15 ? 'A' : span < 24 ? 'B' : span < 36 ? 'C' :
	    span < 52 ? 'D' : span < 65 ? 'E' : 'F');
}

static void
show_message(const char *zh, const char *en)
{
	ptx_menu_ctx_t c = px.ctx;
	c.icao = (px.icao[0] != '\0' ? px.icao : (px.load_icao[0] ?
	    px.load_icao : "----"));
	c.stand_name = NULL;
	c.cjk = ptx_ui_cjk();
	ptx_menu_message(&c, c.cjk ? zh : en, &px.model);
	px.have_plan = false;
	ptx_ui_show(&px.model);
}

static void
rebuild_submenu(void)
{
	if (px.sub == NULL)
		return;
	XPLMClearAllMenuItems(px.sub);
	XPLMAppendMenuItemWithCommand(px.sub, "Open runway menu", px.cmd_open);
	XPLMAppendMenuItemWithCommand(px.sub, "Push to the recommended runway",
	    px.cmd_best);
	XPLMAppendMenuItemWithCommand(px.sub, "Taxi route from here "
	    "(highlighted / recommended runway)", px.cmd_taxi);
	XPLMAppendMenuItemWithCommand(px.sub, "Clear taxi route",
	    px.cmd_taxi_clear);
	if (!px.have_plan || px.g == NULL)
		return;
	XPLMAppendMenuSeparator(px.sub);
	for (int row = 0; row < px.model.n_rows; row++) {
		int ri = px.order[row];
		const ptx_result_t *r = &px.res[ri];
		char buf[160];
		const char *e = ptx_edge_name(px.g, r->edge);
		snprintf(buf, sizeof (buf), "%s%s: nose %03.0f on %s, "
		    "push %.0f m", px.model.rows[row].recommended ?
		    "* " : "  ", ptx_rend_name(px.g, ri), r->end_hdg,
		    e[0] ? e : "apron", r->pb_len);
		XPLMAppendMenuItem(px.sub, buf, (void *)(intptr_t)(row + 1),
		    1);
	}
}

/*
 * Path options for this aircraft. The fillet radii can be overridden in
 * Output/preferences/ProgressiveTaxi.cfg: taxi_route_radius_A .. _F
 * (metres, by the narrower taxiway's width class) and
 * taxi_route_radius_unknown.
 */
static void
path_opts(double wheelbase, double max_steer, ptx_path_opts_t *po)
{
	ptx_path_opts_default(po);
	po->r_min = ptx_path_min_radius(wheelbase, max_steer);
	if (bp_conf == NULL)
		return;
	for (int c = 0; c < 6; c++) {
		char key[32];
		double v;
		snprintf(key, sizeof (key), "taxi_route_radius_%c", 'A' + c);
		if (conf_get_d(bp_conf, key, &v) && v > 0)
			po->r_pref[c] = v;
	}
	{
		double v;
		if (conf_get_d(bp_conf, "taxi_route_radius_unknown", &v) &&
		    v > 0)
			po->r_unknown = v;
	}
}

/* ---------------------------------------------------------------------- */
/* aircraft pose                                                          */
/* ---------------------------------------------------------------------- */

/* main gear and nose gear positions in X-Plane local (x, -z) metres */
static void
acf_pose(vect2_t *main_local, vect2_t *nose_local, double *hdg)
{
	vect2_t pos = VECT2(dr_getf(&px.local_x), -dr_getf(&px.local_z));
	vect2_t dir;

	*hdg = dr_getf(&px.psi);
	dir = hdg2dir(*hdg);
	/* BetterPushback steers by the rear axle: veh_pos2fixed_pos() */
	*main_local = vect2_add(pos, vect2_scmul(dir, bp.veh.fixed_z_off));
	*nose_local = vect2_add(pos, vect2_scmul(dir, -bp.acf.nw_z));
}

static vect2_t
local2plan(vect2_t p)
{
	double lat, lon, alt;
	XPLMLocalToWorld(p.x, dr_getf(&px.local_y), -p.y, &lat, &lon, &alt);
	return (ptx_project(px.g, lat, lon));
}

static vect2_t
plan2local(vect2_t p)
{
	double lat, lon, x, y, z;
	ptx_unproject(px.g, p, &lat, &lon);
	XPLMWorldToLocal(lat, lon, dr_getf(&px.elev), &x, &y, &z);
	return (VECT2(x, -z));
}

/* ---------------------------------------------------------------------- */
/* airport loading                                                        */
/* ---------------------------------------------------------------------- */

static bool
nearest_airport(char icao[16])
{
	float lat = dr_getf(&px.lat), lon = dr_getf(&px.lon);
	char id[32] = "";
	XPLMNavRef ref = XPLMFindNavAid(NULL, NULL, &lat, &lon, NULL,
	    xplm_Nav_Airport);

	if (ref == XPLM_NAV_NOT_FOUND)
		return (false);
	XPLMGetNavAidInfo(ref, NULL, NULL, NULL, NULL, NULL, NULL, id, NULL,
	    NULL);
	strlcpy(icao, id, 16);
	return (icao[0] != '\0');
}

static void
loader_thread(void *arg)
{
	char icao[16];
	char *cachedir;
	ptx_scenery_info_t info;
	ptx_apt_t *apt;

	(void) arg;
	mutex_enter(&px.lock);
	strlcpy(icao, px.load_icao, sizeof (icao));
	mutex_exit(&px.lock);

	cachedir = mkpathname(bp_xpdir, "Output", "caches",
	    "ProgressiveTaxi_aptidx", NULL);
	apt = ptx_scenery_load(bp_xpdir, cachedir, icao, &info);
	free(cachedir);

	mutex_enter(&px.lock);
	px.load_apt = apt;
	px.load_info = info;
	px.load_done = true;
	mutex_exit(&px.lock);
}

static float
loader_floop(float elapsed1, float elapsed2, int counter, void *refcon)
{
	ptx_apt_t *apt;
	ptx_scenery_info_t info;
	bool done;

	(void) elapsed1;
	(void) elapsed2;
	(void) counter;
	(void) refcon;
	mutex_enter(&px.lock);
	done = px.load_done;
	apt = px.load_apt;
	info = px.load_info;
	mutex_exit(&px.lock);
	if (!done)
		return (0.1);

	thread_join(&px.thr);
	px.thr_active = false;
	px.load_done = false;
	px.load_apt = NULL;
	if (apt == NULL) {
		px.route_when_ready = false;
		logMsg("auto pushback: %s not found in any scenery pack",
		    px.load_icao);
		show_message("找不到這座機場的場景資料，請改用手動規劃",
		    "No scenery data for this airport; use the manual planner");
		return (0);
	}
	logMsg("auto pushback: %s from %s (offset %ld, find %.0f ms%s, "
	    "parse %.0f ms): %d runways, %d stands, %d nodes, %d edges",
	    apt->icao, info.apt_path, info.offset, info.ms_find,
	    info.index_built ? " incl. index build" : "", info.ms_load,
	    apt->n_rwys, apt->n_stands, apt->n_nodes, apt->n_edges);
	ptx_results_free(px.res, px.n_res);
	px.n_res = 0;
	ptx_graph_free(px.g);
	ptx_apt_free(px.apt);
	px.apt = apt;
	px.g = ptx_graph_new(apt);
	strlcpy(px.icao, apt->icao, sizeof (px.icao));
	px.full_length_set = false;
	if (px.route_when_ready) {
		px.route_when_ready = false;
		taxi_route();
		return (0);
	}
	plan_and_show();
	return (0);
}

static void
start_loading(const char *icao)
{
	if (px.thr_active)
		return;
	strlcpy(px.load_icao, icao, sizeof (px.load_icao));
	px.load_done = false;
	px.thr_active = true;
	VERIFY(thread_create(&px.thr, loader_thread, NULL));
	XPLMScheduleFlightLoop(px.floop, 0.1, 1);
}

/* ---------------------------------------------------------------------- */
/* planning                                                               */
/* ---------------------------------------------------------------------- */

/*
 * The stand the aircraft sits on: the stand point closest to the main
 * gear - nose axis, within OWN_STAND_DIST. -1 if none.
 */
static int
find_own_stand(vect2_t Pm, vect2_t Pnose)
{
	double best = 1e18;
	int own = -1;

	for (int i = 0; i < px.apt->n_stands; i++) {
		vect2_t s = ptx_project(px.g, px.apt->stands[i].lat,
		    px.apt->stands[i].lon);
		vect2_t ax = vect2_sub(Pnose, Pm);
		double l2 = vect2_dotprod(ax, ax), t = 0, d;
		if (l2 > 0)
			t = MAX(0, MIN(1, vect2_dotprod(vect2_sub(s, Pm), ax) / l2));
		d = vect2_dist(s, vect2_add(Pm, vect2_scmul(ax, t)));
		if (d < best) {
			best = d;
			own = i;
		}
	}
	return (best > OWN_STAND_DIST ? -1 : own);
}

static void
plan_and_show(void)
{
	vect2_t main_l, nose_l, Pm, Pnose, Pn;
	double hdg, d_back, d_front;
	int own = -1;
	const char *type = NULL;
	ptx_aircraft_t ac;
	ptx_opts_t opts;
	ptx_kind_t kind;
	char cls;

	if (px.g == NULL)
		return;
	if (!bp_init()) {
		/* bp_init() already said why (not on ground, moving...) */
		show_message("現在不能後推（原因見語音提示）",
		    "Can't push back right now (see the spoken message)");
		return;
	}
	acf_pose(&main_l, &nose_l, &hdg);
	Pm = local2plan(main_l);
	Pnose = local2plan(nose_l);
	own = find_own_stand(Pm, Pnose);
	if (own >= 0) {
		const ptx_stand_t *st = &px.apt->stands[own];
		Pn = ptx_project(px.g, st->lat, st->lon);
		type = st->type;
		strlcpy(px.stand_name, st->name, sizeof (px.stand_name));
	} else {
		Pn = Pnose;
		px.stand_name[0] = '\0';
	}
	kind = ptx_classify(px.g, Pn, hdg, type, &d_back, &d_front);

	cls = span_class(bp_ls.outline != NULL ?
	    2 * bp_ls.outline->semispan : 0);
	ac.wheelbase = bp.veh.wheelbase;
	ac.max_steer = bp.veh.max_steer;
	ac.route_cls = cls;
	if (!px.full_length_set)
		px.full_length = (cls >= 'E');
	ptx_opts_default(&opts);
	if (px.full_length)
		opts.w_along = PTX_W_ALONG_FULL_LENGTH;

	ptx_results_free(px.res, px.n_res);
	px.n_res = MIN(ptx_n_rends(px.g), MAX_RENDS);
	{
		ptx_result_t *tmp = calloc(ptx_n_rends(px.g) + 1,
		    sizeof (ptx_result_t));
		ptx_plan(px.g, Pm, Pn, hdg, own, &ac, &opts, tmp, NULL);
		memcpy(px.res, tmp, px.n_res * sizeof (ptx_result_t));
		/* anything past MAX_RENDS would leak; airports never have it */
		ptx_results_free(&tmp[px.n_res], ptx_n_rends(px.g) - px.n_res);
		free(tmp);
	}
	px.plan_hdg = hdg;

	memset(&px.ctx, 0, sizeof (px.ctx));
	px.ctx.icao = px.icao;
	px.ctx.stand_name = (own >= 0 ? px.stand_name : NULL);
	px.ctx.kind = kind;
	px.ctx.have_wind = true;
	px.ctx.wind_dir = dr_getf(&px.wind_dir);
	px.ctx.wind_kt = dr_getf(&px.wind_spd) * MS2KT;
	px.ctx.ac_cls = cls;
	px.ctx.full_length = px.full_length;
	px.ctx.cjk = ptx_ui_cjk();
	ptx_menu_build(px.g, px.res, &px.ctx, &px.model, px.order);
	px.have_plan = true;
	logMsg("auto pushback plan at %s stand %s (%s), wheelbase %.2f m, "
	    "steer %.0f deg, class %c, wind %03.0f/%.0f",
	    px.icao, own >= 0 ? px.stand_name : "-", ptx_kind_str(kind),
	    ac.wheelbase, ac.max_steer, cls ? cls : '?', px.ctx.wind_dir,
	    px.ctx.wind_kt);
	for (int row = 0; row < px.model.n_rows; row++) {
		const ptx_result_t *r = &px.res[px.order[row]];
		logMsg("  %-4s %s %s | %s", px.model.rows[row].label,
		    ptx_status_str(r->status), px.model.rows[row].line1,
		    px.model.rows[row].line2);
	}
	ptx_ui_show(&px.model);
	rebuild_submenu();

	if (px.start_best_when_ready) {
		px.start_best_when_ready = false;
		if (px.model.highlight >= 0)
			start_row(px.model.highlight);
	}
}

static void
open_menu(void)
{
	char icao[16];

	if (bp_started) {
		XPLMSpeakString("Pushback already in progress.");
		return;
	}
	/* on ground, stopped, gear down...: bp_init() says what's wrong */
	if (!bp_init())
		return;
	if (!nearest_airport(icao)) {
		show_message("找不到附近的機場", "No airport nearby");
		return;
	}
	if (px.g != NULL && strcmp(px.icao, icao) == 0) {
		plan_and_show();
		return;
	}
	strlcpy(px.load_icao, icao, sizeof (px.load_icao));
	{
		char zh[128], en[128];
		snprintf(zh, sizeof (zh), "正在載入 %s 的機場資料…", icao);
		snprintf(en, sizeof (en), "Loading airport data for %s...",
		    icao);
		show_message(zh, en);
	}
	start_loading(icao);
}

/* ---------------------------------------------------------------------- */
/* doing it                                                               */
/* ---------------------------------------------------------------------- */

static void
start_row(int row)
{
	int ri, n;
	const ptx_result_t *r;
	vect2_t main_l, nose_l, end_l;
	double hdg;
	seg_t *seg;
	bool ok = true;
	char msg[256];

	if (!px.have_plan || row < 0 || row >= px.model.n_rows)
		return;
	ri = px.order[row];
	r = &px.res[ri];
	if (r->status != PTX_OK || !r->pb_allowed) {
		XPLMSpeakString("That runway is not available for an "
		    "automatic pushback.");
		return;
	}
	if (bp_started || !bp_init())
		return;
	acf_pose(&main_l, &nose_l, &hdg);
	end_l = plan2local(r->end);

	/*
	 * The plan was made in the planner's frame; X-Plane's local frame
	 * differs by centimetres, which can tip a borderline case over the
	 * minimum turn radius. If that happens, slide the end point along the
	 * taxiway centreline (it stays on the line) and try again.
	 */
	{
		static const double nudge[] = { 0, 1, -1, 2, -2 };
		vect2_t along = hdg2dir(r->end_hdg);
		for (size_t k = 0; k < ARRAY_NUM_ELEM(nudge); k++) {
			vect2_t e = vect2_add(end_l, vect2_scmul(along,
			    nudge[k]));
			bp_delete_all_segs();
			n = compute_segs(&bp.veh, main_l, hdg, e, r->end_hdg,
			    &bp.segs);
			ok = (n > 0);
			for (seg = list_head(&bp.segs); seg != NULL;
			    seg = list_next(&bp.segs, seg)) {
				seg->have_local_coords = B_TRUE;
				if (!seg->backward)
					ok = false;
			}
			if (ok) {
				if (k > 0) {
					logMsg("auto pushback: end point moved "
					    "%.0f m along the taxiway", nudge[k]);
				}
				end_l = e;
				break;
			}
		}
	}
	if (n <= 0 || !ok) {
		logMsg("auto pushback: compute_segs failed for %s (n=%d)",
		    ptx_rend_name(px.g, ri), n);
		bp_delete_all_segs();
		XPLMSpeakString("Could not build the pushback path. Please "
		    "use the manual planner.");
		return;
	}
	seg = list_tail(&bp.segs);
	seg->user_placed = B_TRUE;

	/* the taxi route goes out once the tug has gone */
	{
		ptx_path_opts_t po;
		ptx_path_t path;
		ptx_path_status_t ps;
		path_opts(bp.veh.wheelbase, bp.veh.max_steer, &po);
		ps = ptx_path_build(px.g, r, ri, &po, &path);
		if (ps == PTX_PATH_OK) {
			ptx_route_publish_after_pushback(&path,
			    "after auto pushback");
		} else {
			logMsg("taxi route: no path for %s (%s)",
			    ptx_rend_name(px.g, ri), ptx_path_status_str(ps));
		}
	}

	ptx_menu_summary(px.g, r, ri, false, msg, sizeof (msg));
	logMsg("auto pushback: %s; %d segments, end %.1f/%.1f hdg %.1f",
	    msg, n, end_l.x, end_l.y, r->end_hdg);
	ptx_ui_hide();
	XPLMSpeakString(msg);
	XPLMCommandOnce(XPLMFindCommand("ProgressiveTaxi/start"));
}

/*
 * X-Plane's own tug can only do "nose 90 left/right" or straight back.
 * Pick the one whose final nose direction is closest to our plan.
 */
static void
start_xp_tug(int row)
{
	const ptx_result_t *r;
	double rel;
	const char *cmd, *what;
	char msg[256];

	if (!px.have_plan || row < 0 || row >= px.model.n_rows)
		return;
	r = &px.res[px.order[row]];
	if (r->status != PTX_OK)
		return;
	rel = rel_hdg(px.plan_hdg, r->end_hdg);
	if (fabs(rel) < 30) {
		cmd = "sim/ground_ops/pushback_straight";
		what = "straight back";
	} else if (rel < 0) {
		cmd = "sim/ground_ops/pushback_left";
		what = "nose left";
	} else {
		cmd = "sim/ground_ops/pushback_right";
		what = "nose right";
	}
	snprintf(msg, sizeof (msg), "X-Plane tug, %s, to face about %03.0f "
	    "for runway %s.", what, r->end_hdg,
	    ptx_rend_name(px.g, px.order[row]));
	logMsg("auto pushback: %s (rel %.0f deg, %s)", msg, rel, cmd);
	ptx_ui_hide();
	XPLMSpeakString(msg);
	XPLMCommandOnce(XPLMFindCommand(cmd));
}

/* ---------------------------------------------------------------------- */
/* taxi route from where we are (no pushback)                             */
/* ---------------------------------------------------------------------- */

static char
acf_span_class(void)
{
	char acf[512], path[1024];

	XPLMGetNthAircraftModel(0, acf, path);
	if (strcmp(path, px.outline_path) != 0) {
		if (px.outline != NULL)
			acf_outline_free(px.outline);
		px.outline = acf_outline_read(path);
		strlcpy(px.outline_path, path, sizeof (px.outline_path));
	}
	return (span_class(px.outline != NULL ? 2 * px.outline->semispan : 0));
}

static void
taxi_route(void)
{
	static ptx_ui_model_t model;
	int order[MAX_RENDS + PTX_UI_MAX_ROWS];
	char icao[16], msg[256];
	ptx_acf_gear_t gear;
	vect2_t pos, dir, Pm, Pn;
	double hdg;
	char cls;
	ptx_aircraft_t ac;
	ptx_opts_t opts;
	ptx_result_t *res;
	ptx_path_opts_t po;
	ptx_path_t path;
	ptx_path_status_t ps;
	int ri = -1, n;
	bool from_menu = false;

	if (bp_started) {
		XPLMSpeakString("Pushback in progress.");
		return;
	}
	if (dr_geti(&px.onground) != 1) {
		XPLMSpeakString("A taxi route needs the aircraft on the "
		    "ground.");
		return;
	}
	if (!nearest_airport(icao)) {
		XPLMSpeakString("No airport nearby.");
		return;
	}
	if (px.g == NULL || strcmp(px.icao, icao) != 0) {
		if (!px.thr_active) {
			px.route_when_ready = true;
			XPLMSpeakString("Loading airport data.");
			start_loading(icao);
		}
		return;
	}
	if (!ptx_acf_gear(&gear)) {
		XPLMSpeakString("Can't read this aircraft's landing gear.");
		return;
	}
	hdg = dr_getf(&px.psi);
	dir = hdg2dir(hdg);
	pos = VECT2(dr_getf(&px.local_x), -dr_getf(&px.local_z));
	Pm = local2plan(vect2_add(pos, vect2_scmul(dir, gear.main_fwd)));
	Pn = local2plan(vect2_add(pos, vect2_scmul(dir, gear.nose_fwd)));
	cls = acf_span_class();
	ac.wheelbase = gear.wheelbase;
	ac.max_steer = gear.max_steer;
	ac.route_cls = cls;
	ptx_opts_default(&opts);
	if (px.full_length_set ? px.full_length : cls >= 'E')
		opts.w_along = PTX_W_ALONG_FULL_LENGTH;
	/* nose-in stand: never "straight ahead", that's the terminal */
	{
		int own = find_own_stand(Pm, Pn);
		if (own >= 0) {
			const ptx_stand_t *st = &px.apt->stands[own];
			double d_back, d_front;
			opts.no_lead = (ptx_classify(px.g,
			    ptx_project(px.g, st->lat, st->lon), hdg, st->type,
			    &d_back, &d_front) == PTX_KIND_PUSHBACK);
			logMsg("taxi route: on stand %s, %s", st->name,
			    opts.no_lead ? "needs a pushback, no straight-ahead "
			    "start" : "straight-ahead start allowed");
		}
	}

	n = ptx_n_rends(px.g);
	res = calloc(n + 1, sizeof (*res));
	ptx_plan_from_pose(px.g, Pm, Pn, hdg, &ac, &opts, res);
	if (ptx_ui_visible() && px.have_plan && px.model.highlight >= 0) {
		ri = px.order[px.model.highlight];
		from_menu = true;
	} else if (n <= MAX_RENDS) {
		/* the menu's recommendation: headwind, then shortest taxi */
		ptx_menu_ctx_t ctx;
		memset(&ctx, 0, sizeof (ctx));
		ctx.icao = px.icao;
		ctx.kind = PTX_KIND_FACES_TAXIWAY;
		ctx.have_wind = true;
		ctx.wind_dir = dr_getf(&px.wind_dir);
		ctx.wind_kt = dr_getf(&px.wind_spd) * MS2KT;
		ctx.ac_cls = cls;
		ptx_menu_build(px.g, res, &ctx, &model, order);
		if (model.highlight >= 0)
			ri = order[model.highlight];
	}
	logMsg("taxi route request at %s: wheelbase %.2f m, steer %.0f deg, "
	    "class %c, %s runway %s", px.icao, ac.wheelbase, ac.max_steer,
	    cls ? cls : '?', from_menu ? "highlighted" : "recommended",
	    ri >= 0 ? ptx_rend_name(px.g, ri) : "-");
	for (int i = 0; i < n; i++) {
		const ptx_result_t *r = &res[i];
		logMsg("  %-4s %s%s taxi %.0f m, start %s, nose %.1f m off",
		    ptx_rend_name(px.g, i), ptx_status_str(r->status),
		    r->has_lead ? " (straight ahead first)" : "", r->taxi,
		    r->edge >= 0 ? ptx_edge_name(px.g, r->edge) : "-",
		    r->off_nose);
	}
	if (ri < 0 || res[ri].status != PTX_OK) {
		snprintf(msg, sizeof (msg), "No taxi route to runway %s from "
		    "here.", ri >= 0 ? ptx_rend_name(px.g, ri) : "");
		XPLMSpeakString(msg);
		ptx_results_free(res, n);
		free(res);
		return;
	}
	path_opts(gear.wheelbase, gear.max_steer, &po);
	ps = ptx_path_build(px.g, &res[ri], ri, &po, &path);
	if (ps != PTX_PATH_OK) {
		logMsg("taxi route: no path for %s (%s)",
		    ptx_rend_name(px.g, ri), ptx_path_status_str(ps));
		XPLMSpeakString("Could not build the taxi route.");
	} else {
		size_t l = snprintf(msg, sizeof (msg), "Taxi route to runway "
		    "%s", ptx_rend_name(px.g, ri));
		int said = 0;
		for (int i = 0; i < path.n_spans && said < 4 &&
		    l < sizeof (msg); i++) {
			const char *nm = path.spans[i].name;
			if (nm[0] == '\0' || strncmp(nm, "RWY ", 4) == 0)
				continue;
			l += snprintf(msg + l, sizeof (msg) - l, "%s%s",
			    said ? ", " : ", via ", nm);
			said++;
		}
		ptx_route_publish(&path, "taxi_route command");
		if (l < sizeof (msg))
			snprintf(msg + l, sizeof (msg) - l, ".");
		XPLMSpeakString(msg);
	}
	ptx_results_free(res, n);
	free(res);
}

static void
ui_click(int row, int btn)
{
	if (row >= 0) {
		px.model.highlight = row;
		start_row(row);
		return;
	}
	switch (btn) {
	case PTX_UI_BTN_FULL:
		px.full_length = !px.full_length;
		px.full_length_set = true;
		plan_and_show();
		break;
	case PTX_UI_BTN_XPTUG:
		start_xp_tug(px.model.highlight);
		break;
	case PTX_UI_BTN_MANUAL:
		ptx_ui_hide();
		XPLMCommandOnce(XPLMFindCommand(
		    "ProgressiveTaxi/start_planner"));
		break;
	case PTX_UI_BTN_CLOSE:
		ptx_ui_hide();
		break;
	default:
		break;
	}
}

static void
move_highlight(int dir)
{
	int n = px.model.n_rows, h = px.model.highlight;
	if (!px.have_plan || n == 0)
		return;
	for (int k = 0; k < n; k++) {
		h = (h + dir + n) % n;
		if (px.model.rows[h].enabled) {
			px.model.highlight = h;
			ptx_ui_show(&px.model);
			return;
		}
	}
}

static int
cmd_cb(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
	(void) refcon;
	if (phase != xplm_CommandBegin)
		return (1);
	if (cmd == px.cmd_open) {
		if (ptx_ui_visible() && px.have_plan)
			start_row(px.model.highlight);
		else
			open_menu();
	} else if (cmd == px.cmd_next) {
		move_highlight(1);
	} else if (cmd == px.cmd_prev) {
		move_highlight(-1);
	} else if (cmd == px.cmd_best) {
		if (ptx_ui_visible() && px.have_plan) {
			start_row(px.model.highlight);
		} else {
			/* plan_and_show() starts it, now or after loading */
			px.start_best_when_ready = true;
			open_menu();
			if (!px.thr_active)
				px.start_best_when_ready = false;
		}
	} else if (cmd == px.cmd_close) {
		ptx_ui_hide();
	} else if (cmd == px.cmd_taxi) {
		taxi_route();
	} else if (cmd == px.cmd_taxi_clear) {
		ptx_route_clear("taxi_route_clear command");
	} else if (cmd == px.cmd_xptug) {
		if (!px.have_plan)
			open_menu();
		else
			start_xp_tug(px.model.highlight);
	}
	return (1);
}

static void
submenu_cb(void *menu_ref, void *item_ref)
{
	int row = (int)(intptr_t)item_ref - 1;
	(void) menu_ref;
	if (row >= 0)
		start_row(row);
}

/* ---------------------------------------------------------------------- */

void
ptx_xp_start(void)
{
	if (px.started)
		return;
	memset(&px, 0, sizeof (px));
	px.cmd_open = XPLMCreateCommand("ProgressiveTaxi/auto_pushback",
	    "Auto pushback: open the runway menu / push to the highlighted "
	    "runway");
	px.cmd_next = XPLMCreateCommand("ProgressiveTaxi/auto_pushback_next",
	    "Auto pushback: highlight the next runway");
	px.cmd_prev = XPLMCreateCommand("ProgressiveTaxi/auto_pushback_prev",
	    "Auto pushback: highlight the previous runway");
	px.cmd_best = XPLMCreateCommand("ProgressiveTaxi/auto_pushback_best",
	    "Auto pushback: push to the recommended runway, no menu");
	px.cmd_close = XPLMCreateCommand(
	    "ProgressiveTaxi/auto_pushback_close",
	    "Auto pushback: close the runway menu");
	px.cmd_xptug = XPLMCreateCommand(
	    "ProgressiveTaxi/auto_pushback_xp_tug",
	    "Auto pushback: use X-Plane's own tug for the highlighted runway");
	px.cmd_taxi = XPLMCreateCommand("ProgressiveTaxi/taxi_route",
	    "Taxi route from here to the highlighted runway (menu open) or "
	    "the recommended one, published as ptaxi/route/...");
	px.cmd_taxi_clear = XPLMCreateCommand(
	    "ProgressiveTaxi/taxi_route_clear",
	    "Withdraw the published taxi route");
	px.started = true;
}

void
ptx_xp_enable(XPLMMenuID root)
{
	XPLMCreateFlightLoop_t fl = {
	    .structSize = sizeof (fl),
	    .phase = xplm_FlightLoop_Phase_AfterFlightModel,
	    .callbackFunc = loader_floop,
	    .refcon = NULL
	};

	if (px.enabled)
		return;
	mutex_init(&px.lock);
	fdr_find(&px.local_x, "sim/flightmodel/position/local_x");
	fdr_find(&px.local_y, "sim/flightmodel/position/local_y");
	fdr_find(&px.local_z, "sim/flightmodel/position/local_z");
	fdr_find(&px.psi, "sim/flightmodel/position/psi");
	fdr_find(&px.elev, "sim/flightmodel/position/elevation");
	fdr_find(&px.lat, "sim/flightmodel/position/latitude");
	fdr_find(&px.lon, "sim/flightmodel/position/longitude");
	fdr_find(&px.wind_dir,
	    "sim/weather/aircraft/wind_now_direction_degt");
	fdr_find(&px.wind_spd, "sim/weather/aircraft/wind_now_speed_msc");
	fdr_find(&px.onground, "sim/flightmodel/failures/onground_any");
	px.floop = XPLMCreateFlightLoop(&fl);

	XPLMRegisterCommandHandler(px.cmd_open, cmd_cb, 1, NULL);
	XPLMRegisterCommandHandler(px.cmd_next, cmd_cb, 1, NULL);
	XPLMRegisterCommandHandler(px.cmd_prev, cmd_cb, 1, NULL);
	XPLMRegisterCommandHandler(px.cmd_best, cmd_cb, 1, NULL);
	XPLMRegisterCommandHandler(px.cmd_close, cmd_cb, 1, NULL);
	XPLMRegisterCommandHandler(px.cmd_xptug, cmd_cb, 1, NULL);
	XPLMRegisterCommandHandler(px.cmd_taxi, cmd_cb, 1, NULL);
	XPLMRegisterCommandHandler(px.cmd_taxi_clear, cmd_cb, 1, NULL);
	ptx_route_init();

	px.root = root;
	px.sub_item = XPLMAppendMenuItem(root, "Auto pushback", NULL, 1);
	px.sub = XPLMCreateMenu("Auto pushback", root, px.sub_item,
	    submenu_cb, NULL);
	rebuild_submenu();

	(void) ptx_ui_init(ui_click);
	px.enabled = true;
}

void
ptx_xp_disable(void)
{
	if (!px.enabled)
		return;
	if (px.thr_active) {
		thread_join(&px.thr);
		px.thr_active = false;
		ptx_apt_free(px.load_apt);
		px.load_apt = NULL;
	}
	XPLMDestroyFlightLoop(px.floop);
	px.floop = NULL;
	XPLMUnregisterCommandHandler(px.cmd_open, cmd_cb, 1, NULL);
	XPLMUnregisterCommandHandler(px.cmd_next, cmd_cb, 1, NULL);
	XPLMUnregisterCommandHandler(px.cmd_prev, cmd_cb, 1, NULL);
	XPLMUnregisterCommandHandler(px.cmd_best, cmd_cb, 1, NULL);
	XPLMUnregisterCommandHandler(px.cmd_close, cmd_cb, 1, NULL);
	XPLMUnregisterCommandHandler(px.cmd_xptug, cmd_cb, 1, NULL);
	XPLMUnregisterCommandHandler(px.cmd_taxi, cmd_cb, 1, NULL);
	XPLMUnregisterCommandHandler(px.cmd_taxi_clear, cmd_cb, 1, NULL);
	ptx_route_fini();
	if (px.outline != NULL) {
		acf_outline_free(px.outline);
		px.outline = NULL;
	}
	px.outline_path[0] = '\0';
	ptx_ui_fini();
	if (px.sub != NULL) {
		XPLMDestroyMenu(px.sub);
		px.sub = NULL;
	}
	ptx_results_free(px.res, px.n_res);
	px.n_res = 0;
	px.have_plan = false;
	ptx_graph_free(px.g);
	px.g = NULL;
	ptx_apt_free(px.apt);
	px.apt = NULL;
	px.icao[0] = '\0';
	mutex_destroy(&px.lock);
	px.enabled = false;
}
