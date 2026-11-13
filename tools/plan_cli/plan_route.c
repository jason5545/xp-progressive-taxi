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
 * plan_cli route / routes: the taxi path published as ptaxi/route/...
 * (design/taxi-route-interface.md), built by the same ptx_path_build()
 * the plugin calls.
 *
 *   plan_cli route --apt FILE [--icao XXXX] --stand NAME --rwy END
 *                  --wheelbase M --max-steer DEG --class C
 *                  [--full-length] [--stand-ref nose|main] [--from-pose]
 *                  [--out route.json] [--png route.png]
 *       One stand to one runway end. JSON keys are the dataref names
 *       (prog_s and the values that depend on it are for prog_s = 0);
 *       "turns", "spans" and "meta" are extra.
 *       --from-pose: no pushback, the plugin's ProgressiveTaxi/taxi_route
 *       from the parked pose.
 *
 *   plan_cli routes --apt FILE [--icao XXXX] --wheelbase M --max-steer DEG
 *                   --class C [--full-length] [--from-pose] [--all-stands]
 *       Every stand (of at least the aircraft's class unless --all-stands)
 *       x every runway end whose plan is OK: builds the path and checks
 *       the acceptance rules; prints one summary line and every failure.
 *       Exit code 0 = all pass.
 *
 *   plan_cli selftest
 *       ptx_path_track() on a made-up hairpin (north 200 m, U-turn, back
 *       south 4 m to the east), the nose 2.5 m east of the first leg, i.e.
 *       closer to the second: progress must stay on the first leg. None of
 *       the five test airports has a route coming back this close.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <cairo.h>

#include "planner/ptx_path.h"
#include "planner/ptx_plan.h"
#include "ptx_ui_render.h"

int cmd_route(int argc, char **argv);
int cmd_routes(int argc, char **argv);
int cmd_selftest(void);

static void
die(const char *msg)
{
	fprintf(stderr, "plan_cli: %s\n", msg);
	exit(2);
}

static char
stand_cls(const ptx_stand_t *s)
{
	if (s->cls != 0)
		return (s->cls);
	if (strstr(s->actypes, "heavy") != NULL)
		return ('E');
	if (strstr(s->actypes, "jets") != NULL)
		return ('C');
	if (strstr(s->actypes, "turboprops") != NULL)
		return ('B');
	return ('A');
}

static bool
is_helo(const char *acts)
{
	char buf[64];
	char *save = NULL, *t;
	bool any = false;

	if (acts[0] == '\0')
		return (false);
	strncpy(buf, acts, sizeof (buf) - 1);
	buf[sizeof (buf) - 1] = '\0';
	for (t = strtok_r(buf, "|", &save); t != NULL;
	    t = strtok_r(NULL, "|", &save)) {
		if (strcmp(t, "helos") != 0)
			return (false);
		any = true;
	}
	return (any);
}

static void
json_str(FILE *fp, const char *s)
{
	fputc('"', fp);
	for (const unsigned char *p = (const unsigned char *)s; *p; p++) {
		if (*p == '"' || *p == '\\')
			fprintf(fp, "\\%c", *p);
		else if (*p < 0x20)
			fprintf(fp, "\\u%04x", *p);
		else if (*p >= 0x80) {
			fputc(0xC0 | (*p >> 6), fp);
			fputc(0x80 | (*p & 0x3F), fp);
		} else
			fputc(*p, fp);
	}
	fputc('"', fp);
}

static double
hdg_of(double dx, double dy)
{
	double h = atan2(dx, dy) * (180.0 / M_PI);
	return (h < 0 ? h + 360 : h);
}

static double
relh(double h1, double h2)
{
	return (fmod(h2 - h1 + 540.0, 360.0) - 180.0);
}

typedef struct {
	const char	*apt_path, *icao, *stand, *rwy, *out, *png;
	double		wb, ms;
	char		cls;
	bool		full, main_ref, from_pose, all_stands;
} args_t;

static void
parse_args(int argc, char **argv, args_t *a)
{
	memset(a, 0, sizeof (*a));
	a->wb = a->ms = NAN;
	for (int i = 0; i < argc; i++) {
		if (strcmp(argv[i], "--apt") == 0 && i + 1 < argc)
			a->apt_path = argv[++i];
		else if (strcmp(argv[i], "--icao") == 0 && i + 1 < argc)
			a->icao = argv[++i];
		else if (strcmp(argv[i], "--stand") == 0 && i + 1 < argc)
			a->stand = argv[++i];
		else if (strcmp(argv[i], "--rwy") == 0 && i + 1 < argc)
			a->rwy = argv[++i];
		else if (strcmp(argv[i], "--out") == 0 && i + 1 < argc)
			a->out = argv[++i];
		else if (strcmp(argv[i], "--png") == 0 && i + 1 < argc)
			a->png = argv[++i];
		else if (strcmp(argv[i], "--wheelbase") == 0 && i + 1 < argc)
			a->wb = atof(argv[++i]);
		else if (strcmp(argv[i], "--max-steer") == 0 && i + 1 < argc)
			a->ms = atof(argv[++i]);
		else if (strcmp(argv[i], "--class") == 0 && i + 1 < argc)
			a->cls = argv[++i][0];
		else if (strcmp(argv[i], "--full-length") == 0)
			a->full = true;
		else if (strcmp(argv[i], "--from-pose") == 0)
			a->from_pose = true;
		else if (strcmp(argv[i], "--all-stands") == 0)
			a->all_stands = true;
		else if (strcmp(argv[i], "--stand-ref") == 0 && i + 1 < argc)
			a->main_ref = (strcmp(argv[++i], "main") == 0);
		else
			die("unknown argument");
	}
	if (a->apt_path == NULL)
		die("needs --apt");
	if (isnan(a->wb) || isnan(a->ms) || a->cls == 0)
		die("needs --wheelbase, --max-steer and --class");
}

/* plans one stand like the plugin does; results has ptx_n_rends() entries */
static void
plan_stand(ptx_graph_t *g, const ptx_apt_t *apt, int si, const args_t *a,
    ptx_result_t *res)
{
	const ptx_stand_t *st = &apt->stands[si];
	ptx_aircraft_t ac = { a->wb, a->ms, a->cls };
	ptx_opts_t opts;
	vect2_t Pn = ptx_project(g, st->lat, st->lon), Pm;
	double h = st->hdg * (M_PI / 180.0);

	ptx_opts_default(&opts);
	if (a->full)
		opts.w_along = PTX_W_ALONG_FULL_LENGTH;
	Pm = (a->main_ref ? Pn : VECT2(Pn.x - sin(h) * a->wb,
	    Pn.y - cos(h) * a->wb));
	if (a->from_pose) {
		/* nose wheel at the ramp start, main gear behind it */
		vect2_t nose = (a->main_ref ? VECT2(Pn.x + sin(h) * a->wb,
		    Pn.y + cos(h) * a->wb) : Pn);
		ptx_plan_from_pose(g, Pm, nose, st->hdg, &ac, &opts, res);
	} else {
		ptx_plan(g, Pm, Pn, st->hdg, si, &ac, &opts, res, NULL);
	}
}

static void
path_opts(const args_t *a, ptx_path_opts_t *po)
{
	ptx_path_opts_default(po);
	po->r_min = ptx_path_min_radius(a->wb, a->ms);
}

/* ---------------------------------------------------------------------- */
/* JSON, same names as the datarefs                                      */
/* ---------------------------------------------------------------------- */

static void
write_json(FILE *fp, const ptx_path_t *p, const char *icao, const char *stand,
    const args_t *a, const ptx_path_opts_t *po)
{
	double nts, ntd;
	const char *cur, *nxt;

	ptx_path_progress(p, 0, &nts, &ntd, &cur, &nxt);
	fprintf(fp, "{\"version\":%d,\"id\":1,\"active\":1,\"ref_lat\":%.10f,"
	    "\"ref_lon\":%.10f,\"n\":%d,", PTX_PATH_VERSION, p->ref_lat,
	    p->ref_lon, p->n);
	fputs("\"x\":[", fp);
	for (int i = 0; i < p->n; i++)
		fprintf(fp, "%s%.9g", i ? "," : "", (float)p->x[i]);
	fputs("],\"y\":[", fp);
	for (int i = 0; i < p->n; i++)
		fprintf(fp, "%s%.9g", i ? "," : "", (float)p->y[i]);
	fputs("],\"s\":[", fp);
	for (int i = 0; i < p->n; i++)
		fprintf(fp, "%s%.9g", i ? "," : "", (float)p->s[i]);
	fputs("],\"flags\":[", fp);
	for (int i = 0; i < p->n; i++)
		fprintf(fp, "%s%d", i ? "," : "", p->flags[i]);
	fprintf(fp, "],\"hs_n\":%d,\"hs_s\":[", p->hs_n);
	for (int i = 0; i < p->hs_n; i++)
		fprintf(fp, "%s%.9g", i ? "," : "", (float)p->hs_s[i]);
	fputs("],\"hs_names\":", fp);
	json_str(fp, p->hs_names);
	fputs(",\"rwy\":", fp);
	json_str(fp, p->rwy);
	fprintf(fp, ",\"rwy_hdg\":%.9g,\"prog_s\":0,\"next_turn_s\":%.9g,"
	    "\"next_turn_deg\":%.9g,\"cur_name\":", (float)p->rwy_hdg,
	    (float)nts, (float)ntd);
	json_str(fp, cur);
	fputs(",\"next_name\":", fp);
	json_str(fp, nxt);
	fputs(",\"turns\":[", fp);
	for (int i = 0; i < p->n_turns; i++) {
		fprintf(fp, "%s{\"s0\":%.2f,\"s1\":%.2f,\"deg\":%.2f,"
		    "\"r_min\":%.2f,\"manual\":%s}", i ? "," : "",
		    p->turns[i].s0, p->turns[i].s1, p->turns[i].deg,
		    p->turns[i].r_min, p->turns[i].manual ? "true" : "false");
	}
	fputs("],\"spans\":[", fp);
	for (int i = 0; i < p->n_spans; i++) {
		fprintf(fp, "%s{\"s\":%.2f,\"name\":", i ? "," : "",
		    p->spans[i].s);
		json_str(fp, p->spans[i].name);
		fputc('}', fp);
	}
	fputs("],\"meta\":{\"apt\":", fp);
	json_str(fp, icao);
	fputs(",\"stand\":", fp);
	json_str(fp, stand);
	fprintf(fp, ",\"wheelbase\":%.3f,\"max_steer\":%.1f,\"class\":\"%c\","
	    "\"from_pose\":%s,\"r_min\":%.2f,\"len\":%.1f,\"r_min_used\":%.2f,"
	    "\"n_shrunk\":%d,\"n_manual\":%d}}\n", a->wb, a->ms, a->cls,
	    a->from_pose ? "true" : "false", po->r_min, p->len, p->r_min_used,
	    p->n_shrunk, p->n_manual);
}

/* ---------------------------------------------------------------------- */
/* PNG                                                                    */
/* ---------------------------------------------------------------------- */

static void
write_png(const char *fn, const ptx_graph_t *g, const ptx_path_t *p,
    int ri, const char *title)
{
	double x0 = 1e18, x1 = -1e18, y0 = 1e18, y1 = -1e18, sc, W, H;
	const double MARGIN = 120, PX = 1600;
	cairo_surface_t *surf;
	cairo_t *cr;
	ptx_ui_fonts_t *fonts = ptx_ui_fonts_new();

	for (int i = 0; i < p->n; i++) {
		x0 = fmin(x0, p->x[i]);
		x1 = fmax(x1, p->x[i]);
		y0 = fmin(y0, p->y[i]);
		y1 = fmax(y1, p->y[i]);
	}
	x0 -= MARGIN; x1 += MARGIN; y0 -= MARGIN; y1 += MARGIN;
	sc = PX / fmax(x1 - x0, y1 - y0);
	W = ceil((x1 - x0) * sc);
	H = ceil((y1 - y0) * sc) + 60;
	surf = cairo_image_surface_create(CAIRO_FORMAT_RGB24, (int)W, (int)H);
	cr = cairo_create(surf);
	cairo_set_source_rgb(cr, 1, 1, 1);
	cairo_paint(cr);
#define	TX(px)	(((px) - x0) * sc)
#define	TY(py)	(H - 60 - ((py) - y0) * sc)

	/* runways */
	for (int k = 0; k < ptx_n_rends(g); k += 2) {
		vect2_t t, u;
		double l, w;
		vect2_t n;
		ptx_rend_geom(g, k, &t, &u, &l, &w);
		n = VECT2(-u.y * w / 2, u.x * w / 2);
		cairo_move_to(cr, TX(t.x + n.x), TY(t.y + n.y));
		cairo_line_to(cr, TX(t.x + u.x * l + n.x), TY(t.y + u.y * l + n.y));
		cairo_line_to(cr, TX(t.x + u.x * l - n.x), TY(t.y + u.y * l - n.y));
		cairo_line_to(cr, TX(t.x - n.x), TY(t.y - n.y));
		cairo_close_path(cr);
		cairo_set_source_rgb(cr, k / 2 == ri / 2 ? 0.80 : 0.86,
		    k / 2 == ri / 2 ? 0.88 : 0.86, k / 2 == ri / 2 ? 0.80 : 0.86);
		cairo_fill(cr);
	}
	/* network */
	cairo_set_line_width(cr, 1.0);
	for (int e = 0; e < ptx_n_edges(g); e++) {
		vect2_t A, B;
		ptx_edge_ends(g, e, &A, &B);
		if (fmax(A.x, B.x) < x0 || fmin(A.x, B.x) > x1 ||
		    fmax(A.y, B.y) < y0 || fmin(A.y, B.y) > y1)
			continue;
		if (ptx_edge_is_runway(g, e))
			cairo_set_source_rgb(cr, 0.45, 0.45, 0.45);
		else
			cairo_set_source_rgb(cr, 0.70, 0.70, 0.70);
		cairo_move_to(cr, TX(A.x), TY(A.y));
		cairo_line_to(cr, TX(B.x), TY(B.y));
		cairo_stroke(cr);
	}
	/* the planned polyline before filleting */
	{
		double dash[] = { 4, 3 };
		cairo_set_dash(cr, dash, 2, 0);
		cairo_set_source_rgb(cr, 0.2, 0.2, 0.2);
		cairo_set_line_width(cr, 1.0);
		for (int k = 0; k < p->n_poly; k++) {
			if (k == 0)
				cairo_move_to(cr, TX(p->poly[k].x), TY(p->poly[k].y));
			else
				cairo_line_to(cr, TX(p->poly[k].x), TY(p->poly[k].y));
		}
		cairo_stroke(cr);
		cairo_set_dash(cr, NULL, 0, 0);
	}
	/* the path, coloured by flags */
	cairo_set_line_width(cr, 3.0);
	cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);
	for (int i = 0; i + 1 < p->n; i++) {
		int f = p->flags[i] | p->flags[i + 1];
		if (f & PTX_PF_MANUAL)
			cairo_set_source_rgb(cr, 0.85, 0.0, 0.75);
		else if ((p->flags[i] & p->flags[i + 1]) & PTX_PF_ARC)
			cairo_set_source_rgb(cr, 0.95, 0.50, 0.0);
		else if (f & PTX_PF_RWY_DEP)
			cairo_set_source_rgb(cr, 0.0, 0.60, 0.15);
		else if (f & PTX_PF_RWY_CROSS)
			cairo_set_source_rgb(cr, 0.85, 0.0, 0.0);
		else
			cairo_set_source_rgb(cr, 0.10, 0.35, 0.90);
		cairo_move_to(cr, TX(p->x[i]), TY(p->y[i]));
		cairo_line_to(cr, TX(p->x[i + 1]), TY(p->y[i + 1]));
		cairo_stroke(cr);
	}
	if (fonts != NULL)
		cairo_set_font_face(cr, ptx_ui_fonts_face(fonts));
	cairo_set_font_size(cr, 15);
	/* hold-short lines */
	for (int h = 0; h < p->hs_n; h++) {
		int i = 0;
		double dx, dy, l;
		while (i + 1 < p->n && p->s[i + 1] < p->hs_s[h])
			i++;
		if (i + 1 >= p->n)
			i = p->n - 2;
		dx = p->x[i + 1] - p->x[i];
		dy = p->y[i + 1] - p->y[i];
		l = hypot(dx, dy);
		dx /= l;
		dy /= l;
		cairo_set_source_rgb(cr, 0.9, 0.0, 0.0);
		cairo_set_line_width(cr, 4.0);
		cairo_move_to(cr, TX(p->x[i] - dy * 25), TY(p->y[i] + dx * 25));
		cairo_line_to(cr, TX(p->x[i] + dy * 25), TY(p->y[i] - dx * 25));
		cairo_stroke(cr);
		cairo_move_to(cr, TX(p->x[i] + dy * 30), TY(p->y[i] - dx * 30));
		cairo_show_text(cr, p->hs_name[h]);
	}
	/* start and end */
	cairo_set_source_rgb(cr, 0, 0, 0);
	cairo_arc(cr, TX(p->x[0]), TY(p->y[0]), 6, 0, 2 * M_PI);
	cairo_fill(cr);
	cairo_move_to(cr, TX(p->x[0]) + 9, TY(p->y[0]) - 9);
	cairo_show_text(cr, "start");
	cairo_move_to(cr, TX(p->x[p->n - 1]) + 9, TY(p->y[p->n - 1]) - 9);
	cairo_show_text(cr, p->rwy);
	/* taxiway names at span starts */
	cairo_set_source_rgb(cr, 0.10, 0.25, 0.60);
	cairo_set_font_size(cr, 13);
	for (int k = 0; k < p->n_spans; k++) {
		int i = 0;
		while (i + 1 < p->n && p->s[i] < p->spans[k].s + 15)
			i++;
		cairo_move_to(cr, TX(p->x[i]) + 6, TY(p->y[i]) + 16);
		cairo_show_text(cr, p->spans[k].name);
	}
	/* legend */
	cairo_set_source_rgb(cr, 0, 0, 0);
	cairo_set_font_size(cr, 16);
	cairo_move_to(cr, 12, H - 36);
	cairo_show_text(cr, title);
	cairo_set_font_size(cr, 14);
	cairo_move_to(cr, 12, H - 14);
	cairo_show_text(cr, "藍=直線 橘=ARC 綠=RWY_DEP 紅=RWY_CROSS 紫=MANUAL "
	    "紅橫線=等待線 灰虛線=規劃折線（圓弧前） 灰=滑行網路");
#undef	TX
#undef	TY
	cairo_destroy(cr);
	if (cairo_surface_write_to_png(surf, fn) != CAIRO_STATUS_SUCCESS)
		die("can't write png");
	cairo_surface_destroy(surf);
	ptx_ui_fonts_free(fonts);
}

/* ---------------------------------------------------------------------- */

static int
find_stand(const ptx_apt_t *apt, const char *name)
{
	for (int i = 0; i < apt->n_stands; i++) {
		if (strcmp(apt->stands[i].name, name) == 0)
			return (i);
	}
	return (-1);
}

static int
find_rend(const ptx_graph_t *g, const char *name)
{
	for (int i = 0; i < ptx_n_rends(g); i++) {
		if (strcmp(ptx_rend_name(g, i), name) == 0)
			return (i);
	}
	return (-1);
}

int
cmd_route(int argc, char **argv)
{
	args_t a;
	ptx_apt_t *apt;
	ptx_graph_t *g;
	ptx_result_t *res;
	ptx_path_opts_t po;
	ptx_path_t p;
	ptx_path_status_t st;
	int si, ri;
	FILE *fp;

	parse_args(argc, argv, &a);
	if (a.stand == NULL || a.rwy == NULL)
		die("'route' needs --stand and --rwy");
	apt = ptx_apt_load(a.apt_path, a.icao);
	if (apt == NULL)
		die("can't load airport");
	g = ptx_graph_new(apt);
	if ((si = find_stand(apt, a.stand)) < 0)
		die("no such stand");
	if ((ri = find_rend(g, a.rwy)) < 0)
		die("no such runway end");
	res = calloc(ptx_n_rends(g) + 1, sizeof (*res));
	plan_stand(g, apt, si, &a, res);
	if (res[ri].status != PTX_OK) {
		fprintf(stderr, "plan_cli: %s %s -> %s: %s\n", apt->icao,
		    a.stand, a.rwy, ptx_status_str(res[ri].status));
		return (1);
	}
	path_opts(&a, &po);
	st = ptx_path_build(g, &res[ri], ri, &po, &p);
	if (st != PTX_PATH_OK) {
		fprintf(stderr, "plan_cli: path: %s\n", ptx_path_status_str(st));
		return (1);
	}
	fp = (a.out != NULL ? fopen(a.out, "w") : stdout);
	if (fp == NULL)
		die("can't open output");
	write_json(fp, &p, apt->icao, a.stand, &a, &po);
	if (fp != stdout)
		fclose(fp);
	if (a.png != NULL) {
		char title[256];
		snprintf(title, sizeof (title), "%s 停機位 %s → 跑道 %s：%d 點、"
		    "%.0f m、轉彎 %d、最小半徑 %.1f m、縮小 %d、MANUAL %d、"
		    "等待線 %s%s", apt->icao, a.stand, a.rwy, p.n, p.len,
		    p.n_turns, p.r_min_used, p.n_shrunk, p.n_manual,
		    p.hs_names, a.from_pose ? "（不後推，從停機位滑出）" : "");
		write_png(a.png, g, &p, ri, title);
		fprintf(stderr, "PNG：%s\n", a.png);
	}
	ptx_path_free(&p);
	ptx_results_free(res, ptx_n_rends(g));
	free(res);
	ptx_graph_free(g);
	ptx_apt_free(apt);
	return (0);
}

/* ---------------------------------------------------------------------- */
/* batch acceptance                                                       */
/* ---------------------------------------------------------------------- */

enum {
	F_BUILD, F_START, F_S, F_STEP, F_N, F_HDG, F_ARC, F_EXT_LAT,
	F_EXT_HDG, F_HS_RANGE, F_HS_ORDER, F_HS_DEP, F_TRACK, NF
};
static const char *F_NAME[NF] = {
	"產不出路徑", "第一點離 result.end（或 lead）> 0.01 m", "s 沒有嚴格遞增",
	"相鄰點 > 2.0 m", "n > 4096", "非 MANUAL 段折角超過 2 m/R + 0.1°",
	"圓弧點離折線超過 R(sec(θ/2)−1)+0.1 m", "延伸段離中心線 > 0.1 m",
	"延伸段航向差 > 0.5°", "hs_s 不在 0..s_max", "hs_s 沒有遞增",
	"起飛跑道等待線不在進跑道之前",
	"進度追蹤跳段（模擬機頭側偏 3 m 沿路徑走，MANUAL 前後 10 m 在線上）"
};

static double
pt_seg_dist(double px, double py, vect2_t A, vect2_t B)
{
	double bx = B.x - A.x, by = B.y - A.y, l2 = bx * bx + by * by, t = 0;
	if (l2 > 0) {
		t = ((px - A.x) * bx + (py - A.y) * by) / l2;
		t = fmin(fmax(t, 0.0), 1.0);
	}
	return (hypot(px - A.x - t * bx, py - A.y - t * by));
}

typedef struct {
	int	fails[NF];
	double	worst_hdg_margin, worst_arc_margin, worst_ext_lat, worst_ext_hdg;
	double	max_step, max_start;
	int	max_n;
} chk_t;

/* returns the number of failed rules for this path */
static int
check_path(const ptx_graph_t *g, const ptx_result_t *r, int ri,
    const ptx_path_t *p, chk_t *c, char *why, size_t cap)
{
	int nf = 0;
	bool bad[NF] = { false };
	int n = p->n;
	float *fx = malloc(n * sizeof (float)), *fy = malloc(n * sizeof (float));
	float *fs = malloc(n * sizeof (float));
	double *chord_r = malloc(n * sizeof (double));
	vect2_t thr, u;
	double rlen, rw;
	int i_dep = -1;

	for (int i = 0; i < n; i++) {
		fx[i] = (float)p->x[i];
		fy[i] = (float)p->y[i];
		fs[i] = (float)p->s[i];
		chord_r[i] = INFINITY;
	}
	/* chord j = point j -> j + 1 */
	for (int a = 0; a < p->n_arcs; a++) {
		const ptx_arc_t *A = &p->arcs[a];
		for (int j = A->i0; j < A->i1; j++)
			chord_r[j] = fmin(chord_r[j], A->r);
	}
	{
		/* without a pushback and parked facing a taxiway, the path
		 * starts at the main gear (lead) instead */
		vect2_t st = (r->has_lead ? r->lead : r->end);
		double d = hypot(fx[0] - st.x, fy[0] - st.y);
		c->max_start = fmax(c->max_start, d);
		if (d > 0.01)
			bad[F_START] = true;
	}
	if (n > PTX_PATH_MAX_PTS)
		bad[F_N] = true;
	c->max_n = (n > c->max_n ? n : c->max_n);
	for (int i = 0; i + 1 < n; i++) {
		double d = hypot(fx[i + 1] - fx[i], fy[i + 1] - fy[i]);
		if (!(fs[i + 1] > fs[i]))
			bad[F_S] = true;
		if (d > 2.0)
			bad[F_STEP] = true;
		c->max_step = fmax(c->max_step, d);
	}
	/* no unhandled corners outside MANUAL */
	for (int i = 1; i + 1 < n; i++) {
		double h1, h2, dh, rr, lim;
		if ((p->flags[i - 1] | p->flags[i] | p->flags[i + 1]) &
		    PTX_PF_MANUAL)
			continue;
		h1 = hdg_of(fx[i] - fx[i - 1], fy[i] - fy[i - 1]);
		h2 = hdg_of(fx[i + 1] - fx[i], fy[i + 1] - fy[i]);
		dh = fabs(relh(h1, h2));
		rr = fmin(chord_r[i - 1], chord_r[i]);
		lim = (isinf(rr) ? 0 : (2.0 / rr) * (180.0 / M_PI)) + 0.1;
		c->worst_hdg_margin = fmax(c->worst_hdg_margin, dh - lim);
		if (dh > lim)
			bad[F_HDG] = true;
	}
	/* arc points stay within R (sec(θ/2) - 1) of the polyline */
	for (int a = 0; a < p->n_arcs; a++) {
		const ptx_arc_t *A = &p->arcs[a];
		double lim, half;
		if (A->r <= 0)
			continue;
		half = fabs(A->theta) / 2 * (M_PI / 180.0);
		lim = A->r * (1 / cos(half) - 1) + 0.1;
		for (int i = A->i0; i <= A->i1; i++) {
			double d = fmin(pt_seg_dist(fx[i], fy[i],
			    p->poly[A->vtx - 1], p->poly[A->vtx]),
			    pt_seg_dist(fx[i], fy[i], p->poly[A->vtx],
			    p->poly[A->vtx + 1]));
			c->worst_arc_margin = fmax(c->worst_arc_margin, d - lim);
			if (d > lim)
				bad[F_ARC] = true;
		}
	}
	/* the runway extension */
	ptx_rend_geom(g, ri, &thr, &u, &rlen, &rw);
	for (int i = 0; i < n; i++) {
		if (p->s[i] + 1e-9 >= p->s_ext) {
			double lat = fabs((fx[i] - thr.x) * u.y -
			    (fy[i] - thr.y) * u.x);
			c->worst_ext_lat = fmax(c->worst_ext_lat, lat);
			if (lat > 0.1)
				bad[F_EXT_LAT] = true;
			if (i + 1 < n) {
				double dh = fabs(relh(p->rwy_hdg, hdg_of(fx[i + 1] -
				    fx[i], fy[i + 1] - fy[i])));
				c->worst_ext_hdg = fmax(c->worst_ext_hdg, dh);
				if (dh > 0.5)
					bad[F_EXT_HDG] = true;
			}
		}
		if (i_dep < 0 && (p->flags[i] & PTX_PF_RWY_DEP))
			i_dep = i;
	}
	for (int h = 0; h < p->hs_n; h++) {
		if (p->hs_s[h] < 0 || p->hs_s[h] > p->s[n - 1])
			bad[F_HS_RANGE] = true;
		if (h > 0 && !(p->hs_s[h] > p->hs_s[h - 1]))
			bad[F_HS_ORDER] = true;
		if (p->hs_dep[h] && (i_dep < 0 || !(p->hs_s[h] < p->s[i_dep])))
			bad[F_HS_DEP] = true;
	}
	/*
	 * What the plugin does every frame: the nose 3 m to the right of the
	 * path, every 4th point; prog_s must follow within 4 m and never go
	 * back by more than 1 m. Within 10 m of a MANUAL point (a corner
	 * tighter than the aircraft can follow, where Safety Pilot has handed
	 * back) the nose is put on the line: 3 m to the side of a 1 m radius
	 * corner is closer to the other leg, which says nothing about the
	 * tracking.
	 */
	{
		double prog = ptx_path_track(p, VECT2(p->x[0], p->y[0]), 0,
		    NULL);
		double *ms = malloc((n + 1) * sizeof (double));
		int nm = 0;
		for (int i = 0; i < n; i++) {
			if (p->flags[i] & PTX_PF_MANUAL)
				ms[nm++] = p->s[i];
		}
		for (int i = 0; i + 1 < n; i += 4) {
			double dx = p->x[i + 1] - p->x[i];
			double dy = p->y[i + 1] - p->y[i];
			double l = hypot(dx, dy), np, side = 3;
			vect2_t q;
			for (int k = 0; k < nm; k++) {
				if (fabs(ms[k] - p->s[i]) <= 10) {
					side = 0;
					break;
				}
			}
			q = VECT2(p->x[i] + dy / l * side,
			    p->y[i] - dx / l * side);
			np = ptx_path_track(p, q, prog, NULL);
			if (np < prog - 1.0 || fabs(np - p->s[i]) > 4.0) {
				bad[F_TRACK] = true;
				break;
			}
			prog = np;
		}
		free(ms);
	}
	why[0] = '\0';
	for (int k = 0; k < NF; k++) {
		if (bad[k]) {
			size_t l = strlen(why);
			c->fails[k]++;
			nf++;
			snprintf(why + l, cap - l, "%s%s", l ? "；" : "",
			    F_NAME[k]);
		}
	}
	free(fx);
	free(fy);
	free(fs);
	free(chord_r);
	return (nf);
}

int
cmd_routes(int argc, char **argv)
{
	args_t a;
	ptx_apt_t *apt;
	ptx_graph_t *g;
	ptx_result_t *res;
	ptx_path_opts_t po;
	chk_t c;
	int n_stands = 0, n_routes = 0, n_bad = 0, n_manual_routes = 0;
	int n_manual_turns = 0, n_shrunk = 0, n_turns = 0, n_cross = 0;
	double r_min_used = 0, max_len = 0;
	int n_hs = 0, n_hs_cross = 0;

	parse_args(argc, argv, &a);
	apt = ptx_apt_load(a.apt_path, a.icao);
	if (apt == NULL)
		die("can't load airport");
	g = ptx_graph_new(apt);
	res = calloc(ptx_n_rends(g) + 1, sizeof (*res));
	path_opts(&a, &po);
	memset(&c, 0, sizeof (c));
	c.worst_hdg_margin = c.worst_arc_margin = -1e9;
	for (int si = 0; si < apt->n_stands; si++) {
		const ptx_stand_t *st = &apt->stands[si];
		if (is_helo(st->actypes))
			continue;
		if (!a.all_stands && stand_cls(st) < a.cls)
			continue;
		n_stands++;
		plan_stand(g, apt, si, &a, res);
		for (int ri = 0; ri < ptx_n_rends(g); ri++) {
			ptx_path_t p;
			ptx_path_status_t ps;
			char why[512];
			if (res[ri].status != PTX_OK)
				continue;
			n_routes++;
			ps = ptx_path_build(g, &res[ri], ri, &po, &p);
			if (ps != PTX_PATH_OK) {
				c.fails[F_BUILD]++;
				n_bad++;
				printf("  失敗 %s → %s：產不出路徑（%s）\n",
				    st->name, ptx_rend_name(g, ri),
				    ptx_path_status_str(ps));
				continue;
			}
			if (check_path(g, &res[ri], ri, &p, &c, why,
			    sizeof (why)) > 0) {
				n_bad++;
				printf("  失敗 %s → %s：%s\n", st->name,
				    ptx_rend_name(g, ri), why);
			}
			if (p.n_manual > 0)
				n_manual_routes++;
			n_manual_turns += p.n_manual;
			n_shrunk += p.n_shrunk;
			n_turns += p.n_turns;
			if (p.r_min_used > 0 && (r_min_used == 0 ||
			    p.r_min_used < r_min_used))
				r_min_used = p.r_min_used;
			max_len = fmax(max_len, p.len);
			n_hs += p.hs_n;
			for (int h = 0; h < p.hs_n; h++)
				n_hs_cross += !p.hs_dep[h];
			for (int i = 0; i < p.n; i++) {
				if (p.flags[i] & PTX_PF_RWY_CROSS) {
					n_cross++;
					break;
				}
			}
			ptx_path_free(&p);
		}
		ptx_results_free(res, ptx_n_rends(g));
	}
	printf("%s（軸距 %.2f m、轉向 %.0f°、等級 %c%s，最小半徑 %.1f m）："
	    "停機位 %d、路線 %d、不合格 %d；MANUAL 路線 %d（轉彎 %d）；"
	    "轉彎 %d、被迫縮小 %d；最小圓弧半徑 %.1f m；穿越跑道的路線 %d、"
	    "等待線 %d（穿越用 %d）；最長 %.0f m、最多 %d 點；"
	    "最大點距 %.3f m、起點最大誤差 %.4f m；折角超出 %.4f°、"
	    "圓弧偏離超出 %.4f m、延伸段離中心線 %.4f m／航向 %.4f°\n",
	    apt->icao, a.wb, a.ms, a.cls, a.from_pose ? "、不後推" : "",
	    po.r_min, n_stands, n_routes, n_bad, n_manual_routes,
	    n_manual_turns, n_turns, n_shrunk, r_min_used, n_cross, n_hs,
	    n_hs_cross, max_len, c.max_n, c.max_step, c.max_start,
	    c.worst_hdg_margin, c.worst_arc_margin, c.worst_ext_lat,
	    c.worst_ext_hdg);
	for (int k = 0; k < NF; k++) {
		if (c.fails[k] > 0)
			printf("  %s：%d\n", F_NAME[k], c.fails[k]);
	}
	free(res);
	ptx_graph_free(g);
	ptx_apt_free(apt);
	return (n_bad == 0 ? 0 : 1);
}

int
cmd_selftest(void)
{
	ptx_path_t p;
	int cap = 512, jumps = 0, gjumps = 0;
	double prog = 0, worst = 0;

	memset(&p, 0, sizeof (p));
	p.x = calloc(cap, sizeof (double));
	p.y = calloc(cap, sizeof (double));
	p.s = calloc(cap, sizeof (double));
	p.flags = calloc(cap, sizeof (int));
	for (int i = 0; i <= 100; i++) {
		p.x[p.n] = 0;
		p.y[p.n++] = 2.0 * i;
	}
	for (int k = 1; k <= 6; k++) {
		double a = M_PI * k / 6;
		p.x[p.n] = 2 - 2 * cos(a);
		p.y[p.n++] = 200 + 2 * sin(a);
	}
	for (int i = 1; i <= 100; i++) {
		p.x[p.n] = 4;
		p.y[p.n++] = 200 - 2.0 * i;
	}
	for (int i = 1; i < p.n; i++) {
		p.s[i] = p.s[i - 1] + hypot(p.x[i] - p.x[i - 1],
		    p.y[i] - p.y[i - 1]);
	}
	for (double y = 0; y < 195; y += 1) {
		double np = ptx_path_track(&p, VECT2(2.5, y), prog, NULL);
		worst = fmax(worst, fabs(np - y));
		if (fabs(np - y) > 4)
			jumps++;
		prog = np;
		/* for comparison: nearest point over the whole path */
		np = ptx_path_track(&p, VECT2(2.5, y), 1e9, NULL);
		if (fabs(np - y) > 4)
			gjumps++;
	}
	printf("進度追蹤（人工 U 形彎，機頭在去程東 2.5 m、離回程 1.5 m）："
	    "195 步、跳段 %d、最大誤差 %.2f m；對照：全路徑最近點跳段 %d 步\n",
	    jumps, worst, gjumps);
	ptx_path_free(&p);
	return (jumps == 0 && gjumps > 0 ? 0 : 1);
}
