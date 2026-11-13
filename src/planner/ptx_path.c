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

#include "ptx_path.h"

#define	MIN_SEG		0.05	/* polyline vertices closer than this merge */
#define	MIN_FILLET	0.05	/* degrees; straighter corners pass through */
#define	MERGE_GAP	10.0	/* same-way arcs this close are one turn */
#define	STEP_MARGIN	0.995	/* keep spacing a hair under opts->step */
#define	TRACK_S_PEN	0.1	/* ptx_path_track(): m of distance per m of s */

/*
 * Preferred radii, step 0 of the route interface work (11/13): painted
 * centreline fillets at network junctions, by the narrower of the two
 * taxiways. RCTP (lighted centreline, type 51) F: p50 57.8 m, E/F 63.9 m;
 * RCSS (types 1, 51) C/E: p50 35.0 m, E/E 51.3 m. A, B and D have no
 * sample at these airports and are interpolated.
 */
static const double R_PREF[6] = { 15.0, 20.0, 35.0, 45.0, 50.0, 55.0 };

static inline vect2_t v2(double x, double y) { vect2_t v = { x, y }; return (v); }
static inline vect2_t vadd(vect2_t a, vect2_t b) { return (v2(a.x + b.x, a.y + b.y)); }
static inline vect2_t vsub(vect2_t a, vect2_t b) { return (v2(a.x - b.x, a.y - b.y)); }
static inline vect2_t vmul(vect2_t a, double s) { return (v2(a.x * s, a.y * s)); }
static inline double vdot(vect2_t a, vect2_t b) { return (a.x * b.x + a.y * b.y); }
static inline double vdist(vect2_t a, vect2_t b) { return (hypot(a.x - b.x, a.y - b.y)); }
static inline double rad(double d) { return (d * (M_PI / 180.0)); }
static inline double deg(double r) { return (r * (180.0 / M_PI)); }

static double
hdg_of(vect2_t d)
{
	double h = deg(atan2(d.x, d.y));
	return (h < 0 ? h + 360 : h);
}

static double
relh(double h1, double h2)
{
	double d = fmod(h2 - h1 + 540.0, 360.0) - 180.0;
	return (d);
}

void
ptx_path_opts_default(ptx_path_opts_t *o)
{
	memset(o, 0, sizeof (*o));
	memcpy(o->r_pref, R_PREF, sizeof (R_PREF));
	o->r_unknown = R_PREF[2];
	o->r_min = 0;
	o->ext_len = 300.0;
	o->turn_deg = 5.0;
	o->manual_deg = 150.0;
	o->step = 2.0;
}

double
ptx_path_min_radius(double wheelbase, double max_steer)
{
	double a = rad(0.9 * max_steer);
	if (wheelbase <= 0 || a <= 0)
		return (0);
	return (wheelbase / sin(a));
}

const char *
ptx_path_status_str(ptx_path_status_t st)
{
	switch (st) {
	case PTX_PATH_OK:
		return ("ok");
	case PTX_PATH_NO_ROUTE:
		return ("no_route");
	case PTX_PATH_TOO_LONG:
		return ("too_long");
	}
	return ("?");
}

/* ---------------------------------------------------------------------- */

typedef struct {
	vect2_t	p;
	int	arr;	/* route index of the edge arriving here; -1 start,
			   -2 the runway extension */
} vx_t;

typedef struct {
	ptx_path_t	*p;
	int		cap;
} pbuf_t;

static void
emit(pbuf_t *b, vect2_t q)
{
	ptx_path_t *p = b->p;
	double ds = 0;

	if (p->n > 0) {
		ds = hypot(q.x - p->x[p->n - 1], q.y - p->y[p->n - 1]);
		if (ds < 1e-6)
			return;
	}
	if (p->n == b->cap) {
		b->cap = (b->cap ? b->cap * 2 : 1024);
		p->x = realloc(p->x, b->cap * sizeof (double));
		p->y = realloc(p->y, b->cap * sizeof (double));
		p->s = realloc(p->s, b->cap * sizeof (double));
	}
	p->x[p->n] = q.x;
	p->y[p->n] = q.y;
	p->s[p->n] = (p->n > 0 ? p->s[p->n - 1] + ds : 0);
	p->n++;
}

static double
cur_s(const pbuf_t *b)
{
	return (b->p->n > 0 ? b->p->s[b->p->n - 1] : 0);
}

static void
seg_name(const ptx_graph_t *g, const ptx_result_t *r, int ri, int arr,
    char *buf, size_t cap)
{
	if (arr == -2) {
		snprintf(buf, cap, "RWY %s", ptx_rend_name(g, ri));
	} else if (arr >= 0) {
		int e;
		ptx_dedge(g, r->route[arr], &e, NULL, NULL);
		if (ptx_edge_is_runway(g, e))
			snprintf(buf, cap, "RWY %s", ptx_edge_name(g, e));
		else
			snprintf(buf, cap, "%s", ptx_edge_name(g, e));
	} else {
		buf[0] = '\0';
	}
}

static char
seg_cls(const ptx_graph_t *g, const ptx_result_t *r, int arr)
{
	int e;
	if (arr < 0)
		return (0);
	ptx_dedge(g, r->route[arr], &e, NULL, NULL);
	return (ptx_edge_cls(g, e));
}

static bool
in_rwy(vect2_t q, vect2_t thr, vect2_t u, double len, double width,
    double slack_along, double *along)
{
	vect2_t rel = vsub(q, thr);
	double al = vdot(rel, u);
	double lat = fabs(rel.x * u.y - rel.y * u.x);
	if (along != NULL)
		*along = al;
	return (al >= -slack_along && al <= len + slack_along &&
	    lat <= width / 2);
}

ptx_path_status_t
ptx_path_build(const ptx_graph_t *g, const ptx_result_t *r, int ri,
    const ptx_path_opts_t *o, ptx_path_t *p)
{
	int nv0, m = 0, off;
	vx_t *v;
	int *map;
	double *L, *th, *T, *Tc, *R, *rp, *vs;
	bool *corner;
	vect2_t thr, u, F = { 0, 0 };
	double rlen, rwidth, along = 0, ext;
	pbuf_t b;
	ptx_hold_t holds[PTX_PATH_MAX_HS];
	int nh;
	double s_dep = 0;
	int i_ext = 0;
	double step = o->step * STEP_MARGIN;

	memset(p, 0, sizeof (*p));
	if (r->status != PTX_OK || r->n_route <= 0)
		return (PTX_PATH_NO_ROUTE);

	snprintf(p->rwy, sizeof (p->rwy), "%s", ptx_rend_name(g, ri));
	p->rwy_hdg = ptx_rend_hdg(g, ri);
	p->ref_lat = ptx_ref_lat(g);
	p->ref_lon = ptx_ref_lon(g);
	ptx_rend_geom(g, ri, &thr, &u, &rlen, &rwidth);

	/* --- the polyline: start, route nodes, entry on the centreline,
	 * extension end --- */
	/* raw vertex k: (lead,) end, to-node of route[k - 1 - off], ... */
	off = (r->has_lead ? 1 : 0);
	nv0 = r->n_route + 2 + off;
	v = calloc(nv0 + 1, sizeof (*v));
	map = calloc(nv0 + 1, sizeof (*map));
	{
		vx_t raw;
		for (int k = 0; k < nv0; k++) {
			if (k < off) {
				raw.p = r->lead;
				raw.arr = -1;
			} else if (k == off) {
				raw.p = r->end;
				raw.arr = (off ? -3 : -1);
			} else if (k <= r->n_route + off) {
				int to;
				ptx_dedge(g, r->route[k - 1 - off], NULL, NULL,
				    &to);
				raw.p = ptx_node_pos(g, to);
				raw.arr = k - 1 - off;
				if (k == r->n_route + off) {
					/* the entry node, moved onto the
					 * runway's 100-row centreline */
					along = vdot(vsub(raw.p, thr), u);
					F = vadd(thr, vmul(u, along));
					raw.p = F;
				}
			} else {
				ext = fmin(o->ext_len, rlen - along);
				if (ext < o->step)
					ext = o->step;
				raw.p = vadd(F, vmul(u, ext));
				raw.arr = -2;
			}
			if (m > 0 && vdist(raw.p, v[m - 1].p) < MIN_SEG &&
			    k < nv0 - 1) {
				map[k] = m - 1;
				continue;
			}
			map[k] = m;
			v[m++] = raw;
		}
	}
	p->n_poly = m;
	p->poly = malloc(m * sizeof (vect2_t));
	for (int k = 0; k < m; k++)
		p->poly[k] = v[k].p;

	L = calloc(m + 1, sizeof (double));
	th = calloc(m + 1, sizeof (double));
	T = calloc(m + 1, sizeof (double));
	Tc = calloc(m + 1, sizeof (double));
	R = calloc(m + 1, sizeof (double));
	rp = calloc(m + 1, sizeof (double));
	vs = calloc(m + 1, sizeof (double));
	corner = calloc(m + 1, sizeof (bool));
	for (int k = 0; k + 1 < m; k++)
		L[k] = vdist(v[k].p, v[k + 1].p);
	for (int k = 1; k + 1 < m; k++) {
		char c1 = seg_cls(g, r, v[k].arr);
		char c2 = seg_cls(g, r, v[k + 1].arr);
		char c = (c1 && c2 ? (c1 < c2 ? c1 : c2) : (c1 ? c1 : c2));
		th[k] = relh(hdg_of(vsub(v[k].p, v[k - 1].p)),
		    hdg_of(vsub(v[k + 1].p, v[k].p)));
		rp[k] = (c >= 'A' && c <= 'F' ? o->r_pref[c - 'A'] :
		    o->r_unknown);
		if (fabs(th[k]) > o->manual_deg)
			corner[k] = true;
		else if (fabs(th[k]) > MIN_FILLET)
			T[k] = rp[k] * tan(rad(fabs(th[k]) / 2));
	}
	/*
	 * A tangent point may use up to half of an edge it shares with
	 * another arc; if that arc needs less than half, the rest is ours.
	 * Edges with nothing at the other end are ours entirely.
	 */
	for (int k = 1; k + 1 < m; k++) {
		double capL, capR;
		if (T[k] == 0)
			continue;
		capL = (T[k - 1] == 0 ? L[k - 1] :
		    fmax(L[k - 1] / 2, L[k - 1] - T[k - 1]));
		capR = (T[k + 1] == 0 ? L[k] : fmax(L[k] / 2, L[k] - T[k + 1]));
		Tc[k] = fmin(T[k], fmin(capL, capR));
		R[k] = Tc[k] / tan(rad(fabs(th[k]) / 2));
	}

	/* --- sample --- */
	b.p = p;
	b.cap = 0;
	emit(&b, v[0].p);
	vs[0] = 0;
	for (int k = 0; k + 1 < m; k++) {
		vect2_t d = vmul(vsub(v[k + 1].p, v[k].p), 1 / L[k]);
		vect2_t a = vadd(v[k].p, vmul(d, Tc[k]));
		vect2_t e = vsub(v[k + 1].p, vmul(d, Tc[k + 1]));
		double ls = vdist(a, e);
		int ns = (int)ceil(ls / step);

		if (k + 1 == m - 1)
			i_ext = p->n - 1;
		for (int j = 1; ls > 1e-6 && j <= ns; j++)
			emit(&b, vadd(a, vmul(vsub(e, a), (double)j / ns)));
		if (k + 1 == m - 1) {
			vs[k + 1] = cur_s(&b);
			break;
		}
		if (Tc[k + 1] > 0) {
			/* arc tangent to both edges */
			double t = th[k + 1], rr = R[k + 1];
			double sg = (t > 0 ? 1.0 : -1.0);
			double h0 = hdg_of(d);
			vect2_t c = vsub(e, vmul(v2(-cos(rad(h0)),
			    sin(rad(h0))), rr * sg));
			double A = rr * rad(fabs(t));
			int na = (int)ceil(A / step);
			ptx_arc_t *ar;

			if (na < 1)
				na = 1;
			p->arcs = realloc(p->arcs, (p->n_arcs + 1) *
			    sizeof (ptx_arc_t));
			ar = &p->arcs[p->n_arcs++];
			ar->vtx = k + 1;
			ar->theta = t;
			ar->r = rr;
			ar->r_pref = rp[k + 1];
			ar->i0 = p->n - 1;
			ar->s0 = cur_s(&b);
			ar->manual = (o->r_min > 0 && rr < o->r_min);
			for (int j = 1; j <= na; j++) {
				double h = rad(h0 + t * j / na);
				emit(&b, vadd(c, vmul(v2(-cos(h), sin(h)),
				    rr * sg)));
			}
			ar->i1 = p->n - 1;
			ar->s1 = cur_s(&b);
			vs[k + 1] = (ar->s0 + ar->s1) / 2;
		} else {
			vs[k + 1] = cur_s(&b);
			if (corner[k + 1]) {
				ptx_arc_t *ar;
				p->arcs = realloc(p->arcs, (p->n_arcs + 1) *
				    sizeof (ptx_arc_t));
				ar = &p->arcs[p->n_arcs++];
				ar->vtx = k + 1;
				ar->theta = th[k + 1];
				ar->r = 0;
				ar->r_pref = rp[k + 1];
				ar->i0 = ar->i1 = p->n - 1;
				ar->s0 = ar->s1 = vs[k + 1];
				ar->manual = true;
			}
		}
	}
	p->len = cur_s(&b);
	p->s_ext = p->s[i_ext];

	if (p->n > PTX_PATH_MAX_PTS) {
		free(v); free(map); free(L); free(th); free(T); free(Tc);
		free(R); free(rp); free(vs); free(corner);
		ptx_path_free(p);
		return (PTX_PATH_TOO_LONG);
	}
	p->flags = calloc(p->n + 1, sizeof (int));

	/* --- turns: same-way arcs close together are one turn --- */
	{
		p->turns = calloc(p->n_arcs + 1, sizeof (ptx_turn_t));
		for (int a = 0; a < p->n_arcs;) {
			int z = a;
			ptx_turn_t tu;
			const ptx_arc_t *A = &p->arcs[a];

			tu.s0 = A->s0;
			tu.s1 = A->s1;
			tu.deg = A->theta;
			tu.r_min = A->r;
			tu.manual = A->manual;
			while (!tu.manual && z + 1 < p->n_arcs) {
				const ptx_arc_t *B = &p->arcs[z + 1];
				if (B->manual || (B->theta > 0) != (tu.deg > 0) ||
				    B->s0 - tu.s1 >= MERGE_GAP)
					break;
				z++;
				tu.s1 = B->s1;
				tu.deg += B->theta;
				tu.r_min = fmin(tu.r_min, B->r);
			}
			if (fabs(tu.deg) > o->turn_deg || tu.manual) {
				p->turns[p->n_turns++] = tu;
				for (int q = a; q <= z; q++) {
					const ptx_arc_t *Q = &p->arcs[q];
					for (int i = Q->i0; i <= Q->i1; i++) {
						p->flags[i] |= (Q->r > 0 ?
						    PTX_PF_ARC : 0);
					}
					if (Q->r > 0 && (p->r_min_used == 0 ||
					    Q->r < p->r_min_used))
						p->r_min_used = Q->r;
					if (Q->r > 0 && Q->r < Q->r_pref - 1e-6 &&
					    fabs(Q->theta) > o->turn_deg)
						p->n_shrunk++;
				}
				if (tu.manual)
					p->n_manual++;
			}
			a = z + 1;
		}
	}
	/* MANUAL: arcs too tight for the aircraft, sharp corners */
	for (int a = 0; a < p->n_arcs; a++) {
		const ptx_arc_t *A = &p->arcs[a];
		if (!A->manual)
			continue;
		if (A->r > 0) {
			for (int i = A->i0; i <= A->i1; i++)
				p->flags[i] |= PTX_PF_MANUAL;
		} else {
			double D = fmin(A->r_pref, fmin(L[A->vtx - 1],
			    L[A->vtx]));
			for (int i = 0; i < p->n; i++) {
				if (fabs(p->s[i] - A->s0) <= D)
					p->flags[i] |= PTX_PF_MANUAL;
			}
		}
	}

	/* --- hold-short lines --- */
	nh = ptx_route_holds(g, r, ri, holds, PTX_PATH_MAX_HS);
	for (int h = 0; h < nh; h++) {
		/* the start node of route[i] is raw vertex i + off */
		double s = vs[map[holds[h].route_idx + off]];
		if (p->hs_n > 0 && s - p->hs_s[p->hs_n - 1] < 1.0) {
			/* same place: one line for both */
			size_t l = strlen(p->hs_name[p->hs_n - 1]);
			snprintf(p->hs_name[p->hs_n - 1] + l,
			    sizeof (p->hs_name[0]) - l, ",%s", holds[h].names);
			p->hs_dep[p->hs_n - 1] |= holds[h].departure;
			if (holds[h].departure)
				s_dep = p->hs_s[p->hs_n - 1];
			continue;
		}
		p->hs_s[p->hs_n] = s;
		snprintf(p->hs_name[p->hs_n], sizeof (p->hs_name[0]), "%s",
		    holds[h].names);
		p->hs_dep[p->hs_n] = holds[h].departure;
		if (holds[h].departure)
			s_dep = s;
		p->hs_n++;
	}
	{
		size_t l = 0;
		p->hs_names[0] = '\0';
		for (int h = 0; h < p->hs_n && l < sizeof (p->hs_names); h++) {
			l += snprintf(p->hs_names + l, sizeof (p->hs_names) - l,
			    "%s%s", h ? ";" : "", p->hs_name[h]);
		}
	}

	/* --- runway flags --- */
	for (int i = 0; i < p->n; i++) {
		vect2_t q = v2(p->x[i], p->y[i]);
		if (i >= i_ext || (p->s[i] >= s_dep &&
		    in_rwy(q, thr, u, rlen, rwidth, rwidth, NULL))) {
			p->flags[i] |= PTX_PF_RWY_DEP;
			continue;
		}
		for (int k = 0; k < ptx_n_rends(g); k += 2) {
			vect2_t t2, u2;
			double l2, w2;
			if (k / 2 == ri / 2)
				continue;
			ptx_rend_geom(g, k, &t2, &u2, &l2, &w2);
			if (in_rwy(q, t2, u2, l2, w2, 0, NULL)) {
				p->flags[i] |= PTX_PF_RWY_CROSS;
				break;
			}
		}
	}

	/* --- taxiway name spans --- */
	p->spans = calloc(m + 1, sizeof (ptx_span_t));
	for (int k = 0; k + 1 < m; k++) {
		char nm[sizeof (p->spans[0].name)];
		seg_name(g, r, ri, v[k + 1].arr, nm, sizeof (nm));
		if (p->n_spans > 0 &&
		    strcmp(p->spans[p->n_spans - 1].name, nm) == 0)
			continue;
		p->spans[p->n_spans].s = vs[k];
		snprintf(p->spans[p->n_spans].name,
		    sizeof (p->spans[0].name), "%s", nm);
		p->n_spans++;
	}

	free(v); free(map); free(L); free(th); free(T); free(Tc);
	free(R); free(rp); free(vs); free(corner);
	return (PTX_PATH_OK);
}

void
ptx_path_free(ptx_path_t *p)
{
	free(p->x);
	free(p->y);
	free(p->s);
	free(p->flags);
	free(p->turns);
	free(p->spans);
	free(p->poly);
	free(p->arcs);
	memset(p, 0, sizeof (*p));
}

int
ptx_path_span_at(const ptx_path_t *p, double s)
{
	int k = -1;
	for (int i = 0; i < p->n_spans; i++) {
		if (p->spans[i].s <= s)
			k = i;
		else
			break;
	}
	return (k);
}

double
ptx_path_track(const ptx_path_t *p, vect2_t q, double prev_s, double *off)
{
	double best_d = 1e18, best_s = prev_s;

	for (int pass = 0; pass < 2; pass++) {
		/*
		 * First only a little behind / ahead of where we were, and
		 * every metre of progress away from there costs TRACK_S_PEN
		 * metres of distance: a parallel leg coming back 1.5 m away
		 * (a hairpin) mustn't win over the leg we're on 2.5 m away.
		 */
		double lo = (pass == 0 ? prev_s - 20.0 : -1e18);
		double hi = (pass == 0 ? prev_s + 150.0 : 1e18);
		double pen = (pass == 0 ? TRACK_S_PEN : 0);
		double best_c = 1e18;
		for (int i = 0; i + 1 < p->n; i++) {
			double ax = p->x[i], ay = p->y[i];
			double bx = p->x[i + 1] - ax, by = p->y[i + 1] - ay;
			double l2 = bx * bx + by * by, t = 0, d, s, c;
			if (p->s[i + 1] < lo || p->s[i] > hi)
				continue;
			if (l2 > 0) {
				t = ((q.x - ax) * bx + (q.y - ay) * by) / l2;
				t = fmin(fmax(t, 0.0), 1.0);
			}
			d = hypot(q.x - ax - t * bx, q.y - ay - t * by);
			s = p->s[i] + t * (p->s[i + 1] - p->s[i]);
			c = d + pen * fabs(s - prev_s);
			if (c < best_c) {
				best_c = c;
				best_d = d;
				best_s = s;
			}
		}
		/* fall back to the whole path only when we're lost */
		if (best_d <= 30.0)
			break;
	}
	if (off != NULL)
		*off = best_d;
	return (best_s);
}

void
ptx_path_progress(const ptx_path_t *p, double prog_s, double *next_turn_s,
    double *next_turn_deg, const char **cur_name, const char **next_name)
{
	int k = ptx_path_span_at(p, prog_s);

	*next_turn_s = -1;
	*next_turn_deg = 0;
	for (int i = 0; i < p->n_turns; i++) {
		if (p->turns[i].s0 >= prog_s) {
			*next_turn_s = p->turns[i].s0;
			*next_turn_deg = p->turns[i].deg;
			break;
		}
	}
	*cur_name = (k >= 0 ? p->spans[k].name : "");
	*next_name = "";
	for (int i = (k < 0 ? 0 : k + 1); i < p->n_spans; i++) {
		if (p->spans[i].name[0] != '\0' &&
		    strcmp(p->spans[i].name, *cur_name) != 0) {
			*next_name = p->spans[i].name;
			break;
		}
	}
}
