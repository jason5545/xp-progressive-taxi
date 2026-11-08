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
 * C port of tools/proto/pbplan.py. Keep the two in lock step: same
 * constants, same iteration order (edges by index, +1 before -1), same
 * strict "<" tie breaking and the same floating point formulas (hypot,
 * x * (pi / 180)). Build with -ffp-contract=off so that no FMA contraction
 * changes the last bit compared to CPython.
 */

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <acfutils/geom.h>
#include <acfutils/list.h>

#include "../driving.h"
#include "ptx_plan.h"

/* ---- constants, must match tools/proto/pbplan.py ---- */
#define	R_SEARCH	300.0
#define	STEP		2.0
#define	W_PB		3.0
#define	W_TIGHT		3.0
#define	OBL_PEN		50.0
#define	ZONE_PEN	200.0
#define	MAX_ROT		150.0
#define	PB_MAX		350.0
#define	NOSE_TOL	1.5
#define	TIE_FRONT	80.0
#define	STAND_PEN	2000.0
#define	FOOT_HALF	6.0
#define	TWX_PEN		5000.0
#define	MAX_TURN	120.0
#define	SHARP_TURN	90.0
#define	SHARP_PEN	100.0
#define	CROSS_PEN	300.0
#define	BACKTRACK_MULT	1.5
#define	GRID_CELL	50.0
#define	CLASSIFY_R	300.0
#define	TIER_STD_LEN	200.0
#define	TIER_SUSPECT_LEN 250.0
#define	SEG_TURN_MULT_P	0.9	/* driving/segs.c SEG_TURN_MULT */
#define	MIN_TURN_RAD_P	1.5	/* driving/segs.c MIN_TURN_RADIUS */
#define	MAX_SEGS	8
#define	MAX_ZR		16

static const double FUSE_LEN[6] = { 9.0, 14.0, 38.0, 55.0, 64.0, 73.0 };

/* ---------------------------------------------------------------------- */
/* geometry, written the way the Python prototype computes it             */
/* ---------------------------------------------------------------------- */

static inline vect2_t v2(double x, double y) { vect2_t v = { x, y }; return (v); }
static inline vect2_t vadd(vect2_t a, vect2_t b) { return (v2(a.x + b.x, a.y + b.y)); }
static inline vect2_t vsub(vect2_t a, vect2_t b) { return (v2(a.x - b.x, a.y - b.y)); }
static inline vect2_t vmul(vect2_t a, double s) { return (v2(a.x * s, a.y * s)); }
static inline double vdot(vect2_t a, vect2_t b) { return (a.x * b.x + a.y * b.y); }
static inline double vlen(vect2_t a) { return (hypot(a.x, a.y)); }
static inline double vdist(vect2_t a, vect2_t b) { return (hypot(a.x - b.x, a.y - b.y)); }
static inline double rad(double d) { return (d * (M_PI / 180.0)); }
static inline double deg(double r) { return (r * (180.0 / M_PI)); }

static inline vect2_t
hdg2v(double h)
{
	double r = rad(h);
	return (v2(sin(r), cos(r)));
}

static double
py_asin(double x)
{
	if (x > 1.0 || x < -1.0)
		return (NAN);
	return (asin(x));
}

static double
v2hdg(vect2_t d)
{
	double a = vlen(d);
	if (d.x >= 0 && d.y >= 0)
		return (deg(py_asin(d.x / a)));
	if (d.x < 0 && d.y >= 0)
		return (360 + deg(py_asin(d.x / a)));
	return (180 - deg(py_asin(d.x / a)));
}

static double
relh(double h1, double h2)
{
	if (h1 > h2) {
		if (h1 > h2 + 180)
			return (360 - h1 + h2);
		return (-(h1 - h2));
	}
	if (h2 > h1 + 180)
		return (-(360 - h2 + h1));
	return (h2 - h1);
}

static double
normh(double h)
{
	h = fmod(h, 360.0);
	if (h < 0)
		h += 360.0;
	if (h <= 0.0 || h > 360.0) {
		if (h < 0.0)
			h = 0.0;
		if (h > 360.0)
			h = 360.0;
	}
	return (h);
}

static double
seg_dist(vect2_t p, vect2_t A, vect2_t B, double *t_out)
{
	vect2_t ab = vsub(B, A);
	double L2 = vdot(ab, ab);
	double t = 0.0;
	vect2_t q;

	if (L2 != 0) {
		t = vdot(vsub(p, A), ab) / L2;
		t = (t < 1.0 ? t : 1.0);
		t = (t > 0.0 ? t : 0.0);
	}
	q = vadd(A, vmul(ab, t));
	if (t_out != NULL)
		*t_out = t;
	return (vdist(p, q));
}

static bool
seg_isect(vect2_t p1, vect2_t p2, vect2_t p3, vect2_t p4, vect2_t *out)
{
	vect2_t d1 = vsub(p2, p1), d2 = vsub(p4, p3), w;
	double den = d1.x * d2.y - d1.y * d2.x, t, u;

	if (den == 0)
		return (false);
	w = vsub(p3, p1);
	t = (w.x * d2.y - w.y * d2.x) / den;
	u = (w.x * d1.y - w.y * d1.x) / den;
	if (0 <= t && t <= 1 && 0 <= u && u <= 1) {
		*out = vadd(p1, vmul(d1, t));
		return (true);
	}
	return (false);
}

static bool
cls_ok(char edge_cls, char ac_cls)
{
	if (edge_cls == 0 || ac_cls == 0)
		return (true);
	return (edge_cls >= ac_cls);
}

static int
cls_idx(char c)
{
	return (c >= 'A' && c <= 'F' ? c - 'A' : -1);
}

/* ---------------------------------------------------------------------- */
/* graph                                                                  */
/* ---------------------------------------------------------------------- */

typedef struct {
	int		a, b;		/* node indices */
	vect2_t		A, B;
	double		L, hdg;
	bool		oneway, runway, leadin;
	char		cls;
	const ptx_edge_t *src;
	int		n_zr;
	char		zr[MAX_ZR][PTX_RWY_LEN];	/* zone runways, normalized */
} gedge_t;

typedef struct {
	int	eid, from, to;
	double	hdg, L;
} gde_t;

typedef struct rtab {
	char		cls;
	double		w_along;
	double		*togo;
	int		*nxt;
	struct rtab	*next;
} rtab_t;

typedef struct {
	char		name[PTX_RWY_LEN];
	char		pair[2][PTX_RWY_LEN];
	vect2_t		thr, u;
	double		len, width, hdg;
	uint8_t		*rw_edge;
	int		target;
	double		target_along, end_along;
	int		n_entries;
	int		*entry_node;
	double		*entry_lost;
	rtab_t		*tabs;
} rend_t;

typedef struct {
	int	*v;
	int	n;
} ilist_t;

struct ptx_graph {
	const ptx_apt_t	*apt;
	double		lat0, lon0, k, cl;

	int		n_nodes;	/* == apt->n_nodes */
	vect2_t		*node_p;
	int		*id_sorted;	/* node indices sorted by (id, idx) */
	int		n_ids;

	gedge_t		*edges;
	int		n_edges;
	gde_t		*de;
	int		n_de;
	int		*de_index;	/* 2 per edge: [+1, -1] */
	ilist_t		*out, *inc;	/* per node */
	int		*deg;

	int		gx0, gy0, gw, gh;
	ilist_t		*cells;

	vect2_t		*stand_p, *foot_a, *foot_b;

	rend_t		*rends;
	int		n_rends;

	/* scratch for near() */
	uint32_t	*mark;
	uint32_t	mark_gen;
};

static void
ilist_push(ilist_t *l, int v)
{
	if ((l->n & (l->n - 1)) == 0 || l->n == 0) {
		int cap = (l->n == 0 ? 4 : l->n * 2);
		l->v = realloc(l->v, cap * sizeof (*l->v));
	}
	l->v[l->n++] = v;
}

vect2_t
ptx_project(const ptx_graph_t *g, double lat, double lon)
{
	return (v2((lon - g->lon0) * g->cl * g->k, (lat - g->lat0) * g->k));
}

void
ptx_unproject(const ptx_graph_t *g, vect2_t p, double *lat, double *lon)
{
	*lat = g->lat0 + p.y / g->k;
	*lon = g->lon0 + p.x / (g->cl * g->k);
}

typedef struct {
	int	id;
	int	idx;
} idpair_t;

static int
cmp_idpair(const void *a, const void *b)
{
	const idpair_t *x = a, *y = b;
	if (x->id != y->id)
		return (x->id < y->id ? -1 : 1);
	return (x->idx < y->idx ? -1 : (x->idx > y->idx ? 1 : 0));
}

/* node index for a file id (the last definition wins, like a dict) */
static int
node_lookup(const ptx_graph_t *g, int id)
{
	int lo = 0, hi = g->n_ids - 1, found = -1;
	while (lo <= hi) {
		int mid = (lo + hi) / 2;
		int mid_id = g->apt->nodes[g->id_sorted[mid]].id;
		if (mid_id < id) {
			lo = mid + 1;
		} else if (mid_id > id) {
			hi = mid - 1;
		} else {
			found = mid;
			lo = mid + 1;	/* keep looking for the last one */
		}
	}
	return (found < 0 ? -1 : g->id_sorted[found]);
}

static int
cmp_int(const void *a, const void *b)
{
	int x = *(const int *)a, y = *(const int *)b;
	return (x < y ? -1 : (x > y ? 1 : 0));
}

/*
 * Edges whose segment lies within R of p, ascending edge index. Returns
 * the count; out must hold n_edges entries.
 */
static int
near_edges(ptx_graph_t *g, vect2_t p, double R, int *out)
{
	int n = 0;
	int x0 = (int)floor((p.x - R) / GRID_CELL);
	int x1 = (int)floor((p.x + R) / GRID_CELL);
	int y0 = (int)floor((p.y - R) / GRID_CELL);
	int y1 = (int)floor((p.y + R) / GRID_CELL);
	int m = 0;

	g->mark_gen++;
	if (g->mark_gen == 0) {
		memset(g->mark, 0, g->n_edges * sizeof (*g->mark));
		g->mark_gen = 1;
	}
	for (int gx = x0; gx <= x1; gx++) {
		if (gx < g->gx0 || gx >= g->gx0 + g->gw)
			continue;
		for (int gy = y0; gy <= y1; gy++) {
			const ilist_t *c;
			if (gy < g->gy0 || gy >= g->gy0 + g->gh)
				continue;
			c = &g->cells[(gx - g->gx0) * g->gh + (gy - g->gy0)];
			for (int i = 0; i < c->n; i++) {
				int e = c->v[i];
				if (g->mark[e] == g->mark_gen)
					continue;
				g->mark[e] = g->mark_gen;
				out[m++] = e;
			}
		}
	}
	qsort(out, m, sizeof (*out), cmp_int);
	for (int i = 0; i < m; i++) {
		const gedge_t *e = &g->edges[out[i]];
		if (seg_dist(p, e->A, e->B, NULL) <= R)
			out[n++] = out[i];
	}
	return (n);
}

/* nearest taxiway edge: distance (1e18 if none), edge index via e_out */
static double
nearest_taxi(ptx_graph_t *g, vect2_t p, double R, bool excl_leadin,
    int *e_out, int *buf)
{
	double best = 1e18;
	int n = near_edges(g, p, R, buf);

	if (e_out != NULL)
		*e_out = -1;
	for (int i = 0; i < n; i++) {
		const gedge_t *e = &g->edges[buf[i]];
		double d;
		if (e->runway)
			continue;
		if (excl_leadin && e->leadin)
			continue;
		d = seg_dist(p, e->A, e->B, NULL);
		if (d < best) {
			best = d;
			if (e_out != NULL)
				*e_out = buf[i];
		}
	}
	return (best);
}

static bool
zr_has(const char zr[][PTX_RWY_LEN], int n, const char *rw)
{
	for (int i = 0; i < n; i++) {
		if (strcmp(zr[i], rw) == 0)
			return (true);
	}
	return (false);
}

static bool
zr_meets_pair(const gedge_t *e, const rend_t *r)
{
	return (zr_has(e->zr, e->n_zr, r->pair[0]) ||
	    zr_has(e->zr, e->n_zr, r->pair[1]));
}

/* rwy_tokens(name) & pair, splitting on / whitespace , - */
static bool
name_meets_pair(const char *name, const rend_t *r)
{
	char buf[PTX_NAME_LEN];
	char *save = NULL, *tok;

	strncpy(buf, name, sizeof (buf) - 1);
	buf[sizeof (buf) - 1] = '\0';
	for (tok = strtok_r(buf, "/ \t,-", &save); tok != NULL;
	    tok = strtok_r(NULL, "/ \t,-", &save)) {
		char n[PTX_RWY_LEN];
		ptx_rwy_norm(tok, n);
		if (strcmp(n, r->pair[0]) == 0 || strcmp(n, r->pair[1]) == 0)
			return (true);
	}
	return (false);
}

static void
build_rends(ptx_graph_t *g)
{
	const ptx_apt_t *apt = g->apt;

	g->n_rends = apt->n_rwys * 2;
	g->rends = calloc(g->n_rends > 0 ? g->n_rends : 1, sizeof (rend_t));
	for (int ri = 0; ri < apt->n_rwys; ri++) {
		const ptx_runway_t *rw = &apt->rwys[ri];
		vect2_t p0 = ptx_project(g, rw->end[0].lat, rw->end[0].lon);
		vect2_t p1 = ptx_project(g, rw->end[1].lat, rw->end[1].lon);
		vect2_t du = vsub(p1, p0);
		double L = vlen(du);
		vect2_t u = vmul(du, 1 / L);

		for (int i = 0; i < 2; i++) {
			rend_t *r = &g->rends[ri * 2 + i];
			int *nodes, nn = 0;
			int *ent;
			int ne = 0;

			strncpy(r->name, rw->end[i].name, PTX_RWY_LEN - 1);
			ptx_rwy_norm(rw->end[0].name, r->pair[0]);
			ptx_rwy_norm(rw->end[1].name, r->pair[1]);
			r->thr = (i == 0 ? p0 : p1);
			r->u = (i == 0 ? u : v2(-u.x, -u.y));
			r->len = L;
			r->width = rw->width;
			r->hdg = v2hdg(r->u);
			r->rw_edge = calloc(g->n_edges > 0 ? g->n_edges : 1, 1);
			r->target = -1;
			r->target_along = NAN;
			r->end_along = NAN;

			nodes = malloc((2 * g->n_edges + 1) * sizeof (int));
			for (int e = 0; e < g->n_edges; e++) {
				const gedge_t *ge = &g->edges[e];
				if (!ge->runway || !name_meets_pair(ge->src->name, r))
					continue;
				r->rw_edge[e] = 1;
				nodes[nn++] = ge->a;
				nodes[nn++] = ge->b;
			}
			if (nn == 0) {
				free(nodes);
				continue;
			}
			/* sorted(set(node ids)) */
			{
				int *ids = malloc(nn * sizeof (int));
				int k = 0;
				for (int j = 0; j < nn; j++)
					ids[j] = apt->nodes[nodes[j]].id;
				qsort(ids, nn, sizeof (int), cmp_int);
				for (int j = 0; j < nn; j++) {
					if (k == 0 || ids[k - 1] != ids[j])
						ids[k++] = ids[j];
				}
				for (int j = 0; j < k; j++)
					nodes[j] = node_lookup(g, ids[j]);
				nn = k;
				free(ids);
			}
			ent = malloc(nn * sizeof (int));
			for (int j = 0; j < nn; j++) {
				int n = nodes[j];
				double al = vdot(vsub(g->node_p[n], r->thr), r->u);
				bool enter = false;
				if (j == 0 || al < r->end_along)
					r->end_along = al;
				for (int d = 0; d < g->inc[n].n; d++) {
					if (!g->edges[g->de[g->inc[n].v[d]].eid].runway) {
						enter = true;
						break;
					}
				}
				if (enter && al <= r->len / 2)
					ent[ne++] = n;
			}
			if (ne > 0) {
				int best = -1;
				double best_al = 0;
				for (int j = 0; j < ne; j++) {
					double al = vdot(vsub(g->node_p[ent[j]],
					    r->thr), r->u);
					if (best < 0 || al < best_al) {
						best = ent[j];
						best_al = al;
					}
				}
				r->target = best;
				r->target_along = best_al;
				r->n_entries = ne;
				r->entry_node = malloc(ne * sizeof (int));
				r->entry_lost = malloc(ne * sizeof (double));
				for (int j = 0; j < ne; j++) {
					double al = vdot(vsub(g->node_p[ent[j]],
					    r->thr), r->u);
					r->entry_node[j] = ent[j];
					r->entry_lost[j] = al - best_al;
				}
			}
			free(ent);
			free(nodes);
		}
	}
}

ptx_graph_t *
ptx_graph_new(const ptx_apt_t *apt)
{
	ptx_graph_t *g = calloc(1, sizeof (*g));
	double slat = 0, slon = 0;
	int nr = 0;
	double xmin = 1e300, xmax = -1e300, ymin = 1e300, ymax = -1e300;

	g->apt = apt;
	for (int i = 0; i < apt->n_rwys; i++) {
		for (int j = 0; j < 2; j++) {
			slat += apt->rwys[i].end[j].lat;
			nr++;
		}
	}
	for (int i = 0; i < apt->n_rwys; i++) {
		for (int j = 0; j < 2; j++)
			slon += apt->rwys[i].end[j].lon;
	}
	if (nr == 0 && apt->n_nodes > 0) {
		slat = apt->nodes[0].lat;
		slon = apt->nodes[0].lon;
		nr = 1;
	}
	g->lat0 = (nr > 0 ? slat / nr : 0);
	g->lon0 = (nr > 0 ? slon / nr : 0);
	g->k = M_PI / 180 * 6371008.8;
	g->cl = cos(rad(g->lat0));

	/* nodes */
	g->n_nodes = apt->n_nodes;
	g->node_p = calloc(g->n_nodes + 1, sizeof (vect2_t));
	for (int i = 0; i < apt->n_nodes; i++)
		g->node_p[i] = ptx_project(g, apt->nodes[i].lat,
		    apt->nodes[i].lon);
	{
		idpair_t *ip = malloc((g->n_nodes + 1) * sizeof (idpair_t));
		for (int i = 0; i < g->n_nodes; i++) {
			ip[i].id = apt->nodes[i].id;
			ip[i].idx = i;
		}
		qsort(ip, g->n_nodes, sizeof (idpair_t), cmp_idpair);
		g->id_sorted = malloc((g->n_nodes + 1) * sizeof (int));
		for (int i = 0; i < g->n_nodes; i++)
			g->id_sorted[i] = ip[i].idx;
		free(ip);
	}
	g->n_ids = g->n_nodes;

	/* edges */
	g->edges = calloc(apt->n_edges + 1, sizeof (gedge_t));
	for (int i = 0; i < apt->n_edges; i++) {
		const ptx_edge_t *pe = &apt->edges[i];
		int a = node_lookup(g, pe->a), b = node_lookup(g, pe->b);
		gedge_t *e;
		double L;

		if (a < 0 || b < 0 || pe->a == pe->b)
			continue;
		L = vdist(g->node_p[a], g->node_p[b]);
		if (L < 0.05)
			continue;
		e = &g->edges[g->n_edges++];
		e->a = a;
		e->b = b;
		e->A = g->node_p[a];
		e->B = g->node_p[b];
		e->L = L;
		e->hdg = v2hdg(vsub(e->B, e->A));
		e->oneway = pe->oneway;
		e->runway = pe->runway;
		e->cls = pe->cls;
		e->src = pe;
		for (int z = 0; z < pe->n_zones; z++) {
			for (int k = 0; k < pe->zones[z].n_rwys; k++) {
				const char *rw = pe->zones[z].rwys[k];
				if (!zr_has(e->zr, e->n_zr, rw) &&
				    e->n_zr < MAX_ZR) {
					strcpy(e->zr[e->n_zr++], rw);
				}
			}
		}
	}

	/* directed edges, out/inc */
	g->de = calloc(2 * g->n_edges + 1, sizeof (gde_t));
	g->de_index = malloc((2 * g->n_edges + 1) * sizeof (int));
	g->out = calloc(g->n_nodes + 1, sizeof (ilist_t));
	g->inc = calloc(g->n_nodes + 1, sizeof (ilist_t));
	for (int i = 0; i < g->n_edges; i++) {
		gedge_t *e = &g->edges[i];
		g->de_index[2 * i] = g->de_index[2 * i + 1] = -1;
		for (int s = 0; s < 2; s++) {
			gde_t *d;
			if (s == 1 && e->oneway)
				break;
			d = &g->de[g->n_de];
			d->eid = i;
			d->from = (s == 0 ? e->a : e->b);
			d->to = (s == 0 ? e->b : e->a);
			d->hdg = (s == 0 ? e->hdg : normh(e->hdg + 180));
			d->L = e->L;
			g->de_index[2 * i + s] = g->n_de;
			ilist_push(&g->out[d->from], g->n_de);
			ilist_push(&g->inc[d->to], g->n_de);
			g->n_de++;
		}
	}
	g->deg = calloc(g->n_nodes + 1, sizeof (int));
	for (int i = 0; i < g->n_edges; i++) {
		g->deg[g->edges[i].a]++;
		g->deg[g->edges[i].b]++;
	}

	/* grid */
	for (int i = 0; i < g->n_edges; i++) {
		const gedge_t *e = &g->edges[i];
		xmin = fmin(xmin, fmin(e->A.x, e->B.x));
		xmax = fmax(xmax, fmax(e->A.x, e->B.x));
		ymin = fmin(ymin, fmin(e->A.y, e->B.y));
		ymax = fmax(ymax, fmax(e->A.y, e->B.y));
	}
	if (g->n_edges == 0)
		xmin = xmax = ymin = ymax = 0;
	g->gx0 = (int)floor(xmin / GRID_CELL);
	g->gy0 = (int)floor(ymin / GRID_CELL);
	g->gw = (int)floor(xmax / GRID_CELL) - g->gx0 + 1;
	g->gh = (int)floor(ymax / GRID_CELL) - g->gy0 + 1;
	g->cells = calloc((size_t)g->gw * g->gh, sizeof (ilist_t));
	for (int i = 0; i < g->n_edges; i++) {
		const gedge_t *e = &g->edges[i];
		int x0 = (int)floor(fmin(e->A.x, e->B.x) / GRID_CELL);
		int x1 = (int)floor(fmax(e->A.x, e->B.x) / GRID_CELL);
		int y0 = (int)floor(fmin(e->A.y, e->B.y) / GRID_CELL);
		int y1 = (int)floor(fmax(e->A.y, e->B.y) / GRID_CELL);
		for (int gx = x0; gx <= x1; gx++) {
			for (int gy = y0; gy <= y1; gy++) {
				ilist_push(&g->cells[(gx - g->gx0) * g->gh +
				    (gy - g->gy0)], i);
			}
		}
	}
	g->mark = calloc(g->n_edges + 1, sizeof (uint32_t));

	/* stands and footprints */
	g->stand_p = calloc(apt->n_stands + 1, sizeof (vect2_t));
	g->foot_a = calloc(apt->n_stands + 1, sizeof (vect2_t));
	g->foot_b = calloc(apt->n_stands + 1, sizeof (vect2_t));
	for (int i = 0; i < apt->n_stands; i++) {
		const ptx_stand_t *s = &apt->stands[i];
		int ci = cls_idx(s->cls ? s->cls : 'C');
		double fl = (ci >= 0 ? FUSE_LEN[ci] : 38.0);
		vect2_t back = vmul(hdg2v(s->hdg), -fl * 0.95);
		g->stand_p[i] = ptx_project(g, s->lat, s->lon);
		g->foot_a[i] = g->stand_p[i];
		g->foot_b[i] = vadd(g->stand_p[i], back);
	}

	/* lead-in stubs: dead-end taxiway node within 20 m of a stand */
	for (int i = 0; i < g->n_edges; i++) {
		gedge_t *e = &g->edges[i];
		int ends[2] = { e->a, e->b };
		if (e->runway)
			continue;
		for (int j = 0; j < 2 && !e->leadin; j++) {
			if (g->deg[ends[j]] != 1)
				continue;
			for (int s = 0; s < apt->n_stands; s++) {
				if (vdist(g->node_p[ends[j]], g->stand_p[s]) < 20) {
					e->leadin = true;
					break;
				}
			}
		}
	}

	build_rends(g);
	return (g);
}

void
ptx_graph_free(ptx_graph_t *g)
{
	if (g == NULL)
		return;
	for (int i = 0; i < g->n_rends; i++) {
		rend_t *r = &g->rends[i];
		rtab_t *t, *tn;
		for (t = r->tabs; t != NULL; t = tn) {
			tn = t->next;
			free(t->togo);
			free(t->nxt);
			free(t);
		}
		free(r->rw_edge);
		free(r->entry_node);
		free(r->entry_lost);
	}
	free(g->rends);
	for (int i = 0; i < g->n_nodes; i++) {
		free(g->out[i].v);
		free(g->inc[i].v);
	}
	for (int i = 0; i < g->gw * g->gh; i++)
		free(g->cells[i].v);
	free(g->cells);
	free(g->out);
	free(g->inc);
	free(g->deg);
	free(g->de);
	free(g->de_index);
	free(g->edges);
	free(g->node_p);
	free(g->id_sorted);
	free(g->mark);
	free(g->stand_p);
	free(g->foot_a);
	free(g->foot_b);
	free(g);
}

/* ---------------------------------------------------------------------- */
/* routing: reverse Dijkstra over directed edges                          */
/* ---------------------------------------------------------------------- */

typedef struct {
	double	c;
	int	d;
} hitem_t;

typedef struct {
	hitem_t	*h;
	int	n, cap;
} heap_t;

static inline bool
hless(hitem_t a, hitem_t b)
{
	return (a.c < b.c || (a.c == b.c && a.d < b.d));
}

static void
hpush(heap_t *hp, double c, int d)
{
	int i;
	if (hp->n == hp->cap) {
		hp->cap = (hp->cap ? hp->cap * 2 : 256);
		hp->h = realloc(hp->h, hp->cap * sizeof (hitem_t));
	}
	i = hp->n++;
	hp->h[i].c = c;
	hp->h[i].d = d;
	while (i > 0) {
		int p = (i - 1) / 2;
		hitem_t t;
		if (!hless(hp->h[i], hp->h[p]))
			break;
		t = hp->h[i];
		hp->h[i] = hp->h[p];
		hp->h[p] = t;
		i = p;
	}
}

static hitem_t
hpop(heap_t *hp)
{
	hitem_t top = hp->h[0];
	int i = 0;

	hp->h[0] = hp->h[--hp->n];
	for (;;) {
		int l = 2 * i + 1, r = l + 1, m = i;
		hitem_t t;
		if (l < hp->n && hless(hp->h[l], hp->h[m]))
			m = l;
		if (r < hp->n && hless(hp->h[r], hp->h[m]))
			m = r;
		if (m == i)
			break;
		t = hp->h[i];
		hp->h[i] = hp->h[m];
		hp->h[m] = t;
		i = m;
	}
	return (top);
}

static double
de_cost(const ptx_graph_t *g, const rend_t *r, int did)
{
	const gde_t *d = &g->de[did];
	const gedge_t *e = &g->edges[d->eid];

	if (e->runway) {
		if (r->rw_edge[d->eid])
			return (d->L * BACKTRACK_MULT);
		return (d->L + CROSS_PEN);
	}
	if (e->n_zr > 0 && !zr_meets_pair(e, r))
		return (d->L + CROSS_PEN / 3);
	return (d->L);
}

static rtab_t *
route_table(ptx_graph_t *g, rend_t *r, char cls, double w_along)
{
	rtab_t *t;
	heap_t hp = { NULL, 0, 0 };

	for (t = r->tabs; t != NULL; t = t->next) {
		if (t->cls == cls && t->w_along == w_along)
			return (t);
	}
	t = calloc(1, sizeof (*t));
	t->cls = cls;
	t->w_along = w_along;
	t->togo = malloc((g->n_de + 1) * sizeof (double));
	t->nxt = malloc((g->n_de + 1) * sizeof (int));
	for (int i = 0; i < g->n_de; i++) {
		t->togo[i] = INFINITY;
		t->nxt[i] = -1;
	}
	t->next = r->tabs;
	r->tabs = t;
	if (r->target < 0)
		return (t);

	for (int j = 0; j < r->n_entries; j++) {
		const ilist_t *inc = &g->inc[r->entry_node[j]];
		for (int k = 0; k < inc->n; k++) {
			int did = inc->v[k];
			const gedge_t *e = &g->edges[g->de[did].eid];
			double c0;
			if (e->runway || !cls_ok(e->cls, cls))
				continue;
			c0 = w_along * r->entry_lost[j];
			if (c0 < t->togo[did]) {
				t->togo[did] = c0;
				hpush(&hp, c0, did);
			}
		}
	}
	while (hp.n > 0) {
		hitem_t it = hpop(&hp);
		const gde_t *d2 = &g->de[it.d];
		double c2;
		const ilist_t *inc;

		if (it.c > t->togo[it.d])
			continue;
		c2 = de_cost(g, r, it.d);
		inc = &g->inc[d2->from];
		for (int k = 0; k < inc->n; k++) {
			int d1 = inc->v[k];
			const gde_t *dd1 = &g->de[d1];
			const gedge_t *e1 = &g->edges[dd1->eid];
			double ang, cand;
			if (dd1->eid == d2->eid)
				continue;
			if (!cls_ok(e1->cls, cls))
				continue;
			if (r->rw_edge[dd1->eid])
				continue;
			ang = fabs(relh(dd1->hdg, d2->hdg));
			if (ang > MAX_TURN)
				continue;
			cand = it.c + c2 + (ang > SHARP_TURN ? SHARP_PEN : 0.0);
			if (cand < t->togo[d1]) {
				t->togo[d1] = cand;
				t->nxt[d1] = it.d;
				hpush(&hp, cand, d1);
			}
		}
	}
	free(hp.h);
	return (t);
}

/* ---------------------------------------------------------------------- */
/* pushback candidates                                                    */
/* ---------------------------------------------------------------------- */

typedef struct {
	bool	turn;
	vect2_t	sp, ep;
	double	sh, eh;
	bool	backward;
	double	len, r;
} pseg_t;

typedef struct {
	double	sc0;
	int	seq;
	int	eid, sgn;
	double	t;
	vect2_t	E;
	double	eh;
	pseg_t	segs[MAX_SEGS];
	int	nseg;
	double	pb_len, rot, r_used, to_node, base;
	bool	zone;
} cand_t;

typedef struct {
	bool	valid;
	cand_t	c;
	vect2_t	*pts;
	int	n_pts;
	int	*conf, n_conf;
	int	*xing, n_xing;
	double	score;
} best_t;

static double
pseg_len(const pseg_t *s)
{
	if (!s->turn)
		return (fabs(s->len));
	return (s->r * fabs(rad(relh(s->sh, s->eh))));
}

static void
pts_push(vect2_t **pts, int *n, int *cap, vect2_t p)
{
	if (*n == *cap) {
		*cap = (*cap ? *cap * 2 : 64);
		*pts = realloc(*pts, *cap * sizeof (vect2_t));
	}
	(*pts)[(*n)++] = p;
}

static void
seg_points(const pseg_t *s, bool first, vect2_t **pts, int *n, int *cap)
{
	double sign = (s->backward ? -1.0 : 1.0);

	if (!s->turn) {
		double L = vdist(s->sp, s->ep);
		int m = (int)ceil(L / 2.0);
		if (m < 1)
			m = 1;
		for (int k = (first ? 0 : 1); k <= m; k++) {
			pts_push(pts, n, cap, vadd(s->sp,
			    vmul(vsub(s->ep, s->sp), (double)k / m)));
		}
	} else {
		double d = relh(s->sh, s->eh);
		double L = pseg_len(s);
		int m = (int)ceil(fabs(d) / 4.0);
		vect2_t p = s->sp;
		if (m < 8)
			m = 8;
		if (first)
			pts_push(pts, n, cap, p);
		for (int k = 0; k < m; k++) {
			double h = s->sh + d * (k + 0.5) / m;
			p = vadd(p, vmul(hdg2v(h), sign * L / m));
			pts_push(pts, n, cap, p);
		}
	}
}

static bool
in_runway(const ptx_graph_t *g, vect2_t p)
{
	for (int i = 0; i < g->n_rends; i += 2) {
		const rend_t *r = &g->rends[i];
		vect2_t rel = vsub(p, r->thr);
		double al = vdot(rel, r->u);
		double lat = fabs(rel.x * r->u.y - rel.y * r->u.x);
		if (-0.0 <= al && al <= r->len && lat <= r->width / 2)
			return (true);
	}
	return (false);
}

static int
cmp_cand(const void *a, const void *b)
{
	const cand_t *x = a, *y = b;
	if (x->eid != y->eid)
		return (x->eid < y->eid ? -1 : 1);
	if (x->sgn != y->sgn)
		return (x->sgn > y->sgn ? -1 : 1);	/* +1 first */
	if (x->sc0 != y->sc0)
		return (x->sc0 < y->sc0 ? -1 : 1);
	return (x->seq < y->seq ? -1 : (x->seq > y->seq ? 1 : 0));
}

static void
best_free(best_t *b)
{
	free(b->pts);
	free(b->conf);
	free(b->xing);
	memset(b, 0, sizeof (*b));
}

/*
 * Returns the per (edge, direction) best candidates, sorted by
 * (edge asc, +1 before -1). *n_out is the count.
 */
static best_t *
find_candidates(ptx_graph_t *g, vect2_t Pm, vect2_t Pn, double H,
    int own_stand, const ptx_aircraft_t *ac, ptx_diag_t *diag, int *n_out)
{
	const ptx_apt_t *apt = g->apt;
	vect2_t fwd = hdg2v(H);
	double wb = ac->wheelbase;
	vehicle_t veh;
	double rmin, r_pref;
	int *buf = malloc((g->n_edges + 1) * sizeof (int));
	int *nearv = malloc((g->n_edges + 1) * sizeof (int));
	int *taxi_near = malloc((g->n_edges + 1) * sizeof (int));
	int n_near, n_taxi_near = 0;
	int *foots = malloc((apt->n_stands + 1) * sizeof (int));
	int n_foots = 0;
	cand_t *pool = NULL;
	int n_pool = 0, cap_pool = 0;
	best_t *best = NULL;
	int n_best = 0;
	list_t segl;

	memset(&veh, 0, sizeof (veh));
	veh.wheelbase = wb;
	veh.max_steer = ac->max_steer;
	rmin = tan(rad(90 - ac->max_steer * SEG_TURN_MULT_P)) * wb;
	if (rmin < MIN_TURN_RAD_P)
		rmin = MIN_TURN_RAD_P;
	r_pref = (wb > rmin * 1.15 ? wb : rmin * 1.15);
	list_create(&segl, sizeof (seg_t), offsetof(seg_t, node));

	for (int s = 0; s < apt->n_stands; s++) {
		vect2_t fa = g->foot_a[s], fb = g->foot_b[s];
		if (s == own_stand || vdist(g->stand_p[s], Pn) < 10.0)
			continue;
		if (seg_dist(Pm, fa, fb, NULL) < FOOT_HALF ||
		    seg_dist(Pn, fa, fb, NULL) < FOOT_HALF)
			continue;
		if (vdist(fa, Pm) > R_SEARCH + 100)
			continue;
		foots[n_foots++] = s;
	}
	n_near = near_edges(g, Pm, R_SEARCH + 50, nearv);
	for (int i = 0; i < n_near; i++) {
		const gedge_t *e = &g->edges[nearv[i]];
		if (!e->runway && !e->leadin)
			taxi_near[n_taxi_near++] = nearv[i];
	}

	/* first pass: cheap geometry */
	n_near = near_edges(g, Pm, R_SEARCH, nearv);
	for (int i = 0; i < n_near; i++) {
		const gedge_t *e = &g->edges[nearv[i]];
		int n;
		if (e->runway)
			continue;
		if (e->leadin) {
			diag->leadin++;
			continue;
		}
		if (!cls_ok(e->cls, ac->route_cls)) {
			diag->narrow++;
			continue;
		}
		n = (int)(e->L / STEP);
		if (n < 1)
			n = 1;
		for (int k = 0; k <= n; k++) {
			double t = (double)k / n;
			vect2_t E = vadd(e->A, vmul(vsub(e->B, e->A), t));
			vect2_t rel = vsub(E, Pm);
			if (vdot(rel, fwd) > -2.0) {
				diag->not_behind++;
				continue;
			}
			if (vlen(rel) > R_SEARCH)
				continue;
			for (int sgn = 1; sgn >= -1; sgn -= 2) {
				double eh;
				int nseg;
				seg_t *sg;
				cand_t c;
				bool fwdseg = false;
				double r_used = INFINITY;
				vect2_t endp, nosep;

				if (e->oneway && sgn == -1)
					continue;
				eh = (sgn == 1 ? e->hdg : normh(e->hdg + 180));
				nseg = compute_segs(&veh, Pm, H, E, eh, &segl);
				diag->tried++;
				if (nseg <= 0) {
					while ((sg = list_remove_head(&segl)) != NULL)
						free(sg);
					diag->segs_fail++;
					continue;
				}
				memset(&c, 0, sizeof (c));
				c.nseg = 0;
				while ((sg = list_remove_head(&segl)) != NULL) {
					if (c.nseg < MAX_SEGS) {
						pseg_t *p = &c.segs[c.nseg++];
						p->turn = (sg->type == SEG_TYPE_TURN);
						p->sp = sg->start_pos;
						p->ep = sg->end_pos;
						p->sh = sg->start_hdg;
						p->eh = sg->end_hdg;
						p->backward = sg->backward;
						p->len = (p->turn ? 0 : sg->len);
						p->r = (p->turn ? sg->turn.r : 0);
					}
					free(sg);
				}
				for (int j = 0; j < c.nseg; j++) {
					if (!c.segs[j].backward)
						fwdseg = true;
				}
				if (fwdseg) {
					diag->has_forward++;
					continue;
				}
				c.pb_len = 0;
				for (int j = 0; j < c.nseg; j++)
					c.pb_len += pseg_len(&c.segs[j]);
				if (c.pb_len > PB_MAX) {
					diag->too_long++;
					continue;
				}
				c.rot = 0;
				for (int j = 0; j < c.nseg; j++) {
					if (c.segs[j].turn)
						c.rot += fabs(relh(c.segs[j].sh,
						    c.segs[j].eh));
				}
				if (c.rot > MAX_ROT) {
					diag->rot++;
					continue;
				}
				endp = c.segs[c.nseg - 1].ep;
				if (vdist(endp, E) > 0.5) {
					diag->off_line++;
					continue;
				}
				nosep = vadd(endp, vmul(hdg2v(eh), wb));
				if (nearest_taxi(g, nosep, 30.0, false, NULL,
				    buf) > NOSE_TOL) {
					diag->nose_off++;
					continue;
				}
				for (int j = 0; j < c.nseg; j++) {
					if (c.segs[j].turn && c.segs[j].r < r_used)
						r_used = c.segs[j].r;
				}
				c.r_used = r_used;
				c.eid = nearv[i];
				c.sgn = sgn;
				c.t = t;
				c.E = E;
				c.eh = eh;
				c.zone = (e->src->n_zones > 0);
				c.to_node = (sgn == 1 ? (1 - t) * e->L : t * e->L);
				c.base = W_PB * c.pb_len +
				    W_TIGHT * fmax(0.0, r_pref - r_used) +
				    (c.nseg > 2 ? OBL_PEN : 0.0) +
				    (c.zone ? ZONE_PEN : 0.0);
				c.sc0 = c.base + c.to_node;
				c.seq = n_pool;
				if (n_pool == cap_pool) {
					cap_pool = (cap_pool ? cap_pool * 2 : 256);
					pool = realloc(pool, cap_pool *
					    sizeof (cand_t));
				}
				pool[n_pool++] = c;
			}
		}
	}
	list_destroy(&segl);

	/* second pass: branch and bound with the expensive checks */
	qsort(pool, n_pool, sizeof (cand_t), cmp_cand);
	best = calloc(n_pool + 1, sizeof (best_t));
	for (int i = 0; i < n_pool;) {
		int j = i;
		best_t cur;
		memset(&cur, 0, sizeof (cur));
		while (j < n_pool && pool[j].eid == pool[i].eid &&
		    pool[j].sgn == pool[i].sgn)
			j++;
		for (int k = i; k < j; k++) {
			const cand_t *c = &pool[k];
			vect2_t *pts = NULL;
			int np = 0, capp = 0;
			int *conf = NULL, nconf = 0, *xing = NULL, nxing = 0;
			double bx0, bx1, by0, by1, score;
			bool on_rwy = false;
			vect2_t endp = c->segs[c->nseg - 1].ep;

			if (cur.valid && c->sc0 >= cur.score)
				break;
			for (int s = 0; s < c->nseg; s++)
				seg_points(&c->segs[s], np == 0, &pts, &np, &capp);
			for (int q = 0; q < np; q += 2) {
				if (in_runway(g, pts[q])) {
					on_rwy = true;
					break;
				}
			}
			if (on_rwy) {
				diag->runway++;
				free(pts);
				continue;
			}
			bx0 = bx1 = pts[0].x;
			by0 = by1 = pts[0].y;
			for (int q = 1; q < np; q++) {
				bx0 = fmin(bx0, pts[q].x);
				bx1 = fmax(bx1, pts[q].x);
				by0 = fmin(by0, pts[q].y);
				by1 = fmax(by1, pts[q].y);
			}
			bx0 -= FOOT_HALF;
			bx1 += FOOT_HALF;
			by0 -= FOOT_HALF;
			by1 += FOOT_HALF;
			conf = malloc((n_foots + 1) * sizeof (int));
			for (int f = 0; f < n_foots; f++) {
				int s = foots[f];
				vect2_t fa = g->foot_a[s], fb = g->foot_b[s];
				if (fmax(fa.x, fb.x) < bx0 || fmin(fa.x, fb.x) > bx1 ||
				    fmax(fa.y, fb.y) < by0 || fmin(fa.y, fb.y) > by1)
					continue;
				for (int q = 0; q < np; q += 2) {
					if (seg_dist(pts[q], fa, fb, NULL) < FOOT_HALF) {
						conf[nconf++] = s;
						break;
					}
				}
			}
			xing = malloc((n_taxi_near + 1) * sizeof (int));
			for (int x = 0; x < n_taxi_near; x++) {
				const gedge_t *e2 = &g->edges[taxi_near[x]];
				if (taxi_near[x] == c->eid)
					continue;
				if (fmax(e2->A.x, e2->B.x) < bx0 ||
				    fmin(e2->A.x, e2->B.x) > bx1 ||
				    fmax(e2->A.y, e2->B.y) < by0 ||
				    fmin(e2->A.y, e2->B.y) > by1)
					continue;
				for (int q = 0; q + 1 < np; q++) {
					vect2_t xp;
					if (seg_isect(pts[q], pts[q + 1], e2->A,
					    e2->B, &xp) && vdist(xp, endp) > 8.0) {
						xing[nxing++] = taxi_near[x];
						break;
					}
				}
			}
			score = c->sc0 + STAND_PEN * nconf + TWX_PEN * nxing;
			if (!cur.valid || score < cur.score) {
				best_free(&cur);
				cur.valid = true;
				cur.c = *c;
				cur.pts = pts;
				cur.n_pts = np;
				cur.conf = conf;
				cur.n_conf = nconf;
				cur.xing = xing;
				cur.n_xing = nxing;
				cur.score = score;
			} else {
				free(pts);
				free(conf);
				free(xing);
			}
		}
		if (cur.valid)
			best[n_best++] = cur;
		i = j;
	}
	diag->n_cand = n_best;
	free(pool);
	free(buf);
	free(nearv);
	free(taxi_near);
	free(foots);
	*n_out = n_best;
	return (best);
}

/* ---------------------------------------------------------------------- */
/* public API                                                             */
/* ---------------------------------------------------------------------- */

void
ptx_opts_default(ptx_opts_t *opts)
{
	opts->w_along = 2.0;
	opts->pb_warn = 200.0;
	opts->pb_limit = 300.0;
}

int
ptx_n_rends(const ptx_graph_t *g)
{
	return (g->n_rends);
}

const char *
ptx_rend_name(const ptx_graph_t *g, int i)
{
	return (g->rends[i].name);
}

double
ptx_rend_hdg(const ptx_graph_t *g, int i)
{
	return (g->rends[i].hdg);
}

double
ptx_rend_len(const ptx_graph_t *g, int i)
{
	return (g->rends[i].len);
}

double
ptx_rend_entry_along(const ptx_graph_t *g, int i)
{
	return (g->rends[i].target_along);
}

int
ptx_n_edges(const ptx_graph_t *g)
{
	return (g->n_edges);
}

const char *
ptx_edge_name(const ptx_graph_t *g, int e)
{
	return (g->edges[e].src->name);
}

bool
ptx_edge_is_runway(const ptx_graph_t *g, int e)
{
	return (g->edges[e].runway);
}

int
ptx_edge_file_a(const ptx_graph_t *g, int e)
{
	return (g->edges[e].src->a);
}

int
ptx_edge_file_b(const ptx_graph_t *g, int e)
{
	return (g->edges[e].src->b);
}

vect2_t
ptx_node_pos(const ptx_graph_t *g, int node)
{
	return (g->node_p[node]);
}

int
ptx_node_file_id(const ptx_graph_t *g, int node)
{
	return (g->apt->nodes[node].id);
}

void
ptx_dedge(const ptx_graph_t *g, int de, int *edge, int *from, int *to)
{
	if (edge != NULL)
		*edge = g->de[de].eid;
	if (from != NULL)
		*from = g->de[de].from;
	if (to != NULL)
		*to = g->de[de].to;
}

ptx_kind_t
ptx_classify(const ptx_graph_t *gc, vect2_t P0, double hdg, const char *type,
    double *d_back, double *d_front)
{
	ptx_graph_t *g = (ptx_graph_t *)gc;	/* near() scratch only */
	vect2_t fwd = hdg2v(hdg);
	double df = INFINITY, db = INFINITY;
	int *buf = malloc((g->n_edges + 1) * sizeof (int));
	int n = near_edges(g, P0, CLASSIFY_R, buf);
	ptx_kind_t kind;

	for (int i = 0; i < n; i++) {
		const gedge_t *e = &g->edges[buf[i]];
		int m;
		if (e->runway || e->leadin)
			continue;
		m = (int)(e->L / 2.0);
		if (m < 1)
			m = 1;
		for (int k = 0; k <= m; k++) {
			vect2_t q = vadd(e->A, vmul(vsub(e->B, e->A),
			    (double)k / m));
			vect2_t rel = vsub(q, P0);
			double d = vlen(rel), c;
			if (d < 1e-6 || d > CLASSIFY_R)
				continue;
			c = vdot(rel, fwd) / d;
			if (c > 0.5)
				df = fmin(df, d);
			else if (c < -0.5)
				db = fmin(db, d);
		}
	}
	free(buf);
	if (d_back != NULL)
		*d_back = db;
	if (d_front != NULL)
		*d_front = df;
	if (isinf(df) && isinf(db))
		return (PTX_KIND_NO_TAXIWAY);
	if (type != NULL && strcmp(type, "gate") == 0) {
		kind = (!isinf(db) ? PTX_KIND_PUSHBACK :
		    PTX_KIND_FACES_TAXIWAY);
	} else if (type != NULL && strcmp(type, "tie_down") == 0) {
		kind = (((df <= TIE_FRONT && df <= db) || isinf(db)) ?
		    PTX_KIND_FACES_TAXIWAY : PTX_KIND_PUSHBACK);
	} else {
		kind = (df < db ? PTX_KIND_FACES_TAXIWAY : PTX_KIND_PUSHBACK);
	}
	return (kind);
}

static bool
is_dep_zone_for(const gedge_t *e, const rend_t *r)
{
	for (int z = 0; z < e->src->n_zones; z++) {
		const ptx_zone_t *zn = &e->src->zones[z];
		bool meets = false;
		if (zn->kind != PTX_ZONE_DEPARTURE && zn->kind != PTX_ZONE_ILS)
			continue;
		for (int k = 0; k < zn->n_rwys; k++) {
			if (strcmp(zn->rwys[k], r->pair[0]) == 0 ||
			    strcmp(zn->rwys[k], r->pair[1]) == 0)
				meets = true;
		}
		if (meets)
			return (true);
	}
	return (false);
}

static void
describe(ptx_graph_t *g, const rend_t *r, const int *seq, int n,
    ptx_result_t *res)
{
	int prev = -1;
	bool in_cross = false;

	res->hs_node = -1;
	res->n_sharp = 0;
	res->crossings = 0;
	res->backtrack = 0;
	res->first_turn = 0;
	res->max_turn = 0;
	for (int i = 0; i < n; i++) {
		const gde_t *d = &g->de[seq[i]];
		const gedge_t *e = &g->edges[d->eid];
		bool is_target, other;
		if (prev >= 0) {
			double a = fabs(relh(g->de[prev].hdg, d->hdg));
			if (prev == seq[0])
				res->first_turn = a;
			if (a > res->max_turn)
				res->max_turn = a;
			if (a > SHARP_TURN)
				res->n_sharp++;
		}
		prev = seq[i];
		is_target = (e->runway && r->rw_edge[d->eid]) ||
		    (e->n_zr > 0 && zr_meets_pair(e, r) && !e->runway &&
		    is_dep_zone_for(e, r));
		if (res->hs_node < 0 && is_target && i > 0)
			res->hs_node = d->from;
		if (e->runway && r->rw_edge[d->eid])
			res->backtrack += d->L;
		other = (e->runway && !r->rw_edge[d->eid]) ||
		    (e->n_zr > 0 && !zr_meets_pair(e, r));
		if (other && !in_cross)
			res->crossings++;
		in_cross = other;
	}
	if (res->hs_node < 0 && n > 0) {
		res->hs_node = g->de[seq[n - 1]].from;
		res->hs_zone = false;
	} else {
		res->hs_zone = true;
	}
	res->entry_node = (n > 0 ? g->de[seq[n - 1]].to : -1);
}

void
ptx_plan(ptx_graph_t *g, vect2_t Pm, vect2_t Pn, double H, int own_stand,
    const ptx_aircraft_t *ac, const ptx_opts_t *opts, ptx_result_t *results,
    ptx_diag_t *diag)
{
	ptx_diag_t dummy;
	int n_best = 0;
	best_t *best;
	int *buf = malloc((g->n_edges + 1) * sizeof (int));

	if (diag == NULL)
		diag = &dummy;
	memset(diag, 0, sizeof (*diag));
	best = find_candidates(g, Pm, Pn, H, own_stand, ac, diag, &n_best);

	for (int ri = 0; ri < g->n_rends; ri++) {
		rend_t *r = &g->rends[ri];
		ptx_result_t *res = &results[ri];
		rtab_t *t = route_table(g, r, ac->route_cls, opts->w_along);
		int bi = -1, did_best = -1;
		double best_total = 0;
		const best_t *b;
		int *seq, nseq = 0;
		vect2_t nosep;
		int ne;

		memset(res, 0, sizeof (*res));
		res->edge = -1;
		res->hs_node = -1;
		res->entry_node = -1;
		res->ang_edge = NAN;
		for (int i = 0; i < n_best; i++) {
			int did = g->de_index[2 * best[i].c.eid +
			    (best[i].c.sgn == 1 ? 0 : 1)];
			double total;
			if (did < 0 || isinf(t->togo[did]))
				continue;
			total = best[i].score + t->togo[did];
			if (bi < 0 || total < best_total) {
				bi = i;
				best_total = total;
				did_best = did;
			}
		}
		if (bi < 0) {
			if (r->target < 0)
				res->status = PTX_NO_RUNWAY_EDGES;
			else if (n_best == 0)
				res->status = PTX_NO_CANDIDATE;
			else
				res->status = PTX_NO_ROUTE;
			continue;
		}
		b = &best[bi];
		res->status = PTX_OK;
		res->edge = b->c.eid;
		res->sgn = b->c.sgn;
		res->end = b->c.segs[b->c.nseg - 1].ep;
		res->end_hdg = b->c.segs[b->c.nseg - 1].eh;
		res->pb_len = b->c.pb_len;
		res->rot = b->c.rot;
		res->r_used = b->c.r_used;
		res->steer = (isinf(b->c.r_used) ? 0.0 :
		    deg(atan(ac->wheelbase / b->c.r_used)));
		res->nseg = b->c.nseg;
		res->n_pts = b->n_pts;
		res->pts = malloc((b->n_pts + 1) * sizeof (vect2_t));
		memcpy(res->pts, b->pts, b->n_pts * sizeof (vect2_t));
		res->n_conflicts = b->n_conf;
		res->conflicts = malloc((b->n_conf + 1) * sizeof (int));
		memcpy(res->conflicts, b->conf, b->n_conf * sizeof (int));
		res->n_xings = b->n_xing;
		res->xings = malloc((b->n_xing + 1) * sizeof (int));
		memcpy(res->xings, b->xing, b->n_xing * sizeof (int));
		res->pb_warn = (res->pb_len > opts->pb_warn);
		res->pb_allowed = (res->pb_len <= opts->pb_limit);

		/* walk the route */
		seq = malloc((g->n_de + 1) * sizeof (int));
		{
			uint8_t *seen = calloc(g->n_de + 1, 1);
			int d = did_best;
			seq[nseq++] = d;
			seen[d] = 1;
			while (t->nxt[d] >= 0) {
				d = t->nxt[d];
				if (seen[d])
					break;
				seen[d] = 1;
				seq[nseq++] = d;
			}
			free(seen);
		}
		describe(g, r, seq, nseq, res);
		res->route = seq;
		res->n_route = nseq;
		res->taxi = b->c.to_node;
		for (int i = 1; i < nseq; i++)
			res->taxi += g->de[seq[i]].L;
		res->entry_lost = (r->target_along);
		for (int j = 0; j < r->n_entries; j++) {
			if (r->entry_node[j] == res->entry_node) {
				res->entry_lost = r->entry_lost[j] +
				    r->target_along;
				break;
			}
		}

		/* checks */
		res->off_main = nearest_taxi(g, res->end, 60.0, false, NULL, buf);
		ne = near_edges(g, res->end, 1.0, buf);
		for (int i = 0; i < ne; i++) {
			const gedge_t *e = &g->edges[buf[i]];
			double a;
			if (e->runway)
				continue;
			a = fabs(relh(res->end_hdg, e->hdg));
			a = fmin(a, 180 - a);
			if (isnan(res->ang_edge) || a < res->ang_edge)
				res->ang_edge = a;
		}
		nosep = vadd(res->end, vmul(hdg2v(res->end_hdg),
		    ac->wheelbase));
		res->off_nose = nearest_taxi(g, nosep, 60.0, false, NULL, buf);

		if (res->n_conflicts > 0 || res->n_xings > 0 ||
		    res->pb_len > TIER_SUSPECT_LEN)
			res->tier = PTX_TIER_SUSPECT;
		else if (res->nseg <= 2 && res->rot <= 120 &&
		    res->pb_len <= TIER_STD_LEN)
			res->tier = PTX_TIER_STANDARD;
		else
			res->tier = PTX_TIER_UNUSUAL;
	}
	for (int i = 0; i < n_best; i++)
		best_free(&best[i]);
	free(best);
	free(buf);
}

void
ptx_results_free(ptx_result_t *results, int n)
{
	for (int i = 0; i < n; i++) {
		free(results[i].pts);
		free(results[i].conflicts);
		free(results[i].xings);
		free(results[i].route);
		memset(&results[i], 0, sizeof (results[i]));
	}
}

const char *
ptx_status_str(ptx_status_t s)
{
	switch (s) {
	case PTX_OK:
		return ("ok");
	case PTX_NO_RUNWAY_EDGES:
		return ("no_runway_edges");
	case PTX_NO_CANDIDATE:
		return ("no_candidate");
	case PTX_NO_ROUTE:
		return ("no_route");
	}
	return ("?");
}

const char *
ptx_kind_str(ptx_kind_t k)
{
	switch (k) {
	case PTX_KIND_PUSHBACK:
		return ("pushback");
	case PTX_KIND_FACES_TAXIWAY:
		return ("faces_taxiway");
	case PTX_KIND_NO_TAXIWAY:
		return ("no_taxiway_300m");
	}
	return ("?");
}

const char *
ptx_tier_str(ptx_tier_t t)
{
	switch (t) {
	case PTX_TIER_STANDARD:
		return ("standard");
	case PTX_TIER_UNUSUAL:
		return ("unusual");
	case PTX_TIER_SUSPECT:
		return ("suspect");
	}
	return ("?");
}
