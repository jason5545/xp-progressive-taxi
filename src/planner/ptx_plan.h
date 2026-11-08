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
 * Automatic pushback endpoint selection and taxi routing.
 *
 * Given an airport (ptx_apt_t), the aircraft's start pose and its steering
 * limits, ptx_plan() picks, for every runway end, a pushback end pose with
 * the main gear on a taxiway centreline and the nose pointing along that
 * taxiway towards the runway, and the taxi route from there to the runway
 * entry. Pushback segments come from BetterPushback's own compute_segs().
 *
 * This is a C port of tools/proto/pbplan.py (the step-0 prototype); the two
 * are kept in lock step and compared by tools/plan_cli/parity.py.
 *
 * Coordinates are metres in a local equirectangular frame centred on the
 * mean of the runway ends: x east, y north. Headings are true degrees.
 * Does not depend on the X-Plane SDK.
 */

#ifndef	_PTX_PLAN_H_
#define	_PTX_PLAN_H_

#include <stdbool.h>

#include <acfutils/geom.h>

#include "ptx_apt.h"

#ifdef	__cplusplus
extern "C" {
#endif

typedef struct ptx_graph ptx_graph_t;

typedef struct {
	double	wheelbase;	/* nose gear to main gear, metres */
	double	max_steer;	/* nose wheel steering limit, degrees */
	char	route_cls;	/* ICAO width class 'A'..'F' for routing */
} ptx_aircraft_t;

typedef struct {
	/* taxi metres charged per metre of runway lost at the entry */
	double	w_along;
	/* above this pushback length the menu shows a warning */
	double	pb_warn;
	/* above this the plugin won't push automatically */
	double	pb_limit;
} ptx_opts_t;

/* w_along that effectively forces the full-length entry when reachable */
#define	PTX_W_ALONG_FULL_LENGTH	1000.0

typedef enum {
	PTX_KIND_PUSHBACK,	/* needs a pushback */
	PTX_KIND_FACES_TAXIWAY,	/* can taxi out forward */
	PTX_KIND_NO_TAXIWAY	/* no taxiway within 300 m */
} ptx_kind_t;

typedef enum {
	PTX_OK,
	PTX_NO_RUNWAY_EDGES,	/* runway not in the taxi network */
	PTX_NO_CANDIDATE,	/* no reachable taxiway behind the aircraft */
	PTX_NO_ROUTE		/* candidates exist but none reaches the rwy */
} ptx_status_t;

typedef enum {
	PTX_TIER_STANDARD,	/* straight back + one turn, <= 120 deg, <= 200 m */
	PTX_TIER_UNUSUAL,	/* S-curve, big rotation or long */
	PTX_TIER_SUSPECT	/* > 250 m, crosses a stand or a taxiway */
} ptx_tier_t;

typedef struct {
	ptx_status_t	status;

	/* --- pushback --- */
	int		edge;		/* graph edge index the end lies on */
	int		sgn;		/* +1 = towards edge node b, -1 = a */
	vect2_t		end;		/* main gear position at the end */
	double		end_hdg;	/* true heading at the end */
	double		pb_len;		/* metres */
	double		rot;		/* total heading change, degrees */
	double		r_used;		/* tightest turn radius, m (inf: none) */
	double		steer;		/* nose wheel angle for r_used, deg */
	int		nseg;		/* compute_segs() segment count */
	vect2_t		*pts;		/* main gear path, n_pts points */
	int		n_pts;
	int		*conflicts;	/* stand indices the path crosses */
	int		n_conflicts;
	int		*xings;		/* edge indices the path crosses */
	int		n_xings;
	bool		pb_warn;	/* pb_len > opts.pb_warn */
	bool		pb_allowed;	/* pb_len <= opts.pb_limit */
	ptx_tier_t	tier;

	/* --- taxi --- */
	int		*route;		/* directed-edge ids, first = end edge */
	int		n_route;
	double		taxi;		/* metres from end pose to entry */
	double		first_turn;	/* degrees */
	double		max_turn;
	int		n_sharp;	/* turns > 90 deg */
	int		crossings;	/* other runways / their zones */
	double		backtrack;	/* metres on the departure runway */
	int		hs_node;	/* node index of the hold-short point */
	bool		hs_zone;	/* hs_node came from a 1204 zone */
	int		entry_node;	/* node index on the runway */
	double		entry_lost;	/* runway metres lost at the entry */

	/* --- checks --- */
	double		off_main;	/* main gear to nearest taxiway, m */
	double		off_nose;	/* nose gear to nearest taxiway, m */
	double		ang_edge;	/* end heading vs edge, deg (NAN: n/a) */
} ptx_result_t;

typedef struct {
	int	tried, segs_fail, has_forward, too_long, rot, off_line;
	int	nose_off, runway, not_behind, leadin, narrow;
	int	n_cand;			/* (edge, direction) keys kept */
} ptx_diag_t;

void ptx_opts_default(ptx_opts_t *opts);

/* The graph keeps a pointer to apt; apt must outlive it. */
ptx_graph_t *ptx_graph_new(const ptx_apt_t *apt);
void ptx_graph_free(ptx_graph_t *g);

vect2_t ptx_project(const ptx_graph_t *g, double lat, double lon);
void ptx_unproject(const ptx_graph_t *g, vect2_t p, double *lat,
    double *lon);

int ptx_n_rends(const ptx_graph_t *g);
const char *ptx_rend_name(const ptx_graph_t *g, int i);
double ptx_rend_hdg(const ptx_graph_t *g, int i);
double ptx_rend_len(const ptx_graph_t *g, int i);
/* along-track position of the closest enterable entry (NAN if none) */
double ptx_rend_entry_along(const ptx_graph_t *g, int i);

int ptx_n_edges(const ptx_graph_t *g);
const char *ptx_edge_name(const ptx_graph_t *g, int e);
bool ptx_edge_is_runway(const ptx_graph_t *g, int e);
int ptx_edge_file_a(const ptx_graph_t *g, int e);	/* node ids */
int ptx_edge_file_b(const ptx_graph_t *g, int e);
vect2_t ptx_node_pos(const ptx_graph_t *g, int node);
int ptx_node_file_id(const ptx_graph_t *g, int node);
/* directed edge -> (edge index, from node, to node) */
void ptx_dedge(const ptx_graph_t *g, int de, int *edge, int *from, int *to);

ptx_kind_t ptx_classify(const ptx_graph_t *g, vect2_t stand_pos,
    double stand_hdg, const char *stand_type, double *d_back,
    double *d_front);

/*
 * Plans a pushback for every runway end. results must hold ptx_n_rends()
 * entries; release them with ptx_results_free().
 *
 *   main_pos	main gear position (BetterPushback steers by the rear axle)
 *   ref_pos	the stand reference point / aircraft nose; used to skip the
 *		aircraft's own stand footprint
 *   own_stand	index of the aircraft's own stand, or -1
 */
void ptx_plan(ptx_graph_t *g, vect2_t main_pos, vect2_t ref_pos,
    double hdg, int own_stand, const ptx_aircraft_t *ac,
    const ptx_opts_t *opts, ptx_result_t *results, ptx_diag_t *diag);
void ptx_results_free(ptx_result_t *results, int n);

const char *ptx_status_str(ptx_status_t s);
const char *ptx_kind_str(ptx_kind_t k);
const char *ptx_tier_str(ptx_tier_t t);

#ifdef	__cplusplus
}
#endif

#endif	/* _PTX_PLAN_H_ */
