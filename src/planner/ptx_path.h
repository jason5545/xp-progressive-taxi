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
 * The taxi path published to Safety Pilot (design/taxi-route-interface.md):
 * a planner route (ptx_result_t) turned into a densely sampled centreline
 * with filleted junctions, a straight extension along the departure
 * runway, per-point flags, hold-short positions and taxiway name spans.
 * The plugin (ptaxi/route/... datarefs) and tools/plan_cli (route JSON) both
 * call ptx_path_build(). Does not depend on the X-Plane SDK.
 */

#ifndef	_PTX_PATH_H_
#define	_PTX_PATH_H_

#include <stdbool.h>

#include "ptx_plan.h"

#ifdef	__cplusplus
extern "C" {
#endif

#define	PTX_PATH_VERSION	1
#define	PTX_PATH_MAX_PTS	4096
#define	PTX_PATH_MAX_HS		16

/* flags, one per point */
#define	PTX_PF_ARC		(1 << 0)
#define	PTX_PF_RWY_DEP		(1 << 1)
#define	PTX_PF_RWY_CROSS	(1 << 2)
#define	PTX_PF_MANUAL		(1 << 3)

typedef struct {
	/* preferred fillet radius per ICAO width class A..F, metres */
	double	r_pref[6];
	/* edges without a width class */
	double	r_unknown;
	/* the aircraft's minimum radius for the nose wheel to follow the
	 * line; smaller arcs are MANUAL (0: don't check) */
	double	r_min;
	/* runway extension past the entry, metres */
	double	ext_len;
	/* a turn (ARC, next_turn_*) must total more than this, degrees */
	double	turn_deg;
	/* corners sharper than this are left as corners and MANUAL */
	double	manual_deg;
	/* maximum point spacing, metres */
	double	step;
} ptx_path_opts_t;

/* one filleted (or MANUAL) corner of the planned polyline */
typedef struct {
	int	vtx;		/* index into poly */
	double	theta;		/* signed turn, degrees, right positive */
	double	r;		/* radius used (0: corner left sharp) */
	double	r_pref;		/* radius wanted */
	int	i0, i1;		/* first and last path point of the arc */
	double	s0, s1;
	bool	manual;
} ptx_arc_t;

/* a turn as reported (consecutive same-direction arcs merged) */
typedef struct {
	double	s0, s1;		/* where it starts / ends on the path */
	double	deg;		/* right positive */
	double	r_min;		/* tightest arc in it (0: sharp corner) */
	bool	manual;
} ptx_turn_t;

/* from s on, the aircraft is on taxiway name ("" if unnamed) */
typedef struct {
	double	s;
	char	name[PTX_NAME_LEN + 8];
} ptx_span_t;

typedef struct {
	int		n;
	double		*x, *y, *s;	/* metres in the planner frame */
	int		*flags;

	int		hs_n;
	double		hs_s[PTX_PATH_MAX_HS];
	char		hs_name[PTX_PATH_MAX_HS][64];
	bool		hs_dep[PTX_PATH_MAX_HS];
	char		hs_names[256];	/* ";"-joined, as published */

	char		rwy[8];
	double		rwy_hdg;
	double		ref_lat, ref_lon;

	ptx_turn_t	*turns;
	int		n_turns;
	ptx_span_t	*spans;
	int		n_spans;

	/* the polyline before filleting, and what was done at each corner */
	vect2_t		*poly;
	int		n_poly;
	ptx_arc_t	*arcs;
	int		n_arcs;
	double		s_ext;		/* the straight runway extension starts */

	/* summary */
	double		len;
	double		r_min_used;	/* tightest arc of a turn > turn_deg */
	int		n_shrunk;	/* turns whose radius had to shrink */
	int		n_manual;	/* MANUAL turns */
} ptx_path_t;

typedef enum {
	PTX_PATH_OK,
	PTX_PATH_NO_ROUTE,	/* the result isn't PTX_OK */
	PTX_PATH_TOO_LONG	/* would need more than PTX_PATH_MAX_PTS */
} ptx_path_status_t;

void ptx_path_opts_default(ptx_path_opts_t *opts);
/*
 * Smallest radius the nose wheel can follow: wheelbase / sin(0.9 x max
 * steer), the same 0.9 margin BetterPushback's compute_segs() keeps.
 */
double ptx_path_min_radius(double wheelbase, double max_steer);

/* Builds the path for result r (runway end ri). Free with ptx_path_free. */
ptx_path_status_t ptx_path_build(const ptx_graph_t *g, const ptx_result_t *r,
    int ri, const ptx_path_opts_t *opts, ptx_path_t *path);
void ptx_path_free(ptx_path_t *path);

/*
 * Where point q (e.g. the nose) is along the path: from 20 m behind to
 * 150 m ahead of prev_s, the point minimising distance + 0.1 x |s -
 * prev_s|, so that a route coming back past the same place doesn't jump;
 * the whole path (nearest point) only if the pick is more than 30 m away.
 * *off gets the distance to the path.
 */
double ptx_path_track(const ptx_path_t *path, vect2_t q, double prev_s,
    double *off);
/*
 * The values that depend on progress: the next turn starting at or after
 * prog_s (-1 / 0 if none), the taxiway we're on and the next differently
 * named one ("" if none).
 */
void ptx_path_progress(const ptx_path_t *path, double prog_s,
    double *next_turn_s, double *next_turn_deg, const char **cur_name,
    const char **next_name);

/* Index of the span containing s (-1 before the first). */
int ptx_path_span_at(const ptx_path_t *path, double s);

const char *ptx_path_status_str(ptx_path_status_t st);

#ifdef	__cplusplus
}
#endif

#endif	/* _PTX_PATH_H_ */
