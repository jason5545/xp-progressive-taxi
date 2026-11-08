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
 * Minimal apt.dat reader for the pushback / taxi planner. Only the rows the
 * planner needs are kept: 100 (runways), 1300/1301 (ramp starts), 1201
 * (taxi nodes), 1202 (taxi edges) and 1204 (active zones). 1206 (ground
 * truck routes) is skipped. Does not depend on the X-Plane SDK.
 */

#ifndef	_PTX_APT_H_
#define	_PTX_APT_H_

#include <stdbool.h>
#include <stddef.h>

#ifdef	__cplusplus
extern "C" {
#endif

#define	PTX_NAME_LEN		64
#define	PTX_RWY_LEN		8
#define	PTX_MAX_ZONE_RWYS	8

typedef struct {
	char	name[PTX_RWY_LEN];	/* as written in the 100 row */
	double	lat, lon;
	double	displ;			/* displaced threshold, m */
} ptx_rwy_end_t;

typedef struct {
	double		width;
	ptx_rwy_end_t	end[2];
} ptx_runway_t;

typedef struct {
	double	lat, lon, hdg;		/* hdg normalized to [0, 360] */
	char	type[16];		/* gate, tie_down, hangar, misc */
	char	actypes[64];		/* e.g. "heavy|jets" */
	char	name[PTX_NAME_LEN];
	char	cls;			/* 'A'..'F' from 1301, 0 if absent */
} ptx_stand_t;

typedef struct {
	int	id;
	double	lat, lon;
} ptx_node_t;

typedef enum {
	PTX_ZONE_DEPARTURE,
	PTX_ZONE_ARRIVAL,
	PTX_ZONE_ILS,
	PTX_ZONE_OTHER
} ptx_zone_kind_t;

typedef struct {
	ptx_zone_kind_t	kind;
	int		n_rwys;
	/* normalized with ptx_rwy_norm() */
	char		rwys[PTX_MAX_ZONE_RWYS][PTX_RWY_LEN];
} ptx_zone_t;

typedef struct {
	int		a, b;		/* node ids as in the file */
	bool		oneway;
	bool		runway;		/* 1202 type "runway" */
	char		cls;		/* 'A'..'F' from taxiway_X, 0 if none */
	char		name[PTX_NAME_LEN];
	int		n_zones;
	ptx_zone_t	*zones;
} ptx_edge_t;

typedef struct {
	char		icao[8];
	char		name[128];
	ptx_runway_t	*rwys;
	int		n_rwys;
	ptx_stand_t	*stands;
	int		n_stands;
	ptx_node_t	*nodes;
	int		n_nodes;
	ptx_edge_t	*edges;
	int		n_edges;
} ptx_apt_t;

/*
 * Loads one airport from an apt.dat file. With icao == NULL the file is
 * expected to hold a single airport (an extract). Returns NULL if the
 * file can't be read or the airport isn't in it.
 */
ptx_apt_t *ptx_apt_load(const char *path, const char *icao);
/* Same, starting at a byte offset (an airport header row). */
ptx_apt_t *ptx_apt_load_at(const char *path, long offset, const char *icao);
void ptx_apt_free(ptx_apt_t *apt);

/* "01R" -> "1R", "05" -> "5", upper-case. */
void ptx_rwy_norm(const char *in, char out[PTX_RWY_LEN]);

#ifdef	__cplusplus
}
#endif

#endif	/* _PTX_APT_H_ */
