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
 * Finds the apt.dat that X-Plane actually uses for an airport: the scenery
 * packs are searched in Custom Scenery/scenery_packs.ini order (disabled
 * packs skipped, *GLOBAL_AIRPORTS* = Global Scenery/Global Airports) and
 * the first pack that defines the airport wins. Big apt.dat files (the
 * 380 MB Global Airports one) are indexed once and the index is cached.
 * Does not depend on the X-Plane SDK; safe to call from a worker thread.
 */

#ifndef	_PTX_SCENERY_H_
#define	_PTX_SCENERY_H_

#include "ptx_apt.h"

#ifdef	__cplusplus
extern "C" {
#endif

typedef struct {
	char	pack[512];	/* scenery pack directory */
	char	apt_path[1024];	/* .../Earth nav data/apt.dat */
	long	offset;		/* byte offset of the airport's header row */
	double	ms_find;	/* time spent locating it */
	double	ms_load;	/* time spent parsing it */
	bool	index_built;	/* a new index had to be built */
} ptx_scenery_info_t;

/*
 * xpdir: X-Plane root. cachedir: where indexes are kept (created if
 * missing), e.g. <xpdir>/Output/caches/ProgressiveTaxi_aptidx.
 * Returns NULL if the airport isn't found in any pack.
 */
ptx_apt_t *ptx_scenery_load(const char *xpdir, const char *cachedir,
    const char *icao, ptx_scenery_info_t *info);

#ifdef	__cplusplus
}
#endif

#endif	/* _PTX_SCENERY_H_ */
