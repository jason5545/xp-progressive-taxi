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
 * Turns planner results into the runway menu's text (Traditional Chinese
 * when a CJK font is available, English otherwise). No X-Plane SDK.
 */

#ifndef	_PTX_MENU_H_
#define	_PTX_MENU_H_

#include <stdbool.h>

#include "planner/ptx_plan.h"
#include "ptx_ui_render.h"

#ifdef	__cplusplus
extern "C" {
#endif

typedef struct {
	const char	*icao;
	const char	*stand_name;	/* NULL: not parked at a stand */
	ptx_kind_t	kind;
	bool		have_wind;
	double		wind_dir;	/* true, from */
	double		wind_kt;
	char		ac_cls;
	bool		full_length;
	bool		cjk;
} ptx_menu_ctx_t;

/*
 * Fills m with one row per usable runway end (best first, at most
 * PTX_UI_MAX_ROWS); unusable ones go into m->footnote. order[row]
 * receives the runway end index (results / ptx_rend_*) of each row.
 */
void ptx_menu_build(const ptx_graph_t *g, const ptx_result_t *res,
    const ptx_menu_ctx_t *c, ptx_ui_model_t *m, int *order);

/* A model that only shows a message (loading, error). */
void ptx_menu_message(const ptx_menu_ctx_t *c, const char *msg,
    ptx_ui_model_t *m);

/* Headwind component (kt, negative = tailwind) for runway end ri. */
double ptx_menu_headwind(const ptx_graph_t *g, int ri,
    const ptx_menu_ctx_t *c);

/* One-line spoken / logged summary of a result, e.g. for XPLMSpeakString. */
void ptx_menu_summary(const ptx_graph_t *g, const ptx_result_t *r, int ri,
    bool cjk, char *buf, size_t cap);

#ifdef	__cplusplus
}
#endif

#endif	/* _PTX_MENU_H_ */
