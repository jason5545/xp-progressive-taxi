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

/* X-Plane window hosting the runway menu (drawn by ptx_ui_render). */

#ifndef	_PTX_UI_H_
#define	_PTX_UI_H_

#include <stdbool.h>

#include "ptx_ui_render.h"

#ifdef	__cplusplus
extern "C" {
#endif

/* row >= 0: a runway row was clicked; btn >= 0: a footer button */
typedef void (*ptx_ui_click_cb_t)(int row, int btn);

bool ptx_ui_init(ptx_ui_click_cb_t cb);
void ptx_ui_fini(void);
bool ptx_ui_cjk(void);
void ptx_ui_show(const ptx_ui_model_t *m);
void ptx_ui_hide(void);
bool ptx_ui_visible(void);

#ifdef	__cplusplus
}
#endif

#endif	/* _PTX_UI_H_ */
