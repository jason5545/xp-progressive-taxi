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
 * Automatic pushback: runway menu, commands and the hand-over to
 * BetterPushback's tug (or X-Plane's own tug).
 *
 * Commands:
 *   ProgressiveTaxi/auto_pushback          open the runway menu; when it
 *                                          is open, push to the highlighted
 *                                          runway
 *   ProgressiveTaxi/auto_pushback_next     highlight the next runway
 *   ProgressiveTaxi/auto_pushback_prev     highlight the previous runway
 *   ProgressiveTaxi/auto_pushback_best     push to the recommended runway
 *                                          right away, no menu
 *   ProgressiveTaxi/auto_pushback_close    close the menu
 *   ProgressiveTaxi/auto_pushback_xp_tug   X-Plane's own tug, direction
 *                                          chosen for the highlighted runway
 */

#ifndef	_PTX_XP_H_
#define	_PTX_XP_H_

#include <XPLMMenus.h>

#ifdef	__cplusplus
extern "C" {
#endif

void ptx_xp_start(void);		/* XPluginStart: create commands */
void ptx_xp_enable(XPLMMenuID root);	/* register, menus, window */
void ptx_xp_disable(void);

#ifdef	__cplusplus
}
#endif

#endif	/* _PTX_XP_H_ */
