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

#include <string.h>

#include <XPLMDisplay.h>
#include <XPLMGraphics.h>

#include <acfutils/geom.h>
#include <acfutils/log.h>
#include <acfutils/mt_cairo_render.h>

#include "ptx_ui.h"

static struct {
	bool			inited;
	ptx_ui_click_cb_t	cb;
	ptx_ui_fonts_t		*fonts;
	XPLMWindowID		win;
	mt_cairo_render_t	*mtcr;
	unsigned		mw, mh;		/* surface size */
	ptx_ui_model_t		model;
	ptx_ui_layout_t		lay;
	bool			dirty;
} ui;

static void
render_cb(cairo_t *cr, unsigned w, unsigned h, void *userinfo)
{
	(void) w;
	(void) h;
	(void) userinfo;
	ptx_ui_render(cr, &ui.model, &ui.lay, ui.fonts);
}

static void
ensure_surface(void)
{
	if (ui.mtcr != NULL && ui.mw == (unsigned)ui.lay.w &&
	    ui.mh == (unsigned)ui.lay.h)
		return;
	if (ui.mtcr != NULL)
		mt_cairo_render_fini(ui.mtcr);
	ui.mw = ui.lay.w;
	ui.mh = ui.lay.h;
	ui.mtcr = mt_cairo_render_init(ui.mw, ui.mh, 0, NULL, render_cb,
	    NULL, NULL);
	if (ui.mtcr != NULL)
		mt_cairo_render_enable_fg_mode(ui.mtcr);
	ui.dirty = true;
}

static void
draw_cb(XPLMWindowID win, void *refcon)
{
	int l, t, r, b;

	(void) refcon;
	XPLMGetWindowGeometry(win, &l, &t, &r, &b);
	ensure_surface();
	if (ui.mtcr == NULL)
		return;
	if (ui.dirty) {
		mt_cairo_render_once_wait(ui.mtcr);
		ui.dirty = false;
	}
	mt_cairo_render_draw(ui.mtcr, VECT2(l, b), VECT2(r - l, t - b));
}

static int
click_cb(XPLMWindowID win, int x, int y, XPLMMouseStatus status, void *refcon)
{
	int l, t, r, b, row, btn;
	double lx, ly;

	(void) refcon;
	if (status != xplm_MouseDown)
		return (1);
	XPLMGetWindowGeometry(win, &l, &t, &r, &b);
	if (r <= l || t <= b)
		return (1);
	/* window boxels -> layout coordinates (y down) */
	lx = (x - l) * (double)ui.lay.w / (r - l);
	ly = (t - y) * (double)ui.lay.h / (t - b);
	row = ptx_ui_hit_row(&ui.model, &ui.lay, lx, ly);
	btn = ptx_ui_hit_btn(&ui.model, &ui.lay, lx, ly);
	if ((row >= 0 || btn >= 0) && ui.cb != NULL)
		ui.cb(row, btn);
	return (1);
}

static int
rclick_cb(XPLMWindowID win, int x, int y, XPLMMouseStatus status,
    void *refcon)
{
	(void) win;
	(void) x;
	(void) y;
	(void) status;
	(void) refcon;
	return (1);
}

static void
key_cb(XPLMWindowID win, char key, XPLMKeyFlags flags, char vkey,
    void *refcon, int losing_focus)
{
	(void) win;
	(void) key;
	(void) flags;
	(void) vkey;
	(void) refcon;
	(void) losing_focus;
}

static XPLMCursorStatus
cursor_cb(XPLMWindowID win, int x, int y, void *refcon)
{
	(void) win;
	(void) x;
	(void) y;
	(void) refcon;
	return (xplm_CursorDefault);
}

static int
wheel_cb(XPLMWindowID win, int x, int y, int wheel, int clicks,
    void *refcon)
{
	(void) win;
	(void) x;
	(void) y;
	(void) wheel;
	(void) clicks;
	(void) refcon;
	return (1);
}

bool
ptx_ui_init(ptx_ui_click_cb_t cb)
{
	if (ui.inited)
		return (true);
	memset(&ui, 0, sizeof (ui));
	ui.cb = cb;
	ui.fonts = ptx_ui_fonts_new();
	logMsg("runway menu font: %s", ptx_ui_fonts_desc(ui.fonts));
	ui.inited = true;
	return (true);
}

void
ptx_ui_fini(void)
{
	if (!ui.inited)
		return;
	if (ui.win != NULL)
		XPLMDestroyWindow(ui.win);
	if (ui.mtcr != NULL)
		mt_cairo_render_fini(ui.mtcr);
	ptx_ui_fonts_free(ui.fonts);
	memset(&ui, 0, sizeof (ui));
}

bool
ptx_ui_cjk(void)
{
	return (ptx_ui_fonts_cjk(ui.fonts));
}

void
ptx_ui_show(const ptx_ui_model_t *m)
{
	int old_w = ui.lay.w, old_h = ui.lay.h;

	if (!ui.inited)
		return;
	ui.model = *m;
	ptx_ui_layout(&ui.model, &ui.lay);
	ui.dirty = true;

	if (ui.win == NULL) {
		int gl, gt, gr, gb, left, top;
		XPLMCreateWindow_t p;

		XPLMGetScreenBoundsGlobal(&gl, &gt, &gr, &gb);
		left = (gl + gr) / 2 - ui.lay.w / 2;
		top = gt - 60;
		memset(&p, 0, sizeof (p));
		p.structSize = sizeof (p);
		p.left = left;
		p.top = top;
		p.right = left + ui.lay.w;
		p.bottom = top - ui.lay.h;
		p.visible = 1;
		p.drawWindowFunc = draw_cb;
		p.handleMouseClickFunc = click_cb;
		p.handleRightClickFunc = rclick_cb;
		p.handleKeyFunc = key_cb;
		p.handleCursorFunc = cursor_cb;
		p.handleMouseWheelFunc = wheel_cb;
		p.refcon = NULL;
		p.decorateAsFloatingWindow = xplm_WindowDecorationRoundRectangle;
		p.layer = xplm_WindowLayerFloatingWindows;
		ui.win = XPLMCreateWindowEx(&p);
		if (ui.win == NULL) {
			logMsg("runway menu: XPLMCreateWindowEx failed");
			return;
		}
		XPLMSetWindowTitle(ui.win, "Progressive Taxi");
		XPLMSetWindowResizingLimits(ui.win, ui.lay.w, ui.lay.h,
		    ui.lay.w, ui.lay.h);
	} else if (old_w != ui.lay.w || old_h != ui.lay.h) {
		int l, t, r, b;
		XPLMGetWindowGeometry(ui.win, &l, &t, &r, &b);
		XPLMSetWindowResizingLimits(ui.win, ui.lay.w, ui.lay.h,
		    ui.lay.w, ui.lay.h);
		XPLMSetWindowGeometry(ui.win, l, t, l + ui.lay.w,
		    t - ui.lay.h);
	}
	XPLMSetWindowIsVisible(ui.win, 1);
	XPLMBringWindowToFront(ui.win);
}

void
ptx_ui_hide(void)
{
	if (ui.win != NULL)
		XPLMSetWindowIsVisible(ui.win, 0);
}

bool
ptx_ui_visible(void)
{
	return (ui.win != NULL && XPLMGetWindowIsVisible(ui.win));
}
