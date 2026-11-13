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
 * The runway menu: data model, layout and cairo drawing. No X-Plane SDK
 * here, so tools/plan_cli can render the very same menu to a PNG.
 */

#ifndef	_PTX_UI_RENDER_H_
#define	_PTX_UI_RENDER_H_

#include <stdbool.h>

#include <cairo.h>

#ifdef	__cplusplus
extern "C" {
#endif

#define	PTX_UI_MAX_ROWS	8	/* runway buttons shown */
#define	PTX_UI_TXT	320

typedef enum {
	PTX_UI_BTN_FULL,	/* full-length entry on/off */
	PTX_UI_BTN_XPTUG,	/* use X-Plane's own tug */
	PTX_UI_BTN_MANUAL,	/* original BetterPushback planner */
	PTX_UI_BTN_CLOSE,
	PTX_UI_N_BTN
} ptx_ui_btn_t;

typedef struct {
	char	label[16];		/* runway, e.g. "05L" */
	char	tag[96];		/* "建議・逆風 12 kt" */
	char	line1[PTX_UI_TXT];	/* end pose and pushback */
	char	line2[PTX_UI_TXT];	/* taxi route */
	bool	enabled;
	bool	recommended;
	bool	warn;			/* long pushback: amber line1 */
} ptx_ui_row_t;

typedef struct {
	char		title[PTX_UI_TXT];
	char		subtitle[PTX_UI_TXT];
	char		message[PTX_UI_TXT];	/* loading / error; hides rows */
	char		footnote[PTX_UI_TXT];	/* unavailable runways */
	char		hint[PTX_UI_TXT];
	int		n_rows;
	ptx_ui_row_t	rows[PTX_UI_MAX_ROWS];
	int		highlight;		/* row index, -1 none */
	char		btn[PTX_UI_N_BTN][64];
	bool		btn_enabled[PTX_UI_N_BTN];
	bool		btn_on[PTX_UI_N_BTN];	/* toggle state shown lit */
} ptx_ui_model_t;

typedef struct {
	double	x, y, w, h;
} ptx_rect_t;

typedef struct {
	int		w, h;
	ptx_rect_t	rows[PTX_UI_MAX_ROWS];
	ptx_rect_t	btns[PTX_UI_N_BTN];
} ptx_ui_layout_t;

typedef struct ptx_ui_fonts ptx_ui_fonts_t;

/* Loads a Traditional Chinese capable font; falls back to Latin only. */
ptx_ui_fonts_t *ptx_ui_fonts_new(void);
void ptx_ui_fonts_free(ptx_ui_fonts_t *f);
bool ptx_ui_fonts_cjk(const ptx_ui_fonts_t *f);
const char *ptx_ui_fonts_desc(const ptx_ui_fonts_t *f);
/* the cairo face of the loaded font (owned by f) */
cairo_font_face_t *ptx_ui_fonts_face(const ptx_ui_fonts_t *f);

void ptx_ui_layout(const ptx_ui_model_t *m, ptx_ui_layout_t *l);
void ptx_ui_render(cairo_t *cr, const ptx_ui_model_t *m,
    const ptx_ui_layout_t *l, const ptx_ui_fonts_t *f);

/* hit test in layout coordinates (y down): row index, or -1 */
int ptx_ui_hit_row(const ptx_ui_model_t *m, const ptx_ui_layout_t *l,
    double x, double y);
/* button index, or -1 */
int ptx_ui_hit_btn(const ptx_ui_model_t *m, const ptx_ui_layout_t *l,
    double x, double y);

#ifdef	__cplusplus
}
#endif

#endif	/* _PTX_UI_RENDER_H_ */
