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

#include <glob.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ft2build.h>
#include FT_FREETYPE_H
#include <cairo-ft.h>

#include "ptx_ui_render.h"

/* ---- geometry (in window pixels / boxels) ---- */
#define	W		860
#define	PAD		16
#define	HEAD_H		80
#define	ROW_H		74
#define	ROW_GAP		6
#define	NOTE_H		30
#define	LABEL_W		104
#define	FOOT_H		64
#define	HINT_H		30
#define	MSG_H		120

struct ptx_ui_fonts {
	FT_Library		ft;
	FT_Face			face;
	cairo_font_face_t	*cr;
	bool			cjk;
	char			desc[256];
};

static bool
face_has_tc(FT_Face f)
{
	/* 機 滑 續 跑 */
	return (FT_Get_Char_Index(f, 0x6A5F) != 0 &&
	    FT_Get_Char_Index(f, 0x6ED1) != 0 &&
	    FT_Get_Char_Index(f, 0x7E8C) != 0 &&
	    FT_Get_Char_Index(f, 0x8DD1) != 0);
}

/* Opens the face of `path` whose family/style match; -1 style = any. */
static bool
try_face(ptx_ui_fonts_t *f, const char *path, const char *family,
    const char *style)
{
	FT_Face probe;
	long n;

	if (FT_New_Face(f->ft, path, -1, &probe) != 0)
		return (false);
	n = probe->num_faces;
	FT_Done_Face(probe);
	for (long i = 0; i < n; i++) {
		FT_Face face;
		if (FT_New_Face(f->ft, path, i, &face) != 0)
			continue;
		if (face->family_name != NULL &&
		    strcmp(face->family_name, family) == 0 &&
		    (style == NULL || (face->style_name != NULL &&
		    strcmp(face->style_name, style) == 0)) &&
		    (strstr(family, "TC") == NULL || face_has_tc(face))) {
			f->face = face;
			snprintf(f->desc, sizeof (f->desc), "%s %s (%s #%ld)",
			    face->family_name, face->style_name ?
			    face->style_name : "", path, i);
			return (true);
		}
		FT_Done_Face(face);
	}
	return (false);
}

ptx_ui_fonts_t *
ptx_ui_fonts_new(void)
{
	ptx_ui_fonts_t *f = calloc(1, sizeof (*f));
	glob_t gl;

	if (FT_Init_FreeType(&f->ft) != 0) {
		free(f);
		return (NULL);
	}
	/* PingFang TC lives in a hashed asset directory */
	memset(&gl, 0, sizeof (gl));
	if (glob("/System/Library/AssetsV2/com_apple_MobileAsset_Font*/*/"
	    "AssetData/PingFang.ttc", 0, NULL, &gl) == 0) {
		for (size_t i = 0; i < gl.gl_pathc && f->face == NULL; i++)
			(void) try_face(f, gl.gl_pathv[i], "PingFang TC",
			    "Medium");
	}
	globfree(&gl);
	if (f->face == NULL)
		(void) try_face(f, "/System/Library/Fonts/STHeiti Medium.ttc",
		    "Heiti TC", NULL);
	if (f->face == NULL)
		(void) try_face(f, "/Library/Fonts/Arial Unicode.ttf",
		    "Arial Unicode MS", NULL);
	if (f->face != NULL) {
		f->cjk = face_has_tc(f->face);
	} else {
		(void) try_face(f, "/System/Library/Fonts/Helvetica.ttc",
		    "Helvetica", "Regular");
		f->cjk = false;
	}
	if (f->face == NULL) {
		FT_Done_FreeType(f->ft);
		free(f);
		return (NULL);
	}
	f->cr = cairo_ft_font_face_create_for_ft_face(f->face, 0);
	return (f);
}

void
ptx_ui_fonts_free(ptx_ui_fonts_t *f)
{
	if (f == NULL)
		return;
	if (f->cr != NULL)
		cairo_font_face_destroy(f->cr);
	if (f->face != NULL)
		FT_Done_Face(f->face);
	FT_Done_FreeType(f->ft);
	free(f);
}

bool
ptx_ui_fonts_cjk(const ptx_ui_fonts_t *f)
{
	return (f != NULL && f->cjk);
}

cairo_font_face_t *
ptx_ui_fonts_face(const ptx_ui_fonts_t *f)
{
	return (f != NULL ? f->cr : NULL);
}

const char *
ptx_ui_fonts_desc(const ptx_ui_fonts_t *f)
{
	return (f != NULL ? f->desc : "(no font)");
}

void
ptx_ui_layout(const ptx_ui_model_t *m, ptx_ui_layout_t *l)
{
	double y = HEAD_H;
	int nb = 0;

	memset(l, 0, sizeof (*l));
	l->w = W;
	if (m->message[0] != '\0') {
		y += MSG_H;
	} else {
		for (int i = 0; i < m->n_rows && i < PTX_UI_MAX_ROWS; i++) {
			l->rows[i] = (ptx_rect_t){ PAD, y, W - 2 * PAD, ROW_H };
			y += ROW_H + ROW_GAP;
		}
	}
	if (m->message[0] == '\0' && m->footnote[0] != '\0')
		y += NOTE_H;
	y += 4;
	for (int b = 0; b < PTX_UI_N_BTN; b++) {
		if (m->btn[b][0] != '\0')
			nb++;
	}
	if (nb > 0) {
		double bw = (W - 2 * PAD - (nb - 1) * 10) / (double)nb;
		double x = PAD;
		for (int b = 0; b < PTX_UI_N_BTN; b++) {
			if (m->btn[b][0] == '\0')
				continue;
			l->btns[b] = (ptx_rect_t){ x, y, bw, FOOT_H - 12 };
			x += bw + 10;
		}
	}
	y += FOOT_H;
	if (m->hint[0] != '\0')
		y += HINT_H;
	l->h = (int)y + 4;
}

static void
rrect(cairo_t *cr, ptx_rect_t r, double rad)
{
	double x = r.x, y = r.y, w = r.w, h = r.h;
	cairo_new_sub_path(cr);
	cairo_arc(cr, x + w - rad, y + rad, rad, -M_PI / 2, 0);
	cairo_arc(cr, x + w - rad, y + h - rad, rad, 0, M_PI / 2);
	cairo_arc(cr, x + rad, y + h - rad, rad, M_PI / 2, M_PI);
	cairo_arc(cr, x + rad, y + rad, rad, M_PI, 3 * M_PI / 2);
	cairo_close_path(cr);
}

static void
rgb(cairo_t *cr, unsigned c)
{
	cairo_set_source_rgb(cr, ((c >> 16) & 0xff) / 255.0,
	    ((c >> 8) & 0xff) / 255.0, (c & 0xff) / 255.0);
}

/* Draws text, shrinking then ellipsizing (UTF-8 aware) to fit max_w. */
static void
text(cairo_t *cr, const char *s, double x, double y, double size,
    double max_w)
{
	char buf[PTX_UI_TXT + 8];
	cairo_text_extents_t te;
	size_t n;

	cairo_set_font_size(cr, size);
	cairo_text_extents(cr, s, &te);
	if (te.x_advance > max_w && size > 12) {
		double s2 = fmax(12, size * max_w / te.x_advance);
		cairo_set_font_size(cr, s2);
		cairo_text_extents(cr, s, &te);
	}
	if (te.x_advance <= max_w) {
		cairo_move_to(cr, x, y);
		cairo_show_text(cr, s);
		return;
	}
	n = strlen(s);
	if (n > PTX_UI_TXT)
		n = PTX_UI_TXT;
	memcpy(buf, s, n);
	buf[n] = '\0';
	while (n > 0) {
		/* drop the last UTF-8 code point */
		do {
			n--;
		} while (n > 0 && ((unsigned char)buf[n] & 0xC0) == 0x80);
		memcpy(&buf[n], "\xE2\x80\xA6", 4);	/* … */
		cairo_text_extents(cr, buf, &te);
		if (te.x_advance <= max_w)
			break;
	}
	cairo_move_to(cr, x, y);
	cairo_show_text(cr, buf);
}

void
ptx_ui_render(cairo_t *cr, const ptx_ui_model_t *m, const ptx_ui_layout_t *l,
    const ptx_ui_fonts_t *f)
{
	cairo_save(cr);
	cairo_set_operator(cr, CAIRO_OPERATOR_SOURCE);
	rgb(cr, 0x1b2026);
	cairo_paint(cr);
	cairo_set_operator(cr, CAIRO_OPERATOR_OVER);
	if (f != NULL)
		cairo_set_font_face(cr, f->cr);

	rgb(cr, 0xffffff);
	text(cr, m->title, PAD, 36, 26, W - 2 * PAD);
	rgb(cr, 0xb0bac4);
	text(cr, m->subtitle, PAD, 66, 18, W - 2 * PAD);

	if (m->message[0] != '\0') {
		rgb(cr, 0xffd54f);
		text(cr, m->message, PAD, HEAD_H + 60, 24, W - 2 * PAD);
	}
	for (int i = 0; m->message[0] == '\0' && i < m->n_rows &&
	    i < PTX_UI_MAX_ROWS; i++) {
		const ptx_ui_row_t *r = &m->rows[i];
		ptx_rect_t rr = l->rows[i];
		ptx_rect_t lab = { rr.x + 8, rr.y + 8, LABEL_W - 16, rr.h - 16 };
		double tx = rr.x + LABEL_W + 4, tw = rr.w - LABEL_W - 16;

		rgb(cr, r->enabled ? 0x2a323b : 0x22282e);
		rrect(cr, rr, 10);
		cairo_fill(cr);
		if (i == m->highlight) {
			rgb(cr, 0x4fc3f7);
			cairo_set_line_width(cr, 4);
			rrect(cr, rr, 10);
			cairo_stroke(cr);
		}
		rgb(cr, !r->enabled ? 0x3a4148 :
		    r->recommended ? 0x2e7d32 : 0x37474f);
		rrect(cr, lab, 8);
		cairo_fill(cr);
		rgb(cr, r->enabled ? 0xffffff : 0x8a939b);
		{
			cairo_text_extents_t te;
			cairo_set_font_size(cr, 30);
			cairo_text_extents(cr, r->label, &te);
			cairo_move_to(cr, lab.x + (lab.w - te.x_advance) / 2,
			    lab.y + lab.h / 2 + 11);
			cairo_show_text(cr, r->label);
		}
		if (r->tag[0] != '\0') {
			cairo_text_extents_t te;
			rgb(cr, r->recommended ? 0x9ccc65 : 0x90a4ae);
			cairo_set_font_size(cr, 16);
			cairo_text_extents(cr, r->tag, &te);
			cairo_move_to(cr, rr.x + rr.w - 14 - te.x_advance,
			    rr.y + 27);
			cairo_show_text(cr, r->tag);
			tw -= te.x_advance + 20;
		}
		rgb(cr, !r->enabled ? 0x8a939b : r->warn ? 0xffb300 : 0xffffff);
		text(cr, r->line1, tx, rr.y + 31, 20, tw);
		rgb(cr, r->enabled ? 0xa9b7c3 : 0x6f777e);
		text(cr, r->line2, tx, rr.y + 59, 16,
		    rr.w - LABEL_W - 16);
	}
	if (m->message[0] == '\0' && m->footnote[0] != '\0') {
		double fy = (m->n_rows > 0 ? l->rows[m->n_rows - 1].y +
		    ROW_H + ROW_GAP : HEAD_H) + 20;
		rgb(cr, 0x90a4ae);
		text(cr, m->footnote, PAD, fy, 16, W - 2 * PAD);
	}
	for (int b = 0; b < PTX_UI_N_BTN; b++) {
		ptx_rect_t br = l->btns[b];
		cairo_text_extents_t te;
		if (m->btn[b][0] == '\0')
			continue;
		rgb(cr, !m->btn_enabled[b] ? 0x2a2f34 :
		    m->btn_on[b] ? 0x1565c0 : 0x37474f);
		rrect(cr, br, 10);
		cairo_fill(cr);
		rgb(cr, m->btn_enabled[b] ? 0xffffff : 0x7a828a);
		cairo_set_font_size(cr, 19);
		cairo_text_extents(cr, m->btn[b], &te);
		if (te.x_advance > br.w - 12) {
			text(cr, m->btn[b], br.x + 6, br.y + br.h / 2 + 7, 19,
			    br.w - 12);
		} else {
			cairo_move_to(cr, br.x + (br.w - te.x_advance) / 2,
			    br.y + br.h / 2 + 7);
			cairo_show_text(cr, m->btn[b]);
		}
	}
	if (m->hint[0] != '\0') {
		rgb(cr, 0x78858f);
		text(cr, m->hint, PAD, l->h - 14, 15, W - 2 * PAD);
	}
	cairo_restore(cr);
}

static bool
in_rect(ptx_rect_t r, double x, double y)
{
	return (x >= r.x && x <= r.x + r.w && y >= r.y && y <= r.y + r.h);
}

int
ptx_ui_hit_row(const ptx_ui_model_t *m, const ptx_ui_layout_t *l, double x,
    double y)
{
	if (m->message[0] != '\0')
		return (-1);
	for (int i = 0; i < m->n_rows && i < PTX_UI_MAX_ROWS; i++) {
		if (in_rect(l->rows[i], x, y))
			return (i);
	}
	return (-1);
}

int
ptx_ui_hit_btn(const ptx_ui_model_t *m, const ptx_ui_layout_t *l, double x,
    double y)
{
	for (int b = 0; b < PTX_UI_N_BTN; b++) {
		if (m->btn[b][0] != '\0' && in_rect(l->btns[b], x, y))
			return (b);
	}
	return (-1);
}
