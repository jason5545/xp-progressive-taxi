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

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ptx_menu.h"

#define	ZH(c)	((c)->cjk)

double
ptx_menu_headwind(const ptx_graph_t *g, int ri, const ptx_menu_ctx_t *c)
{
	double d;
	if (!c->have_wind)
		return (0);
	d = (c->wind_dir - ptx_rend_hdg(g, ri)) * (M_PI / 180.0);
	return (c->wind_kt * cos(d));
}

static void
header(const ptx_menu_ctx_t *c, ptx_ui_model_t *m)
{
	const char *kind;
	char wind[64];

	if (c->stand_name != NULL && c->stand_name[0] != '\0') {
		snprintf(m->title, sizeof (m->title), ZH(c) ?
		    "自動後推　%s　停機位 %s" : "Auto pushback  %s  stand %s",
		    c->icao, c->stand_name);
	} else {
		snprintf(m->title, sizeof (m->title), ZH(c) ?
		    "自動後推　%s" : "Auto pushback  %s", c->icao);
	}
	switch (c->kind) {
	case PTX_KIND_PUSHBACK:
		kind = ZH(c) ? "需要後推" : "pushback needed";
		break;
	case PTX_KIND_FACES_TAXIWAY:
		kind = ZH(c) ? "機頭朝滑行道，也可直接滑出" :
		    "facing a taxiway, can also taxi out";
		break;
	default:
		kind = ZH(c) ? "附近 300 m 沒有滑行道" : "no taxiway within 300 m";
		break;
	}
	if (c->have_wind && c->wind_kt >= 1.0) {
		snprintf(wind, sizeof (wind), ZH(c) ? "風 %03.0f° %.0f kt" :
		    "wind %03.0f/%.0f", c->wind_dir, c->wind_kt);
	} else {
		snprintf(wind, sizeof (wind), ZH(c) ? "無風" : "wind calm");
	}
	snprintf(m->subtitle, sizeof (m->subtitle), ZH(c) ?
	    "%s・%s・%c 級機・全長入口：%s" :
	    "%s · %s · class %c · full-length entry: %s", wind, kind,
	    c->ac_cls ? c->ac_cls : '?', c->full_length ?
	    (ZH(c) ? "開" : "on") : (ZH(c) ? "關" : "off"));

	snprintf(m->btn[PTX_UI_BTN_FULL], sizeof (m->btn[0]), ZH(c) ?
	    "全長入口：%s" : "Full length: %s", c->full_length ?
	    (ZH(c) ? "開" : "on") : (ZH(c) ? "關" : "off"));
	m->btn_on[PTX_UI_BTN_FULL] = c->full_length;
	snprintf(m->btn[PTX_UI_BTN_XPTUG], sizeof (m->btn[0]), "%s",
	    ZH(c) ? "X-Plane 拖車" : "X-Plane tug");
	snprintf(m->btn[PTX_UI_BTN_MANUAL], sizeof (m->btn[0]), "%s",
	    ZH(c) ? "手動規劃" : "Manual plan");
	snprintf(m->btn[PTX_UI_BTN_CLOSE], sizeof (m->btn[0]), "%s",
	    ZH(c) ? "關閉" : "Close");
	for (int b = 0; b < PTX_UI_N_BTN; b++)
		m->btn_enabled[b] = true;
	snprintf(m->hint, sizeof (m->hint), "%s", ZH(c) ?
	    "點一下跑道就開始推。搖桿：auto_pushback 確認、"
	    "auto_pushback_next 換下一條跑道" :
	    "Click a runway to push. Joystick: auto_pushback to confirm, "
	    "auto_pushback_next for the next runway");
}

void
ptx_menu_message(const ptx_menu_ctx_t *c, const char *msg, ptx_ui_model_t *m)
{
	memset(m, 0, sizeof (*m));
	header(c, m);
	snprintf(m->message, sizeof (m->message), "%s", msg);
	m->highlight = -1;
	m->btn_enabled[PTX_UI_BTN_FULL] = false;
	m->btn_enabled[PTX_UI_BTN_XPTUG] = false;
}

typedef struct {
	int	ri;
	bool	ok;
	double	key;	/* -headwind rounded to 1 kt (0 when calm) */
	double	taxi;	/* tie breaker */
} sortent_t;

static int
cmp_sort(const void *a, const void *b)
{
	const sortent_t *x = a, *y = b;
	if (x->ok != y->ok)
		return (x->ok ? -1 : 1);
	if (x->key != y->key)
		return (x->key < y->key ? -1 : 1);
	if (x->taxi != y->taxi)
		return (x->taxi < y->taxi ? -1 : 1);
	return (x->ri < y->ri ? -1 : (x->ri > y->ri ? 1 : 0));
}

static void
route_text(const ptx_graph_t *g, const ptx_result_t *r, int ri, bool zh,
    char *buf, size_t cap)
{
	char names[64][80];
	int n = ptx_route_names(g, r, names, 64);
	const char *entry = ptx_route_entry_name(g, r);
	size_t len = 0;

	buf[0] = '\0';
	for (int i = 0; i < n && len + 8 < cap; i++) {
		len += snprintf(buf + len, cap - len, "%s%s",
		    i ? " → " : "", names[i]);
	}
	if (len < cap) {
		len += snprintf(buf + len, cap - len, zh ?
		    "%s%s 等待點　" : "%shold short %s · ", n ? " → " : "",
		    ptx_rend_name(g, ri));
	}
	if (len < cap && entry[0] != '\0') {
		len += snprintf(buf + len, cap - len, zh ? "從 %s 進入，" :
		    "enter via %s, ", entry);
	}
	if (len < cap) {
		if (r->entry_lost >= 1) {
			len += snprintf(buf + len, cap - len, zh ?
			    "少 %.0f m・" : "%.0f m less · ", r->entry_lost);
		} else if (zh) {
			len += snprintf(buf + len, cap - len, "全長・");
		} else {
			len += snprintf(buf + len, cap - len, "full length · ");
		}
	}
	if (len < cap) {
		(void) snprintf(buf + len, cap - len, zh ? "滑行 %.1f km" :
		    "taxi %.1f km", r->taxi / 1000.0);
	}
}

void
ptx_menu_build(const ptx_graph_t *g, const ptx_result_t *res,
    const ptx_menu_ctx_t *c, ptx_ui_model_t *m, int *order)
{
	int n = ptx_n_rends(g);
	sortent_t *se = calloc(n + 1, sizeof (*se));
	bool calm = (!c->have_wind || c->wind_kt < 1.0);
	bool zh = ZH(c);

	memset(m, 0, sizeof (*m));
	header(c, m);
	for (int i = 0; i < n; i++) {
		se[i].ri = i;
		se[i].ok = (res[i].status == PTX_OK && res[i].pb_allowed);
		se[i].key = calm ? 0 : -round(ptx_menu_headwind(g, i, c));
		/* same trade-off as the planner: 2 taxi m per runway m lost */
		se[i].taxi = (res[i].status == PTX_OK ?
		    res[i].taxi + 2.0 * res[i].entry_lost : 0);
	}
	qsort(se, n, sizeof (*se), cmp_sort);
	m->highlight = -1;
	for (int k = 0; k < n; k++) {
		int ri = se[k].ri;
		const ptx_result_t *r = &res[ri];
		ptx_ui_row_t *rw;
		const char *edge;
		char wind[48];

		if (!se[k].ok || m->n_rows >= PTX_UI_MAX_ROWS) {
			/* footnote: "不可用：07L 到不了、25R 後推太長" */
			size_t len = strlen(m->footnote);
			const char *why = (r->status == PTX_OK ?
			    (zh ? "後推太長" : "pushback too long") :
			    r->status == PTX_NO_ROUTE ? (zh ? "到不了" : "no route") :
			    r->status == PTX_NO_CANDIDATE ?
			    (zh ? "後方沒有滑行道" : "no taxiway behind") :
			    (zh ? "不在滑行網路" : "not in taxi network"));
			if (se[k].ok)
				why = (zh ? "（清單已滿）" : "(list full)");
			snprintf(m->footnote + len, sizeof (m->footnote) - len,
			    "%s%s %s", len == 0 ? (zh ? "不可用：" :
			    "Not available: ") : (zh ? "、" : ", "),
			    ptx_rend_name(g, ri), why);
			continue;
		}
		order[m->n_rows] = ri;
		rw = &m->rows[m->n_rows++];
		snprintf(rw->label, sizeof (rw->label), "%s",
		    ptx_rend_name(g, ri));
		rw->enabled = true;
		if (m->highlight < 0) {
			m->highlight = m->n_rows - 1;
			rw->recommended = true;
		}
		if (c->have_wind && !calm) {
			double h = ptx_menu_headwind(g, ri, c);
			if (h >= 0.5)
				snprintf(wind, sizeof (wind), zh ? "逆風 %.0f kt" :
				    "headwind %.0f kt", h);
			else if (h <= -0.5)
				snprintf(wind, sizeof (wind), zh ? "順風 %.0f kt" :
				    "tailwind %.0f kt", -h);
			else
				snprintf(wind, sizeof (wind), zh ? "側風" :
				    "crosswind");
		} else {
			wind[0] = '\0';
		}
		if (rw->recommended) {
			snprintf(rw->tag, sizeof (rw->tag), "%s%s%s",
			    zh ? "建議" : "best",
			    wind[0] ? (zh ? "・" : " · ") : "",
			    wind[0] ? wind : (calm ? (zh ? "・滑行最短" :
			    " · shortest taxi") : ""));
		} else {
			snprintf(rw->tag, sizeof (rw->tag), "%s", wind);
		}
		edge = ptx_edge_name(g, r->edge);
		if (edge[0] == '\0')
			edge = zh ? "停機坪滑行道" : "apron taxilane";
		snprintf(rw->line1, sizeof (rw->line1), zh ?
		    "推完機頭 %03.0f°・停在 %s・後推 %.0f m%s" :
		    "nose %03.0f° on %s · push %.0f m%s", r->end_hdg,
		    edge, r->pb_len, r->pb_warn ? (zh ? "（偏長）" :
		    " (long)") : "");
		rw->warn = r->pb_warn;
		route_text(g, r, ri, zh, rw->line2, sizeof (rw->line2));
	}
	if (m->n_rows == 0 && m->message[0] == '\0') {
		snprintf(m->message, sizeof (m->message), "%s", zh ?
		    "沒有可自動後推的跑道，請改用手動規劃" :
		    "No runway for an automatic pushback; use the manual "
		    "planner");
		m->btn_enabled[PTX_UI_BTN_FULL] = false;
		m->btn_enabled[PTX_UI_BTN_XPTUG] = false;
	}
	free(se);
}

void
ptx_menu_summary(const ptx_graph_t *g, const ptx_result_t *r, int ri,
    bool cjk, char *buf, size_t cap)
{
	const char *edge = ptx_edge_name(g, r->edge);
	if (cjk) {
		snprintf(buf, cap, "跑道 %s：推完機頭 %03.0f°，停在 %s，"
		    "後推 %.0f 公尺", ptx_rend_name(g, ri), r->end_hdg,
		    edge[0] ? edge : "停機坪滑行道", r->pb_len);
	} else {
		snprintf(buf, cap, "Runway %s: pushing back to face %03.0f "
		    "on taxiway %s, %.0f meters", ptx_rend_name(g, ri),
		    r->end_hdg, edge[0] ? edge : "apron", r->pb_len);
	}
}
