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
 * Offline driver for the C planner (src/planner).
 *
 *   plan_cli all  --apt FILE [--icao XXXX] --fleet FLEET [--stand-ref nose|main]
 *                 [--w-along N] [--out pairs.json] [--timing]
 *       Every stand x every runway end, same flow as tools/proto/pbplan.py
 *       evaluate_airport(); writes the same per-pair JSON as its --pairs-out.
 *
 *   plan_cli menu --apt FILE [--icao XXXX] --stand NAME
 *                 [--wheelbase M --max-steer DEG --class C | --fleet FLEET]
 *                 [--full-length] [--stand-ref nose|main]
 *       The runway menu for one stand, as the plugin will show it.
 *
 * FLEET is the text file written by pbplan.py --fleet-out:
 *   mode class|b738
 *   <stand class> <wheelbase m> <max steer deg> <aircraft width class>
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <acfutils/geom.h>

#include "planner/ptx_plan.h"

typedef struct {
	bool	have;
	double	wheelbase, max_steer;
	char	own;
} fleet_ac_t;

typedef struct {
	bool		b738;
	fleet_ac_t	cls[6];
} fleet_t;

static double
now_ms(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (ts.tv_sec * 1e3 + ts.tv_nsec / 1e6);
}

static void
die(const char *msg)
{
	fprintf(stderr, "plan_cli: %s\n", msg);
	exit(2);
}

static bool
load_fleet(const char *path, fleet_t *f)
{
	FILE *fp = fopen(path, "r");
	char line[256];

	if (fp == NULL)
		return (false);
	memset(f, 0, sizeof (*f));
	while (fgets(line, sizeof (line), fp) != NULL) {
		char c, own;
		double wb, ms;
		char mode[32];
		if (sscanf(line, "mode %31s", mode) == 1) {
			f->b738 = (strcmp(mode, "b738") == 0);
		} else if (sscanf(line, "%c %lf %lf %c", &c, &wb, &ms,
		    &own) == 4 && c >= 'A' && c <= 'F') {
			fleet_ac_t *a = &f->cls[c - 'A'];
			a->have = true;
			a->wheelbase = wb;
			a->max_steer = ms;
			a->own = own;
		}
	}
	fclose(fp);
	return (true);
}

/* JSON string; apt.dat is latin-1, the prototype decodes it as such */
static void
json_str(FILE *fp, const char *s)
{
	fputc('"', fp);
	for (const unsigned char *p = (const unsigned char *)s; *p; p++) {
		if (*p == '"' || *p == '\\') {
			fprintf(fp, "\\%c", *p);
		} else if (*p < 0x20) {
			fprintf(fp, "\\u%04x", *p);
		} else if (*p >= 0x80) {
			fputc(0xC0 | (*p >> 6), fp);
			fputc(0x80 | (*p & 0x3F), fp);
		} else {
			fputc(*p, fp);
		}
	}
	fputc('"', fp);
}

static void
json_num(FILE *fp, double v)
{
	if (isnan(v) || isinf(v))
		fputs("null", fp);
	else
		fprintf(fp, "%.17g", v);
}

static bool
is_helo(const char *acts)
{
	char buf[64];
	char *save = NULL, *t;
	bool any = false;

	if (acts[0] == '\0')
		return (false);
	strncpy(buf, acts, sizeof (buf) - 1);
	buf[sizeof (buf) - 1] = '\0';
	for (t = strtok_r(buf, "|", &save); t != NULL;
	    t = strtok_r(NULL, "|", &save)) {
		if (strcmp(t, "helos") != 0)
			return (false);
		any = true;
	}
	return (any);
}

static char
stand_cls(const ptx_stand_t *s)
{
	if (s->cls != 0)
		return (s->cls);
	if (strstr(s->actypes, "heavy") != NULL)
		return ('E');
	if (strstr(s->actypes, "jets") != NULL)
		return ('C');
	if (strstr(s->actypes, "turboprops") != NULL)
		return ('B');
	return ('A');
}

/* route instruction names, like the prototype's describe_route() */
static int
route_names(const ptx_graph_t *g, const ptx_result_t *r, char names[][80],
    int max)
{
	int n = 0;
	for (int i = 0; i < r->n_route && n < max; i++) {
		int e;
		char nm[80];
		ptx_dedge(g, r->route[i], &e, NULL, NULL);
		if (ptx_edge_is_runway(g, e))
			snprintf(nm, sizeof (nm), "RWY %s", ptx_edge_name(g, e));
		else
			snprintf(nm, sizeof (nm), "%s", ptx_edge_name(g, e));
		if (nm[0] == '\0')
			continue;
		if (n > 0 && strcmp(names[n - 1], nm) == 0)
			continue;
		strcpy(names[n++], nm);
	}
	return (n);
}

static int
cmp_dbl(const void *a, const void *b)
{
	double x = *(const double *)a, y = *(const double *)b;
	return (x < y ? -1 : (x > y ? 1 : 0));
}

static void
print_dist(const char *label, double *v, int n)
{
	if (n == 0)
		return;
	qsort(v, n, sizeof (double), cmp_dbl);
	fprintf(stderr, "%s: n=%d 中位 %.3f / p95 %.3f / 最大 %.3f ms\n", label,
	    n, v[n / 2], v[(int)floor((n - 1) * 0.95)], v[n - 1]);
}

static int
cmd_all(int argc, char **argv)
{
	const char *apt_path = NULL, *icao = NULL, *fleet_path = NULL;
	const char *out_path = NULL;
	bool main_ref = false, timing = false;
	ptx_opts_t opts;
	fleet_t fleet;
	ptx_apt_t *apt;
	ptx_graph_t *g;
	FILE *out;
	bool first = true;
	double t0, t_graph;
	double *warm, *cold;
	int n_warm = 0, n_cold = 0;
	ptx_result_t *res;

	ptx_opts_default(&opts);
	for (int i = 0; i < argc; i++) {
		if (strcmp(argv[i], "--apt") == 0 && i + 1 < argc)
			apt_path = argv[++i];
		else if (strcmp(argv[i], "--icao") == 0 && i + 1 < argc)
			icao = argv[++i];
		else if (strcmp(argv[i], "--fleet") == 0 && i + 1 < argc)
			fleet_path = argv[++i];
		else if (strcmp(argv[i], "--out") == 0 && i + 1 < argc)
			out_path = argv[++i];
		else if (strcmp(argv[i], "--w-along") == 0 && i + 1 < argc)
			opts.w_along = atof(argv[++i]);
		else if (strcmp(argv[i], "--stand-ref") == 0 && i + 1 < argc)
			main_ref = (strcmp(argv[++i], "main") == 0);
		else if (strcmp(argv[i], "--timing") == 0)
			timing = true;
		else
			die("unknown argument to 'all'");
	}
	if (apt_path == NULL || fleet_path == NULL)
		die("'all' needs --apt and --fleet");
	if (!load_fleet(fleet_path, &fleet))
		die("can't read fleet file");
	apt = ptx_apt_load(apt_path, icao);
	if (apt == NULL)
		die("can't load airport");
	t0 = now_ms();
	g = ptx_graph_new(apt);
	t_graph = now_ms() - t0;
	res = calloc(ptx_n_rends(g) + 1, sizeof (ptx_result_t));
	warm = calloc(apt->n_stands + 1, sizeof (double));
	cold = calloc(apt->n_stands + 1, sizeof (double));

	out = (out_path != NULL ? fopen(out_path, "w") : stdout);
	if (out == NULL)
		die("can't open output");
	fputc('[', out);
	for (int si = 0; si < apt->n_stands; si++) {
		const ptx_stand_t *st = &apt->stands[si];
		char cls;
		const fleet_ac_t *fa;
		ptx_aircraft_t ac;
		ptx_kind_t kind;
		double dback, dfront, ts;
		vect2_t Pn, Pm;
		ptx_diag_t diag;

		if (!first)
			fputc(',', out);
		first = false;
		if (is_helo(st->actypes)) {
			fprintf(out, "{\"si\":%d,\"stand\":", si);
			json_str(out, st->name);
			fprintf(out, ",\"kind\":\"helo\",\"cls\":null}");
			continue;
		}
		cls = stand_cls(st);
		if (fleet.b738 && cls < 'C') {
			fprintf(out, "{\"si\":%d,\"stand\":", si);
			json_str(out, st->name);
			fprintf(out, ",\"kind\":\"too_small_for_fleet\","
			    "\"cls\":null}");
			continue;
		}
		fa = &fleet.cls[(fleet.b738 ? 'C' : cls) - 'A'];
		if (!fa->have)
			die("fleet file lacks a class");
		ac.wheelbase = fa->wheelbase;
		ac.max_steer = fa->max_steer;
		ac.route_cls = (fleet.b738 ? 'C' : (cls < fa->own ? cls :
		    fa->own));

		ts = now_ms();
		Pn = ptx_project(g, st->lat, st->lon);
		kind = ptx_classify(g, Pn, st->hdg, st->type, &dback, &dfront);
		Pm = (main_ref ? Pn : VECT2(Pn.x - sin(st->hdg * (M_PI / 180.0)) *
		    ac.wheelbase, Pn.y - cos(st->hdg * (M_PI / 180.0)) *
		    ac.wheelbase));
		ptx_plan(g, Pm, Pn, st->hdg, si, &ac, &opts, res, &diag);
		warm[n_warm++] = now_ms() - ts;

		for (int ri = 0; ri < ptx_n_rends(g); ri++) {
			const ptx_result_t *r = &res[ri];
			if (ri > 0)
				fputc(',', out);
			fprintf(out, "{\"si\":%d,\"stand\":", si);
			json_str(out, st->name);
			fprintf(out, ",\"kind\":\"%s\",\"cls\":\"%c\","
			    "\"ncand\":%d,\"dback\":", ptx_kind_str(kind), cls,
			    diag.n_cand);
			json_num(out, dback);
			fputs(",\"dfront\":", out);
			json_num(out, dfront);
			fputs(",\"rwy\":", out);
			json_str(out, ptx_rend_name(g, ri));
			if (r->status != PTX_OK) {
				fprintf(out, ",\"ok\":false,\"reason\":\"%s\"}",
				    ptx_status_str(r->status));
				continue;
			}
			{
				char names[64][80];
				int nn = route_names(g, r, names, 64);
				fprintf(out, ",\"ok\":true,\"a\":%d,\"b\":%d,"
				    "\"sgn\":%d,\"x\":", ptx_edge_file_a(g, r->edge),
				    ptx_edge_file_b(g, r->edge), r->sgn);
				json_num(out, r->end.x);
				fputs(",\"y\":", out);
				json_num(out, r->end.y);
				fputs(",\"hdg\":", out);
				json_num(out, r->end_hdg);
				fputs(",\"pb\":", out);
				json_num(out, r->pb_len);
				fprintf(out, ",\"nseg\":%d,\"rot\":", r->nseg);
				json_num(out, r->rot);
				fputs(",\"r\":", out);
				json_num(out, r->r_used);
				fprintf(out, ",\"conf\":%d,\"xing\":%d,\"taxi\":",
				    r->n_conflicts, r->n_xings);
				json_num(out, r->taxi);
				fputs(",\"lost\":", out);
				json_num(out, r->entry_lost);
				fputs(",\"ft\":", out);
				json_num(out, r->first_turn);
				fputs(",\"mt\":", out);
				json_num(out, r->max_turn);
				fprintf(out, ",\"cross\":%d,\"names\":[",
				    r->crossings);
				for (int k = 0; k < nn; k++) {
					if (k > 0)
						fputc(',', out);
					json_str(out, names[k]);
				}
				fputs("],\"on\":", out);
				json_num(out, r->off_nose);
				fputs(",\"om\":", out);
				json_num(out, r->off_main);
				fputs(",\"ang\":", out);
				json_num(out, r->ang_edge);
				fprintf(out, ",\"tier\":\"%s\",\"nroute\":%d,"
				    "\"allowed\":%s,\"warn\":%s}",
				    ptx_tier_str(r->tier), r->n_route + 1,
				    r->pb_allowed ? "true" : "false",
				    r->pb_warn ? "true" : "false");
			}
		}
		ptx_results_free(res, ptx_n_rends(g));
		/* "cold": what the plugin pays the first time at this stand */
		if (timing) {
			ptx_graph_t *g2;
			ptx_result_t *res2 = calloc(ptx_n_rends(g) + 1,
			    sizeof (ptx_result_t));
			ts = now_ms();
			g2 = ptx_graph_new(apt);
			(void) ptx_classify(g2, Pn, st->hdg, st->type, NULL,
			    NULL);
			ptx_plan(g2, Pm, Pn, st->hdg, si, &ac, &opts, res2,
			    NULL);
			cold[n_cold++] = now_ms() - ts;
			ptx_results_free(res2, ptx_n_rends(g2));
			free(res2);
			ptx_graph_free(g2);
		}
	}
	fputs("]\n", out);
	if (out != stdout)
		fclose(out);
	fprintf(stderr, "%s：%d 個停機位、%d 個跑道端，建圖 %.3f ms\n",
	    apt->icao, apt->n_stands, ptx_n_rends(g), t_graph);
	print_dist("每個停機位（圖與路由表已建好）", warm, n_warm);
	if (timing)
		print_dist("每個停機位（從載入機場算起：建圖＋路由表＋規劃）",
		    cold, n_cold);
	free(warm);
	free(cold);
	free(res);
	ptx_graph_free(g);
	ptx_apt_free(apt);
	return (0);
}

static int
cmd_menu(int argc, char **argv)
{
	const char *apt_path = NULL, *icao = NULL, *stand = NULL;
	const char *fleet_path = NULL;
	double wb = NAN, ms = NAN;
	char cls = 0;
	bool full = false, main_ref = false;
	ptx_opts_t opts;
	ptx_apt_t *apt;
	ptx_graph_t *g;
	int si = -1;
	const ptx_stand_t *st;
	ptx_aircraft_t ac;
	vect2_t Pn, Pm;
	double dback, dfront, t0, t1;
	ptx_kind_t kind;
	ptx_result_t *res;

	ptx_opts_default(&opts);
	for (int i = 0; i < argc; i++) {
		if (strcmp(argv[i], "--apt") == 0 && i + 1 < argc)
			apt_path = argv[++i];
		else if (strcmp(argv[i], "--icao") == 0 && i + 1 < argc)
			icao = argv[++i];
		else if (strcmp(argv[i], "--stand") == 0 && i + 1 < argc)
			stand = argv[++i];
		else if (strcmp(argv[i], "--fleet") == 0 && i + 1 < argc)
			fleet_path = argv[++i];
		else if (strcmp(argv[i], "--wheelbase") == 0 && i + 1 < argc)
			wb = atof(argv[++i]);
		else if (strcmp(argv[i], "--max-steer") == 0 && i + 1 < argc)
			ms = atof(argv[++i]);
		else if (strcmp(argv[i], "--class") == 0 && i + 1 < argc)
			cls = argv[++i][0];
		else if (strcmp(argv[i], "--full-length") == 0)
			full = true;
		else if (strcmp(argv[i], "--stand-ref") == 0 && i + 1 < argc)
			main_ref = (strcmp(argv[++i], "main") == 0);
		else
			die("unknown argument to 'menu'");
	}
	if (apt_path == NULL || stand == NULL)
		die("'menu' needs --apt and --stand");
	apt = ptx_apt_load(apt_path, icao);
	if (apt == NULL)
		die("can't load airport");
	for (int i = 0; i < apt->n_stands; i++) {
		if (strcmp(apt->stands[i].name, stand) == 0) {
			si = i;
			break;
		}
	}
	if (si < 0)
		die("no such stand");
	st = &apt->stands[si];
	if (fleet_path != NULL) {
		fleet_t fleet;
		char sc = stand_cls(st);
		const fleet_ac_t *fa;
		if (!load_fleet(fleet_path, &fleet))
			die("can't read fleet file");
		fa = &fleet.cls[sc - 'A'];
		wb = fa->wheelbase;
		ms = fa->max_steer;
		cls = (sc < fa->own ? sc : fa->own);
	}
	if (isnan(wb) || isnan(ms) || cls == 0)
		die("give --wheelbase, --max-steer and --class (or --fleet)");
	ac.wheelbase = wb;
	ac.max_steer = ms;
	ac.route_cls = cls;
	if (full)
		opts.w_along = PTX_W_ALONG_FULL_LENGTH;

	t0 = now_ms();
	g = ptx_graph_new(apt);
	Pn = ptx_project(g, st->lat, st->lon);
	kind = ptx_classify(g, Pn, st->hdg, st->type, &dback, &dfront);
	Pm = (main_ref ? Pn : VECT2(Pn.x - sin(st->hdg * (M_PI / 180.0)) * wb,
	    Pn.y - cos(st->hdg * (M_PI / 180.0)) * wb));
	res = calloc(ptx_n_rends(g) + 1, sizeof (ptx_result_t));
	ptx_plan(g, Pm, Pn, st->hdg, si, &ac, &opts, res, NULL);
	t1 = now_ms();

	printf("%s %s 停機位 %s（%s，等級 %c），機頭 %03.0f°\n", apt->icao,
	    apt->name, st->name, st->type, stand_cls(st), st->hdg);
	printf("機型：軸距 %.2f m，最大轉向 %.0f°，滑行寬度等級 %c%s\n", wb,
	    ms, cls, full ? "，全長入口" : "");
	printf("分類：%s（後方滑行道 %.0f m，前方 %s）\n",
	    kind == PTX_KIND_PUSHBACK ? "需要後推" :
	    kind == PTX_KIND_FACES_TAXIWAY ? "機頭朝滑行道，可直接滑出" :
	    "300 m 內沒有滑行道", dback,
	    isinf(dfront) ? "—" : "有");
	for (int ri = 0; ri < ptx_n_rends(g); ri++) {
		const ptx_result_t *r = &res[ri];
		char names[64][80];
		int nn;
		if (r->status != PTX_OK) {
			printf("  %-4s  不可用（%s）\n", ptx_rend_name(g, ri),
			    r->status == PTX_NO_ROUTE ? "到不了這條跑道" :
			    r->status == PTX_NO_CANDIDATE ?
			    "後方沒有可停的滑行道" : "跑道不在滑行網路裡");
			continue;
		}
		nn = route_names(g, r, names, 64);
		printf("  %-4s  推完機頭 %03.0f°，停在 %s，後推 %.0f m%s%s；"
		    "滑行 %.0f m，入口少 %.0f m\n", ptx_rend_name(g, ri),
		    r->end_hdg, ptx_edge_name(g, r->edge)[0] ?
		    ptx_edge_name(g, r->edge) : "無名滑行道", r->pb_len,
		    r->pb_warn ? "（偏長）" : "",
		    r->pb_allowed ? "" : "（太長，不自動推）", r->taxi,
		    r->entry_lost);
		printf("        ");
		for (int k = 0; k < nn; k++)
			printf("%s%s", k ? " → " : "", names[k]);
		printf(" → hold short %s\n", ptx_rend_name(g, ri));
	}
	printf("計算時間（含建圖）：%.2f ms\n", t1 - t0);
	ptx_results_free(res, ptx_n_rends(g));
	free(res);
	ptx_graph_free(g);
	ptx_apt_free(apt);
	return (0);
}

int
main(int argc, char **argv)
{
	if (argc >= 2 && strcmp(argv[1], "all") == 0)
		return (cmd_all(argc - 2, argv + 2));
	if (argc >= 2 && strcmp(argv[1], "menu") == 0)
		return (cmd_menu(argc - 2, argv + 2));
	fprintf(stderr, "usage: plan_cli all|menu ...（見原始檔開頭說明）\n");
	return (2);
}
