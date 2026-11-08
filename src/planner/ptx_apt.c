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

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ptx_apt.h"

#define	MAX_TOKENS	64

/* Growable array helper. */
#define	PUSH(arr, n, cap, elem) \
	do { \
		if ((n) == (cap)) { \
			(cap) = (cap) ? (cap) * 2 : 64; \
			(arr) = realloc((arr), (cap) * sizeof (*(arr))); \
		} \
		(arr)[(n)++] = (elem); \
	} while (0)

static double
norm_hdg(double h)
{
	h = fmod(h, 360.0);
	if (h < 0)
		h += 360.0;
	if (h <= 0.0 || h > 360.0) {
		if (h < 0.0)
			h = 0.0;
		if (h > 360.0)
			h = 360.0;
	}
	return (h);
}

void
ptx_rwy_norm(const char *in, char out[PTX_RWY_LEN])
{
	size_t n = 0;

	while (isspace((unsigned char)*in))
		in++;
	/* strip leading zeros that are followed by a digit */
	while (in[0] == '0' && isdigit((unsigned char)in[1]))
		in++;
	while (*in != '\0' && !isspace((unsigned char)*in) &&
	    n + 1 < PTX_RWY_LEN) {
		out[n++] = toupper((unsigned char)*in);
		in++;
	}
	out[n] = '\0';
}

static int
tokenize(char *line, char *tok[MAX_TOKENS])
{
	int n = 0;
	char *p = line;

	while (*p != '\0' && n < MAX_TOKENS) {
		while (*p != '\0' && isspace((unsigned char)*p))
			p++;
		if (*p == '\0')
			break;
		tok[n++] = p;
		while (*p != '\0' && !isspace((unsigned char)*p))
			p++;
		if (*p != '\0')
			*p++ = '\0';
	}
	return (n);
}

/* Joins tok[from..n-1] with single spaces, like " ".join(p[from:]). */
static void
join_tokens(char *const tok[], int n, int from, char *out, size_t cap)
{
	size_t len = 0;

	out[0] = '\0';
	for (int i = from; i < n; i++) {
		size_t l = strlen(tok[i]);
		if (len > 0 && len + 1 < cap)
			out[len++] = ' ';
		if (len + l >= cap)
			l = cap - len - 1;
		memcpy(&out[len], tok[i], l);
		len += l;
		out[len] = '\0';
	}
}

static void
copy_str(char *dst, size_t cap, const char *src)
{
	size_t l = strlen(src);
	if (l >= cap)
		l = cap - 1;
	memcpy(dst, src, l);
	dst[l] = '\0';
}

ptx_apt_t *
ptx_apt_load(const char *path, const char *icao)
{
	FILE *fp = fopen(path, "r");
	char *line = NULL;
	size_t linecap = 0;
	ptx_apt_t *apt;
	bool in_apt = (icao == NULL), found = (icao == NULL);
	int last_edge = -1, last_stand = -1;
	int cap_r = 0, cap_s = 0, cap_n = 0, cap_e = 0;

	if (fp == NULL)
		return (NULL);
	apt = calloc(1, sizeof (*apt));

	while (getline(&line, &linecap, fp) > 0) {
		char *tok[MAX_TOKENS];
		int n = tokenize(line, tok);
		const char *c;

		if (n == 0)
			continue;
		c = tok[0];
		if ((strcmp(c, "1") == 0 || strcmp(c, "16") == 0 ||
		    strcmp(c, "17") == 0) && n >= 5) {
			if (icao != NULL) {
				if (in_apt)
					break;	/* next airport: done */
				if (strcmp(tok[4], icao) != 0)
					continue;
				in_apt = true;
				found = true;
			}
			copy_str(apt->icao, sizeof (apt->icao), tok[4]);
			join_tokens(tok, n, 5, apt->name, sizeof (apt->name));
			continue;
		}
		if (!in_apt)
			continue;

		if (strcmp(c, "100") == 0 && n >= 21) {
			ptx_runway_t r;
			memset(&r, 0, sizeof (r));
			r.width = atof(tok[1]);
			copy_str(r.end[0].name, PTX_RWY_LEN, tok[8]);
			r.end[0].lat = atof(tok[9]);
			r.end[0].lon = atof(tok[10]);
			r.end[0].displ = atof(tok[11]);
			copy_str(r.end[1].name, PTX_RWY_LEN, tok[17]);
			r.end[1].lat = atof(tok[18]);
			r.end[1].lon = atof(tok[19]);
			r.end[1].displ = atof(tok[20]);
			PUSH(apt->rwys, apt->n_rwys, cap_r, r);
		} else if (strcmp(c, "1300") == 0 && n >= 5) {
			ptx_stand_t s;
			memset(&s, 0, sizeof (s));
			s.lat = atof(tok[1]);
			s.lon = atof(tok[2]);
			s.hdg = norm_hdg(atof(tok[3]));
			copy_str(s.type, sizeof (s.type), tok[4]);
			if (n > 5)
				copy_str(s.actypes, sizeof (s.actypes), tok[5]);
			join_tokens(tok, n, 6, s.name, sizeof (s.name));
			PUSH(apt->stands, apt->n_stands, cap_s, s);
			last_stand = apt->n_stands - 1;
		} else if (strcmp(c, "1301") == 0 && last_stand >= 0) {
			if (n > 1 && strlen(tok[1]) == 1)
				apt->stands[last_stand].cls = tok[1][0];
		} else if (strcmp(c, "1201") == 0 && n >= 5) {
			ptx_node_t nd;
			nd.lat = atof(tok[1]);
			nd.lon = atof(tok[2]);
			nd.id = atoi(tok[4]);
			PUSH(apt->nodes, apt->n_nodes, cap_n, nd);
		} else if (strcmp(c, "1202") == 0 && n >= 4) {
			ptx_edge_t e;
			const char *typ = (n > 4 ? tok[4] : "taxiway");
			memset(&e, 0, sizeof (e));
			e.a = atoi(tok[1]);
			e.b = atoi(tok[2]);
			e.oneway = (strcmp(tok[3], "oneway") == 0);
			e.runway = (strcmp(typ, "runway") == 0);
			if (strncmp(typ, "taxiway_", 8) == 0 &&
			    typ[8] >= 'A' && typ[8] <= 'F' && typ[9] == '\0')
				e.cls = typ[8];
			join_tokens(tok, n, 5, e.name, sizeof (e.name));
			PUSH(apt->edges, apt->n_edges, cap_e, e);
			last_edge = apt->n_edges - 1;
		} else if (strcmp(c, "1204") == 0 && last_edge >= 0) {
			ptx_edge_t *e = &apt->edges[last_edge];
			ptx_zone_t z;
			char *save = NULL, *r;
			if (n < 3)
				continue;
			memset(&z, 0, sizeof (z));
			if (strcmp(tok[1], "departure") == 0)
				z.kind = PTX_ZONE_DEPARTURE;
			else if (strcmp(tok[1], "arrival") == 0)
				z.kind = PTX_ZONE_ARRIVAL;
			else if (strcmp(tok[1], "ils") == 0)
				z.kind = PTX_ZONE_ILS;
			else
				z.kind = PTX_ZONE_OTHER;
			for (r = strtok_r(tok[2], ",", &save); r != NULL &&
			    z.n_rwys < PTX_MAX_ZONE_RWYS;
			    r = strtok_r(NULL, ",", &save)) {
				ptx_rwy_norm(r, z.rwys[z.n_rwys++]);
			}
			e->zones = realloc(e->zones,
			    (e->n_zones + 1) * sizeof (*e->zones));
			e->zones[e->n_zones++] = z;
		} else if (strcmp(c, "1206") == 0) {
			last_edge = -1;
		}
	}
	free(line);
	fclose(fp);
	if (!found) {
		ptx_apt_free(apt);
		return (NULL);
	}
	return (apt);
}

void
ptx_apt_free(ptx_apt_t *apt)
{
	if (apt == NULL)
		return;
	for (int i = 0; i < apt->n_edges; i++)
		free(apt->edges[i].zones);
	free(apt->rwys);
	free(apt->stands);
	free(apt->nodes);
	free(apt->edges);
	free(apt);
}
