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
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "ptx_scenery.h"

#define	BIG_FILE	(16L << 20)	/* index files bigger than this */
#define	IDX_MAGIC	"PTXIDX1"
#define	MAX_PACKS	2048

static double
now_ms(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (ts.tv_sec * 1e3 + ts.tv_nsec / 1e6);
}

static void
rtrim(char *s)
{
	size_t n = strlen(s);
	while (n > 0 && isspace((unsigned char)s[n - 1]))
		s[--n] = '\0';
}

/*
 * If line is an airport header (row code 1, 16 or 17), copies its ICAO
 * (5th field) into icao and returns true.
 */
static bool
header_icao(const char *line, char icao[16])
{
	const char *p = line;
	int field = 0;

	if (!(p[0] == '1' && (isspace((unsigned char)p[1]) ||
	    ((p[1] == '6' || p[1] == '7') && isspace((unsigned char)p[2])))))
		return (false);
	while (*p != '\0') {
		while (*p != '\0' && isspace((unsigned char)*p))
			p++;
		if (*p == '\0')
			break;
		if (field == 4) {
			size_t n = 0;
			while (*p != '\0' && !isspace((unsigned char)*p) &&
			    n + 1 < 16)
				icao[n++] = *p++;
			icao[n] = '\0';
			return (n > 0);
		}
		while (*p != '\0' && !isspace((unsigned char)*p))
			p++;
		field++;
	}
	return (false);
}

/* Linear scan for the airport header; returns its offset or -1. */
static long
scan_file(const char *path, const char *icao)
{
	FILE *fp = fopen(path, "r");
	char *line = NULL;
	size_t cap = 0;
	long pos = 0, found = -1;
	ssize_t n;

	if (fp == NULL)
		return (-1);
	while ((n = getline(&line, &cap, fp)) > 0) {
		char id[16];
		if (header_icao(line, id) && strcmp(id, icao) == 0) {
			found = pos;
			break;
		}
		pos += n;
	}
	free(line);
	fclose(fp);
	return (found);
}

static uint64_t
fnv1a(const char *s)
{
	uint64_t h = 1469598103934665603ULL;
	for (; *s; s++) {
		h ^= (unsigned char)*s;
		h *= 1099511628211ULL;
	}
	return (h);
}

static void
mkdirs(const char *dir)
{
	char buf[1024];
	size_t n = strlen(dir);

	if (n >= sizeof (buf))
		return;
	memcpy(buf, dir, n + 1);
	for (size_t i = 1; i < n; i++) {
		if (buf[i] == '/') {
			buf[i] = '\0';
			(void) mkdir(buf, 0755);
			buf[i] = '/';
		}
	}
	(void) mkdir(buf, 0755);
}

/* Looks icao up in the cached index of a big apt.dat, (re)building it. */
static long
index_lookup(const char *apt, const struct stat *st, const char *cachedir,
    const char *icao, bool *built)
{
	char idx[1200], tmp[1300];
	FILE *fp;
	char *line = NULL;
	size_t cap = 0;
	long found = -1;
	bool valid = false;

	*built = false;
	snprintf(idx, sizeof (idx), "%s/%016llx.idx", cachedir,
	    (unsigned long long)fnv1a(apt));
	fp = fopen(idx, "r");
	if (fp != NULL) {
		long long size, mtime;
		char magic[16];
		if (getline(&line, &cap, fp) > 0 &&
		    sscanf(line, "%15s %lld %lld", magic, &size, &mtime) == 3 &&
		    strcmp(magic, IDX_MAGIC) == 0 &&
		    size == (long long)st->st_size &&
		    mtime == (long long)st->st_mtime)
			valid = true;
		while (valid && getline(&line, &cap, fp) > 0) {
			char id[16];
			long off;
			if (sscanf(line, "%15s %ld", id, &off) == 2 &&
			    strcmp(id, icao) == 0) {
				found = off;
				break;
			}
		}
		fclose(fp);
	}
	if (valid) {
		free(line);
		return (found);
	}

	/* (re)build */
	*built = true;
	mkdirs(cachedir);
	snprintf(tmp, sizeof (tmp), "%s.tmp.%d", idx, (int)getpid());
	{
		FILE *in = fopen(apt, "r"), *out = fopen(tmp, "w");
		long pos = 0;
		ssize_t n;
		if (in == NULL || out == NULL) {
			if (in != NULL)
				fclose(in);
			if (out != NULL)
				fclose(out);
			free(line);
			/* fall back to a plain scan */
			return (scan_file(apt, icao));
		}
		fprintf(out, "%s %lld %lld\n", IDX_MAGIC,
		    (long long)st->st_size, (long long)st->st_mtime);
		while ((n = getline(&line, &cap, in)) > 0) {
			char id[16];
			if (header_icao(line, id)) {
				fprintf(out, "%s %ld\n", id, pos);
				if (found < 0 && strcmp(id, icao) == 0)
					found = pos;
			}
			pos += n;
		}
		fclose(in);
		if (fclose(out) == 0)
			(void) rename(tmp, idx);
		else
			(void) unlink(tmp);
	}
	free(line);
	return (found);
}

static int
read_packs(const char *xpdir, char (*packs)[512])
{
	char path[1024];
	FILE *fp;
	char *line = NULL;
	size_t cap = 0;
	int n = 0;
	bool have_global = false;

	snprintf(path, sizeof (path), "%s/Custom Scenery/scenery_packs.ini",
	    xpdir);
	fp = fopen(path, "r");
	if (fp != NULL) {
		while (getline(&line, &cap, fp) > 0 && n < MAX_PACKS) {
			const char *p;
			rtrim(line);
			if (strncmp(line, "SCENERY_PACK_DISABLED", 21) == 0)
				continue;
			if (strncmp(line, "SCENERY_PACK ", 13) != 0)
				continue;
			p = line + 13;
			while (isspace((unsigned char)*p))
				p++;
			if (strcmp(p, "*GLOBAL_AIRPORTS*") == 0) {
				snprintf(packs[n++], 512,
				    "%s/Global Scenery/Global Airports/", xpdir);
				have_global = true;
			} else if (p[0] == '/') {
				snprintf(packs[n++], 512, "%s%s", p,
				    p[strlen(p) - 1] == '/' ? "" : "/");
			} else {
				snprintf(packs[n++], 512, "%s/%s%s", xpdir, p,
				    p[strlen(p) - 1] == '/' ? "" : "/");
			}
		}
		fclose(fp);
	}
	free(line);
	if (!have_global && n < MAX_PACKS) {
		snprintf(packs[n++], 512, "%s/Global Scenery/Global Airports/",
		    xpdir);
	}
	return (n);
}

ptx_apt_t *
ptx_scenery_load(const char *xpdir, const char *cachedir, const char *icao,
    ptx_scenery_info_t *info)
{
	char (*packs)[512] = calloc(MAX_PACKS, 512);
	int n_packs;
	ptx_scenery_info_t dummy;
	ptx_apt_t *apt = NULL;
	double t0 = now_ms();

	if (info == NULL)
		info = &dummy;
	memset(info, 0, sizeof (*info));
	info->offset = -1;
	n_packs = read_packs(xpdir, packs);
	for (int i = 0; i < n_packs; i++) {
		char apt_path[1024];
		struct stat st;
		long off;
		bool built = false;

		snprintf(apt_path, sizeof (apt_path),
		    "%sEarth nav data/apt.dat", packs[i]);
		if (stat(apt_path, &st) != 0 || !S_ISREG(st.st_mode))
			continue;
		if (st.st_size > BIG_FILE)
			off = index_lookup(apt_path, &st, cachedir, icao, &built);
		else
			off = scan_file(apt_path, icao);
		if (built)
			info->index_built = true;
		if (off < 0)
			continue;
		strncpy(info->pack, packs[i], sizeof (info->pack) - 1);
		strncpy(info->apt_path, apt_path, sizeof (info->apt_path) - 1);
		info->offset = off;
		info->ms_find = now_ms() - t0;
		t0 = now_ms();
		apt = ptx_apt_load_at(apt_path, off, icao);
		info->ms_load = now_ms() - t0;
		break;
	}
	free(packs);
	return (apt);
}
