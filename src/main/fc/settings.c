#include <string.h>
#include <stdint.h>

#include "common/string_light.h"

#include "fc/settings_generated.h"
#include "fc/settings.h"

#include "fc/settings_generated.c"

void clivalue_get_name(const clivalue_t *val, char *buf)
{
	uint8_t bpos = 0;
	uint16_t n = 0;
#ifndef CLIVALUE_ENCODED_NAME_USES_BYTE_INDEXING
	uint8_t shift = 0;
#endif
	for (uint8_t ii = 0; ii < CLIVALUE_ENCODED_NAME_MAX_BYTES; ii++) {
#ifdef CLIVALUE_ENCODED_NAME_USES_BYTE_INDEXING
		n = val->encoded_name[ii];
#else
		// Decode a variable size uint
		uint16_t b = val->encoded_name[ii];
		if (b >= 0x80) {
			// More bytes follow
			n |= (b&0x7f) << shift;
			shift += 7;
			continue;
		}
		// Final byte
		n |= b << shift;
#endif
		const char *word = cliValueWords[n];
		if (!word) {
			// No more words
			break;
		}
		if (bpos > 0) {
			// Word separator
			buf[bpos++] = '_';
		}
		strcpy(&buf[bpos], word);
		bpos += strlen(word);
#ifndef CLIVALUE_ENCODED_NAME_USES_BYTE_INDEXING
		// Reset shift and n
		shift = 0;
		n = 0;
#endif
	}
	buf[bpos] = '\0';
}

bool clivalue_name_contains(const clivalue_t *val, char *buf, const char *cmdline)
{
	clivalue_get_name(val, buf);
	return strstr(buf, cmdline) != NULL;
}

bool clivalue_name_exact_match(const clivalue_t *val, char *buf, const char *cmdline, uint8_t var_name_length)
{
	clivalue_get_name(val, buf);
	return sl_strncasecmp(cmdline, buf, strlen(buf)) == 0 && var_name_length == strlen(buf);
}

pgn_t clivalue_get_pgn(const clivalue_t *val)
{
	uint16_t pos = val - (const clivalue_t *)cliValueTable;
	uint16_t acc = 0;
	for (uint8_t ii = 0; ii < CLIVALUE_PGN_COUNT; ii++) {
		acc += cliValuePgnCounts[ii];
		if (acc > pos) {
			return cliValuePgn[ii];
		}
	}
	return -1;
}

clivalue_min_t clivalue_get_min(const clivalue_t *val)
{
	return cliValueMinMaxTable[CLIVALUE_INDEXES_GET_MIN(val)];
}

clivalue_max_t clivalue_get_max(const clivalue_t *val)
{
	return cliValueMinMaxTable[CLIVALUE_INDEXES_GET_MAX(val)];
}
