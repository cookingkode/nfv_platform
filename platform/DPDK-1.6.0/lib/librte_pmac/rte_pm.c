/*-
 *      Copyright(c) 2010-2013 Intel Corporation. All rights reserved.
 *      All rights reserved.
 *   
 *      Redistribution and use in source and binary forms, with or without
 *      modification, are permitted provided that the following conditions
 *      are met:
 *   
 *        * Redistributions of source code must retain the above copyright
 *          notice, this list of conditions and the following disclaimer.
 *        * Redistributions in binary form must reproduce the above copyright
 *          notice, this list of conditions and the following disclaimer in
 *          the documentation and/or other materials provided with the
 *          distribution.
 *        * Neither the name of Intel Corporation nor the names of its
 *          contributors may be used to endorse or promote products derived
 *          from this software without specific prior written permission.
 *  
 *        * The use of this source code in binary form is permitted to only
 *          be used on Intel® Architecture Processors.
 *   
 *      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *      AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *      IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *      ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *      LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *      DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *      SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *      CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *      OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *      USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#include <rte_pm.h>

#define	PM_BINARY_START	'|'
#define	PM_BINARY_END	PM_BINARY_START
#define PM_BINARY_DELIM	' '
#define PM_BINARY_LEN	2

TAILQ_HEAD(rte_pmx_list, rte_pm_ctx);

const char rte_pm_search_names[RTE_PM_SEARCH_NUM][RTE_PM_SEARCH_NAMELEN] = {
	#define  pm_search_method(num, name)    name,
	#include <rte_pm_def.h>
};

enum rte_pm_search
rte_pm_search_type_by_name(const char *name)
{
	int n;

	if (name == NULL)
		return RTE_PM_SEARCH_UNDEF;

	for (n = RTE_PM_SEARCH_NUM - 1; n != RTE_PM_SEARCH_UNDEF; n--) {
		if (strncmp(name, rte_pm_search_names[n],
				sizeof (rte_pm_search_names[n])) == 0)
			break;
	}
	return ((enum rte_pm_search)n);
}

/*
 * Converts input character string into pattern.
 * Treats "|[hex][hex] ([hex][hex])*| as binary sequnce
 * (aka SNORT 'contents' format).
 * As an example: "ab|0D 0A|c" will be treated as "ab\r\nc"
 *
 * @param in
 *   Input character string.
 * @param out
 *   Pointer to the output pattern buffer.
 * @param len
 *   Length of the output buffer.
 * @return
 *  - -EINVAL if the parameters are invalid.
 *  - -ENOMEM output buffer is not big enough to hold converted pattern.
 */
int
rte_pm_convert_pattern(const char *in, uint8_t *out, uint32_t len)
{
	const char *next, *prev;
	char *end;
	uint32_t i, sz, val;

	if (in == NULL || out == NULL)
		return (-EINVAL);

	prev = in;
	for (i = 0; i != len; ) {

		if ((next = strchr(prev, PM_BINARY_START)) != NULL) 
			sz = next - prev;
		else
			sz = strnlen(prev, len - i);

		sz = RTE_MIN(sz, len - i);

		memcpy(out + i, prev, sz);
		i += sz;

		/* reached end of input string. */
		if (next == NULL)
			return (i);

		prev = next + 1;

		/* convert |[hex][hex]( [hex][hex])*| */
		while (i != len) {
			errno = 0;
			val = strtoul(prev, &end, 16);
			if (errno != 0 || val > UINT8_MAX ||
					end - prev != PM_BINARY_LEN)
				return (-EINVAL);

			out[i] = (uint8_t)val;
			i++;

			prev = end + 1;

			if (end[0] !=  PM_BINARY_DELIM) {
				if (end[0] != PM_BINARY_END)
					return (-EINVAL);
				break;
			}
		}
	}

	/* not enough space to convert entire input string. */
	return (-ENOMEM);
}

struct rte_pm_ctx *
rte_pm_find_existing(const char *name)
{
	struct rte_pm_ctx *pmx;
	struct rte_pmx_list *pmx_list;

	/* check that we have an initialised tail queue */
	if ((pmx_list = RTE_TAILQ_LOOKUP_BY_IDX(RTE_TAILQ_PM,
			rte_pmx_list)) == NULL) {
		rte_errno = E_RTE_NO_TAILQ;
		return NULL;
	}

	rte_rwlock_read_lock(RTE_EAL_TAILQ_RWLOCK);
	TAILQ_FOREACH(pmx, pmx_list, next) {
		if (strncmp(name, pmx->name, sizeof(pmx->name)) == 0)
			break;
	}
	rte_rwlock_read_unlock(RTE_EAL_TAILQ_RWLOCK);

	if (pmx == NULL)
		rte_errno = ENOENT;
	return (pmx);
}

void
rte_pm_free(struct rte_pm_ctx *pmx)
{
	if (pmx == NULL)
		return;

	RTE_EAL_TAILQ_REMOVE(RTE_TAILQ_PM, rte_pmx_list, pmx);

	ac_free(&pmx->ac2);
	rte_free(pmx);
}

struct rte_pm_ctx *
rte_pm_create(const struct rte_pm_param *param)
{
	size_t sz;
	struct rte_pm_ctx *ctx;
	struct rte_pmx_list *pmx_list;
	char name[sizeof (ctx->name)];

	/* check that we have an initialised tail queue */
	if ((pmx_list = RTE_TAILQ_LOOKUP_BY_IDX(RTE_TAILQ_PM,
			rte_pmx_list)) == NULL) {
		rte_errno = E_RTE_NO_TAILQ;
		return NULL;
	}

	/* check that input parameters are valid. */
	if (param == NULL || param->name == NULL) {
		rte_errno = EINVAL;
		return (NULL);
	}

	rte_snprintf(name, sizeof(name), "PM_%s", param->name);

	/* calculate amount of memory required for pattern set. */
	sz = sizeof (*ctx) + param->max_pattern_num * sizeof (*ctx->patterns) +
		param->max_pattern_len;

	/* get EAL TAILQ lock. */
	rte_rwlock_write_lock(RTE_EAL_TAILQ_RWLOCK);

	/* if we already have one with that name */
	TAILQ_FOREACH(ctx, pmx_list, next) {
		if (strncmp(param->name, ctx->name, sizeof (ctx->name)) == 0)
			break;
	}

	/* if PM context with such name doesn't exist, then create a new one. */
	if (ctx == NULL && (ctx = rte_zmalloc_socket(name, sz, CACHE_LINE_SIZE,
			param->socket_id)) != NULL) {

		/* init new allocated context. */
		ctx->patterns = (struct rte_pm_pattern *)(ctx + 1);
		ctx->pattern_buf = (uint8_t *)
			(ctx->patterns + param->max_pattern_num);
		ctx->max_pattern_num = param->max_pattern_num;
		ctx->max_pattern_len = param->max_pattern_len;
		ctx->socket_id = param->socket_id;
		rte_snprintf(ctx->name, sizeof(ctx->name), "%s", param->name);

		TAILQ_INSERT_TAIL(pmx_list, ctx, next);

	} else if (ctx == NULL) {
		RTE_LOG(ERR, PMAC,
			"allocation of %zu bytes on socket %d for %s failed\n",
			sz, param->socket_id, name);
	}

	rte_rwlock_write_unlock(RTE_EAL_TAILQ_RWLOCK);
	return (ctx);
}

int
rte_pm_add_patterns(struct rte_pm_ctx *pmx, const struct rte_pm_pattern *pat,
	uint32_t num)
{
	size_t sz;
	uint32_t i, idx;
	uint8_t *pos;

	if (pmx == NULL || pat == NULL)
		return -EINVAL;

	sz = 0;
	for (i = 0; i != num; i++)
		sz += pat[i].len;

	if (num + pmx->pattern_num > pmx->max_pattern_num ||
		sz + pmx->pattern_len > pmx->max_pattern_len)
		return (-ENOMEM);

	idx = pmx->pattern_num;
	pos = pmx->pattern_buf + pmx->pattern_len;

	for (i = 0; i != num; i++) {
		memcpy(pos, pat[i].pattern, pat[i].len);
		pmx->patterns[idx + i] = pat[i];
		pmx->patterns[idx + i].pattern = pos;
		pos += pat[i].len;
	}

	pmx->pattern_len = (pos - pmx->pattern_buf);
	pmx->pattern_num += num;

	return (0);
}

int
rte_pm_analyze(const struct rte_pm_ctx *pmx, const struct rte_pm_build_opt *opt,
	struct rte_pm_search_avail *res)
{
	int rc;

	if (pmx == NULL || opt == NULL || res == NULL)
		return -EINVAL;

	rc = ac_analyze(pmx, opt, res); 
	return (rc);
}

int
rte_pm_build(struct rte_pm_ctx *pmx, const struct rte_pm_build_opt *opt)
{
	int rc;

	if (pmx == NULL || opt == NULL)
		return (-EINVAL);

	/* free previously builded/loaded context, if any. */
	ac_free(&pmx->ac2);

	rc = ac_build(pmx, opt); 
	return (rc);
}

int
rte_pm_store(const struct rte_pm_ctx *pmx, pm_store_fn_t *fn, void *arg) 
{
	if (pmx == NULL || fn == NULL || arg == NULL)
		return (-EINVAL);

	return (ac_store(pmx, fn, arg));
}


int
rte_pm_load(struct rte_pm_ctx *pmx, pm_load_fn_t *fn, void *arg) 
{
	int rc;

	if (pmx == NULL || fn == NULL || arg == NULL)
		return (-EINVAL);

	/* free previously builded/loaded context, if any. */
	ac_free(&pmx->ac2);

	if ((rc = ac_load(pmx, fn, arg)) != 0)
		ac_free(&pmx->ac2);

	return (rc);
}

/*
 * Dump related routinies.
 */
static void
pm_patterns_dump(const struct rte_pm_pattern *pt, uint32_t num,
	const char *prefix)
{
	uint32_t i, j;

	for (i = 0; i != num; i++) {
		printf("%spattern: \"", prefix);
		for (j = 0; j != pt[i].len; j++) {
			if (isprint(pt[i].pattern[j]))
				printf("%c", pt[i].pattern[j]);
			else
				printf("%c" "%02X" "%c", PM_BINARY_START,
					pt[i].pattern[j], PM_BINARY_END);
		}
		printf("\", len: %" PRIu32 ", userdata: %" PRIu64 "\n",
			pt[i].len, pt[i].userdata);
	}
			
}

static void
pm_bopt_dump(const struct rte_pm_build_opt *bopt, const char *prefix)
{
	printf("%ssearch_type: %" PRIu32 "(%s)\n",
		prefix, bopt->search_type,
		rte_pm_search_names[bopt->search_type]);
	printf("%scase_sense: %" PRIu32 "\n",
		prefix, (uint32_t)bopt->case_sense);
	printf("%sout_of_order: %" PRIu32 "\n",
		prefix, (uint32_t)bopt->out_of_order);
}
	
/*
 * Dump PM context to the stdout.
 */
void
rte_pm_dump(const struct rte_pm_ctx *pmx)
{
	if (!pmx)
		return;
	printf("pm context <%s>@%p\n", pmx->name, pmx);
	printf("  max_pattern_num=%"PRIu32"\n", pmx->max_pattern_num);
	printf("  max_pattern_len=%"PRIu32"\n", pmx->max_pattern_len);
	printf("  pattern_num=%"PRIu32"\n", pmx->pattern_num);
	printf("  pattern_len=%"PRIu32"\n", pmx->pattern_len);
	printf("  pattern_buf=%p\n", pmx->pattern_buf);
	printf("  patterns@%p:\n", pmx->patterns);
	pm_patterns_dump(pmx->patterns, pmx->pattern_num, "    ");
	pm_bopt_dump(&pmx->bopt, "  ");
}

/*
 * Dump all PM contexts to the stdout.
 */
void
rte_pm_list_dump(void)
{
	struct rte_pm_ctx *pmx;
	struct rte_pmx_list *pmx_list;

	/* check that we have an initialised tail queue */
	if ((pmx_list = RTE_TAILQ_LOOKUP_BY_IDX(RTE_TAILQ_PM,
			rte_pmx_list)) == NULL) {
		rte_errno = E_RTE_NO_TAILQ;
		return;
	}

	rte_rwlock_read_lock(RTE_EAL_TAILQ_RWLOCK);
	TAILQ_FOREACH(pmx, pmx_list, next) {
		rte_pm_dump(pmx);
	}
	rte_rwlock_read_unlock(RTE_EAL_TAILQ_RWLOCK);
}
