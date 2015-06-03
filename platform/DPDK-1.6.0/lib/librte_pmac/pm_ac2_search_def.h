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

#ifndef	__func_search_bulk__
#error "__func_search_bulk__ not defined"
#endif

#ifndef	__func_search_first_seg__
#error "__func_search_first_seg__ not defined"
#endif

#ifndef	__func_search_next_seg__
#error "__func_search_next_seg__ not defined"
#endif

#ifndef	__func_search_cur_seg__
#error "__func_search_cur_seg__ not defined"
#endif

#ifndef	__func_setup_bulk_state__
#error "__func_setup_bulk_state__ not defined"
#endif

#ifndef	__func_reset_seg_state__
#error "__func_reset_seg_state__ not defined"
#endif

#ifndef	__func_setup_seg_state__
#error "__func_setup_seg_state__ not defined"
#endif

#ifndef	__func_search__	
#error "__func_search__ not defined"
#endif


/*
 * Starts processing of the new segment in the chain.
 * Stops when end of segment is reached of up to resnum matches is found.
 * Returns number of matches found.
 */
static int
__func_search_next_seg__(const struct rte_pm_ctx *pmx,
	const struct rte_pm_inbuf *buf, struct rte_pm_match res[],
	uint32_t resnum, struct rte_pm_state *st)
{
	struct rte_ac2_state *state;
	uint32_t fill, rc;

	state = &st->ac2;
	fill = 0;

	/* set up state for new segment. */
	__func_setup_seg_state__(buf->buf, buf->len, DIM(state->res), state);

	/* check do we have a match that spawns from previous segment. */
	rc = ac_start_seg(pmx, state);

	do {
		while (rc != 0) {

			/* extract previously found results. */
			fill += ac_extract_matches(pmx, state, res + fill,
				resnum - fill);

			if (fill == resnum)
				return (fill);

			state->match_idx = 0;
			state->match_num = 0;

			/* check if any matches left on current SSE line. */
			rc = ac_step_seg(pmx, state, 0);
		}

	/* continue with the search to the next line. */
	} while ((rc = __func_search__(pmx, state)) != 0);

	/* we have reached end of that segment. */
	if (fill != resnum)
		state->last_byte = state->in_start[state->in_len - 1];

	return (fill);
}

/*
 * Starts processing of the first segment in the chain.
 * Stops when end of segment is reached of up to resnum matches is found.
 * Returns number of matches found.
 */
static int
__func_search_first_seg__(const struct rte_pm_ctx *pmx,
	const struct rte_pm_inbuf *buf, struct rte_pm_match res[],
	uint32_t resnum, struct rte_pm_state *st)
{
	/* set up new state for first segment. */
	__func_reset_seg_state__(&st->ac2);

	/* process  first segment. */
	return (__func_search_next_seg__(pmx, buf, res, resnum, st));
}

/*
 * Continues processing of the new segment in the chain.
 * Stops when end of segment is reached of up to resnum matches is found.
 * Returns number of matches found.
 */
static int
__func_search_cur_seg__(const struct rte_pm_ctx *pmx, struct rte_pm_match res[],
	uint32_t resnum, struct rte_pm_state *st)
{
	uint32_t fill;
	struct rte_ac2_state *state;

        state = &st->ac2;
        fill = 0;

	do {
		do {
			/* Extract previously found matches. */
			fill += ac_extract_matches(pmx, state, res + fill,
				resnum - fill);

			/* No more space in result array. */
			if (fill == resnum)
				return (fill);

			/* reset our match states. */
			state->match_idx = 0;
			state->match_num = 0;

			/*
			 * check, do we have some unfinished trie traversal or
			 * possible unprocessed match.
			 */
		} while (ac_step_seg(pmx, state, 0) != 0);

	/* continue with usual search. */
	} while (__func_search__(pmx, state) != 0);

	/* we have reached end of that segment. */
	if (fill != resnum)
		state->last_byte = state->in_start[state->in_len - 1];

	return (fill);
}

/*
 * Process bulk of independent buffers.
 * Each buffer is processed till match is found or end of buffer is reached.
 * Stops when all buffers are processed.
 * Returns number of buffers where match was found.
 */
static int
__func_search_bulk__(const struct rte_pm_ctx *pmx,
	const struct rte_pm_inbuf buf[], struct rte_pm_match res[],
	uint32_t num)
{
	int ret;
	uint32_t i;
	struct rte_ac2_state st;

	ret = 0;
	for (i = 0; i != num; i++) {

		__func_setup_bulk_state__(buf[i].buf, buf[i].len, 1, &st);

		/* if matches found, extract the first one */
		if (__func_search__(pmx, &st) != 0) {
			ac_extract_match(pmx, st.res, res + i, 1);
			ret++;
		} else {
			res[i].fin = 0;
		}
	}
	return (ret);
}


#undef	__func_search_bulk__
#undef	__func_search_first_seg__
#undef	__func_search_next_seg__
#undef	__func_search_cur_seg__
#undef	__func_setup_bulk_state__
#undef	__func_reset_seg_state__
#undef	__func_setup_seg_state__
#undef	__func_search__	

