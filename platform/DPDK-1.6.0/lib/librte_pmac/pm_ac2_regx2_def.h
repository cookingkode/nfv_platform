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

#ifndef __func_search__
#error "__func_search__ not defined"
#endif

#ifndef __func_calc_mask__
#error "__func_calc_mask__ not defined"
#endif

#ifndef __func_one_byte_pattern__
#error "__func_one_byte_pattern__ not defined"
#endif

static inline uint32_t
__func_search__(const struct rte_pm_ctx *pmx, struct rte_ac2_state *st)
{
	const uint8_t *in;
	uint32_t pos, next_pos, len, len_align, case_sen;
	uint32_t mask, index_mask, displace;
	sse_t line, line1, line2;

	case_sen = pmx->bopt.case_sense;

	len = st->in_len;
	in = st->in_start;
	len_align = RTE_ALIGN(len, SSE_SIZE);

	pos = st->pos;
	index_mask = st->index_mask;

	if (pos == 0)
		line1 = MM_CVT(0);
	else
		line1 = ac_load_caseless(in + pos - SSE_SIZE, case_sen);

	line2 = ac_load_caseless(in + pos, case_sen);

	for (mask = 0; pos < len_align; mask = 0) {

		line = line1;
		line1 = line2;
		line2 = ac_load_caseless(in + pos + SSE_SIZE, case_sen);

		mask = __func_calc_mask__(pmx, line, line1) & index_mask;
		index_mask = -1;

		/* No possible matches found so far. */
		if (likely (mask == 0)) {
			pos += SSE_SIZE;
			continue;

		/* if that's the last and not full SSE line */
		} else if (pos + SSE_SIZE > len) {
			index_mask &= LEN2MASK(len - pos + HASH_RESIDUAL_REG);
			break;
		}

		/* Possible matches found, process them. */
		ac_process_bloom_hits(pmx, st, in + pos - HASH_RESIDUAL_REG,
			mask, in + len);

		/* Adjust input pointer position and index_mask. */
		if (st->bloom_mask == 0) {
			displace = st->in[0] - st->in_start +
				HASH_RESIDUAL_REG;
			next_pos = displace & ~SSE_MASK;

			if (next_pos == pos) {
				pos += SSE_SIZE;
			} else {
				index_mask &= -1 << (displace & SSE_MASK);
				if (next_pos >= pos + 2 * SSE_SIZE) {
					if (next_pos >= pos + 3 * SSE_SIZE)
						line1 = ac_load_caseless(in +
							next_pos - SSE_SIZE,
							case_sen);
					else
						line1 = line2;
					line2 = ac_load_caseless(in + next_pos,
						case_sen);
				}
				pos = next_pos;
			}
		}

		/* we fill all available match slots. */
		if (st->match_num >= st->res_num) {
			st->pos = pos;
			st->index_mask = index_mask;
			return (st->match_num);
		}
	}

	/*
	 * Process remaining possbile matches.
	 * If 1 byte patterns are present, then
	 * always treat last byte as a possible match.
	 * Unless we have a valid index, which means that we already consumed
	 * whole input buffer, and are in the middle of trie traversal.
	 */

#if (__func_one_byte_pattern__ != 0)
	if (mask == 0 && pos == len && len != 0)
		mask = LEN2MASK(HASH_RESIDUAL_REG);
#endif /* __func_one_byte_pattern__ != 0 */

	mask &= index_mask;

	if (mask != 0 && st->index[0] <= RTE_AC2_EMPTY_INDEX) {

		displace = ac_process_bloom_hits(pmx, st,
			in + pos - HASH_RESIDUAL_REG, mask, in + len);

		index_mask = mask & ~LEN2MASK(displace);
	} else {
		/* reset index_mask. */
		index_mask = 0;
	}

	st->pos = pos;
	st->index_mask = index_mask;

	return (st->match_num);
}

#undef __func_search__
#undef __func_calc_mask__
#undef __func_one_byte_pattern__

