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
 *          be used on Intel� Architecture Processors.
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
#include "pm_ac2.h"

#define	AC_MB_OVERLAP(tr, n, ov)	\
	(((const struct rte_ac2_bloom_dfa32 *)((tr) + (n)))->level  <= (ov))

#define	AC_MB_UNROLL	(RTE_AC2_MB_SPLIT * RTE_AC2_MB_SPLIT)

#define	AC_DFA_INDEX(tr, n, v)	\
	(((const struct rte_ac2_bloom_dfa32 *)((tr) + (n)))->index[(v)])

#define	HASH_RESIDUAL_L1x4	3
#define	HASH_RESIDUAL_REG	1
#define	HASH_RESIDUAL_P1	HASH_RESIDUAL_L1x4

#define	BIT0_SET	0x11111111
#define	BIT1_SET	0x22222222
#define	BIT2_SET	0x44444444
#define	BIT3_SET	0x88888888

#define	EVEN_BITS_SET	(BIT0_SET | BIT2_SET)
#define	ODD_BITS_SET	(BIT1_SET | BIT3_SET)

#define	COPY_MB_SPLIT_ARRAY(d, s)	do { \
	(d)[0] = (s)[0];                     \
	(d)[1] = (s)[1];                     \
	(d)[2] = (s)[2];                     \
	(d)[3] = (s)[3];                     \
} while (0);


static const mmreg_t mm_ltA = {
	.ui8 = {
		'A' - 1, 'A' - 1, 'A' - 1, 'A' - 1, 
		'A' - 1, 'A' - 1, 'A' - 1, 'A' - 1, 
		'A' - 1, 'A' - 1, 'A' - 1, 'A' - 1, 
		'A' - 1, 'A' - 1, 'A' - 1, 'A' - 1,
	},
};

static const mmreg_t mm_gtZ = {
	.ui8 = {
		'Z' + 1, 'Z' + 1, 'Z' + 1, 'Z' + 1, 
		'Z' + 1, 'Z' + 1, 'Z' + 1, 'Z' + 1, 
		'Z' + 1, 'Z' + 1, 'Z' + 1, 'Z' + 1, 
		'Z' + 1, 'Z' + 1, 'Z' + 1, 'Z' + 1, 
	},
};

static const mmreg_t mm_0f = {
	.ui32 = {
		0x0f0f0f0f, 0x0f0f0f0f, 0x0f0f0f0f, 0x0f0f0f0f,
	},
};

static const mmreg_t mm_case = {
	.ui8 = {
		'a' ^ 'A', 'a' ^ 'A', 'a' ^ 'A', 'a' ^ 'A', 
		'a' ^ 'A', 'a' ^ 'A', 'a' ^ 'A', 'a' ^ 'A', 
		'a' ^ 'A', 'a' ^ 'A', 'a' ^ 'A', 'a' ^ 'A', 
		'a' ^ 'A', 'a' ^ 'A', 'a' ^ 'A', 'a' ^ 'A', 
	},
};

static const mmreg_t mm_bit_shuffle = {
	.ui8 = {
		1, 2, 4, 8, 0x10, 0x20, 0x40, 0x80,
		1, 2, 4, 8, 0x10, 0x20, 0x40, 0x80,
	},
};

static const mmreg_t mm_shuffle_mask = {
	.ui32 = {
		0x1f1f1f1f, 0x1f1f1f1f, 0x1f1f1f1f, 0x1f1f1f1f,
	},
};


static inline rte_ac2_index_t
ac_add_match(const struct rte_pm_ctx *pmx, uint32_t fin, uint32_t overlap,
	rte_ac2_index_t index, struct rte_ac2_match *res)
{
	struct rte_ac2_bloom_match32 *m;

	m = (struct rte_ac2_bloom_match32 *)(pmx->ac2.bloom_trie32 + index);
	res->match_index = m->match_index;
	res->fin = fin;
	res->overlap = overlap;

	return (m->index);
}

static inline uint32_t
ac_extract_match(const struct rte_pm_ctx *pmx, struct rte_ac2_match *match,
	struct rte_pm_match res[], uint32_t resnum)
{
	struct rte_ac2_match_data *md;
	uint32_t i, index;

	index = match->match_index;
	for (i = 0; i != resnum && index != 0;) {
		md = pmx->ac2.match_index + index;
		index = md->next;

		/* excludes duplicates for MB. */
		if (match->overlap < md->len) {
			res[i].fin = match->fin;
			res[i].len = md->len;
			res[i].userdata = md->userdata;
			i++;
		}
	}

	match->match_index = index;
	return (i);
}

/* Extract previously found matches. */
static inline uint32_t
ac_extract_matches(const struct rte_pm_ctx *pmx, struct rte_ac2_state *state,
	struct rte_pm_match res[], uint32_t resnum)
{
	uint32_t fill, k;

	fill = 0;
	for (k = state->match_idx;
			k != state->match_num && fill != resnum;
			k += (state->res[k].match_index == 0)) {

		fill += ac_extract_match(pmx, state->res + k, res + fill,
			resnum - fill);
	}

	state->match_idx = k;
	return (fill);
}

static inline sse_t
ac_load_caseless(const void *p, int case_sen)
{
	sse_t data, tmp;

	data = MM_LOADU(p);
	if (case_sen == RTE_PM_CASE_LESS) {
		tmp = MM_CMPGT8(data, mm_ltA.m);
		tmp = MM_AND(tmp, MM_CMPGT8(mm_gtZ.m, data));
		data = MM_XOR(data, MM_AND(tmp, mm_case.m));
	}

	return (data);
}

static inline const uint8_t *
ac_l3_type_match(const struct rte_pm_ctx *pmx, struct rte_ac2_state *st,
	const uint8_t *in, const uint8_t *end, uint32_t type, int match_offset)
{
	const struct rte_ac2_bloom_match32 *m;
	rte_ac2_index_t index;

	index = RTE_AC2_GET_INDEX(st->index[0]);
        m = (struct rte_ac2_bloom_match32 *)(pmx->ac2.bloom_trie32 + index);

	if (type == RTE_AC2_BNODE_TYPE_MATCH && index > RTE_AC2_EMPTY_INDEX) {

		if (match_offset)
			in += m->match_offset;

		if (in <= end) {
			st->index[0] = ac_add_match(pmx, in - st->in_start,
				0, index, st->res + st->match_num);
			st->match_num++;
		}
	}
	return (in);
} 

static inline const uint8_t *
ac_process_fanout(const rte_ac2_index_t *trie, struct rte_ac2_state *st,
	const uint8_t *in)
{
	const uint8_t *p;
	const struct rte_ac2_bloom_fanout32 *fan;
	rte_ac2_index_t index;
	uint32_t x;
	sse_t input, values;

	p = in;
	index = RTE_AC2_GET_INDEX(st->index[0]);
	fan = (const struct rte_ac2_bloom_fanout32 *)(trie + index);

	input = MM_SET1_8(*p);
	values = MM_LOAD((const sse_t *)fan->string);
	input = MM_CMPEQ8(input, values);
	x = MM_MOVEMASK8(input);

	/* set 'match not found' bit. */
	x |= 1 << RTE_AC2_BLOOM_FANOUT32_LEN;

	/* count length of match sequence, note only one match can occur. */
	x = rte_bsf32(x);                          

	p += (x == RTE_AC2_BLOOM_FANOUT32_LEN) ? 0 : 1;
	st->index[0] = fan->index[x >> 1];
	return (p);
}

static inline const uint8_t *
ac_process_seq_vect(const struct rte_ac2_bloom_seq32 *seq,
	struct rte_ac2_state *st, const uint8_t *in, const uint8_t *end)
{
	const uint8_t *p;
	uint32_t len, lenmask, seqnum, x;
	sse_t input, values;

	p = in;
	input = MM_LOADU((const sse_t *)p);

	if ((seqnum = st->sequence) != 0) {
		values = MM_CVT(seqnum);
		seqnum /= CHAR_BIT;
		input = MM_SLL64(input, values);
	}

	values = MM_LOAD((const sse_t *)seq->string);

	len = end - p + seqnum;
	lenmask = (len > sizeof (lenmask) * CHAR_BIT) ?
		UINT32_MAX : LEN2MASK(len);

	/*
	 * MM_SHUFFLE32(X, 0x44) puts 32bits in the following order:
	 *  {x0, x1, x2, x3} -> {x0, x1, x0, x1};
	 */
	input = MM_SHUFFLE32(input, 0x44);
	input = MM_CMPEQ8(input, values);
	x = MM_MOVEMASK8(input);
	x |= (1 << seqnum) - 1;
	x |= x >> RTE_AC2_BLOOM_SEQ32_NUM;
	x &= seq->mask & lenmask;

	/* count length of match sequence. */
	x = rte_bsf32(x + 1);

	/*
	 * if match is beyond end of data, then stay on the same trie's node,
	 * save state to resume sequence on next search.
	 */
	if (x == len && lenmask < seq->mask) {
		/* offset in bits from the start of the pattern. */
		st->sequence = len * CHAR_BIT;
		p = end;

	/* point to the new node. */
	} else {
		st->index[0] = seq->index[x];
		st->sequence = 0;
		p += x - seqnum;
	}

	return (p);
}

static inline const uint8_t *
ac_process_seq(const rte_ac2_index_t *trie, struct rte_ac2_state *st,
	const uint8_t *in, const uint8_t *end)
{
	const uint8_t *p;
	const struct rte_ac2_bloom_seq32 *seq;
	rte_ac2_index_t index;
	uint32_t seqnum, seqmask;

	p = in;
	index = RTE_AC2_GET_INDEX(st->index[0]);
	seq = (const struct rte_ac2_bloom_seq32 *)(trie + index);

	seqnum = st->sequence / CHAR_BIT;
	seqmask = LEN2MASK(seqnum);

	/* quick first byte of sequence check, node typically fails here. */
	if (seq->string[seqnum] != *p &&
			seq->string[seqnum + RTE_AC2_BLOOM_SEQ32_NUM] != *p) {

		st->index[0] = seq->index[seqnum];
		st->sequence = 0;
	} else if (seq->mask == seqmask) {
		st->index[0] = seq->index[seqnum + 1];
		st->sequence = 0;
		p++;

	/* need to do a proper sequence scan. */
	} else {
		p = ac_process_seq_vect(seq, st, p, end);

	}

	return (p);
}

/*
 * Traverse the trie until an empty transition is encountered. An empty
 * transition represent a transition back to the first 2 (n) levels which
 * are covered by the Bloom filter and have no associated node. There are
 *  four types of transitions.
 * DFA with 256 pointers to next nodes.
 * SEQ which represent a sequence of nodes with single forward transitions.
 * This is basically a character sequence or token.
 * FANOUT which represents a limited number of forward transitions.
 * MATCH which means a match has been found.
 * The SEQ node may run out of input data before processing its complete
 * sequence. When this happens, it saves the relevant info in the state
 * and on a subsequent search, the comparison is resumend via a call to
 * ac_continue_processing.
 */
static inline const uint8_t *
ac_process_bloom(const struct rte_pm_ctx *pmx, struct rte_ac2_state *st,
	const uint8_t *start, const uint8_t *end)
{
	const uint8_t *p;
	const rte_ac2_index_t *trie;
	uint32_t type;

	p = start;
	trie = pmx->ac2.bloom_trie32;

	while (st->index[0] > RTE_AC2_EMPTY_INDEX && p < end &&
			st->res_num > st->match_num) {

		type = RTE_AC2_GET_TYPE(st->index[0]);

		/* Walk through DFA nodes. */
		if (type == RTE_AC2_BNODE_TYPE_DFA) {
			do {
				st->index[0] = AC_DFA_INDEX(trie,
					st->index[0], *p);
				type = RTE_AC2_GET_TYPE(st->index[0]);
				p++;
			} while (type == RTE_AC2_BNODE_TYPE_DFA && p < end);

			/* No matches found. */
			if (st->index[0] <= RTE_AC2_EMPTY_INDEX)
				p--;

		/* Process Sequence node. */
		} else if (type == RTE_AC2_BNODE_TYPE_SEQ) {
			p = ac_process_seq(trie, st, p, end);

		/* Process Sequence node. */
		} else if (type == RTE_AC2_BNODE_TYPE_FANOUT) {
			p = ac_process_fanout(trie, st, p);
		}

		type = RTE_AC2_GET_TYPE(st->index[0]);

		/* Process the match, if found. */
		ac_l3_type_match(pmx, st, p, end, type, 0);
        }

	return (p);
}

/*
 * Process bloom filter hits. mask represents an array of bloom filter hits.
 * This routine does trie traversals for those hits and resets the mask
 * according tothe amount input processed during the trie traversal.
 */
static inline uint32_t
ac_process_bloom_hits(const struct rte_pm_ctx *pmx, struct rte_ac2_state *st,
	const uint8_t *start, uint32_t mask, const uint8_t *end)
{
	const uint8_t *p, *prev;
	uint32_t displace;
	uint16_t idx;
	int c;

	st->in[0] = start;
	p = start;
	displace = 0;

	while (mask != 0 && p < end && st->res_num > st->match_num) {

		prev = p;
		p += rte_bsf32(mask);

		idx = rte_le_to_cpu_16(*(const uint16_t *)p);

		/* Calculate level 3 index for this hit.*/
		st->index[0] = pmx->ac2.bloom_l3[idx];

		/*
		 * Skip repeating character(s) that false hit in the
		 * bloom filter
		 */

		if (st->index[0] == 0) {
			c = *p++;
			if (c == p[0])
				while (c == p[1] && p < end)
					p++;
		} else {
			p += sizeof (uint16_t);

			/* If this is a match from the 2 byte check,
			 * then do 'Add Match' and allow the routine to
			 * adjust p, if its a 1-byte pattern.
			 * This is only done when setting index from pBloomL3
			 * array, otherwise no adjustment to p is needed.
			 * If possible match from L3 spawns beyond the end of 
			 * the buffer, then ignore it and reset the index.
			 */
			if ((p = ac_l3_type_match(pmx, st, p, end,
				RTE_AC2_GET_TYPE(st->index[0]), 1)) > end)
				st->index[0] = 0;
		}


		/* Walk through trie nodes to find the real match, if any. */
		p = ac_process_bloom(pmx, st, p, end);

		/* How much we need to go backwards on failure path. */
		if (st->index[0] <= RTE_AC2_EMPTY_INDEX) {
			p -= st->index[0] >> RTE_AC2_BNODE_TYPE_SHIFT;
		}

		/* Calculate the displacement into mask where trie traversal
		 * stopped and reset mask bits accordingly.
		 */
		displace = p - prev;
        
		mask = (displace >= sizeof (mask) * CHAR_BIT) ?
			0 : (mask >> displace);
	}

	st->in[0] = p;
	st->bloom_mask = mask;

	return (p - start);
}

/*
 * Calculates Bloom filter for one SSE line (8 x 16 bits patterns).
 * Bloom filter supposed to be fit in one SSE register.
 * Non-zero return value of each 16 bits means a possbile match.
 */
static inline sse_t
ac_bloom_filter_r128x2(sse_t bloom, sse_t index)
{
	sse_t input_bits;

	/* Calculate hash values for the input patterns. */
	index = MM_ADD16(index, mm_prime_byte.m);
	index = MM_MADD8(index, mm_prime_byte.m);

	/* SHUFFLE8(mm_bit_shuffle, X) = {1 << (X[0-15] & 7)}; */
	input_bits = MM_SHUFFLE8(mm_bit_shuffle.m,
		MM_AND(index, mm_shuffle_mask.m));

	/* Get stored Bloom fileter values for the input patterns. */
	index = MM_SHUFFLE8(bloom, MM_AND(MM_SRL16(index, AC2_BLOOM_SHIFT),
					mm_shuffle_mask.m));

	/* Check that computed hash values are present in the Bloom filter. */
	index = MM_ANDNOT(index, input_bits);
	return (index);
}

/*
 * Calculates Bloom filter for one SSE line (8 x 16 bits patterns).
 * Bloom filter supposed to be fit into 2 SSE registers.
 * Non-zero return value of each 16 bits means a possbile match.
 */
static inline sse_t
ac_bloom_filter_r256x2(sse_t bloom_low, sse_t bloom_high, sse_t index)
{
	sse_t filter, t1, t2;

	/* Calculate hash values for the input patterns. */
	index = MM_ADD16(index, mm_prime_byte.m);
	index = MM_MADD8(index, mm_prime_byte.m);

	/* get byte's postions in the bloom array for the given input. */
	filter = MM_AND(MM_SRL16(index, AC2_BLOOM_SHIFT), mm_shuffle_mask.m);

	/* Get stored Bloom fileter values for the input patterns. */
	t1 = MM_SHUFFLE8(bloom_low, filter);
	t2 = MM_SHUFFLE8(bloom_high, filter);
	filter = MM_BLENDV8(t1, t2, MM_CMPGT8(filter, mm_0f.m));

	/* SHUFFLE8(mm_bit_shuffle, X) = {1 << (X[0-15] & 7)}; */
	index = MM_SHUFFLE8(mm_bit_shuffle.m, MM_AND(index, mm_shuffle_mask.m));
		
	/* Check that computed hash values are present in the Bloom filter. */
	filter = MM_ANDNOT(filter, index);
	return (filter);
}

/*
 * Calculates One Byte filter for one SSE line (16 x 8 bits patterns)
 * Non-zero return value of each 8 bits means a match.
 */
static inline sse_t
ac_one_byte_filter(sse_t one_byte_low, sse_t one_byte_high, sse_t index)
{
	sse_t filter, t1, t2;

	/* get byte's postions in one_byte_low(high) for given input. */
	filter = MM_AND(MM_SRL16(index, AC2_BLOOM_SHIFT), mm_shuffle_mask.m);

	/* get one_byte_low(high) values for selected positions. */
	t1 = MM_SHUFFLE8(one_byte_low, filter);
	t2 = MM_SHUFFLE8(one_byte_high, filter);
	filter = MM_BLENDV8(t1, t2, MM_CMPGT8(filter, mm_0f.m));

	/* calculate one_byte_low(high) values for the given input. */
	index = MM_SHUFFLE8(mm_bit_shuffle.m, MM_AND(index, mm_shuffle_mask.m));

	/* compare calculated and stored values and calculate mask. */
        filter = MM_ANDNOT(filter, index);
	return (filter);
}

/*
 * For all calc_mask_r*x2[_1b] routinies:
 * index0 = {
 * 	<N-1,N>, <N+1,N+2>, <N+3,N+4>, <N+5,N+6>,
 * 	<N+7,N+8>, <N+9,N+10>, <N+11,N+12>, <N+13,N+14>
 * };
 * e.g: {P0, P2, P4, P6, P8, P10, P12, P14};
 *
 * index1 = {
 * 	<N,N+1>, <N+2,N+3>, <N+4,N+5>, <N+6,N+7>,
 * 	<N+8,N+9>, <N+10,N+11>, <N+12,N+13>, <N+14,N+15>
 * };
 * e.g: {P1, P3, P5, P7, P9, P11, P13, P15};
 */
static inline uint32_t
ac_calc_mask_r128x2(const struct rte_pm_ctx *pmx, sse_t line, sse_t line1)
{
	uint32_t mask;
	sse_t bloom, index0, index1;
	
	bloom = MM_LOAD((const sse_t *)pmx->ac2.bloom);

	index0 = MM_ALIGNR8(line1, line, SSE_SIZE - HASH_RESIDUAL_REG);
	index1 = line1;

	index0 = ac_bloom_filter_r128x2(bloom, index0);
	index1 = ac_bloom_filter_r128x2(bloom, index1);

	mask = MM_MOVEMASK8(MM_CMPEQ16(index0, MM_XOR(index0, index0))) &
		EVEN_BITS_SET;
	mask |= MM_MOVEMASK8(MM_CMPEQ16(index1, MM_XOR(index0, index0))) &
		ODD_BITS_SET;
	return (mask);
}

static inline uint32_t
ac_calc_mask_r256x2(const struct rte_pm_ctx *pmx, sse_t line, sse_t line1)
{
	uint32_t mask;
	sse_t bloom_low, bloom_high;
	sse_t index0, index1;
	
	bloom_low = MM_LOAD((const sse_t *)pmx->ac2.bloom);
	bloom_high = MM_LOAD(&((const sse_t *)pmx->ac2.bloom)[1]);

	index0 = MM_ALIGNR8(line1, line, SSE_SIZE - HASH_RESIDUAL_REG);
	index1 = line1;

	index0 = ac_bloom_filter_r256x2(bloom_low, bloom_high, index0);
	index1 = ac_bloom_filter_r256x2(bloom_low, bloom_high, index1);

	mask = MM_MOVEMASK8(MM_CMPEQ16(index0, MM_XOR(index0, index0))) &
		EVEN_BITS_SET;
	mask |= MM_MOVEMASK8(MM_CMPEQ16(index1, MM_XOR(index0, index0))) &
		ODD_BITS_SET;
	return (mask);
}

static inline uint32_t
ac_calc_mask_r128x2_1b(const struct rte_pm_ctx *pmx, sse_t line, sse_t line1)
{
	uint32_t mask;
	sse_t bloom, index0, index1, one_byte;

	bloom = MM_LOAD((const sse_t *)pmx->ac2.bloom);
	
	index0 = MM_ALIGNR8(line1, line, SSE_SIZE - HASH_RESIDUAL_REG);
	index1 = line1;

	/* check for one byte patterns match. */
	one_byte = ac_one_byte_filter(pmx->ac2.one_byte_low.m,
		pmx->ac2.one_byte_high.m, index0);

	index0 = ac_bloom_filter_r128x2(bloom, index0);
	index1 = ac_bloom_filter_r128x2(bloom, index1);

	mask = MM_MOVEMASK8(MM_CMPEQ16(index0, MM_XOR(index0, index0))) &
		EVEN_BITS_SET;
	mask |= MM_MOVEMASK8(MM_CMPEQ16(index1, MM_XOR(index0, index0))) &
		ODD_BITS_SET;
	mask |= MM_MOVEMASK8(MM_CMPEQ8(one_byte, MM_XOR(index0, index0)));
	return (mask);
}

static inline uint32_t
ac_calc_mask_r256x2_1b(const struct rte_pm_ctx *pmx, sse_t line, sse_t line1)
{
	uint32_t mask;
	sse_t bloom_low, bloom_high;
	sse_t index0, index1, one_byte;
	
	bloom_low = MM_LOAD((const sse_t *)pmx->ac2.bloom);
	bloom_high = MM_LOAD(&((const sse_t *)pmx->ac2.bloom)[1]);

	index0 = MM_ALIGNR8(line1, line, SSE_SIZE - HASH_RESIDUAL_REG);
	index1 = line1;

	/* check for one byte patterns match. */
	one_byte = ac_one_byte_filter(pmx->ac2.one_byte_low.m,
		pmx->ac2.one_byte_high.m, index0);

	index0 = ac_bloom_filter_r256x2(bloom_low, bloom_high, index0);
	index1 = ac_bloom_filter_r256x2(bloom_low, bloom_high, index1);

	mask = MM_MOVEMASK8(MM_CMPEQ16(index0, MM_XOR(index0, index0))) &
		EVEN_BITS_SET;
	mask |= MM_MOVEMASK8(MM_CMPEQ16(index1, MM_XOR(index0, index0))) &
		ODD_BITS_SET;
	mask |= MM_MOVEMASK8(MM_CMPEQ8(one_byte, MM_XOR(index0, index0)));
	return (mask);
}

static inline uint32_t
ac_calc_mask_p1(const struct rte_pm_ctx *pmx, sse_t line, sse_t line1)
{
	uint32_t mask;
	sse_t pattern, sse_mask;
	sse_t index0, index1, index2, index3;

	sse_mask = pmx->ac2.sse_mask;
	pattern = pmx->ac2.first_pattern;

	index0 = MM_ALIGNR8(line1, line, SSE_SIZE - 3);
	index1 = MM_ALIGNR8(line1, line, SSE_SIZE - 2);
	index2 = MM_ALIGNR8(line1, line, SSE_SIZE - 1);

	index3 = MM_AND(sse_mask, line1);
	index0 = MM_AND(sse_mask, index0);
	index1 = MM_AND(sse_mask, index1);
	index2 = MM_AND(sse_mask, index2);

	mask = MM_MOVEMASK8(MM_CMPEQ32(index3, pattern)) & BIT3_SET;
	mask |= MM_MOVEMASK8(MM_CMPEQ32(index0, pattern)) & BIT0_SET;
	mask |= MM_MOVEMASK8(MM_CMPEQ32(index1, pattern)) & BIT1_SET;
	mask |= MM_MOVEMASK8(MM_CMPEQ32(index2, pattern)) & BIT2_SET;

	return (mask);
}

/*
 *  MM_SHUFFLE32(X, 0x39): puts 32 bits in the following order:
 *  {x0, x1, x2, x3} -> {x1, x2, x3, x0}; - after first one
 *  {x1, x2, x3, x0} -> {x2, x3, x0, x1}; - after second one
 *  {x2, x3, x0, x1} -> {x3, x0, x1, x2}; - after third one
 *  So GATHER3 is just an unroll of the loop:
 *  for (i = 1; i != 4; i++) {
 *  	x = index0.v32[i];
 *  	y = index1.v32[i];
 *  	z = bloom[x] | bloom[y] << 8;
 *  	((uint16_t *)&base_index1)[n + (i - 1) * 2] = z;
 *  }
 */

#define GATHER3(index0, index1, base_index1, bloom, n)	do {  \
	(index0) = MM_SHUFFLE32((index0), 0x39);              \
	(index1) = MM_SHUFFLE32((index1), 0x39);              \
	(c) = MM_CVT32((index0));                             \
	(d) = MM_CVT32((index1));                             \
	(base_index1) = MM_INSERT16((base_index1),            \
		((bloom)[(d)] << 8) | (bloom)[(c)], (n));     \
	(index0) = MM_SHUFFLE32((index0), 0x39);              \
	(index1) = MM_SHUFFLE32((index1), 0x39);              \
	(a) = MM_CVT32((index0));                             \
	(b) = MM_CVT32((index1));                             \
	(base_index1) = MM_INSERT16((base_index1),            \
		((bloom)[(b)] << 8) | (bloom)[(a)], (n) + 2); \
	(index0) = MM_SHUFFLE32((index0), 0x39);              \
	(index1) = MM_SHUFFLE32((index1), 0x39);              \
	(c) = MM_CVT32((index0));                             \
	(d) = MM_CVT32((index1));                             \
	(base_index1) = MM_INSERT16((base_index1),            \
		((bloom)[(d)] << 8) | (bloom)[(c)], (n) + 4); \
} while (0)

/*	 
 * GATHER_EVEN - gathers bloom[hash(P)] and puts them at appropriate
 * place into base_index, e.g:
 * base_index =  {
 * 	bloom[hash(P0)],  pBloom[hash(P1)],  0, 0,
 *	bloom[hash(P4)],  pBloom[hash(P5)],  0, 0,
 *	bloom[hash(P8)],  pBloom[hash(P9)],  0, 0,
 *	bloom[hash(P12)], pBloom[hash(P13)], 0, 0,
 * };
 */
#define	GATHER_EVEN(index0, index1, base_index1, bloom)	do {        \
	uint32_t a, b, c, d;                                        \
	a = MM_CVT32(index0);                                       \
	b = MM_CVT32(index1);                                       \
	(base_index1) = MM_CVT(((bloom)[(b)] << 8) | (bloom)[(a)]); \
	GATHER3(index0, index1, base_index1, bloom, 2);              \
} while (0)

#define	GATHER_ODD(index0, index1, base_index1, bloom)	do {       \
	uint32_t a,b,c,d;                                          \
	a = MM_CVT32(index0);                                      \
	b = MM_CVT32(index1);                                      \
	(base_index1) = MM_INSERT16((base_index1),                 \
		((bloom)[(b)] << 8) | (bloom)[(a)], 1);            \
	GATHER3(index0, index1, base_index1, bloom, 3);             \
} while (0)

/* Bloom filter function for the BYTE INDEX into the filter is
 * Hash(Value & RelevantBitsMask) & FilterSizeMask
 *
 * Mask 4-byte Value to relevant number of bits (18-32 bits)
 * The mask value is determined at generation phase as a function of
 * the number of 1, 2, and 3 byte patterns in the set.
 * Simple universal hash function is 
 *  w0 * p0 + w1 * p1
 * where p0 and p1 are primes and 
 * w0 and w1 are the first and second
 * words of the 4 byte value
 * Mask hash value to filter size
*/
#define	CALC_HASH(index0, index1, mask, prime_word, input_mask)		do { \
	(index0) = MM_AND((index0), (mask));                                 \
	(index1) = MM_AND((index1), (mask));                                 \
	(index0) = HASH32((index0), (prime_word));                           \
	(index1) = HASH32((index1), (prime_word));                           \
	(index0) = MM_AND((index0), (input_mask));                           \
	(index1) = MM_AND((index1), (input_mask));                           \
} while (0)


static inline uint32_t
ac_calc_mask_l1x4(const struct rte_pm_ctx *pmx, sse_t line, sse_t line1)
{
	const uint8_t *bloom;
	uint32_t mask;
	sse_t sse_mask, input_mask;
	sse_t index0, index1, input_bits, base_index;

	bloom = pmx->ac2.bloom; 
	sse_mask = pmx->ac2.sse_mask;
	input_mask = pmx->ac2.input_mask;

	/*
	 * index0 contains first 4 possible 4 patterns,
	 * e.g starting from input position N:
	 * index0 = {
	 * 	<N-3,N-2,N-1,N>, <N+1,N+2,N+3,N+4>,
	 *	<N+5,N+6,N+7,N+8>, <N+9,N+10,N+11,N+12>
	 * };
	 * e.g: {P0, P4, P8, P12}
	 * index1 = {
	 *	{<N-2,N-1,N,N+1>, <N+2,N+3,N+4,N+5>,
	 *	<N+6,N+7,N+8,N+9>, <N+10,N+11,N+12,N+13>
	 * };
	 * e.g: {P1, P5, P9, P13}
	 */
	index0 = MM_ALIGNR8(line1, line, SSE_SIZE - 3);
	index1 = MM_ALIGNR8(line1, line, SSE_SIZE - 2);

	/*
	 * Calculate the bloom mask based on which bytes are the
	 * first byte of a pattern. Note: the bloom filter uses the
	 * low 3 bits of each input byte (unchanged) to determine
	 * which bits are set in the filter, however, the byte index
	 * into the filter is calculated via a hash function.	
	 */

	/* SHUFFLE8(mm_bit_shuffle, X) = {1 << (X[0-15] & 7)}; */
	input_bits = MM_SHUFFLE8(mm_bit_shuffle.m,
		MM_AND(index0, mm_shuffle_mask.m));

	/*
	 * Bits for Bloom filter function(s) = (1 << (input[N] & 3)) where
	 * multiple functions simply use a different bytes.
	 * For 2 functions, N = 0, 1, for 3 functions N = 0,1,2, ...
	 * NOTE: 2 input bytes are always available, however, 3 may not be.
	 */
	input_bits = MM_OR(input_bits,
			MM_SHUFFLE8(mm_bit_shuffle.m,
				MM_AND(index1, mm_shuffle_mask.m)));

	/*
	 * Calculate Bloom Filter value for each 4 bytes pattern from
	 * index0 & index1.
	 * CALC_HASH - hashes 4 bytes pattern into 2 byte value
	 * (index into Bloom Filter array).
	 * Bloom filter value for pattern P: bloom[hash(P)]
	 * So for each 4 byte pattern P  we select 1 byte value from bloom[].
	 */
	CALC_HASH(index0, index1, sse_mask, mm_prime_word.m, input_mask);

	/*	 
	 * base_index =  {
	 * 	bloom[hash(P0)],  pBloom[hash(P1)],  0, 0,
	 *	bloom[hash(P4)],  pBloom[hash(P5)],  0, 0,
	 *	bloom[hash(P8)],  pBloom[hash(P9)],  0, 0,
	 *	bloom[hash(P12)], pBloom[hash(P13)], 0, 0,
	 * };
	 */
	GATHER_EVEN(index0, index1, base_index, bloom);

	/*
	 * index0 = {
	 * 	<N-1,N,N+1,N+2>, <N+3,N+4,N+5,N+6>,
	 * 	<N+7,N+8,N+9,N+10>, <N+11,N+12,N+13,N+14>
	 * };
	 * e.g: {P2, P6, P10, P14}
	 * index1 = {
	 * 	<N,N+1,N+2,N+3>, <N+4,N+5,N+6,N+7>,
	 * 	<N+8,N+9,N+10,N+11>, <N+12,N+13,N+14,N+15>i
	 * };
	 * e.g: {P3, P7, P11, P15}
	 */
	index0 = MM_ALIGNR8(line1, line, SSE_SIZE - 1);
	index1 = line1;

	/*
	 * nBaseIndex = { bloom[hash(P0)], ..., bloom[hash(P15)]};
	 */
	CALC_HASH(index0, index1, sse_mask, mm_prime_word.m, input_mask);
	GATHER_ODD(index0, index1, base_index, bloom);

	/*
 	 * Calculate hits in the bloom filter such that 1 represents a hit.
 	 * For each byte, a hit means that all bits set in input_bits are
	 * also set in base_index.
 	 */
	index0 = MM_XOR(line, line);
	mask = MM_MOVEMASK8(MM_CMPEQ8(MM_ANDNOT(base_index, input_bits),
					index0));

	return (mask);
}

/*
 * RTE_PM_SEARCH_AC2_R128 – specific search method, siutable when number of
 * patterns in the pattern set is less or equal 6.
 * Vector implementation. Bloom filter fits into one SSE register.
 * At each iteration it consumes up to (16+1) input bytes and simultaneously
 * process 16 2-byte sequences to find a possible match in Bloom Filter.
 * If possible match is found it traverses the trie to either find a real match
 * or drop it as false positive.
 * Bloom Filter Mask for R128 is 16 bits bits long.
 * When 1 byte patterns are present in the pattern set it acually uses two
 * different filters - normal one and additional one for 1 byte patterns.
 */

/*
 * AC2_R128
 */

#define	__func_search__			ac_search_r128x2
#define	__func_calc_mask__		ac_calc_mask_r128x2
#define	__func_one_byte_pattern__	0

#include "pm_ac2_regx2_def.h"

/*
 * AC2_R128_1B
 */

#define	__func_search__			ac_search_r128x2_1b
#define	__func_calc_mask__		ac_calc_mask_r128x2_1b
#define	__func_one_byte_pattern__	1

#include "pm_ac2_regx2_def.h"

/*
 * RTE_PM_SEARCH_AC2_R256 – specific search method, siutable when number of
 * patterns in the pattern set is less or equal 13.
 * Vector implementation. Bloom filter fits into two SSE registers.
 * At each iteration it consumes up to (16+1) input bytes and simultaneously
 * process 16 2-byte sequences to find a possible match in Bloom Filter.
 * If possible match is found it traverses the trie to either find a real match
 * or drop it as false positive.
 * Bloom Filter Mask for R256 is 16 bits bits long.
 * When 1 byte patterns are present in the pattern set it acually uses two
 * different filters - normal one and additional one for 1 byte patterns.
 */

/*
 * AC2_R256
 */

#define	__func_search__			ac_search_r256x2
#define	__func_calc_mask__		ac_calc_mask_r256x2
#define	__func_one_byte_pattern__	0

#include "pm_ac2_regx2_def.h"

/*
 * AC2_R256_1B
 */

#define	__func_search__			ac_search_r256x2_1b
#define	__func_calc_mask__		ac_calc_mask_r256x2_1b
#define	__func_one_byte_pattern__	1

#include "pm_ac2_regx2_def.h"

/*
 * RTE_PM_SEARCH_AC2_L1x4_MH (MULTIHASH) – specific search method,
 * siutable for pattern set with all patterns have length >= 4 bytes.
 * Vector implementation.
 * Always uses whole 32 bit mask.
 * It can work with Bloom Filter extended to first 5/7/11 bytes of pattern
 * Of course for that length of all our patterns has to be >= (5/7/11) bytes.
 * As in base L1x4 we operate on (16+3) bytes per iteration -
 * still devide it into 16 4B sequences and calucalte Bloom Filter for them.
 * Though now it consider that match is possible when Bloom Filter is set
 * for all 1/2/4/8 sequences.
 * So now for 5/7/11 byte sequence can span into 2 SSE lines.
 * That's why we need to use sort of 'merged  mask'.
 * We process mask got from previous iteration merged with mask for first
 * (2/4/8) bytes from current iteration.
 * MH5 (5 bytes pattern) example:
 * pattern = {1,2,3,4,5};
 * line0={<0 X 16>}; line1={<0 X 12>,1,2,3,4}; line2={5,<0 X 15>};
 * So at first iteration, after we processed line0[13-15]line1,
 * we got prev_mask=0x8000.
 * Then at next iteration, after processing line1[13-15]line2,
 * we got new_mask=0x0001.
 * So our 'meged mask' will be:
 * mask = prev_mask | (new_mask << SSE_SIZE); // == 0x18000
 * mask &= mask >> 1; // == 0x8000
 * Which means we have a possble match to process starting at line1[12].
 */
static inline uint32_t
ac_search_l1x4mh(const struct rte_pm_ctx *pmx, struct rte_ac2_state *st)
{
	const uint8_t *in;
	uint32_t pos, next_pos, len, case_sen;
	uint32_t mask, index_mask, new_mask, displace;
	sse_t line, line1, line2;

	case_sen = pmx->bopt.case_sense;

	len = st->in_len;
	in = st->in_start;

	pos = st->pos;
	index_mask = st->index_mask;

	if (pos == 0)
		line1 = MM_CVT(0);
	else
		line1 = ac_load_caseless(in + pos - SSE_SIZE, case_sen);

	line2 = ac_load_caseless(in + pos, case_sen);

	while (pos < len) {

		line = line1;
		line1 = line2;
		line2 = ac_load_caseless(in + pos + SSE_SIZE, case_sen);

		new_mask = ac_calc_mask_l1x4(pmx, line, line1) & index_mask;
		index_mask = -1;

		mask = st->prev_mask | (new_mask << SSE_SIZE);
		mask &= mask >> pmx->ac2.shift1;
		mask &= mask >> pmx->ac2.shift2;
		mask &= mask >> pmx->ac2.shift3;
		mask &= LEN2MASK(SSE_SIZE);
		st->prev_mask = new_mask;

		/* No possible matches found so far. */
		if (likely (mask == 0)) {
			pos += SSE_SIZE;
			continue;
		}

		/* Possible matches found, process them. */
		ac_process_bloom_hits(pmx, st,
			in + pos - SSE_SIZE - HASH_RESIDUAL_L1x4,
			mask, in + len);

		/* Adjust input pointer position and index_mask. */
		if (st->bloom_mask == 0) {
			displace = st->in[0] - st->in_start +
				HASH_RESIDUAL_L1x4;
			next_pos = displace & ~SSE_MASK;

			if (next_pos <= pos) {
				if (next_pos == pos)
					st->prev_mask &=
						-1 << (displace & SSE_MASK);
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
				st->prev_mask = 0;
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
	 * Treat all bits set in prev_mask and last 3 bytes in the buffer,
	 * as possbile matches.
	 * Unless we have a valid index, which means that we already consumed
	 * whole input buffer, and are in the middle of trie traversal.
	 */

	mask = st->prev_mask & LEN2MASK(len - pos + SSE_SIZE +
		HASH_RESIDUAL_L1x4);
	mask |= LEN2MASK(HASH_RESIDUAL_L1x4) << (len - pos + SSE_SIZE);
	mask &= index_mask;

	if (mask != 0 && st->index[0] <= RTE_AC2_EMPTY_INDEX) {

		displace = ac_process_bloom_hits(pmx, st,
			in + pos - SSE_SIZE - HASH_RESIDUAL_L1x4,
			mask, in + len);

		index_mask = mask & ~LEN2MASK(displace);
	}

	st->pos = pos;
	st->index_mask = index_mask;
	st->prev_mask = 0;

	return (st->match_num);
}

/*
 * RTE_PM_SEARCH_AC2_P1 – specific search method, siutable for pattern set
 * with only one pattern.
 * Vector implementation.
 * Similar to L1x4 it consumes up to (16+3) input bytes and simultaneously
 * process 16 4-byte sequences to find a possible match.
 * If possible match is found it traverses the trie to either find a real match
 * or drop it as false positive.
 * Bloom fiter for that case - first 4 bytes of the pattern.
 */

#define	__func_search__		ac_search_p1
#define	__func_calc_mask__	ac_calc_mask_p1

#include "pm_ac2_l1x4_def.h"

/*
 * RTE_PM_SEARCH_AC2_L1x4 – generic search method, siutable (though not always
 * optimal) for any pattern set.
 * Vector implementation.
 * At each iteration it consumes up to (16+3) input bytes and simultaneously
 * process 16 4-byte sequences to find a possible match in Bloom Filter.
 * If possible match is found it traverses the trie to either find a real match
 * or drop it as false positive.
 * Bloom Filter Mask for L1x4 can be 32/28/24/20/18 bits long.
 * It determines how many bits form first 4-byte sequence are actually used
 * to generate Bloom Filter value. 
 */

#define	__func_search__		ac_search_l1x4
#define	__func_calc_mask__	ac_calc_mask_l1x4

#include "pm_ac2_l1x4_def.h"

/*
 * Reset state at the beginning of new segment in the chain.
 * Suitable for all Bloom filter algorithms.
 */
static inline void
ac_set_bloom_state(const uint8_t *start, uint32_t len, uint32_t resnum,
	struct rte_ac2_state *st)
{
	st->in_start = start;
	st->in_len = len;

	st->res_num = RTE_MIN(DIM(st->res), resnum);
	st->match_num = 0;
	st->match_idx = 0;

	st->prev_mask = 0;
	st->index_mask = ~LEN2MASK(st->hash_residual);
	st->pos = 0;

	st->in[0] = start;
	st->segnum++;
}

static inline void
ac_reset_l1x4_state(struct rte_ac2_state *st)
{
	st->segnum = 0;
	st->sequence = 0;
	st->bloom_mask = 0;
	st->hash_residual = HASH_RESIDUAL_L1x4;
	st->index[0] = RTE_AC2_EMPTY_INDEX;
}

static inline void
ac_start_l1x4_first(const uint8_t *start, uint32_t len, uint32_t resnum,
	struct rte_ac2_state *st)
{
	ac_reset_l1x4_state(st);
	ac_set_bloom_state(start, len, resnum, st);
}

static inline void
ac_reset_regx2_state(struct rte_ac2_state *st)
{
	st->segnum = 0;
	st->sequence = 0;
	st->bloom_mask = 0;
	st->hash_residual = HASH_RESIDUAL_REG;
	st->index[0] = RTE_AC2_EMPTY_INDEX;
}

static inline void
ac_start_regx2_first(const uint8_t *start, uint32_t len, uint32_t resnum,
	struct rte_ac2_state *st)
{
	ac_reset_regx2_state(st);
	ac_set_bloom_state(start, len, resnum, st);
}

/*
 * Check if we have L3 escape spawns through multiple segments, e.g.
 * last byte of previous segment and first byte of current one.
 */
static inline void
ac_seg_last_byte(const struct rte_pm_ctx *pmx, struct rte_ac2_state *st,
	uint16_t first_byte)
{
	uint16_t idx;  

	if (st->segnum != 1 && st->index[0] <= RTE_AC2_EMPTY_INDEX) {
		idx = (uint16_t)(first_byte << CHAR_BIT | st->last_byte);
		if ((st->index[0] = pmx->ac2.bloom_l3[idx]) != 0)
			st->in[0] = st->in_start + 1;
	}
}

/*
 * Continue processing from previous segment in a chain.
 * Returns new input pointer.
 */
static inline const uint8_t *
ac_continue_process(const struct rte_pm_ctx *pmx, struct rte_ac2_state *st)
{
	const uint8_t *end, *start, *p;

	start = st->in[0];
	end = st->in_start + st->in_len;
	p = start;

	/* If we are in the middle of trie traversal. */
	if (st->index[0] > RTE_AC2_EMPTY_INDEX) {

		p = ac_process_bloom(pmx, st, start, end);

		/* How much we need to go backwards on failure path. */
		if (st->index[0] <= RTE_AC2_EMPTY_INDEX)
			p -= st->index[0] >> RTE_AC2_BNODE_TYPE_SHIFT;
	}
	return (p);
}

/*
 * Before starting to calculate bloom mask for next SSE line,
 * we need to check is there any unfinished trie traversal and/or
 * unpreocessed bloom mask from previous search.
 * Previous search can left some possible matches unprocessed by one
 * of two reasons:
 *  - End of previous segment was reached.
 *  - State's results array became full.
 */
static inline uint32_t
ac_step_seg(const struct rte_pm_ctx *pmx, struct rte_ac2_state *st,
	uint32_t displace)
{
	const uint8_t *end, *start, *p;
	uint32_t  mask, pos;

	start = st->in[0];
	end = st->in_start + st->in_len;
	mask = st->bloom_mask;

	p = ac_continue_process(pmx, st);

	/* calculate displace */
	if (p > start) {
		displace += p - start;
		mask = (displace >= sizeof (mask) * CHAR_BIT) ?
			0 : (mask >> displace);
	} else if (p < start) {
		displace = start - p;
		if (mask != 0)
			mask = mask << displace | LEN2MASK(displace);
	}

	/* update bloom_mask */
	st->bloom_mask = mask;

	/* update input pointer position. */
	st->in[0] = p;
	start = p;

	/* If we still have unprocessed possible hits. */
	if (mask != 0) {
		displace += ac_process_bloom_hits(pmx, st, start, mask, end);
	}
	
	/* Adjust input pointer position and index_mask. */
	if (displace != 0 && st->bloom_mask == 0) {

		displace = st->in[0] - st->in_start + st->hash_residual;
		pos = displace & ~SSE_MASK;

		/* if we adjusted our bloom mask by ac_process_bloom_hits. */
		if (pos <= st->pos && mask != 0) {

			/* adjust prev_mask. */
			if (pos == st->pos)
				st->prev_mask &= -1 << (displace & SSE_MASK);

			/*
			 * if we reached the end of the buffer, then
			 * reset index_mask, to mark that seach is finished.
			 * Otherwise adjust input pointer position and
			 * index_mask.
			 */
			if (st->pos < st->in_len) {
				st->pos += SSE_SIZE;
				st->index_mask = -1;
			} else {
				st->index_mask = 0;
			}

		} else {
			st->pos = pos;
			st->index_mask = -1 << (displace & SSE_MASK);
			st->prev_mask = 0;
		}
	}

	return (st->match_num);
}

/*
 * Start processing of new segment in the chain.
 */
static inline uint32_t
ac_start_seg(const struct rte_pm_ctx *pmx, struct rte_ac2_state *st)
{
	const uint8_t *p;
	uint32_t displace, prev_index;

	displace = 0;
	prev_index = st->index[0];

	p = ac_continue_process(pmx, st);
	st->in[0] = p;

	/*
	 * If there is no match from previous segment, that spawns futher
	 * then first byte of then new segment, then check if we have L3
	 * escape from last byte of previous segment, first byte of new 
	 * segment.
	 */
	if (prev_index <= RTE_AC2_EMPTY_INDEX ||
			(st->index[0] <= RTE_AC2_EMPTY_INDEX &&
			p < st->in_start)) {
		st->in[0] = st->in_start;
		ac_seg_last_byte(pmx, st, st->in_start[0]);
		p = st->in[0];
	}

	displace = p - st->in_start;
	return (ac_step_seg(pmx, st, displace));
}

/*
 * AC2_L1x4 search_bulk/search_chain functions definitions.
 */

/* search algorithm implementaion. */
#define	__func_search__			ac_search_l1x4

/* search bulk routinies. */
#define	__func_setup_bulk_state__	ac_start_l1x4_first
#define	__func_search_bulk__		ac_search_l1x4_bulk

/* search chain routinies. */
#define	__func_reset_seg_state__	ac_reset_l1x4_state
#define	__func_setup_seg_state__	ac_set_bloom_state
#define	__func_search_first_seg__	ac_search_l1x4_first
#define	__func_search_next_seg__	ac_search_l1x4_next
#define	__func_search_cur_seg__		ac_search_l1x4_seg

#include "pm_ac2_search_def.h"

/*
 * AC2_L1x4_MH search_bulk/search_chain functions definitions.
 */

/* search algorithm implementaion. */
#define	__func_search__			ac_search_l1x4mh

/* search bulk routinies. */
#define	__func_setup_bulk_state__	ac_start_l1x4_first
#define	__func_search_bulk__		ac_search_l1x4mh_bulk

/* search chain routinies. */
#define	__func_reset_seg_state__	ac_reset_l1x4_state
#define	__func_setup_seg_state__	ac_set_bloom_state
#define	__func_search_first_seg__	ac_search_l1x4mh_first
#define	__func_search_next_seg__	ac_search_l1x4mh_next
#define	__func_search_cur_seg__		ac_search_l1x4mh_seg

#include "pm_ac2_search_def.h"

/*
 * AC2_R128x2 search_bulk/search_chain functions definitions.
 */

/* search algorithm implementaion. */
#define	__func_search__			ac_search_r128x2

/* search bulk routinies. */
#define	__func_setup_bulk_state__	ac_start_regx2_first
#define	__func_search_bulk__		ac_search_r128x2_bulk

/* search chain routinies. */
#define	__func_reset_seg_state__	ac_reset_regx2_state
#define	__func_setup_seg_state__	ac_set_bloom_state
#define	__func_search_first_seg__	ac_search_r128x2_first
#define	__func_search_next_seg__	ac_search_r128x2_next
#define	__func_search_cur_seg__		ac_search_r128x2_seg

#include "pm_ac2_search_def.h"

/*
 * AC2_R128x2_1B search_bulk/search_chain functions definitions.
 */

/* search algorithm implementaion. */
#define	__func_search__			ac_search_r128x2_1b

/* search bulk routinies. */
#define	__func_setup_bulk_state__	ac_start_regx2_first
#define	__func_search_bulk__		ac_search_r128x2_1b_bulk

/* search chain routinies. */
#define	__func_reset_seg_state__	ac_reset_regx2_state
#define	__func_setup_seg_state__	ac_set_bloom_state
#define	__func_search_first_seg__	ac_search_r128x2_1b_first
#define	__func_search_next_seg__	ac_search_r128x2_1b_next
#define	__func_search_cur_seg__		ac_search_r128x2_1b_seg

#include "pm_ac2_search_def.h"

/*
 * AC2_R256x2 search_bulk/search_chain functions definitions.
 */

/* search algorithm implementaion. */
#define	__func_search__			ac_search_r256x2

/* search bulk routinies. */
#define	__func_setup_bulk_state__	ac_start_regx2_first
#define	__func_search_bulk__		ac_search_r256x2_bulk

/* search chain routinies. */
#define	__func_reset_seg_state__	ac_reset_regx2_state
#define	__func_setup_seg_state__	ac_set_bloom_state
#define	__func_search_first_seg__	ac_search_r256x2_first
#define	__func_search_next_seg__	ac_search_r256x2_next
#define	__func_search_cur_seg__		ac_search_r256x2_seg

#include "pm_ac2_search_def.h"

/*
 * AC2_R256x2_1B search_bulk/search_chain functions definitions.
 */

/* search algorithm implementaion. */
#define	__func_search__			ac_search_r256x2_1b

/* search bulk routinies. */
#define	__func_setup_bulk_state__	ac_start_regx2_first
#define	__func_search_bulk__		ac_search_r256x2_1b_bulk

/* search chain routinies. */
#define	__func_reset_seg_state__	ac_reset_regx2_state
#define	__func_setup_seg_state__	ac_set_bloom_state
#define	__func_search_first_seg__	ac_search_r256x2_1b_first
#define	__func_search_next_seg__	ac_search_r256x2_1b_next
#define	__func_search_cur_seg__		ac_search_r256x2_1b_seg

#include "pm_ac2_search_def.h"

/*
 * AC2_P1 search_bulk/search_chain functions definitions.
 */

/* search algorithm implementaion. */
#define	__func_search__			ac_search_p1

/* search bulk routinies. */
#define	__func_setup_bulk_state__	ac_start_l1x4_first
#define	__func_search_bulk__		ac_search_p1_bulk

/* search chain routinies. */
#define	__func_reset_seg_state__	ac_reset_l1x4_state
#define	__func_setup_seg_state__	ac_set_bloom_state
#define	__func_search_first_seg__	ac_search_p1_first
#define	__func_search_next_seg__	ac_search_p1_next
#define	__func_search_cur_seg__		ac_search_p1_seg

#include "pm_ac2_search_def.h"

/*
 * AC2_MB search algorithm implementation.
 */

static inline void
ac_start_mb(uint32_t root_index, const uint8_t *start, uint32_t len,
	uint32_t resnum, struct rte_ac2_state *st)
{
	uint32_t input_index, rem;

	st->index[0] = root_index;
	st->index[1] = root_index;
	st->index[2] = root_index;
	st->index[3] = root_index;

	input_index = len / RTE_AC2_MB_SPLIT;
	rem = len % RTE_AC2_MB_SPLIT;

	st->in_start = start;
	st->in_len = len;

	st->in[0] = start;
	st->in[1] = st->in[0] + input_index + (rem > RTE_AC2_MB_SPLIT - 1);
	st->in[2] = st->in[1] + input_index + (rem > RTE_AC2_MB_SPLIT - 2);
	st->in[3] = st->in[2] + input_index + (rem > RTE_AC2_MB_SPLIT - 3);

	st->len16 = len % AC_MB_UNROLL;
	st->overlap = 0;

	st->res_num = RTE_MIN(DIM(st->res), resnum);
	st->match_idx = 0;
	st->match_num = 0;

	st->segnum++;
}

static inline uint32_t
ac_mb_type_match(const struct rte_pm_ctx *pmx, uint32_t type, uint32_t index,
	const uint8_t *in, struct rte_ac2_state *st)
{
	if (type == RTE_AC2_BNODE_TYPE_MATCH) {

		index = ac_add_match(pmx, in - st->in_start,
			st->overlap, RTE_AC2_GET_INDEX(index),
			st->res + st->match_num);
		st->match_num++;
	}
	return (index);
} 

/*
 * RTE_PM_SEARCH_AC2_MB – generic search algorithm.
 * Divides input buffer into  RTE_AC2_MB_SPLIT(4) chuncks and processes all
 * chunks in parallel.
 * Doesn’t use Bloom filter.
 * Supposed that only 2 types of nodes in the trie are supported:
 * DFA and MATCH
 * (this is enforced at build stage if that search method was selected).
 * Can find matches out of order (e.g. if they are in different chunks).
 * Scalar implementation.
 */
static inline uint32_t
ac_search_mb(const struct rte_pm_ctx *pmx, struct rte_ac2_state *st)
{
	const rte_ac2_index_t *trie;
	uint32_t res_num, types, type[RTE_AC2_MB_SPLIT];
	const uint8_t *end, *in[RTE_AC2_MB_SPLIT];
	uint32_t index[RTE_AC2_MB_SPLIT];

	trie = pmx->ac2.bloom_trie32;
	end = st->in_start + st->in_len;

	res_num = st->res_num;

	COPY_MB_SPLIT_ARRAY(index, st->index);
	COPY_MB_SPLIT_ARRAY(in, st->in);

	/*
	 * unroll 16 (4x4) times, while we have more then
	 * 16 input bytes to process.
	 */
	while (in[3] + RTE_AC2_MB_SPLIT < end) {
		index[0] = trie[index[0] + *in[0]++];
		index[1] = trie[index[1] + *in[1]++];
		index[2] = trie[index[2] + *in[2]++];
		index[3] = trie[index[3] + *in[3]++];

		types = RTE_AC2_GET_TYPE(index[0] | index[1] | index[2] |
			index[3]);

		if (likely (types == RTE_AC2_BNODE_TYPE_DFA)) {
			index[0] = trie[index[0] + *in[0]++];
			index[1] = trie[index[1] + *in[1]++];
			index[2] = trie[index[2] + *in[2]++];
			index[3] = trie[index[3] + *in[3]++];

			types = RTE_AC2_GET_TYPE(index[0] | index[1] |
				index[2] | index[3]);

			if (likely (types == RTE_AC2_BNODE_TYPE_DFA)) {
				index[0] = trie[index[0] + *in[0]++];
				index[1] = trie[index[1] + *in[1]++];
				index[2] = trie[index[2] + *in[2]++];
				index[3] = trie[index[3] + *in[3]++];

				types = RTE_AC2_GET_TYPE(index[0] | index[1] |
					index[2] | index[3]);

				if (likely (types == RTE_AC2_BNODE_TYPE_DFA)) {
					index[0] = trie[index[0] + *in[0]++];
					index[1] = trie[index[1] + *in[1]++];
					index[2] = trie[index[2] + *in[2]++];
					index[3] = trie[index[3] + *in[3]++];

					types = RTE_AC2_GET_TYPE(index[0] |
						index[1] | index[2] | index[3]);

                        		if (types == RTE_AC2_BNODE_TYPE_DFA)
                            			continue;
				}
			}
		}

		type[0] = RTE_AC2_GET_TYPE(index[0]);
		type[1] = RTE_AC2_GET_TYPE(index[1]);
		type[2] = RTE_AC2_GET_TYPE(index[2]);
		type[3] = RTE_AC2_GET_TYPE(index[3]);

		index[0] = ac_mb_type_match(pmx, type[0], index[0], in[0], st);
		index[1] = ac_mb_type_match(pmx, type[1], index[1], in[1], st);
		index[2] = ac_mb_type_match(pmx, type[2], index[2], in[2], st);
		index[3] = ac_mb_type_match(pmx, type[3], index[3], in[3], st);

		st->len16 = (st->in_len - (in[0] - st->in_start) *
			RTE_AC2_MB_SPLIT) % AC_MB_UNROLL;

		if (res_num <= st->match_num + 4) {
			COPY_MB_SPLIT_ARRAY(st->index, index);
			COPY_MB_SPLIT_ARRAY(st->in, in);
			return (st->match_num);
		}
	}

	/* unroll remaining up to 16 bytes, (4x4) times. */
	while (in[3] < end) {
		switch (st->len16) {
		case 0:
		index[0] = trie[index[0] + *in[0]++];
		case 15:
		index[1] = trie[index[1] + *in[1]++];
		case 14:
		index[2] = trie[index[2] + *in[2]++];
		case 13:
		index[3] = trie[index[3] + *in[3]++];

		types = RTE_AC2_GET_TYPE(index[0] | index[1] | index[2] |
			index[3]);
		st->len16 = 12;

		if (likely (types == RTE_AC2_BNODE_TYPE_DFA)) {
			case 12:
			index[0] = trie[index[0] + *in[0]++];
			case 11:
			index[1] = trie[index[1] + *in[1]++];
			case 10:
			index[2] = trie[index[2] + *in[2]++];
			case 9:
			index[3] = trie[index[3] + *in[3]++];

			types = RTE_AC2_GET_TYPE(index[0] | index[1] |
				index[2] | index[3]);
			st->len16 = 8;

			if (likely (types == RTE_AC2_BNODE_TYPE_DFA)) {
				case 8:
				index[0] = trie[index[0] + *in[0]++];
				case 7:
				index[1] = trie[index[1] + *in[1]++];
				case 6:
				index[2] = trie[index[2] + *in[2]++];
				case 5:
				index[3] = trie[index[3] + *in[3]++];

				types = RTE_AC2_GET_TYPE(index[0] | index[1] |
					index[2] | index[3]);
				st->len16 = 4;

				if (likely (types == RTE_AC2_BNODE_TYPE_DFA)) {
					case 4:
					index[0] = trie[index[0] + *in[0]++];
					case 3:
					index[1] = trie[index[1] + *in[1]++];
					case 2:
					index[2] = trie[index[2] + *in[2]++];
					case 1:
					index[3] = trie[index[3] + *in[3]++];

					types = RTE_AC2_GET_TYPE(index[0] |
						index[1] | index[2] | index[3]);
					st->len16 = 0;

                        		if (types == RTE_AC2_BNODE_TYPE_DFA)
                            			continue;
				}
			}
		}
		}

		type[0] = RTE_AC2_GET_TYPE(index[0]);
		type[1] = RTE_AC2_GET_TYPE(index[1]);
		type[2] = RTE_AC2_GET_TYPE(index[2]);
		type[3] = RTE_AC2_GET_TYPE(index[3]);

		index[0] = ac_mb_type_match(pmx, type[0], index[0], in[0], st);
		index[1] = ac_mb_type_match(pmx, type[1], index[1], in[1], st);
		index[2] = ac_mb_type_match(pmx, type[2], index[2], in[2], st);
		index[3] = ac_mb_type_match(pmx, type[3], index[3], in[3], st);

		if (res_num <= st->match_num + 4) {
			COPY_MB_SPLIT_ARRAY(st->index, index);
			COPY_MB_SPLIT_ARRAY(st->in, in);
			return (st->match_num);
		}
	}

	while (in[2] < end) {
		if (AC_MB_OVERLAP(trie, index[2], st->overlap))
			break;
		if (AC_MB_OVERLAP(trie, index[1], st->overlap)) {
			index[1] = index[2];
			in[1] = in[2];

			index[2] = RTE_AC2_EMPTY_INDEX;
			in[2] = end;
			break;
		}
		if (AC_MB_OVERLAP(trie, index[0], st->overlap)) {
			index[0] = index[1];
			in[0] = in[1];
			index[1] = index[2];
			in[1] = in[2];

			index[2] = RTE_AC2_EMPTY_INDEX;
			in[2] = end;
			break;
		}

		st->overlap++;

		index[0] = trie[index[0] + *in[0]++];
		index[1] = trie[index[1] + *in[1]++];
		index[2] = trie[index[2] + *in[2]++];
		types = RTE_AC2_GET_TYPE(index[0] | index[1] | index[2]);
		if (types == RTE_AC2_BNODE_TYPE_DFA)
			continue;

		type[0] = RTE_AC2_GET_TYPE(index[0]);
		type[1] = RTE_AC2_GET_TYPE(index[1]);
		type[2] = RTE_AC2_GET_TYPE(index[2]);

		index[0] = ac_mb_type_match(pmx, type[0], index[0], in[0], st);
		index[1] = ac_mb_type_match(pmx, type[1], index[1], in[1], st);
		index[2] = ac_mb_type_match(pmx, type[2], index[2], in[2], st);

		if (res_num <= st->match_num + 3) {
			COPY_MB_SPLIT_ARRAY(st->index, index);
			COPY_MB_SPLIT_ARRAY(st->in, in);
			return (st->match_num);
		}
	}

	while (in[1] < end) {
		if (AC_MB_OVERLAP(trie, index[1], st->overlap))
			break;
		if (AC_MB_OVERLAP(trie, index[0], st->overlap)) {
			index[0] = index[1];
			in[0] = in[1];

			index[1] = RTE_AC2_EMPTY_INDEX;
			in[1] = end;
			break;
		}

		st->overlap++;

		index[0] = trie[index[0] + *in[0]++];
		index[1] = trie[index[1] + *in[1]++];
		types = RTE_AC2_GET_TYPE(index[0] | index[1]);
		if (types == RTE_AC2_BNODE_TYPE_DFA)
			continue;

		type[0] = RTE_AC2_GET_TYPE(index[0]);
		type[1] = RTE_AC2_GET_TYPE(index[1]);

		index[0] = ac_mb_type_match(pmx, type[0], index[0], in[0], st);
		index[1] = ac_mb_type_match(pmx, type[1], index[1], in[1], st);

		if (res_num <= st->match_num + 2) {
			COPY_MB_SPLIT_ARRAY(st->index, index);
			COPY_MB_SPLIT_ARRAY(st->in, in);
			return (st->match_num);
		}
	}

	while (in[0] < end) {
		if (AC_MB_OVERLAP(trie, index[0], st->overlap))
			break;

		st->overlap++;

		index[0] = trie[index[0] + *in[0]++];
		type[0] = RTE_AC2_GET_TYPE(index[0]);
		if (type[0] == RTE_AC2_BNODE_TYPE_DFA)
			continue;

		index[0] = ac_mb_type_match(pmx, type[0], index[0], in[0], st);
		if (res_num <= st->match_num + 1) {
			COPY_MB_SPLIT_ARRAY(st->index, index);
			COPY_MB_SPLIT_ARRAY(st->in, in);
			return (st->match_num);
		}
	}

	COPY_MB_SPLIT_ARRAY(st->index, index);
	COPY_MB_SPLIT_ARRAY(st->in, in);
	return (st->match_num);
}

static int
ac_search_mb_bulk(const struct rte_pm_ctx *pmx,
	const struct rte_pm_inbuf buf[], struct rte_pm_match res[],
	uint32_t num)
{
	int ret;
	uint32_t i;
	struct rte_ac2_state st;

	ret = 0;
	for (i = 0; i != num; i++) {

		ac_start_mb(pmx->ac2.root_index, buf[i].buf, buf[i].len,
			1, &st);

		/* if matches found, extract the first one */
		if (ac_search_mb(pmx, &st) != 0) {
			ac_extract_match(pmx, st.res, res + i, 1);
			ret++;
		} else {
			res[i].fin = 0;
		}
	}
	return (ret);
}

/*
 * Starts processing of the new segment in the chain.
 * Stops when end of segment is reached of up to resnum matches is found.
 * Returns number of matches found.
 */
static int
ac_search_mb_next(const struct rte_pm_ctx *pmx, const struct rte_pm_inbuf *buf,
	struct rte_pm_match res[], uint32_t resnum, struct rte_pm_state *st)
{
	struct rte_ac2_state *state;
	uint32_t fill, mseg_idx, i;

	state = &st->ac2;
	fill = 0;

	/*
	 * check is there unfinished search from previous one,
	 * e.g. possible match that spans though segments boundaries.
	 * If there are few possible matches, then select the longest one.
	 */

	mseg_idx = RTE_AC2_EMPTY_INDEX;

	for (i = 0; i != RTE_AC2_MB_SPLIT; i++) {
		if (state->in[i] >= state->in_start + state->in_len &&
				(mseg_idx = state->index[i]) >
				RTE_AC2_EMPTY_INDEX)
			break;
	}

	ac_start_mb(pmx->ac2.root_index, buf->buf, buf->len, DIM(state->res),
		state);

	if (mseg_idx > RTE_AC2_EMPTY_INDEX) {
		state->index[0] = mseg_idx;
	}

	/* find and extract next matches in current segment. */
	while (ac_search_mb(pmx, state) != 0) {

		fill += ac_extract_matches(pmx, state, res + fill,
			resnum - fill);

		if (fill == resnum)
			break;

		state->match_idx = 0;
		state->match_num = 0;
	}

	return (fill);
}

/*
 * Starts processing of the first segment in the chain.
 * Stops when end of segment is reached of up to resnum matches is found.
 * Returns number of matches found.
 */
static int
ac_search_mb_first(const struct rte_pm_ctx *pmx, const struct rte_pm_inbuf *buf,
	struct rte_pm_match res[], uint32_t resnum, struct rte_pm_state *st)
{
	memset(st, 0, sizeof (*st));
	return (ac_search_mb_next(pmx, buf, res, resnum, st));
}

/*
 * Continues processing of the new segment in the chain.
 * Stops when end of segment is reached of up to resnum matches is found.
 * Returns number of matches found.
 */
static int
ac_search_mb_seg(const struct rte_pm_ctx *pmx, struct rte_pm_match res[],
	uint32_t resnum, struct rte_pm_state *st)
{
	uint32_t fill;
	struct rte_ac2_state *state;

	state = &st->ac2;
	fill = 0;

	do {
		/* Extract previously found matches. */
		fill += ac_extract_matches(pmx, state, res + fill,
			resnum - fill);

		/* No more space in result array. */
		if (fill == resnum)
			break;

		state->match_idx = 0;
		state->match_num = 0;

	/* find next matches in the current segment. */
	} while (ac_search_mb(pmx, state) != 0);

	return (fill);
}

/*
 * Structures and types that define all available search functions.
 */
enum {
	RTE_PM_FN_AC2_UNDEF,
	RTE_PM_FN_AC2_L1x4,
	RTE_PM_FN_AC2_L1x4_MB,
	RTE_PM_FN_AC2_L1x4_MH,
	RTE_PM_FN_AC2_R128x2,
	RTE_PM_FN_AC2_R128x2_1B,
	RTE_PM_FN_AC2_R256x2,
	RTE_PM_FN_AC2_R256x2_1B,
	RTE_PM_FN_AC2_P1,
	RTE_PM_FN_AC2_NUM
};

/*
 * This array *should* be in sync with the enum above.
 */
const struct rte_pm_search_fn rte_pm_search_fn[] = {

	/* RTE_PM_FN_AC2_UNDEF */
	{
		.search_bulk = NULL,
		.search_seg = NULL,
		.next_seg = NULL,
		.first_seg = NULL,
	},

	/* RTE_PM_FN_AC2_L1x4 */
	{
		.search_bulk = ac_search_l1x4_bulk,
		.search_seg = ac_search_l1x4_seg,
		.next_seg = ac_search_l1x4_next,
		.first_seg = ac_search_l1x4_first,
	},

	/*RTE_PM_FN_AC2_L1x4_MB */
	{
		.search_bulk = ac_search_mb_bulk,
		.search_seg = ac_search_mb_seg,
		.next_seg = ac_search_mb_next,
		.first_seg = ac_search_mb_first,
	},

	/* RTE_PM_FN_AC2_L1x4_MH */
	{
		.search_bulk = ac_search_l1x4mh_bulk,
		.search_seg = ac_search_l1x4mh_seg,
		.next_seg = ac_search_l1x4mh_next,
		.first_seg = ac_search_l1x4mh_first,
	},

	/* RTE_PM_FN_AC2_R128x2 */
	{
		.search_bulk = ac_search_r128x2_bulk,
		.search_seg = ac_search_r128x2_seg,
		.next_seg = ac_search_r128x2_next,
		.first_seg = ac_search_r128x2_first,
	},

	/* RTE_PM_FN_AC2_R128x2_1B */
	{
		.search_bulk = ac_search_r128x2_1b_bulk,
		.search_seg = ac_search_r128x2_1b_seg,
		.next_seg = ac_search_r128x2_1b_next,
		.first_seg = ac_search_r128x2_1b_first,
	},

	/* RTE_PM_FN_AC2_R256x2 */
	{
		.search_bulk = ac_search_r256x2_bulk,
		.search_seg = ac_search_r256x2_seg,
		.next_seg = ac_search_r256x2_next,
		.first_seg = ac_search_r256x2_first,
	},

	/* RTE_PM_FN_AC2_R256x2_1B */
	{
		.search_bulk = ac_search_r256x2_1b_bulk,
		.search_seg = ac_search_r256x2_1b_seg,
		.next_seg = ac_search_r256x2_1b_next,
		.first_seg = ac_search_r256x2_1b_first,
	},

	/* RTE_PM_FN_AC2_P1 */
	{
		.search_bulk = ac_search_p1_bulk,
		.search_seg = ac_search_p1_seg,
		.next_seg = ac_search_p1_next,
		.first_seg = ac_search_p1_first,
	}
};

/*
 * Setup runtime search function to use.
 */
int
ac_set_runtime(struct rte_pm_ctx *pmx, int one_byte)
{
	int rc;

	pmx->search_fn = RTE_PM_FN_AC2_UNDEF;

	if (pmx->bopt.search_type == RTE_PM_SEARCH_AC2_L1x4) {
		pmx->search_fn = RTE_PM_FN_AC2_L1x4;
	} else if (pmx->bopt.search_type == RTE_PM_SEARCH_AC2_L1x4_MB) {
		pmx->search_fn = RTE_PM_FN_AC2_L1x4_MB;
	} else if (RTE_PM_SEARCH_AC2_L1x4_MH_FAMILY(pmx->bopt.search_type)) {
		pmx->search_fn = RTE_PM_FN_AC2_L1x4_MH;
	} else if (pmx->bopt.search_type == RTE_PM_SEARCH_AC2_R128x2) {
		if (one_byte == 0)
			pmx->search_fn = RTE_PM_FN_AC2_R128x2;
		 else
			pmx->search_fn = RTE_PM_FN_AC2_R128x2_1B;
	} else if (pmx->bopt.search_type == RTE_PM_SEARCH_AC2_R256x2) {
		if (one_byte == 0)
			pmx->search_fn = RTE_PM_FN_AC2_R256x2;
		else
			pmx->search_fn = RTE_PM_FN_AC2_R256x2_1B;
	} else if (pmx->bopt.search_type == RTE_PM_SEARCH_AC2_P1) {
		pmx->search_fn = RTE_PM_FN_AC2_P1;
	}

	rc = (pmx->search_fn != RTE_PM_FN_AC2_UNDEF) ? 0 : -ENOTSUP;
	return (rc);
}

