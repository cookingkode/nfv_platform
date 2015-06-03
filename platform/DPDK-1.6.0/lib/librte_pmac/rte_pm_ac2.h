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

#ifndef _RTE_PM_AC2_H_
#define _RTE_PM_AC2_H_

/**
 * @file
 *
 * RTE Pattern Match AC2 header
 */

#include <rte_pmac_vect.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
	RTE_AC2_L1x4_MH_LEN =   4,
	RTE_AC2_L1x4_MH5_LEN =  5,
	RTE_AC2_L1x4_MH7_LEN =  7,
	RTE_AC2_L1x4_MH11_LEN = 11,
	RTE_AC2_L1x4_MH_LEN_MAX = RTE_AC2_L1x4_MH11_LEN,
};

enum {
	RTE_AC2_BNODE_TYPE_SHIFT = 2,
	RTE_AC2_BNODE_TYPE_MASK = (1 << RTE_AC2_BNODE_TYPE_SHIFT) - 1,
	RTE_AC2_BNODE_TYPE_DFA = 0,
	RTE_AC2_BNODE_TYPE_FANOUT = 1,
	RTE_AC2_BNODE_TYPE_SEQ = 2,
	RTE_AC2_BNODE_TYPE_MATCH = 3,
};

#define	RTE_PM_SEARCH_AC2_L1x4_MH_FAMILY(t)	\
	((t) >=  RTE_PM_SEARCH_AC2_L1x4_MH &&   \
	(t) <=  RTE_PM_SEARCH_AC2_L1x4_MH11)

#define	RTE_PM_SEARCH_AC2_L1x4_FAMILY(t)	\
	((t) >=  RTE_PM_SEARCH_AC2_L1x4 && (t) <=  RTE_PM_SEARCH_AC2_L1x4_MH11)

#define	RTE_PM_SEARCH_AC2_FAMILY(t)	\
	((t) >=  RTE_PM_SEARCH_AC2_L1x4 && (t) <=  RTE_PM_SEARCH_AC2_P1)

#define	RTE_AC2_EMPTY_INDEX	\
	(RTE_AC2_BLOOM_DFA32_SIZE | RTE_AC2_BNODE_TYPE_MATCH)
#define	RTE_AC2_EMPTY_LEVEL(x)	\
	((x) << RTE_AC2_BNODE_TYPE_SHIFT | RTE_AC2_BNODE_TYPE_MATCH)

#define	RTE_AC2_GET_INDEX(x)	((x) & ~RTE_AC2_BNODE_TYPE_MASK)

#define	RTE_AC2_GET_TYPE(x)	((x) & RTE_AC2_BNODE_TYPE_MASK)

#define	RTE_AC2_PATTERN_BYTES	(RTE_AC2_L1x4_MH_LEN_MAX + 1)

struct rte_ac2_match_data {
	uint64_t userdata;
	uint32_t len;
	uint32_t next;
	uint32_t match_point;
	uint8_t  pattern[RTE_AC2_PATTERN_BYTES];
} __rte_cache_aligned;

typedef uint32_t rte_ac2_index_t;

struct rte_ac2_bloom_match32 {
    rte_ac2_index_t index;
    uint32_t        match_index;
    uint32_t        match_point;
    int32_t         match_offset;
    uint32_t        level;
}  __rte_cache_aligned;

#define	RTE_AC2_BLOOM_MATCH32_SIZE	\
	(sizeof (struct rte_ac2_bloom_match32) / sizeof (rte_ac2_index_t))


/*
 *  even elements of string[] contains TO_CASELESS(c),
 *  odd elements of the string[] contains TO_OTHER_CASE(c). 
 */
#define	RTE_AC2_BLOOM_FANOUT32_NUM	8
#define	RTE_AC2_BLOOM_FANOUT32_LEN	(RTE_AC2_BLOOM_FANOUT32_NUM * 2)

struct rte_ac2_bloom_fanout32 {
    uint8_t         string[RTE_AC2_BLOOM_FANOUT32_LEN];
    rte_ac2_index_t index[10];
    uint32_t        level;
    uint32_t        mask;
}  __rte_cache_aligned;

#define	RTE_AC2_BLOOM_FANOUT32_SIZE	\
	(sizeof (struct rte_ac2_bloom_fanout32) / sizeof (rte_ac2_index_t))

/*
 *  first RTE_AC2_BLOOM_SEQ32_NUM elements of string[] contains TO_CASELESS(c),
 *  other RTE_AC2_BLOOM_SEQ32_LEN elements of the string[] contains
 *  TO_OTHER_CASE(c). 
 */
#define	RTE_AC2_BLOOM_SEQ32_NUM	8
#define	RTE_AC2_BLOOM_SEQ32_LEN	(RTE_AC2_BLOOM_SEQ32_NUM * 2)

struct rte_ac2_bloom_seq32 {
    uint8_t         string[RTE_AC2_BLOOM_SEQ32_LEN];
    rte_ac2_index_t index[10];
    uint32_t        level;
    uint32_t        mask;
}  __rte_cache_aligned;

#define	RTE_AC2_BLOOM_SEQ32_SIZE	\
	(sizeof (struct rte_ac2_bloom_seq32) / sizeof (rte_ac2_index_t))

struct rte_ac2_bloom_dfa32 {
    rte_ac2_index_t index[UINT8_MAX + 1];
    uint32_t        level;
}  __rte_cache_aligned;

#define	RTE_AC2_BLOOM_DFA32_SIZE	\
	(sizeof (struct rte_ac2_bloom_dfa32) / sizeof (rte_ac2_index_t))

/*
 * Run-time state.
 */

struct rte_ac2_match {
	uint32_t fin;
	uint32_t overlap;
	uint32_t match_index;
};

/* For how many search chunks to split input buffer (L1x4_MB). */
#define	RTE_AC2_MB_SPLIT	4

/* Should be multiply of RTE_AC2_MB_SPLIT. */
#define	RTE_AC2_STATE_RESNUM	(4 *  RTE_AC2_MB_SPLIT)

struct rte_ac2_state {
	const uint8_t *in_start;
	uint32_t       in_len;
	uint32_t       len16;
	uint32_t       overlap;
	uint32_t       match_idx;
	uint32_t       match_num;
	uint32_t       res_num;
	uint32_t       prev_mask;
	uint32_t       bloom_mask;
	uint32_t       index_mask;
	uint32_t       pos;
	uint32_t       hash_residual;
	uint32_t       sequence;
	uint32_t       segnum;
	uint32_t       last_byte;
	uint32_t       index[RTE_AC2_MB_SPLIT];
	const uint8_t *in[RTE_AC2_MB_SPLIT];
	struct rte_ac2_match res[RTE_AC2_STATE_RESNUM];
};

/*
 * AC2 PM context.
 */

struct rte_pm_ac2 {
	struct rte_ac2_match_data *match_index;
	rte_ac2_index_t *bloom_l3;
	rte_ac2_index_t *bloom_trie32;
	uint8_t         *bloom;
	mmreg_t          one_byte_low;
	mmreg_t          one_byte_high;
	sse_t            sse_mask;
	sse_t            input_mask;
	sse_t            first_pattern;
	uint32_t         root_index;
	uint32_t         shift1;
	uint32_t         shift2;
	uint32_t         shift3;
	void            *mem;
	/* fields below are used for statistics and load/store. */
	uint64_t         mem_sz;
	uint64_t         ofs_match;
	uint64_t         ofs_l3;
	uint64_t         ofs_trie32;
	uint64_t         ofs_bloom;
};

int
ac_analyze(const struct rte_pm_ctx *pmx, const struct rte_pm_build_opt *opt,
	struct rte_pm_search_avail *res);

int
ac_build(struct rte_pm_ctx *pmx, const struct rte_pm_build_opt *opt);

int
ac_store(const struct rte_pm_ctx *pmx, pm_store_fn_t *fn, void *arg);

int
ac_load(struct rte_pm_ctx *pmx, pm_load_fn_t *fn, void *arg);

void
ac_free(struct rte_pm_ac2 *ctx);

#ifdef __cplusplus
}
#endif

#endif /* _RTE_PM_AC2_H_ */
