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

#ifndef _PM_AC2_H_
#define _PM_AC2_H_

/**
 * @file
 *
 * Pattern Match AC2 internal header
 */

#include <rte_pm.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Bloom filter
 * The minimum Bloom size (BLOOMBITS) targets 16 bit values. Optimally,
 * the Bloom filter fits in L1 cache (32KB or 256Kb) and the max size
 * (AC2_BLOOM_MAX) is limited to L2 cache (256KB or 2Mb). The code
 * automatically sizes the the bloom filter based on the number of nodes
 * in the 4th level of the trie (L3 escapes).
 * It attempts to have AC2_BLOOM_RATIO times as many bits in the bloom filter
 * as there are L3 escapes for a small false positive rate.
 * These sizing parameters could be expanded to handle LL cache
 * and/or E-DRAM in order to accomodate extra large DBs,
 * however, the algorithm would need tuning to optimize around the
 * increased memory latency.
 */

/* max bloom filter size in bytes (L2 cache). */
#define	AC2_BLOOM_MAX            0x1fffff
/* default bloom filter size in bytes (L1 cache). */
#define	AC2_BLOOM_BITS           0x3ffff
/* number of bloom filter bits per pattern. */
#define	AC2_BLOOM_RATIO          200

#define	AC2_BLOOM_SHIFT		3
#define	AC2_BLOOM_MASK		((1 << AC2_BLOOM_SHIFT) - 1)

enum {
    AC2_NODES_TOTAL,     /* number of sequential and none sequential nodes */
    AC2_NODES_SEQ,       /* number of sequential nodes */
    AC2_NODES_START,     /* number of sequential starts */
    AC2_NODES_MATCH,     /* number of match nodes */
    AC2_NODES_SEQ11,     /* seq nodes for 32 bit links */
    AC2_NODES_FAN11,     /* fanout nodes for 32 bit links */
    AC2_NODES_FANOVER11, /* full fanout nodes for 32 bit links */
    AC2_NODES_NUM
};

typedef uint32_t ac2_nodes_count_t[AC2_NODES_NUM];

struct ac2_match_data {
	const struct rte_pm_pattern *pmp;
	struct ac2_match_data *next;
	struct ac2_match_data *match_list;
        int               match_id;
	int               match_point;
	int               last_search_id;
	uint32_t          index;
	int               allocated;
	int               bloom_filter;
	int               duplicate;
};

#define	HASH32(a, b)	MM_MADD16(a, b)

static const mmreg_t mm_prime_word = {
	.ui32 = {0xeaad8405, 0xeaad8405, 0xeaad8405, 0xeaad8405},
};

static const mmreg_t mm_prime_byte = {
	.ui32 = {0xa343a343, 0xa343a343, 0xa343a343, 0xa343a343},
};

int
ac_set_runtime(struct rte_pm_ctx *pmx, int one_byte);

#ifdef __cplusplus
}
#endif

#endif /* _PM_AC2_H_ */
