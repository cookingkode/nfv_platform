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

#ifndef _PM_AC2_BLD_H_
#define _PM_AC2_BLD_H_

/**
 * @file
 *
 * RTE Pattern Match AC2 build header
 * Contains temporary build context and related structures.
 * These structures are expected to be alive only while AC2 build is active.
 */

#include "pmac_mem.h"
#include "pm_ac2.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * The following macros are default values for construction DFA nodes.
 * AC2_FORCE_LEVEL - forces DFA type for all nodes of this depth or less.
 * AC2_FORCE_FAILURES - forces all failure path references to be DFA nodes.
 * AC2_FORCE_MATCHES - forces all nodes referenced by a match node to be DFA
 * nodes.
*/

#ifndef AC2_FORCE_LEVEL
#define	AC2_FORCE_LEVEL		8
#endif	/* AC2_FORCE_LEVEL */

#ifndef AC2_FORCE_FAILURES
#define	AC2_FORCE_FAILURES	0
#endif	/* AC2_FORCE_FAIULURES */

#ifndef AC2_FORCE_MATCHES
#define	AC2_FORCE_MATCHES	0
#endif	/* AC2_FORCE_MATCHES */

#define	AC2_POOL_ALIGN		8
#define	AC2_POOL_ALLOC_MIN	0x100000

#define	AC2_MAX_LEVEL		31
#define	AC2_LEVEL_NUM		(AC2_MAX_LEVEL + 1)

#define	AC2_BLOOM_LEVEL		3

/*
 * we can have up to 2^16 Level 3 escapes
 * (first 2 bytes/levels of all patterns).
 * 
 */
#define	AC2_L3_ESCAPE_NUM	(1 << (AC2_BLOOM_LEVEL - 1) * CHAR_BIT)

#define	AC2_NUM_CHAR_STEP	16

#define	AC2_MAX_MB_LEN		(RTE_AC2_L1x4_MH11_LEN  - 1)

#define	OTHER_CASE(c)		\
	((typeof (c))((islower(c)) ? (toupper(c)) : (tolower(c))))

#define	TO_CASELESS(c, s)	\
	((typeof (c))(((s) == RTE_PM_CASE_LESS) ? (tolower(c)) : (c)))

#define	TO_OTHER_CASE(c, s)	\
	((typeof (c))(((s) == RTE_PM_CASE_LESS) ? OTHER_CASE(c) : (c)))

struct ac2_trie_build {
	struct ac2_match_data  *match_data;
	struct ac2_match_data  *match_end;
	struct ac2_trie_build  *link;
	struct ac2_trie_build  *failure;
	int                     node_id;
	uint32_t                depth;
	const uint8_t          *pattern;
	int                     num_char_save;
	int                     num_char;
	int                     size_char;
	uint8_t                 a_match_char[UINT8_MAX + 1];
	uint8_t                *p_match_char;
	struct ac2_trie_build **next_trie;
	uint32_t                id;
	int                     level;
	int                     flag_dfa[8];
	struct ac2_trie_build  *mult;
	int                     sequential;
	int                     start;
	int                     bloom_type;
	int                     bloom_position;
	int                     bloom_failure_position;
	int                     force256;
	int                     failure_refs;
	int                     link_length;
	int                     sparse_min;
	int                     exclude;
	int                     patterns;
};

struct ac2_bld_ctx {
	struct rte_pm_ctx      pmx;
	struct pmac_mem_pool    pool;
	struct ac2_trie_build *build_list;
	struct ac2_trie_build *build_last;
	struct ac2_trie_build *build_root;
	struct ac2_match_data *match_list;
	struct ac2_match_data *match_last;
	uint32_t               nodes;
	uint32_t               stateless_mask;
	uint32_t               match_index;
	uint32_t               bloom_bits;
	uint32_t               bloom_size;
	uint32_t               mask_len;
	uint32_t               mask;
	uint32_t               shift_max;
	uint32_t               min_pattern_len;
	int                    num_nodes[AC2_LEVEL_NUM];
	int                    num_links[AC2_LEVEL_NUM];
	uint32_t               num_match_points;
	uint32_t               pattern_len_num[RTE_AC2_PATTERN_BYTES];
	uint32_t               sum_pattern_len[RTE_AC2_PATTERN_BYTES];
	ac2_nodes_count_t      node_counts;
	uint32_t               num_failure_dfa;
	uint32_t               max_alloc_index;
	uint32_t               bloom_hits;
	uint32_t               bloom_iter;
};

/*Structure for managing a string during trie construction */
struct ac2_search_data {
    const uint8_t *search_string;
    uint32_t       index;
    uint32_t       size;
};

#ifdef __cplusplus
}
#endif

#endif /* _PM_AC2_BLD_H_ */
