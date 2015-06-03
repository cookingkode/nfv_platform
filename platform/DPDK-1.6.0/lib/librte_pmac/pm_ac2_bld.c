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
#include "pm_ac2_bld.h"

#define	 DEPTH1BR	64

#define	MAX_ADDS	(1 + 2048 + 256)

#define	 AC2_BLOOM_MB_NODES	2500

#define	AC2_R128x2_MAX_PATTERNS	6
#define	AC2_R256x2_MAX_PATTERNS	13

#define	AC2_EXCLUDE_NODE	1
#define	AC2_FAIL_NODE		2

static const struct {
	uint32_t min_pattern_len;
	enum rte_pm_search search_type;
	uint32_t shift_max;
	uint32_t shift1;
	uint32_t shift2;
	uint32_t shift3;
} ac2_mh_select[] = {
	{RTE_AC2_L1x4_MH_LEN,   RTE_PM_SEARCH_AC2_L1x4_MH,   1, 0, 0, 0},
	{RTE_AC2_L1x4_MH5_LEN,  RTE_PM_SEARCH_AC2_L1x4_MH5,  2, 0, 0, 1},
	{RTE_AC2_L1x4_MH7_LEN,  RTE_PM_SEARCH_AC2_L1x4_MH7,  4, 0, 2, 1},
	{RTE_AC2_L1x4_MH11_LEN, RTE_PM_SEARCH_AC2_L1x4_MH11, 8, 4, 2, 1},
};

static rte_ac2_index_t ac_get_bloom_index32(rte_ac2_index_t *trie,
	const struct ac2_trie_build * node, uint8_t index, int case_sen);

static int
ac_match_list(struct ac2_bld_ctx *ctx)
{
	uint32_t i;
	struct ac2_match_data *match, **match_list;

	match_list = &ctx->match_list;
	for (i = 0; i != ctx->pmx.pattern_num; i++) {

		if ((match = pmac_alloc(&ctx->pool, sizeof (*match))) == NULL)
			return (-ENOMEM);

		match->pmp = ctx->pmx.patterns + i;

		ctx->match_index++;
		*match_list = match;
		ctx->match_last = match;
		match_list = &match->match_list;
	}

	return (0);
}

/* Create a Build node for trie construction */
static struct ac2_trie_build *
ac_create_build_node(struct ac2_bld_ctx *ctx, const uint8_t * patt,
	uint32_t len, int level)
{
	struct ac2_trie_build *node;

	if ((node = pmac_alloc(&ctx->pool, sizeof(*node))) == NULL)
		return (NULL);

	if (ctx->build_root == NULL)
		ctx->build_root = node;

	if (ctx->build_list == NULL)
		ctx->build_list = node;
	else
		ctx->build_last->link = node;

	ctx->build_last = node;

	node->failure = ctx->build_root;
	node->depth = len;
	node->pattern = patt;
	node->level = level;
	level = RTE_MIN((uint32_t)level, DIM(ctx->num_nodes) - 1);
	ctx->num_nodes[level]++;
	ctx->nodes++;
	node->id = ctx->nodes;
	memset(node->a_match_char, UINT8_MAX, sizeof (node->a_match_char));

	return (node);
}

/* Get pointer to linked node for character c. */
static struct ac2_trie_build *
ac_get_build_node(const struct ac2_trie_build *node, uint8_t c, int case_sen)
{
	uint8_t lc;

	lc = TO_CASELESS(c, case_sen);
	if (node->a_match_char[lc] < node->num_char)
		return (node->next_trie[node->a_match_char[lc]]);
	return (NULL);
}

/* Add a node to the trie. */
static int
ac_add_build_node(struct ac2_bld_ctx *ctx, struct ac2_trie_build *node,
	struct ac2_trie_build *next_node, uint8_t c, int case_sen)
{
	uint8_t lc;
	uint8_t *ptr;
	struct ac2_trie_build **build;

	lc = TO_CASELESS(c, case_sen);
	if (node->num_char >= node->size_char) {
		node->size_char = node->num_char + AC2_NUM_CHAR_STEP;
		build = pmac_alloc(&ctx->pool,
			node->size_char * sizeof (*build));
		ptr = pmac_alloc(&ctx->pool, node->size_char);

		if (build == NULL || ptr == NULL)
			return (-ENOMEM);

		if (node->next_trie != NULL) {
			memcpy(build, node->next_trie,
				node->num_char * sizeof (*build));
			memcpy(ptr, node->p_match_char, node->num_char);
		}
		node->next_trie = build;
		node->p_match_char = ptr;
	}
	node->next_trie[node->num_char] = next_node;
	node->p_match_char[node->num_char] = lc;
	node->a_match_char[lc] = (uint8_t)node->num_char;
	node->num_char++;

	return (0);
}

/*
 * Starting at node, traverse trie to find match position for the given string
 * Returns matching node, or NULL if no match was found.
 */
static struct ac2_trie_build *
ac_search_build_node(struct ac2_trie_build *node,
	const struct ac2_search_data *data, int case_sen)
{
	uint32_t index;

	/* follow the Trie to find match or NULL if no match. */
	for (index = data->index; index < data->size && node != NULL; index++) {
        	node = ac_get_build_node(node, data->search_string[index],
			case_sen);
	}
	return (node);
}

/*
 * Gets the link to the next node for this character in the pattern.
 * If the next node doesn't exist yet, creates a new node and populates it.
 * Returns next node on success or NULL on failure.
 */
static struct ac2_trie_build *
ac_trie_next_node(struct ac2_bld_ctx *ctx, struct ac2_trie_build *node,
	const uint8_t *search, int index, int case_sen)
{
	struct ac2_trie_build *next_node;

	/*
	 * if no link established yet, then create new node and
	 * link it in.
	 */
	if ((next_node = ac_get_build_node(node, search[index],
			case_sen))== NULL) {

		if ((next_node = ac_create_build_node(ctx, search, index + 1,
				node->level + 1)) == NULL)
				return (NULL);

		next_node->patterns = 1;
		if (ac_add_build_node(ctx, node, next_node, search[index],
				case_sen) != 0)
			return (NULL);
	} else {
		node->patterns++;
	}
	return (next_node);
}


/*
 * Builds a trie from the list of patterns.
 */
static int
ac_build_trie(struct ac2_bld_ctx *ctx, int case_sen)
{
	struct ac2_trie_build *node, *next_node;
	struct ac2_match_data *match, *match_next;
	uint32_t index, size;
	const uint8_t *search;

	/* for each pattern in the list. */
	for (match = ctx->match_list; match != NULL; match = match_next) {

		match_next = match->match_list;

		node = ctx->build_root;
		ctx->build_root->failure_refs++;

		size = match->pmp->len;
		search = match->pmp->pattern;
		next_node = NULL;

		/* for each character in the pattern. */
		for (index = 0, next_node = NULL;
				index < size;
				node = next_node, index++) {
			/*
			 * get the the next node for this character
			 * in the pattern.
			 */
			if ((next_node = ac_trie_next_node(ctx, node, search,
					index, case_sen)) == NULL)
				return (-ENOMEM);
		}

		/*
		 * Now at last node in pattern, add match data to
		 * nodes list of matches.
		 */
		if (size != 0) {
			match->next = NULL;

			if (node->match_end == NULL) {
				node->match_data = match;
			} else {
				node->match_end->next = match;
				match->index = 1;
			}
			node->match_end = match;
		}
	}
	return (0);
}

/*
 * Constructs the failure path link for each node in the trie.
 */
static void
ac_build_nfa(struct ac2_bld_ctx *ctx, int case_sen)
{
	uint32_t n;
	struct ac2_trie_build *node;
	struct ac2_trie_build *root;
	struct ac2_trie_build *string_node;
	struct ac2_search_data s;

	root = ctx->build_root;

	/* for each node in the trie */
	for (node = ctx->build_list; node != NULL; node = node->link) {

		/* Get pattern up to this node. */
		s.size = node->depth;
        	s.search_string = node->pattern;

		/*
		 * Try to find the failure path for this node by
		 * looking for the longest substring that is in the Trie.
		 */
		for (n = 1; n < node->depth; n++) {
			s.index = n;
			string_node = ac_search_build_node(root, &s, case_sen);
			if (string_node != NULL) {
				node->failure = string_node;
				string_node->failure_refs++;
				break;
			}
		}
	}
}

/* Traverses failure path until a node with a match is found. */
static struct ac2_match_data *
ac_get_match_path(struct ac2_trie_build *node)
{
    /* if there's a match or this is the root node. */
    if (node->match_data != NULL || node == node->failure)
        return (node->match_data);
    else
        return (ac_get_match_path(node->failure));
}

/*
 * Links the match chain from a node to matches on all
 * of the nodes on its failure path.
 */
static void
ac_link_match_data(struct ac2_bld_ctx *ctx)
{
	struct ac2_match_data *match_link, *md;
	struct ac2_trie_build *node;
	int num_match_points;

	num_match_points = 1;

	/* For each node in the Trie. */
	for (node = ctx->build_list; node != NULL; node = node->link) {

		if ((md = node->match_data) != NULL) {
			md->match_point = num_match_points++;
			if (md->pmp->len < DIM(ctx->pattern_len_num))
				ctx->pattern_len_num[md->pmp->len]++;
		}

		match_link = ac_get_match_path(node->failure);

		if (match_link != NULL) {
			if (node->match_end == NULL) {
				node->match_data = match_link;
			} else {
				node->match_end->next = match_link;
				match_link->index = 1;
			}
			node->match_end = match_link;
		}
	}
	ctx->num_match_points = num_match_points - 1;
}

/*
 * Traverses match list of the build context and fills match index.
 * This routine places ac2_match_data into an array sorted in list order.
 * Simply find all heads of lists and insert them into the array in list order.
 */
static void
ac_fill_match_index(struct ac2_bld_ctx *ctx, int case_sen)
{
	struct ac2_match_data *match_link, *match;
	struct rte_ac2_match_data *ac_match;
	uint32_t index, n;

	index = 1;
	for (match_link = ctx->match_list; match_link != NULL;
			match_link = match_link->match_list) {

		if (match_link->index != 0)
			continue;

		/*
		 * if index == 0, then this ac2_match_data is at head
		 * of the list
		 */

		for (match = match_link;
				match != NULL && match->allocated == 0;
				match = match->next) {

			match->index = index++;
			match->allocated = 1;

			ac_match = ctx->pmx.ac2.match_index + match->index;
			ac_match->userdata = match->pmp->userdata;
			ac_match->len = match->pmp->len;
			ac_match->match_point = match->match_point;

			for (n = 0;
					n < DIM(ac_match->pattern) &&
					n < match->pmp->len;
					n++)
				ac_match->pattern[n] =
					TO_CASELESS(match->pmp->pattern[n],
						case_sen);

			if (match->next != NULL)
				ac_match->next =
					(match->next->allocated == 0) ?
					index : match->next->index;
		}
	}
}

static void
ac_sum_pattern_len(struct ac2_bld_ctx *ctx)
{
	uint32_t i, sum;

	memcpy(ctx->sum_pattern_len, ctx->pattern_len_num,
		sizeof (ctx->sum_pattern_len));

	sum = 0;
	for (i = 0; i != DIM(ctx->sum_pattern_len); i++) {
		ctx->sum_pattern_len[i] += sum;
		sum += ctx->sum_pattern_len[i];
	}
	
}

static void
ac_get_search_types(struct ac2_bld_ctx *ctx, const struct rte_pm_build_opt *opt,
	struct rte_pm_search_avail *res)
{
	uint32_t i, num_patterns, plen;

	memset(res, 0, sizeof (*res));
	num_patterns = ctx->num_match_points;

	/* Generic case. */
	RTE_PM_SET_BIT(res->avail, RTE_PM_SEARCH_AC2_L1x4);
	res->prefer = RTE_PM_SEARCH_AC2_L1x4;

	/* is MULTIBUFFER suitable? */
	if (opt->out_of_order != 0) {
		RTE_PM_SET_BIT(res->avail, RTE_PM_SEARCH_AC2_L1x4_MB);
		if (ctx->num_nodes[AC2_BLOOM_LEVEL] < AC2_BLOOM_MB_NODES &&
				ctx->sum_pattern_len[AC2_MAX_MB_LEN] != 0)
			res->prefer = RTE_PM_SEARCH_AC2_L1x4_MB;
	}

	/* Is MULTIHASH suitable? */
	for (i = 0; i != DIM(ac2_mh_select); i++) {
		plen = ac2_mh_select[i].min_pattern_len - 1;
		if (ctx->sum_pattern_len[plen] != 0)
			break;
		RTE_PM_SET_BIT(res->avail, ac2_mh_select[i].search_type);
		if (res->prefer != RTE_PM_SEARCH_AC2_L1x4_MB)
			res->prefer = ac2_mh_select[i].search_type;
	}

	if (num_patterns <= AC2_R256x2_MAX_PATTERNS) {
		RTE_PM_SET_BIT(res->avail, RTE_PM_SEARCH_AC2_R256x2);
		res->prefer = RTE_PM_SEARCH_AC2_R256x2;
	}

	if (num_patterns <= AC2_R128x2_MAX_PATTERNS) {
		RTE_PM_SET_BIT(res->avail, RTE_PM_SEARCH_AC2_R128x2);
		res->prefer = RTE_PM_SEARCH_AC2_R128x2;
	}

	if (num_patterns == 1) {
		RTE_PM_SET_BIT(res->avail, RTE_PM_SEARCH_AC2_P1);
		res->prefer = RTE_PM_SEARCH_AC2_P1;
	}
}

/*
 * Creates match list and builds the trie.
 */
static int
ac_compile(struct ac2_bld_ctx *ctx, int case_sen)
{
	int rc;
	uint8_t dummy;

	if ((rc = ac_match_list(ctx)) != 0)
		return (rc);

	dummy = 0;
	 /* Create the root for the build trie. */
	if(ac_create_build_node(ctx, &dummy, 0, 0) == NULL)
		return (-ENOMEM);

	/* Build Trie structure from patterns. */
	if ((rc = ac_build_trie(ctx, case_sen)) != 0)
		return (rc);

	/* Calculate failure paths */
	ac_build_nfa(ctx, case_sen);

	/* For each node, link match data to failure node's match data. */
	ac_link_match_data(ctx);

	return (0);
}

/* Force DFA for any 1 byte repeated up to 64 more levels. */
static void
ac_force_dfa_one_byte(struct ac2_bld_ctx *ctx, int case_sen)
{
	uint8_t n;
	uint32_t m;
	struct ac2_trie_build *p;

	for (n = 0; n != UINT8_MAX; n++) {
		p = ctx->build_root;
		for (m = 0; m < DEPTH1BR; m++) {
			if ((p = ac_get_build_node(p, n, case_sen)) == NULL)
				break;
			p->bloom_type = RTE_AC2_BNODE_TYPE_DFA;
			p->force256 = 1;
		}
	}
}

/*
 * L1x4: Setting mask.
 * mask for L1x4 determinies how many first bits (max 32) of the pattern
 * are used to calculate Bloom hash value.
 * Also same number of bits in input string will be used to apply
 * Bloom filter for. Basically it defines number of bits that will be used
 * to search for possible match. Let say pattern_len(p) == 4B, but
 * mask_len == 20. So only first 20 bits will be used for fast initial
 * (Bloom) comparision. Remaining 12 bits will be just ignored.
 * Ideally for each pattern we would like to have:
 * mask_len(p) == MIN(pattern_len(p), 4).
 * The problem here is that we neeed to have one nMask for all patterns.
 * So setting mask_len to MAX of mask_len(p) for all patterns in the set,
 * will cause a problem: patterns with less amount of bytes will have to
 * generate more values in Bloom array (more bits will be set).
 * Let say we have mask_len == 32 and 1 byte pattern.
 * So for that pattern (as we can't just ignore remaining 3 bytes anymore)
 * we have to generate bloom values for all possible combinations: 'A***'.
 * That means we'll have to generate 2^24 bloom values.
 * That's too expencive (memory usage, time to generate), plus setting all
 * these bits in Bloom array will increase probability of false positive.
 * So to find the compromise value for mask_len the heuristic below is used.
 */
static void
ac_l1x4_gen_param(struct ac2_bld_ctx *ctx)
{
	uint32_t bloom_bits, bloom_max;
	const uint32_t *num = ctx->pattern_len_num;

	if (ctx->sum_pattern_len[3] == 0)
		ctx->mask_len = 32;
	else if (num[1] * 0x100000 + num[2] * 0x800 + num[3] * 0x10 < MAX_ADDS)
		ctx->mask_len = 28;
	else if (num[1] * 0x10000 + num[2] * 0x100 < MAX_ADDS)
		ctx->mask_len = 24;
	else if (num[1] * 0x800 + num[2] * 0x10 < MAX_ADDS)
		ctx->mask_len = 20;
    	else
		ctx->mask_len = 18;

	ctx->mask = LEN2MASK(ctx->mask_len);

	bloom_max = (ctx->num_nodes[AC2_BLOOM_LEVEL] + 1) * AC2_BLOOM_RATIO;
	bloom_max = RTE_MIN((uint32_t)AC2_BLOOM_MAX - 1, bloom_max);

	for (bloom_bits = AC2_BLOOM_BITS;
			bloom_bits < bloom_max;
			bloom_bits = bloom_bits * 2 + 1)
		;

	ctx->bloom_bits = bloom_bits;
	ctx->bloom_size = RTE_ALIGN(bloom_bits, CHAR_BIT);

	ctx->stateless_mask = -1 << 3;
}

static void
ac_r256x2_gen_param(struct ac2_bld_ctx *ctx)
{
	ctx->min_pattern_len = 2;
	ctx->bloom_bits = UINT8_MAX;
	ctx->bloom_size = 32;
	ctx->stateless_mask = -1 << 1;
	ctx->mask_len = 16;
	ctx->mask = LEN2MASK(ctx->mask_len);
}

static void
ac_r128x2_gen_param(struct ac2_bld_ctx *ctx)
{
	ctx->min_pattern_len = 2;
	ctx->bloom_bits = INT8_MAX;
	ctx->bloom_size = 16;
	ctx->stateless_mask = -1 << 1;
	ctx->mask_len = 16;
	ctx->mask = LEN2MASK(ctx->mask_len);
}

static void
ac_p1_gen_param(struct ac2_bld_ctx *ctx)
{
	uint32_t n;

	ctx->bloom_bits = INT8_MAX;
	ctx->bloom_size = 16;
	ctx->stateless_mask = -1;

	/*
	 * MASK_BITS = MIN(min_patterns_length, 4) * 8, e.g:
	 * if all patterns have length >=4, then MASK_BITS = 32.
	 * else if all patterns have length >= 3, then MASK_BITS = 24,
	 * else if all patterns have length >= 2, then MASK_BITS = 16,
	 * else  MASK_BITS = 8.
	 */

	for (n = 1; n != 4 && ctx->pattern_len_num[n] == 0; n++)
		;

	ctx->mask_len = n * CHAR_BIT;
	ctx->mask = LEN2MASK(ctx->mask_len);
}

/*
 * Sets the following fields in build context:
 *  - min_pattern_len
 *  - bloom_bits
 *  - bloom_size
 *  - stateless_mask
 *  - mask_len
 *  - mask
 *  (L1x4* only):
 *  - shift_max 
 *  - shift1
 *  - shift2
 *  - shift3
 */
static void
ac_calc_gen_param(struct ac2_bld_ctx *ctx)
{
	enum rte_pm_search st;
	uint32_t i;

	ctx->min_pattern_len = 1;

	st = ctx->pmx.bopt.search_type;
	if (st == RTE_PM_SEARCH_AC2_P1) {
		ac_p1_gen_param(ctx);
	} else if (st == RTE_PM_SEARCH_AC2_R128x2) {
		ac_r128x2_gen_param(ctx);
	} else if (st == RTE_PM_SEARCH_AC2_R256x2) {
		ac_r256x2_gen_param(ctx);
	} else {
		ac_l1x4_gen_param(ctx);

		ctx->shift_max = 1;
		for (i = 0; i != DIM(ac2_mh_select); i++) {
			if (ac2_mh_select[i].search_type == st) {
				ctx->shift_max = ac2_mh_select[i].shift_max;
				ctx->pmx.ac2.shift1 = ac2_mh_select[i].shift1;
				ctx->pmx.ac2.shift2 = ac2_mh_select[i].shift2;
				ctx->pmx.ac2.shift3 = ac2_mh_select[i].shift3;
				break;
			}
		}
	}
}

static int
ac_exclude_nodes(struct ac2_trie_build *node, int min_level, int min_fail,
	int include)
{
	int n, include_cnt;

	include_cnt = 0;
	if (node->level >= min_level && node->level >= min_fail)
		return (include_cnt);

	include |= (node->match_data != NULL);
	if (include == 0) {
		if (node->level < min_level)
			node->exclude |= AC2_EXCLUDE_NODE;

		if (node->level < min_fail)
			node->exclude |= AC2_FAIL_NODE;

        } else if (node->level < min_level) {
                include_cnt++;
        }

        for (n = 0; n < node->num_char; n++)
		include_cnt += ac_exclude_nodes(node->next_trie[n], min_level,
			min_fail, include);

	return (include_cnt);
}

static void
ac_set_seq_counts(ac2_nodes_count_t counts, uint32_t started)
{
	counts[AC2_NODES_SEQ11] += (started + RTE_AC2_BLOOM_SEQ32_NUM - 1) /
		RTE_AC2_BLOOM_SEQ32_NUM;
}

static uint32_t
ac_count_sequential(struct ac2_trie_build *node, ac2_nodes_count_t counts,
	uint32_t started)
{
	if ((node->exclude & AC2_EXCLUDE_NODE) == 0) {

		counts[AC2_NODES_TOTAL]++;

		if (node->match_data != NULL) {
			counts[AC2_NODES_MATCH]++;

			/* matches will get separate nodes. */
			if (node->num_char != 0)
				counts[AC2_NODES_TOTAL]++;
		}

		if (node->bloom_type == RTE_AC2_BNODE_TYPE_SEQ) {
			counts[AC2_NODES_SEQ]++;
			if (!started || node->match_data != NULL ||
					node->failure_refs != 0) {
				counts[AC2_NODES_START]++;
				node->start = counts[AC2_NODES_START];
				ac_set_seq_counts(counts, started);
                		started = 0;
            		}
			started++;

		} else if (node->bloom_type == RTE_AC2_BNODE_TYPE_FANOUT) {
			counts[AC2_NODES_FAN11]++;
		} else {
			counts[AC2_NODES_FANOVER11]++;
		}
	}

	if (node->bloom_type != RTE_AC2_BNODE_TYPE_SEQ) {
		ac_set_seq_counts(counts, started);
		started = 0;
	}

	/* 1, 2, 3, ... for 1st 2nd 3rd node in sequential list. */
	node->sequential = started;

	return (started);
}

static uint32_t
ac_set_node_types(struct ac2_trie_build *node, int force_level, int failures,
	int matches, ac2_nodes_count_t counts, uint32_t started)
{
	struct ac2_trie_build *p;
	uint32_t force_cnt;
	int n;

	force_cnt = 0;
	if ((node->exclude & AC2_EXCLUDE_NODE) == 0 && node->force256 == 0) {

		/*
		 * Force DFA for any node at a level below FORCE256.
		 * Force DFA for any node with failure references.
		 * Force DFA for any node referenced by a match node.
		 */
		if (node->num_char > 0 &&
				(node->level < force_level ||
				(node->failure_refs > 0 && failures != 0) ||
				(node->match_data != NULL && matches != 0))) {

			// Don't count those below the min level
			if (node->level >= force_level)
				force_cnt++;

			node->bloom_type = RTE_AC2_BNODE_TYPE_DFA;
			node->force256 = 1;

        	} else if (node->num_char == 0)  {
			if((node->failure->exclude & AC2_EXCLUDE_NODE) == 0 &&
					matches != 0) {

				for (p = node->failure;
						p->num_char == 0;
						p = p->failure)
					;

				if (p->bloom_type != RTE_AC2_BNODE_TYPE_DFA)
					force_cnt++;

				p->bloom_type = RTE_AC2_BNODE_TYPE_DFA;
				p->force256 = 1;
			}
			node->bloom_type = RTE_AC2_BNODE_TYPE_MATCH;

		} else if (node->num_char == 1) {
			node->bloom_type = RTE_AC2_BNODE_TYPE_SEQ;

		/* if FANOUT exceeds a FANOUT node then its a DFA. */
		} else if (node->num_char > RTE_AC2_BLOOM_FANOUT32_NUM) {
			node->bloom_type = RTE_AC2_BNODE_TYPE_DFA;

		/* Must be a FANOUT. */
		} else {
			node->bloom_type = RTE_AC2_BNODE_TYPE_FANOUT;
		}
	}

	started = ac_count_sequential(node, counts, started);

	for (n = 0; n < node->num_char; n++)
		force_cnt += ac_set_node_types(node->next_trie[n], force_level,
			failures, matches, counts, started);

	return (force_cnt);
}

/*
 * allocates and sets expected indexes for the trie nodes
 * in the future transision table and return maximum allocated index
 */
static uint32_t
ac_alloc_sequential32(struct ac2_trie_build *node, uint32_t alloc)
{
	int n, type;
	struct ac2_trie_build *p;

	if ((node->exclude & AC2_EXCLUDE_NODE) == 0) {

		node->bloom_position = alloc | node->bloom_type;

		if (node->match_data != NULL) {
            		node->bloom_position = alloc | RTE_AC2_BNODE_TYPE_MATCH;
            		alloc += RTE_AC2_BLOOM_MATCH32_SIZE;
        	}

        	node->bloom_failure_position = alloc | node->bloom_type;

		if ((type = node->bloom_type) == RTE_AC2_BNODE_TYPE_MATCH) {
			node->bloom_failure_position = 0;
		} else if (type == RTE_AC2_BNODE_TYPE_DFA) {
			alloc += RTE_AC2_BLOOM_DFA32_SIZE;
		} else if (type == RTE_AC2_BNODE_TYPE_FANOUT) { 
			alloc += RTE_AC2_BLOOM_FANOUT32_SIZE;
		} else if (type == RTE_AC2_BNODE_TYPE_SEQ &&
				node->sequential == 1 &&
				node->num_char == 1) {

			for (p = node, n = 1;
					p->sequential + 1 ==
					p->next_trie[0]->sequential;
					n++, p = p->next_trie[0])
				;
			
			alloc += (n + RTE_AC2_BLOOM_SEQ32_NUM - 1) /
				RTE_AC2_BLOOM_SEQ32_NUM *
				RTE_AC2_BLOOM_SEQ32_SIZE;
            	}
	}

	for (n = 0; n < node->num_char; n++)
        	alloc = ac_alloc_sequential32(node->next_trie[n], alloc);

	return (alloc);
}

/*
 * allocates space from RTE shared memory fori runtime used search data:
 * - match_index
 * - bloom_l3
 * - bloom_trie32   
 * - bloom
 */
static int
ac_gen_mem_alloc(struct ac2_bld_ctx *ctx)
{
	uint64_t sz, ofs_l3, ofs_trie32, ofs_bloom;
	uint8_t *ptr;

	/* match_index */
	sz = (ctx->match_index + 1) * sizeof (*ctx->pmx.ac2.match_index);

	/* bloom_l3 */
	ofs_l3 = sz;
	sz += AC2_L3_ESCAPE_NUM * sizeof (*ctx->pmx.ac2.bloom_l3);

	/* bloom_trie32 */
	ofs_trie32 = sz;
	sz += ctx->max_alloc_index * sizeof (*ctx->pmx.ac2.bloom_trie32);

	/* bloom */
	ofs_bloom = sz;
	sz += RTE_ALIGN(ctx->bloom_size, CACHE_LINE_SIZE);

	RTE_LOG(DEBUG, PMAC, "CTX \"%s\" runtime memory footprint:\n"
		"match indexes: %" PRIu64 " bytes\n"
		"L3 escapes: %" PRIu64 " bytes\n"
		"transition table: %" PRIu64 " bytes\n"
		"bloom table: %" PRIu64 " bytes\n"
		"total: %" PRIu64 " bytes\n",
		ctx->pmx.name, ofs_l3, ofs_trie32 - ofs_l3,
		ofs_bloom - ofs_trie32, sz - ofs_bloom, sz);

	if (sz > SIZE_MAX ||
			(ptr = rte_zmalloc_socket(ctx->pmx.name, sz,
				CACHE_LINE_SIZE, ctx->pmx.socket_id)) == NULL) {
		 RTE_LOG(ERR, PMAC,
			"allocation of %" PRIu64 " bytes on socket %d "
			"for %s failed\n",
                        sz, ctx->pmx.socket_id, ctx->pmx.name);
		return (-ENOMEM);
	}

	ctx->pmx.ac2.match_index = (struct rte_ac2_match_data *)ptr;
	ctx->pmx.ac2.bloom_l3 = (rte_ac2_index_t *)(ptr + ofs_l3);
	ctx->pmx.ac2.bloom_trie32 = (rte_ac2_index_t *)(ptr + ofs_trie32);
	ctx->pmx.ac2.bloom = ptr + ofs_bloom;
	ctx->pmx.ac2.mem = ptr;

	ctx->pmx.ac2.ofs_match = 0;
	ctx->pmx.ac2.ofs_l3 = ofs_l3;
	ctx->pmx.ac2.ofs_trie32 = ofs_trie32;
	ctx->pmx.ac2.ofs_bloom = ofs_bloom;
	ctx->pmx.ac2.mem_sz = sz;

	return (0);
}

static void
ac_resolve_failures(struct ac2_trie_build *node, int recurse)
{
	int n;

	if (node->bloom_failure_position == 0) {
		if ((node->exclude & AC2_EXCLUDE_NODE) == 0) {
			ac_resolve_failures(node->failure, 0);
			node->bloom_failure_position =
				node->failure->bloom_failure_position;
		} else {
			node->bloom_failure_position =
				RTE_AC2_EMPTY_LEVEL(node->level);
		}
	}

	if (recurse == 0)
		return;

	for (n = 0; n < node->num_char; n++)
		ac_resolve_failures(node->next_trie[n], recurse);
}

static rte_ac2_index_t
ac_get_bloom_index_dfa32(rte_ac2_index_t *trie,
	const struct ac2_trie_build * node, uint8_t index, int case_sen)
{
	struct ac2_trie_build *next_node;
	rte_ac2_index_t ret;

	/* Get the 'forward' node. */
	if ((next_node = ac_get_build_node(node, index, case_sen)) != NULL) {

		ret = ((next_node->exclude & AC2_EXCLUDE_NODE) != 0) ?
			RTE_AC2_EMPTY_LEVEL(next_node->failure->level) :
			next_node->bloom_position; 

	/* get the 'failure' path node */
	} else if (node->failure->exclude != 0) {
		ret = RTE_AC2_EMPTY_LEVEL(node->failure->level);
	} else if (node == node->failure) {
		ret = node->bloom_failure_position;
	} else {
		ret = ac_get_bloom_index32(trie, node->failure,
			index, case_sen);
	}
	return (ret);
}

static rte_ac2_index_t
ac_get_bloom_index32(rte_ac2_index_t *trie, const struct ac2_trie_build *node,
	uint8_t index, int case_sen)
{
	rte_ac2_index_t i;
	struct rte_ac2_bloom_dfa32 *pd;
	struct ac2_trie_build *next_node;

	i = RTE_AC2_GET_INDEX(node->bloom_failure_position);

	if (node->bloom_type == RTE_AC2_BNODE_TYPE_DFA) {

		pd = (struct rte_ac2_bloom_dfa32 *)(trie + i);
		if (pd->index[index] == 0)
			pd->index[index] = ac_get_bloom_index_dfa32(trie, node,
				index, case_sen);

		return (pd->index[index]);

	/* Resolve link for non-DFA node. */
	} else  if ((next_node = ac_get_build_node(node, index,
			case_sen)) != NULL) {
		return (next_node->bloom_position);

	} else if (node->failure->exclude != 0) {
		return (RTE_AC2_EMPTY_LEVEL(node->failure->level));
	} else {
            return (ac_get_bloom_index32(trie, node->failure, index, case_sen));
        }
}

static rte_ac2_index_t
ac_resolve_match(const struct ac2_trie_build *node,
	struct rte_ac2_bloom_match32 *match)
{
	if (node->num_char > 0) {
		match->index = node->bloom_failure_position;
	} else if (node->match_data->pmp->len == 1) {
		/* Allow 1 byte patterns to reference root node. */
		match->index = node->failure->bloom_position;
	} else {
		match->index = (node->failure->exclude != 0) ?
			RTE_AC2_EMPTY_LEVEL(node->failure->level) :
			node->failure->bloom_failure_position;
	}
	match->match_index = node->match_data->index;
	match->match_point = node->match_data->match_point;
	match->match_offset = (node->match_data->pmp->len == 1) ? -1 : 0;
	match->level = node->level;
	return (RTE_AC2_GET_INDEX(node->bloom_failure_position));	
}

static void
ac_resolve_dfa(const struct ac2_trie_build *node, rte_ac2_index_t *trie,
	struct rte_ac2_bloom_dfa32 *dfa, int case_sen)
{
	int n;

	for (n = 0; n != DIM(dfa->index); n++)
		dfa->index[n] = ac_get_bloom_index32(trie, node, (uint8_t)n,
			case_sen);

	dfa->level = node->level;
}

static void
ac_resolve_fanout(const struct ac2_trie_build *node,
	struct rte_ac2_bloom_fanout32 *fan, int case_sen)
{
	int n;
	uint8_t c;

	fan->level = node->level;
	fan->mask = 1 << (node->num_char * 2);

	for (n = 0; n < node->num_char; n++) {
		c = node->p_match_char[n];
		fan->string[n * 2] = c;
		fan->string[n * 2 + 1] = TO_OTHER_CASE(c, case_sen);
		fan->index[n] = node->next_trie[n]->bloom_position;
	}

	for (; n < RTE_AC2_BLOOM_FANOUT32_NUM; n++) {
		fan->string[n * 2] = fan->string[0];
		fan->string[n * 2 + 1] = fan->string[1];
	}

	fan->index[n] = (node->failure->exclude != 0) ?
		RTE_AC2_EMPTY_LEVEL(node->failure->level) :
		node->failure->bloom_failure_position;
}

static void
ac_resolve_seq(const struct ac2_trie_build *node,
	struct rte_ac2_bloom_seq32 *ps, int case_sen)
{
	rte_ac2_index_t index;
	int n, nseq;
	uint8_t c;

	if (node->sequential != 1 || node->num_char != 1)
		return;

	nseq = 0;
	index = node->bloom_failure_position;

	while (node->sequential == nseq + 1) {

		ps->level = node->level;
		for (n = 0;
				n < RTE_AC2_BLOOM_SEQ32_NUM &&
				node->sequential == nseq + 1;
				n++) {

			nseq = node->sequential;
			c = node->p_match_char[0];

			ps->string[n] = c;
			ps->string[n + RTE_AC2_BLOOM_SEQ32_NUM] =
				TO_OTHER_CASE(c, case_sen);

			ps->index[n] = (node->failure->exclude != 0) ?
				RTE_AC2_EMPTY_LEVEL(node->failure->level) :
				node->failure->bloom_failure_position;
			ps->index[n + 1] = node->next_trie[0]->bloom_position;

			node = node->next_trie[0];
		}

		index += RTE_AC2_BLOOM_SEQ32_SIZE;
		if (node->sequential == nseq + 1)
			ps->index[RTE_AC2_BLOOM_SEQ32_NUM] = index;

		ps->mask = (1 << n) - 1;
		ps++;
	}
}

static void
ac_resolve_sequential32(const struct ac2_trie_build *node,
	rte_ac2_index_t *trie, int case_sen)
{
	rte_ac2_index_t index;
	struct rte_ac2_bloom_match32 *match;
	struct rte_ac2_bloom_dfa32 *dfa;
	struct rte_ac2_bloom_fanout32 *fan;
	struct rte_ac2_bloom_seq32 *seq;
	int bt, n;

	if ((node->exclude & AC2_EXCLUDE_NODE) == 0) {

		index = RTE_AC2_GET_INDEX(node->bloom_position);
		if (node->match_data != NULL) {
			match = (struct rte_ac2_bloom_match32 *)(trie + index);
			index = ac_resolve_match(node, match);
		}

		if ((bt = node->bloom_type) == RTE_AC2_BNODE_TYPE_DFA) {
			dfa = (struct rte_ac2_bloom_dfa32 *)(trie + index);
			ac_resolve_dfa(node, trie, dfa, case_sen);
		} else if (bt ==  RTE_AC2_BNODE_TYPE_FANOUT) {
			fan = (struct rte_ac2_bloom_fanout32 *)(trie + index);
			ac_resolve_fanout(node, fan, case_sen);
		} else if (bt == RTE_AC2_BNODE_TYPE_SEQ) {
			seq = (struct rte_ac2_bloom_seq32 *)(trie + index);
			ac_resolve_seq(node, seq, case_sen);
		}
	}

	for (n = 0; n < node->num_char; n++)
		ac_resolve_sequential32(node->next_trie[n], trie, case_sen);
}

/*
 * Populates Bloom L3 Escapes array with transition table indexes.
 * Note that for 1 byte pattern we need to set indexes for all possible
 * 256 combinations, e.g. for pattern: "A" we'll set indexes for 'A*'.
 */
static void
ac_set_l3(struct ac2_bld_ctx *ctx, struct rte_ac2_match_data *match,
	rte_ac2_index_t index, int case_sen)
{
	uint32_t k, end, val0, val1;

	val0 = match->pattern[0];

	if (match->len == 1) {
		val1 = 0;
		end = UINT8_MAX + 1;
	} else {
		val1 = match->pattern[1];
		end = val1 + 1;
	}

	while (val1 < end) {

		/* for the pattern with first 2 bytes: 'xy': */

		/* set bloom_l3 for 'xy' */
		k = val0 | val1 << CHAR_BIT;
		ctx->pmx.ac2.bloom_l3[k] = index;

		/* set bloom_l3 for 'Xy' */
		k = TO_OTHER_CASE(val0, case_sen) | val1 << CHAR_BIT;
		ctx->pmx.ac2.bloom_l3[k] = index;

		/* set bloom_l3 for 'xY' */
		k = val0 | TO_OTHER_CASE(val1, case_sen) << CHAR_BIT;
		ctx->pmx.ac2.bloom_l3[k] = index;

		/* set bloom_l3 for 'XY' */
		k = TO_OTHER_CASE(val0, case_sen);
		k |= TO_OTHER_CASE(val1, case_sen) << CHAR_BIT;
			
		ctx->pmx.ac2.bloom_l3[k] = index;

		val1++;
	}
}

static void
ac_fill_l3(struct ac2_bld_ctx *ctx, int case_sen)
{
	uint16_t k;
	uint32_t i;
	rte_ac2_index_t index;
	struct ac2_trie_build *node, *node1;
	struct rte_ac2_match_data *match, *pm;

	/* For each match in the set. */
	for (i = 0; i <= ctx->match_index; i++) {
		match = ctx->pmx.ac2.match_index + i;

		/* Ignore duplicate patterns. */
		if (match->match_point == 0)
			continue;

		k = (uint16_t)(match->pattern[0] |
			match->pattern[1] << CHAR_BIT);

		/* Get node for first byte of the pattern. */
		if ((node = ac_get_build_node(ctx->build_root,
				match->pattern[0], case_sen)) == NULL)
			continue;

		index = ctx->pmx.ac2.bloom_l3[k];
		if (index == 0 && match->len != 1) {

			/* Get node for second byte of the pattern. */
			if ((node1 = ac_get_build_node(node, match->pattern[1],
					case_sen)) != NULL)
				node = node1;

			ac_set_l3(ctx, match, node->bloom_position, case_sen);

		/* update L3 table for single byte pattern. */
		} else if (match->len == 1) {
			pm = ctx->pmx.ac2.match_index + node->match_data->index;
			ac_set_l3(ctx, pm, node->bloom_position, case_sen);
		}
	}
}

static uint32_t
ac_calc_bloom_index4(uint32_t val)
{
	sse_t index;

	index = MM_CVT(val);
	index = HASH32(index, mm_prime_word.m);
	return (MM_CVT32(index));
}

static void
ac_bloom_r128(struct ac2_bld_ctx *ctx, struct rte_ac2_match_data *match)
{
	mmreg_t pattern;
	sse_t temp;
	uint32_t *p;
	uint32_t bpos, bval;
	uint8_t c;

	if (match->len < ctx->min_pattern_len)
		return;

	p = (uint32_t *)match->pattern;
	temp = MM_CVT(rte_le_to_cpu_32(p[0]));
	temp = MM_ADD16(temp, mm_prime_byte.m);
	temp = MM_MADD8(temp, mm_prime_byte.m);
	MM_STORE(&pattern.m, temp);

	c = pattern.ui8[0];
	bpos = (ctx->bloom_size - 1) & (c >> 3);
	bval = ctx->pmx.ac2.bloom[bpos] | 1 << (c & 7);
	ctx->pmx.ac2.bloom[bpos] = (uint8_t)bval;

	c = pattern.ui8[1];
	bpos = (ctx->bloom_size - 1) & (c >> 3);
	bval = ctx->pmx.ac2.bloom[bpos] | 1 << (c & 7);
	ctx->pmx.ac2.bloom[bpos] = (uint8_t)bval;
}

/*
 * Fill Bloom filter array.
 * For L1x4 we have:
 * k1 = 1 << (P[0] & 7)
 * k2 = 1 << (P[1] & 7)
 * bloom[(hash(P)] |= (k1 | k2);
 */
static void
ac_bloom_l1x4(struct ac2_bld_ctx *ctx, struct rte_ac2_match_data *match)
{
	uint32_t b, i, k, shift;
	uint64_t adder, max_adder, val;

	if (match->len < ctx->min_pattern_len)
		return;

	for (i = 1; i != 4 && i != match->len; i++)
		;

	max_adder = ((uint64_t)1 << i * CHAR_BIT);
	adder = ctx->mask;
	adder = RTE_MIN(max_adder, adder + 1);
		
	for (shift = 0; shift < ctx->shift_max; shift++) {

		val = match->pattern[shift];
		val += match->pattern[shift + 1] << 8;
		val += match->pattern[shift + 2] << 16;
		val += match->pattern[shift + 3] << 24;
		val &= adder - 1;

		while (ctx->mask >= val) {

			/* Calculate position in Bloom array. */
			k = ac_calc_bloom_index4(val);
			k &= (ctx->bloom_bits >> AC2_BLOOM_SHIFT);

			/* Calculate Bloom filter bits. */
			b = 1 << (val & AC2_BLOOM_MASK);
			b |= 1 << ((val >> CHAR_BIT) & AC2_BLOOM_MASK);

			if ((ctx->pmx.ac2.bloom[k] & b) != b) {
				b |= ctx->pmx.ac2.bloom[k];
				ctx->pmx.ac2.bloom[k] = (uint8_t)b;
				ctx->bloom_hits++;
			}
			ctx->bloom_iter++;
			val += adder;
		}
	}
}

static void
ac_fill_bloom(struct ac2_bld_ctx *ctx)
{
	uint32_t i;
	enum rte_pm_search st;
	struct rte_ac2_match_data *match;

	st = ctx->pmx.bopt.search_type;

	/* For each match in the set. */
	for (i = 0; i <= ctx->match_index; i++) {
		match = ctx->pmx.ac2.match_index + i;

		/* Ignore duplicate patterns. */
		if (match->match_point == 0)
			continue;

		if (RTE_PM_SEARCH_AC2_L1x4_FAMILY(st))
			ac_bloom_l1x4(ctx, match);
		else if (st == RTE_PM_SEARCH_AC2_R128x2 ||
				st == RTE_PM_SEARCH_AC2_R256x2)
			ac_bloom_r128(ctx, match);
	}
}

static void
ac_fill_one_byte(struct ac2_bld_ctx *ctx, int case_sen)
{
	uint8_t c;
	uint32_t bpos, bval, i;
	struct rte_ac2_match_data *match;

	/* For each match in the set. */
	for (i = 0; i <= ctx->match_index; i++) {
		match = ctx->pmx.ac2.match_index + i;

		if (match->len >= ctx->min_pattern_len)
			continue;

		c = match->pattern[0];

		if (c & INT8_MIN) {
			bpos = (c & INT8_MAX) >> 3;
			bval = ctx->pmx.ac2.one_byte_high.ui8[bpos] |
				1 << (c & 7);
			ctx->pmx.ac2.one_byte_high.ui8[bpos] = (uint8_t)bval;
		} else {
			bpos = c >> 3;
			bval = ctx->pmx.ac2.one_byte_low.ui8[bpos] |
				1 << (c & 7);
			ctx->pmx.ac2.one_byte_low.ui8[bpos] = (uint8_t)bval;
		}

		c = TO_OTHER_CASE(c, case_sen);

		if (c & INT8_MIN) {
			bpos = (c & INT8_MAX) >> 3;
			bval = ctx->pmx.ac2.one_byte_high.ui8[bpos] |
				1 << (c & 7);
			ctx->pmx.ac2.one_byte_high.ui8[bpos] = (uint8_t)bval;
		} else {
			bpos = c >> 3;
			bval = ctx->pmx.ac2.one_byte_low.ui8[bpos] |
				1 << (c & 7);
			ctx->pmx.ac2.one_byte_low.ui8[bpos] = (uint8_t)bval;
		}
	}
}

static int
ac_gen(struct ac2_bld_ctx *ctx)
{
	uint32_t alloc_idx;
	const uint32_t *first_pattern;
	struct rte_ac2_bloom_dfa32 *dfa;
	int max_build_level, max_fail_level;
	int force_level, force_failures, force_matches;
	int i, rc;

	/* For L1x4_MB all nodes have to be DFA type. */
	if (ctx->pmx.bopt.search_type == RTE_PM_SEARCH_AC2_L1x4_MB) {
		max_build_level = 0;
		max_fail_level = 0;
		force_matches = 1;
		force_failures = 1;
		force_level = INT32_MAX;
	} else {
		max_build_level = AC2_BLOOM_LEVEL - 1;
		max_fail_level = AC2_BLOOM_LEVEL - 1;
		force_matches = AC2_FORCE_MATCHES;
		force_failures = AC2_FORCE_FAILURES;
		force_level = AC2_FORCE_LEVEL;
	}

	/* Start of GEN phase. */
	ac_force_dfa_one_byte(ctx, ctx->pmx.bopt.case_sense);

	ac_calc_gen_param(ctx);

	if (ac_exclude_nodes(ctx->build_root, max_build_level,
			max_fail_level, 0) != 0)
		ctx->build_root->exclude &= ~AC2_EXCLUDE_NODE;

	ctx->num_failure_dfa = ac_set_node_types(ctx->build_root, force_level,
		force_failures, force_matches, ctx->node_counts, 0);

	/* reserve one DFA and one MATCH at the start of transition table. */
	alloc_idx = RTE_AC2_BLOOM_DFA32_SIZE + RTE_AC2_BLOOM_MATCH32_SIZE;

	/* number of required indexes in the transition table. */
	ctx->max_alloc_index = ac_alloc_sequential32(ctx->build_root,
		alloc_idx);

	ac_resolve_failures(ctx->build_root, 1);

	/* allocate space for runtime search structures. */
	if ((rc = ac_gen_mem_alloc(ctx)) != 0)
		return (rc);

	/* fill match_index array. */
	ac_fill_match_index(ctx, ctx->pmx.bopt.case_sense);

	/* fill reserved DFA at the start of transition table. */
	dfa = (struct rte_ac2_bloom_dfa32 *)ctx->pmx.ac2.bloom_trie32;
	for (i = 0; i != DIM(dfa->index); i++)
		dfa->index[i] = RTE_AC2_EMPTY_LEVEL(0);

	/* fill the transition table. */
	ac_resolve_sequential32(ctx->build_root, ctx->pmx.ac2.bloom_trie32,
		ctx->pmx.bopt.case_sense);

	/* fill Bloom L3 escapes table. */
	ac_fill_l3(ctx, ctx->pmx.bopt.case_sense);

	/* fill Bloom filter array. */
	ac_fill_bloom(ctx);

	/* Fill one byte patterns bitmask. */
	ac_fill_one_byte(ctx, ctx->pmx.bopt.case_sense);

	/* Setup  remaining fields. */
	ctx->pmx.ac2.sse_mask = MM_SET1_32(ctx->mask);
	ctx->pmx.ac2.input_mask = MM_SET1_32(ctx->bloom_bits >>
		AC2_BLOOM_SHIFT);

	first_pattern = (const uint32_t *)ctx->pmx.ac2.match_index[1].pattern;
	ctx->pmx.ac2.first_pattern =
		MM_SET1_32(rte_le_to_cpu_32(first_pattern[0]));

	ctx->pmx.ac2.root_index = ctx->build_root->bloom_position;
	return (0);
}

static int
ac_parse(const struct rte_pm_ctx *pmx, const struct rte_pm_build_opt *opt,
	struct ac2_bld_ctx *ctx, struct rte_pm_search_avail *res)
{
	int rc;

	/* Init AC Build Context. */
	memset(ctx, 0, sizeof (*ctx));
	ctx->pmx = *pmx;
	ctx->pool.alignment = AC2_POOL_ALIGN;
	ctx->pool.min_alloc = AC2_POOL_ALLOC_MIN;

	if ((rc = ac_compile(ctx, opt->case_sense)) != 0)
		return (rc);

	ac_sum_pattern_len(ctx);

	/* analyze build trie and build opts and select search type.*/
	ac_get_search_types(ctx, opt, res);

	return (0);
}
	

int
ac_analyze(const struct rte_pm_ctx *pmx, const struct rte_pm_build_opt *opt,
	struct rte_pm_search_avail *res)
{
	int rc;
	struct ac2_bld_ctx ctx;

	rc = ac_parse(pmx, opt, &ctx, res);
	pmac_free_pool(&ctx.pool);
	return (rc);
}

int
ac_build(struct rte_pm_ctx *pmx, const struct rte_pm_build_opt *opt)
{
	int rc;
	enum rte_pm_search st;
	struct ac2_bld_ctx ctx;
	struct rte_pm_search_avail avail;

	do {
		if ((rc = ac_parse(pmx, opt, &ctx, &avail)) != 0)
			break;

		if (avail.prefer == RTE_PM_SEARCH_UNDEF) {
			RTE_LOG(ERR, PMAC, "failed to determine search type "
				"for %s\n", ctx.pmx.name);
			rc = -EINVAL;
			break;
		}
		if ((st = opt->search_type) == RTE_PM_SEARCH_UNDEF)
			st = avail.prefer;

		if (RTE_PM_GET_BIT(avail.avail, st) == 0) {
			RTE_LOG(ERR, PMAC, "search type %u is not supported "
				"for %s\n", st, ctx.pmx.name);
			rc = -EINVAL;
			break;
		}

		ctx.pmx.bopt = *opt;
		ctx.pmx.bopt.search_type = st;

		rc = ac_gen(&ctx);

	} while (0);

	if (rc == 0) {
		pmx->bopt = ctx.pmx.bopt;
		pmx->ac2 = ctx.pmx.ac2;
		rc = ac_set_runtime(pmx, ctx.pattern_len_num[1] != 0);
	}

	pmac_free_pool(&ctx.pool);
	return (rc);
}

void
ac_free(struct rte_pm_ac2 *ctx)
{
	if (ctx->mem != NULL)
		rte_free(ctx->mem);

	memset(ctx, 0, sizeof (*ctx));
}
