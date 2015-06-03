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

#include <rte_acl.h>
#include "pmac_mem.h"
#include "acl.h"

#define	ACL_POOL_ALIGN		8
#define	ACL_POOL_ALLOC_MIN	0x800000

/*
 * Macros used in ipv4vlan heuristics, to decide how many tries to build,
 * and how to distribute rules between the tries.
 */

#define	WILD16_NUM_RULES	750
#define	WILDSRCDST16_PERCENT	0.01
#define	WILDSRC16_PERCENT	0.05
#define	WILDDST16_PERCENT	0.05

#define	WILDSRCDST24_PERCENT	0.10
#define	WILDSRC24_PERCENT	0.49

#define	SRCMSK_PERCENT	0.50

/* number of pointers per alloc */
#define ACL_PTR_ALLOC 32

/* variable for dividing rule sets */
#define NODE_MAX 2500
#define NODE_PERCENTAGE 0.40
#define RULE_PERCENTAGE 0.40

enum {
	WILDSRC0 = 0,
	WILDDST0,
	WILDSRC8,
	WILDDST8,
	WILDSRC16,
	WILDDST16,
	WILDSRC24,
	WILDDST24,
	WILDSRCDST0,
	WILDSRCDST8,
	WILDSRCDST16,
	WILDSRCDST24,
	WILDSRCDST32,
	WILDNUM
};

enum {
    ACL_INTERSECT_NONE = 0,
    ACL_INTERSECT_A = 1,    /* set A is a superset of A and B intersect */
    ACL_INTERSECT_B = 2,    /* set B is a superset of A and B intersect */
    ACL_INTERSECT = 4,	    /* sets A and B intersect */
};

struct acl_mem_block {
	uint32_t			block_size;
	void *				mem_ptr;
};

/* Context for build phase */
struct acl_build_context {
	const struct rte_acl_ctx *acx;
	uint32_t                  node;
	uint32_t                  num_nodes;
	uint32_t                  num_categories;
	uint32_t                  category_mask;
	uint32_t                  num_rules;
	uint32_t                  node_id;
	uint32_t                  src_mask;
	uint32_t                  num_build_rules;
	uint32_t                  num_tries;
	struct pmac_mem_pool      pool;
	struct rte_acl_trie       tries[RTE_ACL_MAX_TRIES];
	struct rte_acl_bld_trie   bld_tries[RTE_ACL_MAX_TRIES];

	/* trie indexes. */
	uint32_t                  default_trie;
	uint32_t                  wildest_trie;
	uint32_t                  dst_port_trie;
	uint32_t                  src_dst_trie;
	uint32_t                  src_trie;
	uint32_t                  dst_trie;

	/* statistics for given rule set. */
	uint32_t                  wild_dst_ports;
	uint32_t                  wild_vlan;
	uint32_t                  wild_src[33];
	uint32_t                  wild_dst[33];
	uint32_t                  wild_src_dst [33][33];
	double                    wild_array[WILDNUM];

	/* memory free lists for nodes and blocks used for node ptrs */
	struct acl_mem_block      blocks[16];
	struct rte_acl_node       *node_free_list;
};
static int acl_merge_trie(struct acl_build_context *context,
	struct rte_acl_node *node_a, struct rte_acl_node *node_b,
	uint32_t level, uint32_t subtree_id, struct rte_acl_node **node_c);

static int acl_merge(struct acl_build_context *context,
	struct rte_acl_node *node_a, struct rte_acl_node *node_b,
	int move, int a_subset, int level);

static void
acl_deref_ptr(struct acl_build_context *context,
              struct rte_acl_node *node, int index);

static void
*acl_build_alloc(struct acl_build_context *context, size_t n, size_t s)
{
	uint32_t m;
	size_t alloc_size = n * s;

	/*
	*	look for memory in free lists
	*/
	for (m=0; m<DIM(context->blocks); m++) {
		if (context->blocks[m].block_size == 
		   alloc_size && context->blocks[m].mem_ptr != NULL) {
			void * p = context->blocks[m].mem_ptr;
			context->blocks[m].mem_ptr = *((void **)p);
			memset(p, 0, alloc_size);
			return p;
		}
	}
	/*
	*	return allocation from memory pool
	*/
	void *p = pmac_alloc(&context->pool, alloc_size);
	return p;
}

/*
*	Free memory blocks (kept in context for reuse)
*/
static void
acl_build_free(struct acl_build_context *context, size_t s, void *p)
{
	uint32_t n;
	for (n=0; n<DIM(context->blocks); n++) {
		if (context->blocks[n].block_size == s) {
			*((void **)p) = context->blocks[n].mem_ptr;
			context->blocks[n].mem_ptr = p;
			return;
		}
	}
	for (n=0; n<DIM(context->blocks); n++) {
		if (context->blocks[n].block_size == 0) {
			context->blocks[n].block_size = s;
			*((void **)p) = NULL;
			context->blocks[n].mem_ptr = p;
			return;
		}
	}
}

/*
 * Allocate and initialize a new node.
 */
static struct rte_acl_node
*acl_alloc_node(struct acl_build_context * context, int level)
{
	struct rte_acl_node *node;
	if (context->node_free_list != NULL) {
		node = context->node_free_list;
		context->node_free_list = node->next;
		memset(node, 0, sizeof(struct rte_acl_node));
	} else
		node = (struct rte_acl_node *) acl_build_alloc(context, sizeof(struct rte_acl_node), 1);
	if (node != NULL) {
		node->num_ptrs = 0;
		node->level = level;
		node->node_type = RTE_ACL_NODE_UNDEFINED;
		node->node_index = RTE_ACL_NODE_UNDEFINED;
		context->num_nodes++;
		node->id = context->node_id++;
	}
	return (node);
}

/*
*	Dereference all nodes to which this node points
*/
static void
acl_free_node(struct acl_build_context *context,
              struct rte_acl_node *node)
{
	uint32_t n;
	if (node->prev != NULL)
		node->prev->next = NULL;
	for (n = 0; n < node->num_ptrs; n++)
		acl_deref_ptr(context, node, n);

	/* free mrt if this is a match node */
	if (node->mrt != NULL)
		acl_build_free(
			context,
			sizeof(struct rte_acl_match_results),
			node->mrt);
	node->mrt = NULL;

	/* free transitions to other nodes */
	if (node->ptrs != NULL)
		acl_build_free(
			context,
			node->max_ptrs * sizeof(struct rte_acl_ptr_set),
			node->ptrs);
	node->ptrs = NULL;

	/* put it on the free list */
	context->num_nodes--;
	node->next = context->node_free_list;
	context->node_free_list = node;
}


/*
 * Include src bitset in dst bitset
 */
static void
acl_include(struct rte_acl_bitset *dst,
            struct rte_acl_bitset *src,
            bits_t mask)
{
	uint32_t n;
	for (n = 0; n < RTE_ACL_BIT_SET_SIZE; n++)
		dst->bits[n] = (dst->bits[n] & mask) | src->bits[n];
}

/*
 *	Set dst to bits of src1 that are not in src2
 */
static int
acl_exclude(struct rte_acl_bitset *dst,
            struct rte_acl_bitset *src1,
            struct rte_acl_bitset *src2)
{
	uint32_t n;
	bits_t all_bits = 0;

	for (n = 0; n < RTE_ACL_BIT_SET_SIZE; n++) {
		dst->bits[n] = src1->bits[n] & ~src2->bits[n];
		all_bits |= dst->bits[n];
	}
	return (all_bits != 0);
}

/*
 * Add a pointer (ptr) to a node.
 */
static int
acl_add_ptr(struct acl_build_context *context,
            struct rte_acl_node *node,
            struct rte_acl_node *ptr,
            struct rte_acl_bitset *bits)
{
	uint32_t n, num_ptrs;
	struct rte_acl_ptr_set *ptrs = NULL;

	/*
	 * If there's already a pointer to the same node, just add to the bitset
	 */
	for (n = 0; n < node->num_ptrs; n++) {
		if (node->ptrs[n].ptr != NULL) {
			if (node->ptrs[n].ptr == ptr) {
				acl_include(&node->ptrs[n].values, bits, -1);
				acl_include(&node->values, bits, -1);
				return (0);
			}
		}
	}

	/* if there's no room for another pointer, make room */
	if (node->num_ptrs >= node->max_ptrs) {
		/* add room for more pointers */
		num_ptrs = node->max_ptrs + ACL_PTR_ALLOC;
		if ((ptrs = acl_build_alloc(context, num_ptrs, 
				sizeof(*ptrs))) == NULL)
			return (-ENOMEM);

		/* copy current points to new memory allocation */
		if (node->ptrs != NULL) {
			memcpy(ptrs, node->ptrs,
				node->num_ptrs * sizeof(*ptrs));
			acl_build_free(context, node->max_ptrs * sizeof(*ptrs),
				node->ptrs);
		}
		node->ptrs = ptrs;
		node->max_ptrs = num_ptrs;
	}

	/* Find available ptr and add a new pointer to this node */
	for (n = node->min_add; n < node->max_ptrs; n++) {
		if (node->ptrs[n].ptr == NULL) {
			node->ptrs[n].ptr = ptr;
			acl_include(&node->ptrs[n].values, bits, 0);
			acl_include(&node->values, bits, -1);
			if (ptr != NULL)
				ptr->ref_count++;
			if (node->num_ptrs <= n)
				node->num_ptrs = n + 1;
			return (0);
		}
	}

	return (0);
}

/*
 * Add a pointer for a range of values
 */
static int
acl_add_ptr_range(struct acl_build_context *context,
                  struct rte_acl_node *root,
                  struct rte_acl_node *node,
                  uint8_t low,
                  uint8_t high)
{
	uint32_t n;
	struct rte_acl_bitset bitset;

	/* clear the bitset values */
	for (n = 0; n < RTE_ACL_BIT_SET_SIZE; n++)
		bitset.bits[n] = 0;

	/* for each bit in range, add bit to set */
	for (n = 0; n < UINT8_MAX + 1; n++)
		if (n >= low && n <= high)
			bitset.bits[n / (sizeof(bits_t) * 8)] |= 
				1 << (n % (sizeof(bits_t) * 8));

	return (acl_add_ptr(context, root, node, &bitset));
}

/*
 * Generate a bitset from a byte value and mask.
 */
static int
acl_gen_mask(struct rte_acl_bitset *bitset, uint32_t value, uint32_t mask)
{
	int range = 0;
	uint32_t n;

	/* clear the bitset values */
	for (n = 0; n < RTE_ACL_BIT_SET_SIZE; n++)
		bitset->bits[n] = 0;

	/* for each bit in value/mask, add bit to set */
	for (n = 0; n < UINT8_MAX + 1; n++) {
		if ((n & mask) == value) {
			range++;
			bitset->bits[n / (sizeof(bits_t) * 8)] |=
				1 << (n % (sizeof(bits_t) * 8));
		}
	}
	return (range);
}

/*
*  Determine how A and B intersect.
*  Determine if A and/or B are supersets of the intersection
*/
static int
acl_intersect_type(struct rte_acl_bitset *a_bits,
                   struct rte_acl_bitset *b_bits,
                   struct rte_acl_bitset *intersect)
{
	uint32_t n;
	bits_t intersect_bits = 0;
	bits_t a_superset = 0;
	bits_t b_superset = 0;

	/* 
	* calculate and store intersection and check if A and/or B have
	* bits outside the intersection (superset)
	*/
	for (n = 0; n < RTE_ACL_BIT_SET_SIZE; n++) {
		intersect->bits[n] = a_bits->bits[n] & b_bits->bits[n];
		a_superset |= a_bits->bits[n] ^ intersect->bits[n];
		b_superset |= b_bits->bits[n] ^ intersect->bits[n];
		intersect_bits |= intersect->bits[n];
	}

	n = (intersect_bits == 0 ? ACL_INTERSECT_NONE : ACL_INTERSECT) |
	    (b_superset == 0 ? 0 : ACL_INTERSECT_B) |
	    (a_superset == 0 ? 0 : ACL_INTERSECT_A);

	return (n);
}

/*
*	Check if all bits in the bitset are on
*/
static int
acl_full(struct rte_acl_node *node)
{
	uint32_t n;
	bits_t all_bits = -1;

	for (n = 0; n < RTE_ACL_BIT_SET_SIZE; n++)
		all_bits &= node->values.bits[n];
	return (all_bits == -1);
}

/*
 * Check if all bits in the bitset are off
 */
static int
acl_empty(struct rte_acl_node *node)
{
	uint32_t n;
	if (node->ref_count == 0) {
		for (n = 0; n < RTE_ACL_BIT_SET_SIZE; n++) {
			if (0 != node->values.bits[n])
				return 0;
		}
		return (1);
	} else {
		return (0);
	}
}

/*
 * Compute intersection of A and B
 * return 1 if there is an intersection else 0.
 */
static int
acl_intersect(struct rte_acl_bitset *a_bits,
              struct rte_acl_bitset *b_bits,
              struct rte_acl_bitset *intersect)
{
	uint32_t n;
	bits_t all_bits = 0;

	for (n = 0; n < RTE_ACL_BIT_SET_SIZE; n++) {
		intersect->bits[n] = a_bits->bits[n] & b_bits->bits[n];
		all_bits |= intersect->bits[n];
	}
	return (all_bits != 0);
}

/*
 * Duplicate a node
 */
static struct rte_acl_node *
acl_dup_node(struct acl_build_context *context, struct rte_acl_node *node)
{
	uint32_t n;
	struct rte_acl_node *next;

	/*
	* if this is not a match node and its already been duplicated
	*  return the duplicate
	*/
	if (node->match_flag == 0 && node->next != NULL)
		return node->next;

	if ((next = acl_alloc_node(context, node->level)) == NULL)
		return (NULL);

	/* allocate the pointers */
	if (node->num_ptrs > 0) {
		next->ptrs = acl_build_alloc(context,
			node->max_ptrs,
			sizeof(struct rte_acl_ptr_set));
		if (next->ptrs == NULL)
			return (NULL);
		next->max_ptrs = node->max_ptrs;
	}

	/* copy over the pointers */
	for (n = 0; n < node->num_ptrs; n++) {
		if (node->ptrs[n].ptr != NULL) {
			next->ptrs[n].ptr = node->ptrs[n].ptr;
			next->ptrs[n].ptr->ref_count++;
			acl_include(&next->ptrs[n].values,
			            &node->ptrs[n].values, -1);
		}
	}

	next->num_ptrs = node->num_ptrs;

	/* copy over node's match results */
	if (node->match_flag == 0)
		next->match_flag = 0;
	else {
		next->match_flag = -1;
		next->mrt = (struct rte_acl_match_results *)
			acl_build_alloc(context, 1, sizeof(struct rte_acl_match_results));
		memcpy(next->mrt, node->mrt, sizeof(struct rte_acl_match_results));
	}

	/* copy over node's bitset */
	acl_include(&next->values, &node->values, -1);

	node->next = next;
	next->prev = node;

	return (next);
}

/*
 * Dereference a pointer from a node
 */
static void
acl_deref_ptr(struct acl_build_context *context,
              struct rte_acl_node *node, int index)
{
	struct rte_acl_node *ref_node;

	/* De-reference the node at the specified pointer */
	if (node != NULL && node->ptrs[index].ptr != NULL) {
		ref_node = node->ptrs[index].ptr;
		ref_node->ref_count--;
		if (ref_node->ref_count == 0)
			acl_free_node(context, ref_node);
	}
}

/*
 * Exclude bitset from a node pointer
 * returns  0 if poiter was deref'd
 *          1 otherwise
 */
static int
acl_exclude_ptr(struct acl_build_context *context,
                struct rte_acl_node *node,
                int index,
                struct rte_acl_bitset *b_bits)
{
	int retval = 1;
	/*
	 * remove bitset from node pointer and deref
	 * if the bitset becomes empty.
	 */
	if (!acl_exclude(&node->ptrs[index].values,
	                 &node->ptrs[index].values,
	                 b_bits)) {
		acl_deref_ptr(context, node, index);
		node->ptrs[index].ptr = NULL;
		retval = 0;
	}

	/* exclude bits from the composite bits for the node */
	acl_exclude(&node->values, &node->values, b_bits);
	return retval;
}

/*
 * Remove a bitset from src ptr and move remaining ptr to dst
 */
static int
acl_move_ptr(struct acl_build_context *context,
             struct rte_acl_node *dst,
             struct rte_acl_node *src,
             int index,
             struct rte_acl_bitset *b_bits)
{
	int rc;

	if (b_bits != NULL)
		if (!acl_exclude_ptr(context, src, index, b_bits))
			return (0);

	/* add src pointer to dst node */
	if ((rc = acl_add_ptr(context, dst, src->ptrs[index].ptr,
			&src->ptrs[index].values)) < 0)
		return (rc);

	/* remove ptr from src */
	acl_exclude_ptr(context, src, index, &src->ptrs[index].values);
	return (1);
}

/*
 * acl_exclude rte_acl_bitset from src and copy remaining pointer to dst
 */
static int
acl_copy_ptr(struct acl_build_context *context,
             struct rte_acl_node *dst,
             struct rte_acl_node *src,
             int index,
             struct rte_acl_bitset *b_bits)
{
	int rc;
	struct rte_acl_bitset bits;

	if (b_bits != NULL)
		if (!acl_exclude(&bits, &src->ptrs[index].values, b_bits))
			return (0);

	if ((rc = acl_add_ptr(context, dst, src->ptrs[index].ptr, &bits)) < 0)
		return (rc);
	return (1);
}

/*
 * Fill in gaps in ptrs list with the ptr at the end of the list
 */
static void
acl_compact_node_ptrs(struct rte_acl_node *node_a)
{
	uint32_t n;
	int min_add = node_a->min_add;

	while (node_a->num_ptrs > 0  &&
	        node_a->ptrs[node_a->num_ptrs - 1].ptr == NULL)
		node_a->num_ptrs--;

	for (n = min_add; n + 1 < node_a->num_ptrs; n++) {

		/* if this entry is empty */
		if (node_a->ptrs[n].ptr == NULL) {

			/* move the last pointer to this entry */
			acl_include(&node_a->ptrs[n].values,
			            &node_a->ptrs[node_a->num_ptrs - 1].values,
			            0);
			node_a->ptrs[n].ptr =
				node_a->ptrs[node_a->num_ptrs - 1].ptr;

			/*
			 * mark the end as empty and adjust the number
			 * of used pointer enum_tries
			 */
			node_a->ptrs[node_a->num_ptrs - 1].ptr = NULL;
			while (node_a->num_ptrs > 0  &&
			        node_a->ptrs[node_a->num_ptrs - 1].ptr == NULL)
				node_a->num_ptrs--;
		}
	}
}

/*
 * acl_merge helper routine.
 */
static int
acl_merge_intersect(struct acl_build_context *context,
	struct rte_acl_node *node_a, uint32_t idx_a,
	struct rte_acl_node *node_b, uint32_t idx_b,
	int next_move, int level,
	struct rte_acl_bitset *intersect_ptr)
{
	struct rte_acl_node *node_c;

	/* Duplicate A for intersection */
	if ((node_c = acl_dup_node(context, node_a->ptrs[idx_a].ptr)) == NULL)
		return (-1);

	/* Remove intersection from A */
	acl_exclude_ptr(context, node_a, idx_a, intersect_ptr);
								                
	/*
	 * Added link from A to C for all transitions
	 * in the intersection
	 */
	if (acl_add_ptr(context, node_a, node_c, intersect_ptr) < 0)
		return (-1);

	/* merge B->node into C */
	return (acl_merge(context, node_c, node_b->ptrs[idx_b].ptr, next_move,
		0, level + 1));
}


/*
 * Merge the children of nodes A and B together
 *
 * if match node
 *	For each category
 *		node A result = highest priority result
 * if any pointers in A intersect with any in B
 *	For each intersection
 *		C = copy of node that A points to
 *		remove intersection from A pointer
 *		add a pointer to A that points to C for the intersection
 *		Merge C and node that B points to
 * Compact the pointers in A and B
 * if move flag
 *	If B has only one reference
 *		Move B pointers to A
 *	else
 *		Copy B pointers to A
 */
static int
acl_merge(struct acl_build_context *context,
	struct rte_acl_node *node_a, struct rte_acl_node *node_b,
	int move, int a_subset, int level)
{
	uint32_t n, m, ptrs_a, ptrs_b;;
	uint32_t min_add_a, min_add_b;
	int intersect_type;
	int node_intersect_type;
	int b_full, next_move, rc;
	struct rte_acl_bitset intersect_values;
	struct rte_acl_bitset intersect_ptr;

	min_add_a = 0;
	min_add_b = 0;
	intersect_type = 0;
	node_intersect_type = 0;

	if (level == 0)
		a_subset = 1;

	/*
	*  Resolve match priorities
	*/
	if (node_a->match_flag != 0 || node_b->match_flag != 0) {

		if (node_a->match_flag == 0 || node_b->match_flag == 0)
			RTE_LOG(ERR, PMAC, "Not both matches\n");

		if (node_b->match_flag < node_a->match_flag)
			RTE_LOG(ERR, PMAC, "Not same match\n");

		for (n = 0; n < context->num_categories; n++) {
			if (node_a->mrt->priority[n] < node_b->mrt->priority[n]) {
				node_a->mrt->priority[n] =
					node_b->mrt->priority[n];
				node_a->mrt->results[n] = node_b->mrt->results[n];
			}
		}
	}

	/*
	 * If the two node transitions intersect then merge the transitions.
	 * Check intersection for entire node (all pointers)
	 */
	node_intersect_type = acl_intersect_type(&node_a->values,
	                      &node_b->values,
	                      &intersect_values);

	if (node_intersect_type & ACL_INTERSECT) {

		b_full = acl_full(node_b);

		min_add_b = node_b->min_add;
		node_b->min_add = node_b->num_ptrs;
		ptrs_b = node_b->num_ptrs;

		min_add_a = node_a->min_add;
		node_a->min_add = node_a->num_ptrs;
		ptrs_a = node_a->num_ptrs;

		for (n = 0; n < ptrs_a; n++) {
			for (m = 0; m < ptrs_b; m++) {

				if (node_a->ptrs[n].ptr == NULL ||
						node_b->ptrs[m].ptr == NULL ||
						node_a->ptrs[n].ptr ==
						node_b->ptrs[m].ptr)
						continue;

				intersect_type = acl_intersect_type(
					&node_a->ptrs[n].values,
					&node_b->ptrs[m].values,
					&intersect_ptr);

				/* If this node is not a 'match' node */
				if ((intersect_type & ACL_INTERSECT) &&
					(context->num_categories != 1 ||
					!(node_a->ptrs[n].ptr->match_flag))) {

					/*
					 * next merge is a 'move' pointer,
					 * if this one is and B is a 
					 * subset of the intersection.
					 */
					next_move = move &&
						(intersect_type &
						ACL_INTERSECT_B) == 0;
							                 
					if (a_subset && b_full) {
						rc = acl_merge(context,
							node_a->ptrs[n].ptr,
							node_b->ptrs[m].ptr,
							next_move,
							1, level + 1);
						if (rc != 0)
							return (rc);
					} else {
						rc = acl_merge_intersect(
							context, node_a, n,
							node_b, m, next_move,
							level,&intersect_ptr);
						if (rc != 0)
							return (rc);
					}
				}
			}
		}
	}

	/* Compact pointers */
	node_a->min_add = min_add_a;
	acl_compact_node_ptrs(node_a);
	node_b->min_add = min_add_b;
	acl_compact_node_ptrs(node_b);

	/*
	 *  Either COPY or MOVE pointers from B to A
	 */
	acl_intersect(&node_a->values, &node_b->values, &intersect_values);

	if (move && node_b->ref_count == 1) {
		for (m = 0; m < node_b->num_ptrs; m++) {
			if (node_b->ptrs[m].ptr != NULL &&
					acl_move_ptr(context, node_a, node_b, m,
					&intersect_values) < 0)
				return (-1);
		}
	} else {
		for (m = 0; m < node_b->num_ptrs; m++) {
			if (node_b->ptrs[m].ptr != NULL &&
					acl_copy_ptr(context, node_a, node_b, m,
					&intersect_values) < 0)
				return (-1);
		}
	}

	/*
	 *  Free node if its empty (no longer used)
	 */
	if (acl_empty(node_b)) {
		acl_free_node(context, node_b);
	}
	return (0);
}

enum {
	ACL_PRIORITY_EQUAL = 0,
	ACL_PRIORITY_NODE_A = 1,
	ACL_PRIORITY_NODE_B = 2,
	ACL_PRIORITY_MIXED = 3
};

static int
acl_resolve_leaf(struct acl_build_context *context,
          struct rte_acl_node *node_a,
          struct rte_acl_node *node_b,
          struct rte_acl_node **node_c)
{
	uint32_t n;
	int combined_priority = ACL_PRIORITY_EQUAL;
	for (n = 0; n < context->num_categories; n++) {
		if (node_a->mrt->priority[n] != node_b->mrt->priority[n]) {
			combined_priority |= (node_a->mrt->priority[n] > node_b->mrt->priority[n]) ? ACL_PRIORITY_NODE_A : ACL_PRIORITY_NODE_B;
		}
	}

	/* if node a is higher or equal priority for all categories then return node_a */
	if (combined_priority == ACL_PRIORITY_NODE_A || combined_priority == ACL_PRIORITY_EQUAL) {
		*node_c = node_a;
		return 0;
	}
	/* if node b is higher or equal priority for all categories then return node_b */
	if (combined_priority == ACL_PRIORITY_NODE_B) {
		*node_c = node_b;
		return 0;
	}
	/* mixed priorities - create a new node with the highest priority for each category */
	node_a->next = NULL;	// force new duplication
	*node_c = acl_dup_node(context, node_a);
	for (n = 0; n < context->num_categories; n++) {
		if ((*node_c)->mrt->priority[n] < node_b->mrt->priority[n]) {
			(*node_c)->mrt->priority[n] = node_b->mrt->priority[n];
			(*node_c)->mrt->results[n] = node_b->mrt->results[n];
		}
	}
	return 0;
}
/*
* Within the existing trie structure, determine which nodes are
* part of the subtree of the trie to be merged.
*
* For these purposes, a subtree is defined as the set of nodes that
* are 1) not a superset of the intersection with the same level of
* the merging tree, and 2) do not have any references from a node
* outside of the subtree.
*/
static void
mark_subtree(struct rte_acl_node *node, 
	struct rte_acl_bitset * level_bits, 
	uint32_t level, 
	uint32_t id)
{
	uint32_t n;
	/* mark this node as part of the subtree */
	node->subtree_id = id | RTE_ACL_SUBTREE_NODE;

	for (n = 0; n < node->num_ptrs; n++) {
		if (node->ptrs[n].ptr != NULL) {
			struct rte_acl_bitset intersect_bits;
			/*
			* Item 1) :
			* check if this child pointer is not a superset of the
			* same level of the merging tree. 
			*/
			int intersect = acl_intersect_type(&node->ptrs[n].values, 
				&level_bits[level], 
				&intersect_bits);
			if ((intersect & ACL_INTERSECT_A) == 0 ) {
				struct rte_acl_node *child = node->ptrs[n].ptr;
				/* 
				* reset subtree reference if this is
				* the first visit by this subtree.
				*/
				if (child->subtree_id != id) {
					child->subtree_id = id;
					child->subtree_ref_count = 0;
				}
				/* 
				* Item 2) :
				* increment the subtree reference count and if
				* all references are from this subtree then
				* recurse to that child
				*/
				child->subtree_ref_count++;
				if (child->subtree_ref_count == child->ref_count)
					mark_subtree(child, level_bits, level+1, id);
			} 
		}
	}
}
/*
* Build the set of bits that define the set of transitions
* for each level of a trie.
*/
static void
build_subset_mask(
	struct rte_acl_node *node, 
	struct rte_acl_bitset * level_bits, 
	int level)
{
	uint32_t n;
	/* Add this node's transitions to the set for this level */
	for (n = 0; n < RTE_ACL_BIT_SET_SIZE; n++)
		level_bits[level].bits[n] &= node->values.bits[n];

	/* For each child, add the transitions for the next level */
	for (n = 0; n < node->num_ptrs; n++) 
		if (node->ptrs[n].ptr != NULL) 
			build_subset_mask(node->ptrs[n].ptr, level_bits, level+1);
}


/*
 * Merge nodes A and B together, 
 *   returns a node that is the path for the intersection
 *
 * If match node (leaf on trie)
 *	For each category
 *		return node = highest priority result
 *
 * Create C as a duplicate of A to point to child intersections
 * If any pointers in C intersect with any in B
 *	For each intersection
 *		merge children 
 *		remove intersection from C pointer
 *		add a pointer from C to child intersection node
 * Compact the pointers in A and B
 * Copy any B pointers that are outside of the intersection to C
 * If C has no references to the B trie
 *   free C and return A
 * Else If C has no references to the A trie
 *   free C and return B
 * Else
 *   return C
 */
static int
acl_merge_trie(struct acl_build_context *context,
	struct rte_acl_node *node_a, struct rte_acl_node *node_b,
	uint32_t level, uint32_t subtree_id, struct rte_acl_node **return_c)
{
	uint32_t n, m, ptrs_c, ptrs_b;;
	uint32_t min_add_c, min_add_b;
	int node_intersect_type;
	struct rte_acl_bitset node_intersect;
	struct rte_acl_node *node_c;
	struct rte_acl_node *node_a_next;
	int node_b_refs;
	int node_a_refs;

	node_c = node_a;
	node_a_next = node_a->next;
	min_add_c = 0;
	min_add_b = 0;
	node_a_refs = node_a->num_ptrs;
	node_b_refs = 0;
	node_intersect_type = 0;

	/* Resolve leaf nodes (matches) */
	if (node_a->match_flag != 0) {
		acl_resolve_leaf(context, node_a, node_b, return_c);
		return 0;
	}
	/*
	* Create node C as a copy of node A if node A is not part of 
	* a subtree of the merging tree (node B side). Otherwise,
	* just use node A.
	*/
	if (level > 0 && 
		node_a->subtree_id != (subtree_id | RTE_ACL_SUBTREE_NODE)) 
	{
		node_c = acl_dup_node(context, node_a);
		node_c->subtree_id = subtree_id | RTE_ACL_SUBTREE_NODE;
	}

	/*
	 * If the two node transitions intersect then merge the transitions.
	 * Check intersection for entire node (all pointers)
	 */
	node_intersect_type = acl_intersect_type(&node_c->values,
	                      &node_b->values,
	                      &node_intersect);
	if (node_intersect_type & ACL_INTERSECT) {

		min_add_b = node_b->min_add;
		node_b->min_add = node_b->num_ptrs;
		ptrs_b = node_b->num_ptrs;

		min_add_c = node_c->min_add;
		node_c->min_add = node_c->num_ptrs;
		ptrs_c = node_c->num_ptrs;

		for (n = 0; n < ptrs_c; n++) {
			if (node_c->ptrs[n].ptr == NULL) {
				node_a_refs--;
				continue;
			}
			node_c->ptrs[n].ptr->next = NULL;
			for (m = 0; m < ptrs_b; m++) {
				struct rte_acl_node *child_node_c = NULL;
				struct rte_acl_bitset child_intersect;
				int	child_intersect_type;

				if (node_b->ptrs[m].ptr == NULL ||
						node_c->ptrs[n].ptr ==
						node_b->ptrs[m].ptr)
						continue;

				child_intersect_type = acl_intersect_type(&node_c->ptrs[n].values,
		                &node_b->ptrs[m].values,
			            &child_intersect);
				if ((child_intersect_type & ACL_INTERSECT) != 0) {
					if (acl_merge_trie(context, node_c->ptrs[n].ptr,
									node_b->ptrs[m].ptr,
									level + 1, subtree_id, &child_node_c))
						return 1;
					if (child_node_c != NULL && 
							child_node_c != node_c->ptrs[n].ptr) {
						node_b_refs++;
						/*
						* Added link from C to child_C for all transitions
						*  in the intersection
						*/
						acl_add_ptr(context, node_c,
									child_node_c, &child_intersect);

						/* inc refs if pointer is not to node b */
						if (child_node_c != node_b->ptrs[m].ptr)
							node_a_refs++;

						/* Remove intersection from C pointer */
						if (!acl_exclude(&node_c->ptrs[n].values,
											&node_c->ptrs[n].values,
											&child_intersect)) {
							acl_deref_ptr(context, node_c, n);
							node_c->ptrs[n].ptr = NULL;
							node_a_refs--;
						}
					}
				}
			}
		}
		/* Compact pointers */
		node_c->min_add = min_add_c;
		acl_compact_node_ptrs(node_c);
		node_b->min_add = min_add_b;
		acl_compact_node_ptrs(node_b);
	} 

	/*
	*  Copy pointers outside of the intersection from B to C
	*/
	if ((node_intersect_type & ACL_INTERSECT_B) != 0) {
		node_b_refs++;
		for (m = 0; m < node_b->num_ptrs; m++)
			if (node_b->ptrs[m].ptr != NULL)
				acl_copy_ptr(context, node_c, 
					node_b, m, &node_intersect);
	}

	/* 
	* Free node C if top of trie is contained in A or B
	* 	if	node C is a duplicate of node A &&
	* 		node C was not an existing duplicate
	*/
	if (node_c != node_a && node_c != node_a_next) {
		/*
		* if the intersection has no references to the
		* B side, then it is contained in A
		*/
		if (node_b_refs == 0) {
			acl_free_node(context, node_c);
			node_c = node_a;
		} else {
			/*
			* if the intersection has no references to the
			* A side, then it is contained in B
			*/
			if (node_a_refs == 0) {
				acl_free_node(context, node_c);
				node_c = node_b;
			}
		}
	}

	if (return_c != NULL)
		*return_c = node_c;

	if (level == 0)
		acl_free_node(context, node_b);

	return 0;
}



/*
 * Reset current runtime fields before next build:
 *  - free allocated RT memory.
 *  - reset all RT related fields to zero.
 */
static void
acl_build_reset(struct rte_acl_ctx *ctx)
{
	rte_free(ctx->mem);
	memset(&ctx->num_categories, 0,
		sizeof(*ctx) - offsetof (struct rte_acl_ctx, num_categories));
}

static void
acl_gen_range(struct acl_build_context * context,
	uint8_t *hi, uint8_t *lo, int size, int level,
	struct rte_acl_node *root, struct rte_acl_node *end)
{
	struct rte_acl_node *node, *prev;
	uint32_t n;

	prev = root;
	for (n = size - 1; n > 0; n--) {
		node = acl_alloc_node(context, level++);
		acl_add_ptr_range(context, prev, node, lo[n], hi[n]);
		prev = node;
	}
	acl_add_ptr_range(context, prev, end, lo[0], hi[0]);
}
/*
*
*/
static struct rte_acl_node
*acl_gen_range_trie(struct acl_build_context * context,
	void	*min,
	void	*max,
	int		size,
	int		level,
	struct rte_acl_node **pend)
{
	int32_t n;
	struct rte_acl_node *root;
	*pend = acl_alloc_node(context, level+size);
	root = acl_alloc_node(context, level++);
	uint8_t *lo = (uint8_t *)min;
	uint8_t *hi = (uint8_t *)max;

	if (lo[size-1] == hi[size-1]) {
		acl_gen_range(context, hi, lo, size, level, root, *pend);
	} else {
		uint8_t limit_lo[64];
		uint8_t limit_hi[64];
		uint8_t hi_ff = 0xff;
		uint8_t lo_00 = 0;
		memset(limit_lo, 0, DIM(limit_lo));
		memset(limit_hi, -1, DIM(limit_hi));

		for (n = size - 2; n >= 0; n--) {
			hi_ff &= hi[n];
			lo_00 |= lo[n];
		}

		if (hi_ff != 0xff) {
			limit_lo[size-1] = hi[size-1];
			acl_gen_range(context, hi, limit_lo, size, level, root, *pend);
		}

		if (lo_00 != 0) {
			limit_hi[size-1] = lo[size-1];
			acl_gen_range(context, limit_hi, lo, size, level, root, *pend);
		}

		if (hi[size-1] - lo[size-1] > 1 || lo_00 == 0 || hi_ff == 0xff) {
			limit_lo[size-1] = lo[size-1] + (lo_00 != 0 ? 1 : 0);
			limit_hi[size-1] = hi[size-1] - (hi_ff != 0xff ? 1 : 0);
			acl_gen_range(context, limit_hi, limit_lo, size, level, root, *pend);
		}
	}
	return (root);
}

static struct rte_acl_node
*acl_gen_mask_trie(struct acl_build_context * context,
	void	*value,
	void	*mask,
	int		size,
	int		level,
	struct rte_acl_node **pend)
{
	int32_t n;
	struct rte_acl_node *root = acl_alloc_node(context, level++);
	struct rte_acl_node *node, *prev;
	struct rte_acl_bitset bits;
	uint8_t *val = (uint8_t *)value;
	uint8_t *msk = (uint8_t *)mask;
	prev = root;
	for (n = size - 1; n >= 0; n--) {
		node = acl_alloc_node(context, level++);
		acl_gen_mask(&bits, val[n], msk[n]);
		acl_add_ptr(context, prev, node, &bits);
		prev = node;
	}
	*pend = prev;
	return (root);
}
//static int node_counts;
static struct rte_acl_node 
*build_trie(struct acl_build_context *context, struct rte_acl_build_rule *head, struct rte_acl_build_rule **last, uint32_t *count)
{
	uint32_t n, m;
	int node_count;
	struct rte_acl_build_rule *prev = head;
	struct rte_acl_build_rule *rule = head;
	struct rte_acl_node *trie = acl_alloc_node(context, 0);
	while (rule != NULL) {
		struct rte_acl_node *end, *merge, *root, *end_prev;
		root = acl_alloc_node(context, 0);
		root->ref_count = 1;
		end = root;
		for (n=0; n<rule->config->num_fields; n++) {
			int field_index = rule->config->defs[n].field_index;
			end_prev = end;
			/* build a mini-trie for this field */
			switch(rule->config->defs[n].type) {
			case RTE_ACL_FIELD_TYPE_BITMASK:
				merge = acl_gen_mask_trie(context,
							&rule->f.field[field_index].value, 
							&rule->f.field[field_index].mask_range,
							rule->config->defs[n].size,
							end->level+1,
							&end);
				break;
			case RTE_ACL_FIELD_TYPE_MASK:
				{
					/* set msb for the size of the field and all higher bits */
					long long mask = -1 << (rule->config->defs[n].size * 8 - 1);

					/* arithmetic right shift for the length of the mask less the msb */
					if (rule->f.field[field_index].mask_range.u32 == 0)
						mask = 0;
					else
						mask >>= rule->f.field[field_index].mask_range.u32 - 1;

					/* gen a mini-trie for this field */
					merge = acl_gen_mask_trie(context,
								&rule->f.field[field_index].value, 
								(char *)&mask,
								rule->config->defs[n].size,
								end->level+1,
								&end);
				}
				break;
			case RTE_ACL_FIELD_TYPE_RANGE:
				merge = acl_gen_range_trie(context, 
							&rule->f.field[field_index].value, 
							&rule->f.field[field_index].mask_range,
							rule->config->defs[n].size,
							end->level+1,
							&end);
				break;
			default:
				printf("Error in rule[%d] type - %d\n", rule->f.userdata, rule->config->defs[n].type);
				return NULL;
			}

			/* merge this field on to the end of the rule */
			if (acl_merge_trie(context, end_prev, merge, 0, 0, NULL))
				return NULL;
		}
		(*count)++;
		end->match_flag = context->num_build_rules + *count;
		//printf("[%d, %d] ", ++node_counts, end->match_flag);

		/*
		*	Setup the results for this rule.
		*	The result and priority of each category.
		*/
		if (rule->f.category_mask == 0)
			rule->f.category_mask = 1;
		if (end->mrt == NULL)
			end->mrt = (struct rte_acl_match_results *)
				acl_build_alloc(context, 1, sizeof(struct rte_acl_match_results));
		for (m = 0; m < context->num_categories; m++) {
			if (rule->f.category_mask & (1 << m)) {
				end->mrt->results[m] = rule->f.userdata;
				end->mrt->priority[m] = rule->f.priority;
			} else {
				end->mrt->results[m] = 0;
				end->mrt->priority[m] = 0;
			}
		}

		node_count = context->num_nodes;

		{
			struct rte_acl_bitset level_bits[RTE_ACL_MAX_LEVELS];
			memset(&level_bits[0], -1, sizeof(level_bits));
			build_subset_mask(root, &level_bits[0], 0);
			mark_subtree(trie, &level_bits[0], 0, end->match_flag);
		}

		/* merge this rule into the trie */
		if (acl_merge_trie(context, trie, root, 0, end->match_flag, NULL))
			return NULL;
		node_count = context->num_nodes - node_count;

		if (node_count > NODE_MAX) {
			*last = prev;
			return trie;
		}

		prev = rule;
		rule = rule->next;
	}
	*last = NULL;
	return trie;
}

uint32_t wild_limits[5] = {0, 25, 50, 75, 100};

// TALLY are statistics per field
enum {
	TALLY_0 = 0,		// number of rules that are   0% or more wild
	TALLY_25,			//                           25%
	TALLY_50,			//                           50%
	TALLY_75,			//                           75%
	TALLY_100,			//                          100%
	TALLY_DEACTIVATED,	// deactivated fields (all 100% wild in all rules)
	TALLY_DEPTH,		// number of rules that are 100% wild for this field and higher

	TALLY_NUM
};
static void
acl_calc_wildness(struct rte_acl_build_rule *head, struct rte_acl_config *config)
{
	uint32_t n;
	struct rte_acl_build_rule *rule;
	for (rule = head; rule != NULL; rule = rule->next) {
		for (n=0; n<config->num_fields; n++) {
			double wild = 0;
			double size = 8 * config->defs[n].size;
			int field_index = config->defs[n].field_index;
			switch (rule->config->defs[n].type) {
			case RTE_ACL_FIELD_TYPE_BITMASK:
					wild = (size - _mm_popcnt_u64(rule->f.field[field_index].mask_range.u8)) / size;
				break;
			case RTE_ACL_FIELD_TYPE_MASK:
				wild = (size - rule->f.field[field_index].mask_range.u32) / size;
				break;
			case RTE_ACL_FIELD_TYPE_RANGE:
				switch (rule->config->defs[n].size) {
				case 1:
					wild = ((double) rule->f.field[field_index].mask_range.u8 - 
						rule->f.field[field_index].value.u8) / UINT8_MAX;
					break;
				case 2:
					wild = ((double) rule->f.field[field_index].mask_range.u16 - 
						rule->f.field[field_index].value.u16) / UINT16_MAX;
					break;
				case 4:
					wild = ((double) rule->f.field[field_index].mask_range.u32 - 
						rule->f.field[field_index].value.u32) / UINT32_MAX;
					break;
				default:
					wild = ((double) rule->f.field[field_index].mask_range.u64 - 
						rule->f.field[field_index].value.u64) / UINT64_MAX;
					break;
				}
				break;
			default:
				printf("Error in rule[%d] type - %d\n", rule->f.userdata, rule->config->defs[n].type);
				return;
			}
			rule->f.field[field_index].wildness = wild * 100;
		}
	}
}

static int
acl_rule_stats(struct rte_acl_build_rule *head, struct rte_acl_config *config, uint32_t *wild_limit)
{
	struct rte_acl_build_rule *rule;
	uint32_t n, m, fields_deactivated = 0;
	uint32_t start = 0, deactivate = 0;
	int tally[RTE_ACL_MAX_LEVELS][TALLY_NUM];
	memset(tally, 0, sizeof(tally));
	for (rule = head; rule != NULL; rule = rule->next) {
		for (n=0; n<config->num_fields; n++) {
			uint32_t field_index = config->defs[n].field_index;
			tally[n][TALLY_0]++;
			for (m=1; m<DIM(wild_limits); m++) {
				if (rule->f.field[field_index].wildness >= wild_limits[m])
					tally[n][m]++;
			}
		}
		for (n = config->num_fields - 1; n > 0; n--) {
			uint32_t field_index = config->defs[n].field_index;
			if (rule->f.field[field_index].wildness == 100)
				tally[n][TALLY_DEPTH]++;
			else
				break;
		}
	}

	/* 
	* Look for any field that is always wild and drop it from the config 
	*	Only deactivate if all fields for a given input loop are deactivated.
	*/
	for (n = 1; n < config->num_fields; n++) {
		if (config->defs[n].input_index != config->defs[n-1].input_index) {
			for (m = start; m < n; m++) 
				tally[m][TALLY_DEACTIVATED] = deactivate;
			fields_deactivated += deactivate;
			start = n;
			deactivate = 1;
		}

		/* if the field is not always completely wild */
		if (tally[n][TALLY_100] != tally[n][TALLY_0])
			deactivate = 0;
	}

	for (m = start; m < n; m++) 
		tally[m][TALLY_DEACTIVATED] = deactivate;
	fields_deactivated += deactivate;

	if (fields_deactivated) {
		/* remove deactivated fields */
		uint32_t k, l=0;
		for (k = 0; k < config->num_fields; k++) {
			if (tally[k][TALLY_DEACTIVATED] == 0) {
				memcpy(&tally[l][0], &tally[k][0], TALLY_NUM*sizeof(tally[0][0]));
				memcpy(&config->defs[l++],
					&config->defs[k], 
					sizeof(struct rte_acl_field_def));
			}
		}
		config->num_fields = l;
	}

//	show_tally(tally, config);
	{
		int min = RTE_ACL_SINGLE_TRIE_SIZE;
		if (config->num_fields == 2)
			min *= 4;
		else if (config->num_fields == 3)
			min *= 3;
		else if (config->num_fields == 4)
			min *= 2;

		if (tally[0][TALLY_0] < min)
			return 0;
		for (n = 0; n < config->num_fields; n++)
			wild_limit[n] = 0;
	}

	/* 
	* If trailing fields are 100% wild, group those together.
	* This allows the search length of the trie to be shortened.
	*/
	for (n = 1; n < config->num_fields; n++) {
		double rule_percentage = (double)tally[n][TALLY_DEPTH] / tally[n][0];
		if (rule_percentage > RULE_PERCENTAGE) {
			/* if it crosses an input boundary then round up */
			while (config->defs[n-1].input_index == config->defs[n].input_index)
				n++;

			/* set the limit for selecting rules */
			while (n < config->num_fields)
				wild_limit[n++] = 100;
				
			if (wild_limit[n-1] == 100)
				return 1;
		}
	}

	/* look for the most wild that's 40% or more of the rules */
	for (n = 1; n < config->num_fields; n++) {
		for (m = TALLY_100; m > 0; m--) {
			double rule_percentage = (double)tally[n][m] / tally[n][0];
			if ((tally[n][TALLY_DEACTIVATED] == 0) &&
				(tally[n][TALLY_0] > RTE_ACL_SINGLE_TRIE_SIZE) &&
					(rule_percentage > NODE_PERCENTAGE) &&
					(rule_percentage < 0.80)
					) {
				wild_limit[n] = wild_limits[m];
				return 1;
			}
		}
	}
	return 0;
}

static int
order(struct rte_acl_build_rule **insert, struct rte_acl_build_rule *rule)
{
	uint32_t n;
	struct rte_acl_build_rule *left = *insert;
	if (left == NULL)
		return (0);
	for (n=1; n<left->config->num_fields; n++) {
		int field_index = left->config->defs[n].field_index;
		if (left->f.field[field_index].wildness != rule->f.field[field_index].wildness)
			return (left->f.field[field_index].wildness >= rule->f.field[field_index].wildness);
	}
	return (0);
}

static struct rte_acl_build_rule 
*ordered_insert_rule(struct rte_acl_build_rule *head, struct rte_acl_build_rule *rule)
{
	struct rte_acl_build_rule **insert;
	if (rule == NULL)
		return head;
	rule->next = head;
	if (head == NULL)
		return rule;
	insert = &head;
	while (order(insert, rule)) {
		insert = &(*insert)->next;
	}
	rule->next = *insert;
	*insert = rule;
	return head;
}

static struct rte_acl_build_rule *
sort_rules(struct rte_acl_build_rule * head)
{
	struct rte_acl_build_rule *rule, *reordered_head = NULL;
	struct rte_acl_build_rule *last_rule = NULL;

	for (rule = head; rule != NULL; rule = rule->next) {
		reordered_head = ordered_insert_rule(reordered_head, last_rule);
		last_rule = rule;
	}

	if (last_rule != reordered_head) {
		reordered_head = ordered_insert_rule(reordered_head, last_rule);
	}

	return reordered_head;
}

static uint32_t
*acl_build_index(struct rte_acl_config *config)
{
	uint32_t n, m = 0;
	int32_t last_header = -1;
	uint32_t *index = (uint32_t *) calloc(config->num_fields, sizeof(int));
	for (n = 0; n < config->num_fields; n++)
		if (last_header != config->defs[n].input_index) {
			last_header = config->defs[n].input_index;
			index[m++] = config->defs[n].offset;
		}
	return index;
}

static int
acl_build_tries(struct acl_build_context *context, struct rte_acl_build_rule *head)
{
	uint32_t n, m;
	uint32_t wild_limit[RTE_ACL_MAX_LEVELS];

	struct rte_acl_config *config = head->config;
	struct rte_acl_build_rule *rule = head;

	uint32_t num_tries=1;
	struct rte_acl_build_rule *rule_sets[RTE_ACL_MAX_TRIES];
	rule_sets[0] = head;

	/* initialize tries */
	for (n = 0; n < DIM(context->tries); n++) {
		context->tries[n].type = RTE_ACL_UNUSED_TRIE;
		context->bld_tries[n].trie = NULL;
		context->tries[n].count = 0;
		context->tries[n].smallest = 0x7fffffff;
	}

	context->tries[0].type = RTE_ACL_FULL_TRIE;

	/* calc wildness of each field of each rule */
	acl_calc_wildness(head, config);

	n = acl_rule_stats(head, config, &wild_limit[0]);
	/* put all rules that fit the wildness criteria into a seperate trie */
	while (n > 0 && num_tries < RTE_ACL_MAX_TRIES) {
		struct rte_acl_config *new_config = (struct rte_acl_config *) 
			acl_build_alloc(context, 1, sizeof(struct rte_acl_config));
		struct rte_acl_build_rule **prev = &rule_sets[num_tries-1];
		struct rte_acl_build_rule *next = head->next;
		memcpy(new_config, config, sizeof(*new_config));
		config = new_config;
		rule_sets[num_tries] = NULL;
		for (rule = head; rule != NULL; rule = next) {
			next = rule->next;
			int move = 1;
			for (m = 0; m < config->num_fields; m++) {
				int x = config->defs[m].field_index;
				if (rule->f.field[x].wildness < wild_limit[m]) {
					move = 0;
					break;
				}
			}
			if (move) {
				rule->config = new_config;
				rule->next = rule_sets[num_tries];
				rule_sets[num_tries] = rule;
				*prev = next;
			} else
				prev = &rule->next;
		}
		head = rule_sets[num_tries];
		n = acl_rule_stats(rule_sets[num_tries], config, &wild_limit[0]);
		num_tries++;
	}

	if (n > 0)
		printf("Number of tries(%d) exceeded.\n", RTE_ACL_MAX_TRIES);

	{
		struct rte_acl_build_rule *last;
		for (n=0; n<num_tries; n++) {
			rule_sets[n] = sort_rules(rule_sets[n]);
			context->tries[n].data_index = acl_build_index(rule_sets[n]->config);
			context->tries[n].type = RTE_ACL_FULL_TRIE;
			context->tries[n].count = 0;
//			printf("\n###%s---1\n", __FUNCTION__);
			context->bld_tries[n].trie = build_trie(context, rule_sets[n], &last, &context->tries[n].count);
			if (last != NULL) {
				rule_sets[num_tries++] = last->next;
				last->next = NULL;
				acl_free_node(context, context->bld_tries[n].trie);
				context->tries[n].count = 0;
//				printf("\n###%s---2\n", __FUNCTION__);
				context->bld_tries[n].trie = build_trie(context, rule_sets[n], &last, &context->tries[n].count);
			}
			context->num_build_rules += context->tries[n].count;
		}
	}
	context->num_tries = num_tries;

	return 0;
}
/*
*
*/
enum {
	SRC_ADDR_OFFSET = 0,
	DST_ADDR_OFFSET = 16,
	SRC_PORT_OFFSET = 32,
	DST_PORT_OFFSET = 34,
	PROTOCOL_OFFSET = 36,
	VLAN_OFFSET = 40,
};

static struct rte_acl_field_def ipv4_defs[] = {
	{RTE_ACL_FIELD_TYPE_BITMASK,	1,  0,  0, PROTOCOL_OFFSET}, 
	{RTE_ACL_FIELD_TYPE_BITMASK,	2,  1,  1, VLAN_OFFSET}, 
	{RTE_ACL_FIELD_TYPE_BITMASK,	2,  2,  1, VLAN_OFFSET+2}, 
	{RTE_ACL_FIELD_TYPE_MASK,		4,  3,  2, SRC_ADDR_OFFSET}, 
	{RTE_ACL_FIELD_TYPE_MASK,		4,  4,  3, DST_ADDR_OFFSET}, 
	{RTE_ACL_FIELD_TYPE_RANGE,		2,  5,  4, SRC_PORT_OFFSET},
	{RTE_ACL_FIELD_TYPE_RANGE,		2,  6,  4, DST_PORT_OFFSET},
};


static struct rte_acl_build_rule *
acl_convert_ip_rule(
	struct acl_build_context *context, 
	struct rte_acl_ipv4vlan_rule *head, uint32_t num_rules,
	const uint32_t *layout)
{
	struct rte_acl_ipv4vlan_rule *ip_rule = head;
	struct rte_acl_build_rule *rule, *acl_head = NULL;

	struct rte_acl_config *config = (struct rte_acl_config *) 
		acl_build_alloc(context, 1, sizeof(struct rte_acl_config));
	memcpy(&config->defs, ipv4_defs, sizeof(ipv4_defs));
	config->num_fields = DIM(ipv4_defs);
	config->defs[0].offset = layout[RTE_ACL_IPV4VLAN_PROTO];
	config->defs[1].offset = layout[RTE_ACL_IPV4VLAN_VLAN];
	config->defs[2].offset = layout[RTE_ACL_IPV4VLAN_VLAN]+2;
	config->defs[3].offset = layout[RTE_ACL_IPV4VLAN_SRC];
	config->defs[4].offset = layout[RTE_ACL_IPV4VLAN_DST];
	config->defs[5].offset = layout[RTE_ACL_IPV4VLAN_PORTS];
	config->defs[6].offset = layout[RTE_ACL_IPV4VLAN_PORTS]+2;

	while (num_rules != 0) {
		rule = (struct rte_acl_build_rule *) 
			acl_build_alloc(context, 1, sizeof(struct rte_acl_build_rule));
		rule->config = config;
		rule->f.field = (struct rte_acl_field *)
			acl_build_alloc(context, config->num_fields, sizeof(struct rte_acl_field));
		rule->f.field[0].value.u8 = ip_rule->proto;
		rule->f.field[1].value.u16 = ip_rule->vlan;
		rule->f.field[2].value.u16 = ip_rule->domain;
		rule->f.field[3].value.u32 = ip_rule->src_addr;
		rule->f.field[4].value.u32 = ip_rule->dst_addr;
		rule->f.field[5].value.u16 = ip_rule->src_port_low;
		rule->f.field[6].value.u16 = ip_rule->dst_port_low;

		rule->f.field[0].mask_range.u8 = ip_rule->proto_mask;
		rule->f.field[1].mask_range.u16 = ip_rule->vlan_mask;
		rule->f.field[2].mask_range.u16 = ip_rule->domain_mask;
		rule->f.field[3].mask_range.u32 = ip_rule->src_mask_len;
		rule->f.field[4].mask_range.u32 = ip_rule->dst_mask_len;
		rule->f.field[5].mask_range.u16 = ip_rule->src_port_high;
		rule->f.field[6].mask_range.u16 = ip_rule->dst_port_high;
		rule->f.userdata = ip_rule->data.userdata;
		rule->f.priority = ip_rule->data.priority;
		rule->f.category_mask = ip_rule->data.category_mask;
		rule->next = acl_head;
		acl_head = rule;

		ip_rule++;
		num_rules--;
	}
	return acl_head;
}

static void
acl_build_log(const struct acl_build_context *ctx)
{
	uint32_t n;
	RTE_LOG(DEBUG, PMAC, "Build phase for ACL \"%s\":\n"
		"memory consumed: %zu\n",
		ctx->acx->name,
		ctx->pool.alloc);
	for (n = 0; n < DIM(ctx->tries); n++) {
		if (ctx->tries[n].count != 0)
			RTE_LOG(DEBUG, PMAC, 
				"trie %d: number of rules: %u\n",
				n, ctx->tries[n].count);
	}
}


int
rte_acl_build(struct rte_acl_ctx *ctx,
	uint32_t num_categories)
{
	int rc;
	struct acl_build_context bcx;
	struct rte_acl_build_rule *acl_head = ctx->build_rules;

	if (ctx == NULL ||
		num_categories > RTE_ACL_MAX_CATEGORIES)
		return -(EINVAL);

	acl_build_reset(ctx);

	memset(&bcx, 0, sizeof (bcx));
	bcx.acx = ctx;
	bcx.pool.alignment = ACL_POOL_ALIGN;
	bcx.pool.min_alloc = ACL_POOL_ALLOC_MIN;
	bcx.num_categories = num_categories;

	/* build internal trie representation. */
	if ((rc = acl_build_tries(&bcx, acl_head)) == 0) { 

		/* allocate and fill run-time  structures. */
		if ((rc = rte_acl_gen(ctx, bcx.tries, bcx.bld_tries,
				bcx.num_tries, bcx.num_categories,
				RTE_ACL_IPV4VLAN_NUM * DIM(bcx.tries),
				bcx.num_build_rules)) == 0) {
		}
	}

	acl_build_log(&bcx);

	/* cleanup after build. */
	pmac_free_pool(&bcx.pool);

	return rc;
}
int
rte_acl_ipv4vlan_build(struct rte_acl_ctx *ctx,
	const uint32_t layout[RTE_ACL_IPV4VLAN_NUM],
	uint32_t num_categories)
{
	int rc;
	struct rte_acl_build_rule *acl_head;
	struct acl_build_context bcx;

	if (ctx == NULL || layout == NULL ||
		num_categories > RTE_ACL_MAX_CATEGORIES)
		return -(EINVAL);

	acl_build_reset(ctx);

	memset(&bcx, 0, sizeof (bcx));
	bcx.acx = ctx;
	bcx.pool.alignment = ACL_POOL_ALIGN;
	bcx.pool.min_alloc = ACL_POOL_ALLOC_MIN;
	bcx.num_categories = num_categories;

	/* convert from old rule format to new format */
	acl_head = acl_convert_ip_rule(&bcx, ctx->rules, ctx->num_rules, layout);

	/* build internal trie representation. */
	if ((rc = acl_build_tries(&bcx, acl_head)) == 0) { 
		/* allocate and fill run-time  structures. */
		if ((rc = rte_acl_gen(ctx, bcx.tries, bcx.bld_tries,
				bcx.num_tries, bcx.num_categories,
				RTE_ACL_IPV4VLAN_NUM * DIM(bcx.tries),
				bcx.num_build_rules)) == 0) {
		}
	}

	/* cleanup after build. */
	pmac_free_pool(&bcx.pool);
	return rc;

}

