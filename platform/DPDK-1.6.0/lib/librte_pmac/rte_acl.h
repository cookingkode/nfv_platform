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

#ifndef _RTE_ACL_H_
#define _RTE_ACL_H_

/**
 * @file
 *
 * RTE Classifier.
 */

#include <rte_pmac_osdep.h>
#include <rte_pmac_vect.h>

#ifdef __cplusplus
extern "C" {
#endif

#define	RTE_ACL_MAX_CATEGORIES	16

#define	RTE_ACL_RESULTS_MULTIPLIER	(SSE_SIZE / sizeof(uint32_t))

#define RTE_ACL_MAX_LEVELS 64
#define RTE_ACL_MAX_FIELDS 64

union rte_acl_field_types {
	uint8_t			u8;
	uint16_t		u16;
	uint32_t		u32;
	uint64_t		u64;
};

enum {
	RTE_ACL_FIELD_TYPE_MASK = 0,
	RTE_ACL_FIELD_TYPE_RANGE,
	RTE_ACL_FIELD_TYPE_BITMASK
};

struct rte_acl_field_def {
	uint16_t		type;		// type - ACL_FIELD_TYPE
	uint8_t			size;		// size of field 1,2,4, or 8
	uint8_t			field_index;
	uint16_t		input_index; // 0-N input index
	uint16_t		offset;		 // offset to start of field
};

// defines the fields of an ACL trie (only used for build)
struct rte_acl_config {
	struct rte_acl_field_def	defs[RTE_ACL_MAX_FIELDS];	// array of field definitions
	uint32_t					num_fields;
};

// defines the value of a field for a rule
struct rte_acl_field {
	union rte_acl_field_types		value;		// a 1,2,4, or 8 byte value of the field
	union rte_acl_field_types		mask_range;	/* depending on field type
										mask -> 1.2.3.4/32 value=0x1234, mask_range=32
										range -> 0 : 65535 value=0, mask_range=65535
										bitmask -> 0x06/0xff value=6, mask_range=0xff */
	uint32_t				wildness;	// measure of how wild the field is (0 to 100)
};

// A single ACL rule
struct rte_acl_rule {
	struct rte_acl_field	*field;		// value and mask or range for each field.
	uint32_t				category_mask;	// bit mask of relevant categories
	int32_t					priority;	// priority for this rule, higher value = higher priority
	uint32_t				userdata;	// rule number (return value when match this rule)
};

enum {
	RTE_ACL_TYPE_SHIFT = 29,
	RTE_ACL_MAX_INDEX = LEN2MASK(RTE_ACL_TYPE_SHIFT),
	RTE_ACL_MAX_PRIORITY = RTE_ACL_MAX_INDEX,
	RTE_ACL_MIN_PRIORITY = 0,
};

#define	RTE_ACL_INVALID_USERDATA	0

struct rte_acl_rule_data {
	uint32_t category_mask;
	int32_t  priority;
	uint32_t userdata;
};

struct rte_acl_ipv4vlan_rule {
	struct rte_acl_rule_data data;
	uint8_t proto;
	uint8_t proto_mask;
	uint16_t vlan;
	uint16_t vlan_mask;
	uint16_t domain;
	uint16_t domain_mask;
	uint32_t src_addr;
	uint32_t src_mask_len;
	uint32_t dst_addr;
	uint32_t dst_mask_len;
	uint16_t src_port_low;
	uint16_t src_port_high;
	uint16_t dst_port_low;
	uint16_t dst_port_high;
};

/*
 * That effectively defines order of IPV4VLAN classifications:
 *  - PROTO
 *  - VLAN (TAG and DOMAIN)
 *  - SRC IP ADDRESS
 *  - DST IP ADDRESS
 *  - PORTS (SRC and DST)
 */
enum {
	RTE_ACL_IPV4VLAN_PROTO,
	RTE_ACL_IPV4VLAN_VLAN,
	RTE_ACL_IPV4VLAN_SRC,
	RTE_ACL_IPV4VLAN_DST,
	RTE_ACL_IPV4VLAN_PORTS,
	RTE_ACL_IPV4VLAN_NUM
};

/** Max number of characters in name.*/
#define	RTE_ACL_NAMESIZE		32

/**
 * Parameters used when creating the ACL context.
 */
struct rte_acl_param {
	const char *name;         /**< Name of the ACL context. */
	int         socket_id;    /**< Socket ID to allocate memory for. */
	uint32_t    rule_size;    /**< Size of each rule. */
	uint32_t    max_rule_num; /**< Maximum number of rules. */
}; 


/**
 * Create a new ACL context.
 *
 * @param param
 *   Parameters used to create and initialise the ACL context.
 * @return
 *   Pointer to ACL context structure that is used in future ACL
 *   operations, or NULL on error, with error code set in rte_errno.
 *   Possible rte_errno errors include:
 *   - E_RTE_NO_TAILQ - no tailq list could be got for the ACL context list
 *   - EINVAL - invalid parameter passed to function
 */
struct rte_acl_ctx *
rte_acl_create(const struct rte_acl_param *param);

/**
 * Find an existing ACL context object and return a pointer to it.
 *
 * @param name
 *   Name of the ACL context as passed to rte_acl_create()
 * @return
 *   Pointer to ACL context or NULL if object not found
 *   with rte_errno set appropriately. Possible rte_errno values include:
 *    - ENOENT - value not available for return
 */
struct rte_acl_ctx *
rte_acl_find_existing(const char *name);

/**
 * De-allocate all memory used by ACL context.
 *
 * @param ctx
 *   ACL context to free
 */
void
rte_acl_free(struct rte_acl_ctx *ctx);

/**
 * Add rules to an existing ACL context.
 * This function is not multi-thread safe.
 *
 * @param ctx
 *   ACL context to add patterns to.
 * @param rules
 *   Array of rules to add to the ACL context.
 *   Note that all fields in rte_acl_ipv4vlan_rule structures are expected
 *   to be in host byte order.
 * @param num
 *   Number of elements in the input array of rules.
 * @return
 *   - -ENOMEM if there is no space in the ACL context for these rules.
 *   - -EINVAL if the parameters are invalid.
 *   - Zero if operation completed successfully.
 */
int
rte_acl_ipv4vlan_add_rules(struct rte_acl_ctx *ctx,
	const struct rte_acl_ipv4vlan_rule *rules,
	uint32_t num);

/**
 * Analyze set of rules and build required internal run-time structures.
 * This function is not multi-thread safe.
 *
 * @param ctx
 *   ACL context to build.
 * @param layout
 *   Layout of input data to search through.
 * @param num_categories
 *   Maximum number of categories to use in that build.
 * @return
 *   - -ENOMEM if couldn't allocate enough memory.
 *   - -EINVAL if the parameters are invalid.
 *   - Negative error code if operation failed.
 *   - Zero if operation completed successfully.
 */
int
rte_acl_ipv4vlan_build(struct rte_acl_ctx *ctx,
	const uint32_t layout[RTE_ACL_IPV4VLAN_NUM],
	uint32_t num_categories);
int
rte_acl_build(struct rte_acl_ctx *ctx, 
	uint32_t num_categories);

int
rte_acl_set_config(struct rte_acl_ctx *ctx,
	const struct rte_acl_config *cfg);

void
rte_acl_ctx_rules_reset(struct rte_acl_ctx *ctx);

int
rte_acl_add_rules(struct rte_acl_ctx *ctx,
	const struct rte_acl_rule *rules,
	uint32_t num);
/**
 * Search for a matching ACL rule for each input data buffer.
 * Each input data buffer can have up to *categories* matches.
 * That implies that results array should be big enough to hold
 * (categories * num) elements.
 * Also categories parameter should be either one or multiple of
 * RTE_ACL_RESULTS_MULTIPLIER and can't be bigger than RTE_ACL_MAX_CATEGORIES.
 * If more than one rule is applicable for given input buffer and
 * given category, then rule with highest priority will be returned as a match.
 * Note, that it is a caller responsibility to ensure that input parameters
 * are valid and point to correct memory locations.
 *
 * @param ctx
 *   ACL context to search with.
 * @param data
 *   Array of pointers to input data buffers to perform search.
 *   Note that all fields in input data buffers supposed to be in network
 *   byte order (MSB).
 * @param results
 *   Array of search results, *categories* results per each input data buffer.
 * @param num
 *   Number of elements in the input data buffers array.
 * @param categories
 *   Number of maximum possible matches for each input buffer, one possible
 *   match per category.
 * @return
 *   zero on successful completion.
 *   -EINVAL for incorrect arguments.
 */
int
rte_acl_classify(const struct rte_acl_ctx *ctx, const uint8_t **data,
	uint32_t *results, uint32_t num, uint32_t categories);

/**
 * Perform scalar search for a matching ACL rule for each input data buffer.
 * Note, that while the search itself will avoid explicit use of SSE/AVX
 * intrinsics, code for comparing matching results/priorities sill might use
 * vector intrinsics (for  categories > 1).
 * Each input data buffer can have up to *categories* matches.
 * That implies that results array should be big enough to hold
 * (categories * num) elements.
 * Also categories parameter should be either one or multiple of
 * RTE_ACL_RESULTS_MULTIPLIER and can't be bigger than RTE_ACL_MAX_CATEGORIES.
 * If more than one rule is applicable for given input buffer and
 * given category, then rule with highest priority will be returned as a match.
 * Note, that it is a caller's responsibility to ensure that input parameters
 * are valid and point to correct memory locations.
 *
 * @param ctx
 *   ACL context to search with.
 * @param data
 *   Array of pointers to input data buffers to perform search.
 *   Note that all fields in input data buffers supposed to be in network
 *   byte order (MSB).
 * @param results
 *   Array of search results, *categories* results per each input data buffer.
 * @param num
 *   Number of elements in the input data buffers array.
 * @param categories
 *   Number of maximum possible matches for each input buffer, one possible
 *   match per category.
 * @return
 *   zero on successful completion.
 *   -EINVAL for incorrect arguments.
 */
int
rte_acl_classify_scalar(const struct rte_acl_ctx *ctx, const uint8_t **data,
	uint32_t *results, uint32_t num, uint32_t categories);


/**
 * Dump an ACL context structure to the console.
 *
 * @param ctx
 *   ACL context to dump.
 */
void
rte_acl_dump(const struct rte_acl_ctx *ctx);

/**
 * Dump all ACL context structures to the console.
 */
void
rte_acl_list_dump(void);


#ifdef __cplusplus
}
#endif

#endif /* _RTE_ACL_H_ */
