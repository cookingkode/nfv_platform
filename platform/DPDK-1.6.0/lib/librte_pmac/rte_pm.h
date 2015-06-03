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

#ifndef _RTE_PM_H_
#define _RTE_PM_H_

/**
 * @file
 *
 * RTE Pattern Match
 */

#include <rte_pmac_osdep.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Max number of characters in PM name.*/
#define	RTE_PM_NAMESIZE		32

/**
 * Parameters used when creating the PM context.
 */
struct rte_pm_param {
	const char *name;            /**< Name of the PM context. */
	int         socket_id;       /**< Socket ID to allocate memory for. */
	uint32_t    max_pattern_num; /**< Maximum number of patterns. */
	uint32_t    max_pattern_len; /**< Total max length of all pattterns. */
}; 


/**
 * Pattern structure.
 */
struct rte_pm_pattern {
	const uint8_t *pattern;  /**< Pattern. */
	uint64_t       userdata; /**< Associated with it user data. */
	uint32_t       len;      /**< Length of the pattern. */
};

/*
 * Supported PM search algorithms.
 */
enum rte_pm_search {
	#define	pm_search_method(num, name)	num,
	#define	pm_search_method_end(num)	num
	#include <rte_pm_def.h>
};

/*
 * Names of supported PM search algorithms.
 */
#define	RTE_PM_SEARCH_NAMELEN	0x40
extern const char rte_pm_search_names[RTE_PM_SEARCH_NUM][RTE_PM_SEARCH_NAMELEN];

#define	RTE_PM_MASK_BITNUM	(CHAR_BIT * sizeof (uint32_t))
#define	RTE_PM_MASK_BITMASK	(RTE_PM_MASK_BITNUM - 1)

#define	RTE_PM_SEARCH_BITNUM	\
	RTE_ALIGN(RTE_PM_SEARCH_NUM, RTE_PM_MASK_BITNUM)

typedef uint32_t rte_pm_search_mask_t[RTE_PM_SEARCH_BITNUM];

/**
 * Bitmask of all available search algorithms and preferred seach algorithm.
 */
struct rte_pm_search_avail {
	enum rte_pm_search   prefer; /**< preferred search algorithm. */
	rte_pm_search_mask_t avail;
	/** bitmask of all availble search algorithms. */
};

#define	RTE_PM_SET_BIT(m, b)	\
	((m)[(b) / RTE_PM_MASK_BITNUM] |= 1 << ((b) & RTE_PM_MASK_BITMASK))
#define	RTE_PM_CLEAR_BIT(m, b)	\
	((m)[(b) / RTE_PM_MASK_BITNUM] &= ~(1 << ((b) & RTE_PM_MASK_BITMASK)))
#define	RTE_PM_GET_BIT(m, b)	\
	(((m)[(b) / RTE_PM_MASK_BITNUM] >> ((b) & RTE_PM_MASK_BITMASK)) & 1)

enum {
	RTE_PM_CASE_LESS,
	RTE_PM_CASE_SENSE,
	RTE_PM_CASE_NUM
};

/**
 * Build Options for the PM context.
 */
struct rte_pm_build_opt {
	enum rte_pm_search search_type;  /**< search algorithm to use. */
	uint32_t case_sense : 1;      /**< Treat patterns as case-sensitive. */
	uint32_t out_of_order : 1;    /**< Allow out of order search. */
}; 

/**
 * Input buffer to perfrom search on.
 */
struct rte_pm_inbuf {
	const uint8_t *buf;  /**< input buffer to perform search on. */
	uint32_t       len;  /**< length of the input buffer. */
};

/**
 * Structure that describes the match found.
 */
struct rte_pm_match {
	uint32_t  fin;      /**< Offset of next byte beyond found match. */
	uint32_t  len;      /**< Length of the matched pattern. */
	uint64_t  userdata;
	/** User data associated with the matched pattern. */
};

struct rte_pm_ctx;

/*
 * Store/Load support for PM context.
 */
typedef int (pm_store_fn_t)(void * /*arg*/, const void * /*buf*/,
	uint64_t /*offset*/, uint64_t /*size*/);
	

typedef int (pm_load_fn_t)(void * /*arg*/, void * /*buf*/,
	uint64_t /*offset*/, uint64_t /*size*/);


#include <rte_pm_ac2.h>

/**
 * PM search state structure.
 */
struct rte_pm_state {
	union {
		struct rte_ac2_state ac2;
	};
};

typedef int (* pm_search_bulk_fn_t)(const struct rte_pm_ctx *,
	const struct rte_pm_inbuf *, struct rte_pm_match *, uint32_t);

typedef int (*pm_first_seg_fn_t)(const struct rte_pm_ctx *,
	const struct rte_pm_inbuf *, struct rte_pm_match *, uint32_t,
	struct rte_pm_state *);

typedef int (*pm_next_seg_fn_t)(const struct rte_pm_ctx *,
	const struct rte_pm_inbuf *, struct rte_pm_match *, uint32_t,
	struct rte_pm_state *);

typedef int (*pm_search_seg_fn_t)(const struct rte_pm_ctx *,
	struct rte_pm_match *, uint32_t, struct rte_pm_state *);

struct rte_pm_search_fn {
	pm_search_bulk_fn_t search_bulk;
	/** Pointer to search bulk function. */
	pm_first_seg_fn_t    first_seg;
	/** Pointer to search first segment function. */
	pm_next_seg_fn_t    next_seg;
	/** Pointer to search next segment function. */
	pm_search_seg_fn_t  search_seg;
	/** Pointer to search current segment function. */
};
	
extern const struct rte_pm_search_fn rte_pm_search_fn[];

/**
 * Pattern Match context structure.
 */
struct rte_pm_ctx {
	TAILQ_ENTRY(rte_pm_ctx) next;    /**< Next in list. */
	char     name[RTE_PM_NAMESIZE];  /**< Name of the PM context. */
	struct rte_pm_pattern *patterns; /**< Set of patterns. */
	uint8_t *pattern_buf;            /**< Buffer to store patterns. */
	uint32_t pattern_len;
	/** Actual total length of all patterns in the set. */
	uint32_t pattern_num;
	/**< Actual number of patterns in the set. */
	uint32_t max_pattern_num;        /**< Max allowed number of patterns.*/
	uint32_t max_pattern_len;
	/** Max allowed total pattern length.*/
	int32_t  socket_id;
	uint32_t search_fn;              /**< Runtime search function id. */
	/**< Socket ID to allocate memory from. */
	struct   rte_pm_build_opt bopt;  /**< Build options for the context. */
	union {
		struct rte_pm_ac2 ac2;
	};
};

/**
 * Get search algorithm id by name.
 *
 * @param name
 *   name of the search algorithm.
 * @return
 *   search algorithm id.
 */
enum rte_pm_search
rte_pm_search_type_by_name(const char *name);


/**
 * Create a new Pattern Match context.
 *
 * @param param
 *   Parameters used to create and initialise the pattern match context.
 * @return
 *   Pointer to PM context structure that is used in future PM
 *   operations, or NULL on error, with error code set in rte_errno.
 *   Possible rte_errno errors include:
 *   - E_RTE_NO_TAILQ - no tailq list could be got for the PM context list
 *   - EINVAL - invalid parameter passed to function
 */
struct rte_pm_ctx *
rte_pm_create(const struct rte_pm_param *param);

/**
 * Find an existing PM context object and return a pointer to it.
 *
 * @param name
 *   Name of the PM context as passed to rte_pm_create()
 * @return
 *   Pointer to PM context or NULL if object not found
 *   with rte_errno set appropriately. Possible rte_errno values include:
 *    - ENOENT - value not available for return
 */
struct rte_pm_ctx *
rte_pm_find_existing(const char *name);

/**
 * De-allocate all memory used by PM context.
 *
 * @param pmx
 *   PM context to free
 */
void
rte_pm_free(struct rte_pm_ctx *pmx);

/**
 * Convert  input character string into pattern.
 * Treat "|[hex][hex] ([hex][hex])*|" as binary sequence
 * (same as SNORT 'contents' format).
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
rte_pm_convert_pattern(const char *in, uint8_t *out, uint32_t len);

/**
 * Add patterns to an existing PM context.
 * This function is not multi-thread safe.
 *  
 * @param pmx
 *   PM context to add patterns to.
 * @param pat
 *   Array of patterns to add to the PM context.
 * @param num
 *   Number of elements in the input array of patterns.
 * @return
 *   - -ENOMEM if there is no space in the PM context for these patterns.
 *   - -EINVAL if the parameters are invalid.
 *   - Zero if operation completed successfully.
 */
int
rte_pm_add_patterns(struct rte_pm_ctx *pmx, const struct rte_pm_pattern *pat,
	uint32_t num);

/**
 * Analyze the set of patterns in the context and input build options.
 * At successful completion will update res parameter with all supported
 * and preferred for given pattern set and build options search algorithms.
 * PM context will not be updated.
 *
 * @param pmx
 *   PM context to analyze.
 * @param opt
 *   Build options.
 * @param res
 *   Output containing bitmask of all supported search algorithms and
 *   preferred search algorithm. 
 * @return
 *   - -ENOMEM if couldn't allocate enough memory.
 *   - -EINVAL if the parameters are invalid.
 *   - Negative error code if operation failed.
 *   - Zero if operation completed successfully.
 */
int
rte_pm_analyze(const struct rte_pm_ctx *pmx, const struct rte_pm_build_opt *opt,
	struct rte_pm_search_avail *res);

/**
 * Analyze pattern set and build required for selected search algorithm
 * internal run-time structures.
 * Caller can either specify search algorithm explicitly,
 * or just use RTE_PM_SEARCH_UNDEF to make the PM library select the
 * preferred one for the given set of patterns and build options.
 * The function will analyze the pattern set to make sure that specified by
 * user search algorithm is applicable. If not, then function will return
 * with an error, no internal structures will be created/updated in that case.
 * This function is not multi-thread safe.
 * 
 * @param pmx
 *   PM context to build.
 * @param opt
 *   Build options.
 * @return
 *   - -ENOMEM if couldn't allocate enough memory.
 *   - -EINVAL if the parameters are invalid.
 *   - Negative error code if operation failed.
 *   - Zero if operation completed sucessfully.
 */
int
rte_pm_build(struct rte_pm_ctx *pmx, const struct rte_pm_build_opt *opt);

/**
 * Store (serialize) runtime structures of given PM context,
 * using store function pointed to by fn, which is called
 * with four arguments:
 * int fn(void *arg, const void *buf, uint64_t offset, uint64_t size);
 *   arg - opaque pointer to caller related data.
 *   buf - pointer to the memory containing PM context runtime data to
 *     store(serialize).
 *   offset - offset to store buf.
 *   size - size of the buf.
 * The store function must return zero if operation was completed successfully
 * or negative error code otherwise.
 * This function is not multi-thread safe.
 * 
 * @param pmx
 *   PM context to store(serialize).
 * @param fn
 *   Pointer to the store function provided by caller.
 * @param arg
 *   Opaque pointer to the caller provided data for store function.
 * @return
 *   - Negative error code if operation failed.
 *   - Zero if operation completed successfully.
 */
int
rte_pm_store(const struct rte_pm_ctx *pmx, pm_store_fn_t *fn, void *arg);
	
/**
 * Load (de-serialize) runtime structures to given PM context,
 * using load function pointed to by fn, which is called
 * with four arguments:
 * int fn(void *arg, void *buf, uint64_t offset, uint64_t size);
 *   arg - opaque pointer to caller related data.
 *   buf - pointer to the memory where PM context runtime data should be copied.
 *   offset - offset to load buf from.
 *   size - size of the buf.
 * The load function must return zero if operation was completed successfully
 * or negative error code otherwise.
 * Note that any previously built/loaded runtime data for that PM context
 * will be lost and it's memory will be freed.
 * This function is not multi-thread safe.
 * 
 * @param pmx
 *   PM context to load(de-serialize).
 * @param fn
 *   Pointer to the load function provided by caller.
 * @param arg
 *   Opaque pointer to the caller provided data for load function.
 * @return
 *   - -ENOMEM if couldn't allocate enough memory.
 *   - Negative error code if operation failed.
 *   - Zero if operation completed successfully.
 */
int
rte_pm_load(struct rte_pm_ctx *pmx, pm_load_fn_t *fn, void *arg);
	

/**
 * Search through each input buffer for a match.
 * Match canâ€™t span through different buffers, e.g. each buffer is treated
 * as a separate entity (one-segment packet case).
 * Can find up to one match per buffer.
 * Search results for N-th input buffer, will be written into N-th
 * element of results array.
 * Note, that arrays of input buffers and search results supposed to
 * have the same number of elements.
 * Note, that it is a caller responsibility to ensure that input parameters
 * are valid and point to correct memory locations.
 *
 * @param pmx
 *   PM context to search with.
 * @param buf
 *   Array of input buffers to search through.
 * @param res
 *   Array of search results, one result per input buffer.
 * @param num
 *   Number of elements in the input buffers and search results arrays.
 * @return
 *   - number of matches found.
 */
static inline int
rte_pm_search_bulk(const struct rte_pm_ctx *pmx,
	const struct rte_pm_inbuf *buf, struct rte_pm_match *res,
	uint32_t num)
{
	return (rte_pm_search_fn[pmx->search_fn].search_bulk(pmx, buf,
		res, num));
}

/**
 * Start new search though the chain of logically continuous input buffers.
 * Match can span through different buffers in chain, e.g. multi-segment
 * packet case.
 * The function will stop if one of the following occur:
 *  - end of input segment was reached.
 *  - resnum matches was found.
 * If resnum matches are found before the end of the segment,
 * then function saves its current state, sufficient to resume the search
 * from the same position in future and returns.
 * Note, that it is a caller responsibility to ensure that input parameters
 * are valid and point to correct memory locations.
 * @param pmx
 *   PM context to search with.
 * @param buf
 *   Input buffers to search through.
 * @param res
 *   Array of search results.
 * @param resnum
 *   Number of elements in the array of search results.
 * @param state
 *   Search state used to save current search position and state,
 *   so further search could be resumed from the known state.
 * @return
 *   - number of matches found.
 *
 */
static inline int
rte_pm_search_chain_start(const struct rte_pm_ctx *pmx,
	const struct rte_pm_inbuf *buf, struct rte_pm_match *res,
	uint32_t resnum, struct rte_pm_state *state)
{
	return (rte_pm_search_fn[pmx->search_fn].first_seg(pmx, buf,
		res, resnum, state));
}

/**
 * Continue search to the new segment of the chain of logically continuous
 * input buffers.
 * Match can span through different buffers in chain, e.g. multi-segment
 * packet case.
 * The function will stop if one of the following occur:
 *  - end of input segment was reached.
 *  - resnum matches was found.
 * If resnum matches are found before the end of the segment,
 * then function saves its current state, sufficient to resume the search
 * from the same position in future and returns.
 * Note, that it is a caller responsibility to ensure that input parameters
 * are valid and point to correct memory locations.
 *
 * @param pmx
 *   PM context to search with.
 * @param buf
 *   Input buffers to search through.
 * @param res
 *   Array of search results.
 * @param resnum
 *   Number of elements in the array of search results.
 * @param state
 *   Search state used to save current search position and state,
 *   so further search could be resumed from the known state.
 * @return
 *   - number of matches found.
 *
 */
static inline int
rte_pm_search_chain_next(const struct rte_pm_ctx *pmx,
	const struct rte_pm_inbuf *buf, struct rte_pm_match *res,
	uint32_t resnum, struct rte_pm_state *state)
{
	return (rte_pm_search_fn[pmx->search_fn].next_seg(pmx, buf, res,
		resnum, state));
}

/**
 * Continue search through the same input segment of the chain of logically
 * continuous input buffers.
 * Match can span through different buffers in chain, e.g. multi-segment
 * packet case.
 * The function will stop if one of the following occur:
 *  - end of input segment was reached.
 *  - resnum matches was found.
 * If resnum matches are found before the end of the segment,
 * then function saves its current state, sufficient to resume the search
 * from the same position in future and returns.
 * Note, that it is a caller responsibility to ensure that input parameters
 * are valid and point to correct memory locations.
 *
 * @param pmx
 *   PM context to search with.
 * @param res
 *   Array of search results.
 * @param resnum
 *   Number of elements in the array of search results.
 * @param state
 *   Search state used to save current search position and state,
 *   so further search could be resumed from the known state.
 * @return
 *   - number of matches found.
 *
 */
static inline int
rte_pm_search_chain(const struct rte_pm_ctx *pmx, struct rte_pm_match *res,
	uint32_t resnum, struct rte_pm_state *state)
{
	return (rte_pm_search_fn[pmx->search_fn].search_seg(pmx, res,
		resnum, state));
}


/**
 * Dump an PM context structure to the console.
 * Dump context's pattern set, build options, internal structure, etc.
 *
 * @param pmx
 *   PM context to dump.
 */
void
rte_pm_dump(const struct rte_pm_ctx *pmx);

/**
 * Dump all PM context structures to the console.
 */
void
rte_pm_list_dump(void);


#ifdef __cplusplus
}
#endif

#endif /* _RTE_PM_H_ */
