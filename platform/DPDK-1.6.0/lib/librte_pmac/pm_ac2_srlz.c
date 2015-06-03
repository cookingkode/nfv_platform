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

static const uint32_t ac_serialize_type = 0xAC2;
static const uint32_t ac_serialize_version = 1;

#define	STORE_FIELD(fn, arg, buf, ofs, sz)	do {                \
	int rc;                                                     \
	if ((rc = (*(fn))(arg, buf, ofs, sz)) != 0) {               \
		RTE_LOG(ERR, PMAC,                                  \
			"%s: pm_store_fn(%p, %" PRIu64 ", %" PRIu64 \
			") failed with error code: %d\n",           \
			__func__, arg, ofs, (uint64_t)(sz), rc);    \
		return (rc);                                        \
	}                                                           \
	(ofs) += (sz);                                              \
} while (0)

#define	LOAD_FIELD(fn, arg, buf, ofs, sz)	do {                \
	int rc;                                                     \
	if ((rc = (*(fn))(arg, buf, ofs, sz)) != 0) {               \
		RTE_LOG(ERR, PMAC,                                  \
			"%s: pm_load_fn(%p, %" PRIu64 ", %" PRIu64  \
			") failed with error code: %d\n",           \
			__func__, arg, ofs, (uint64_t)(sz), rc);    \
		return (rc);                                        \
	}                                                           \
	(ofs) += (sz);                                              \
} while (0)

int
ac_store(const struct rte_pm_ctx *pmx, pm_store_fn_t *fn, void *arg) 
{
	uint64_t ofs;

	ofs = 0;

	/* store common PM context fields. */

	STORE_FIELD(fn, arg, &ac_serialize_type, ofs,
		sizeof(ac_serialize_type));
	STORE_FIELD(fn, arg, &ac_serialize_version, ofs,
		sizeof(ac_serialize_version));
	STORE_FIELD(fn, arg, pmx->name, ofs, sizeof(pmx->name));
	STORE_FIELD(fn, arg, &pmx->search_fn, ofs,
		sizeof(pmx->search_fn));
	STORE_FIELD(fn, arg, &pmx->bopt, ofs, sizeof(pmx->bopt));

	/* store AC specific fields. */

	/* store amount of memory required */
	STORE_FIELD(fn, arg, &pmx->ac2.mem_sz, ofs, sizeof(pmx->ac2.mem_sz));
	STORE_FIELD(fn, arg, &pmx->ac2.ofs_match, ofs,
		sizeof(pmx->ac2.ofs_match));
	STORE_FIELD(fn, arg, &pmx->ac2.ofs_l3, ofs, sizeof(pmx->ac2.ofs_l3));
	STORE_FIELD(fn, arg, &pmx->ac2.ofs_trie32, ofs,
		sizeof(pmx->ac2.ofs_trie32));
	STORE_FIELD(fn, arg, &pmx->ac2.ofs_bloom, ofs,
		sizeof(pmx->ac2.ofs_bloom));

	/* store runtime structures */
	STORE_FIELD(fn, arg, &pmx->ac2.one_byte_low, ofs,
		sizeof(pmx->ac2.one_byte_low));
	STORE_FIELD(fn, arg, &pmx->ac2.one_byte_high, ofs,
		sizeof(pmx->ac2.one_byte_high));
	STORE_FIELD(fn, arg, &pmx->ac2.sse_mask, ofs,
		sizeof(pmx->ac2.sse_mask));
	STORE_FIELD(fn, arg, &pmx->ac2.input_mask, ofs,
		sizeof(pmx->ac2.input_mask));
	STORE_FIELD(fn, arg, &pmx->ac2.first_pattern, ofs,
		sizeof(pmx->ac2.first_pattern));
	STORE_FIELD(fn, arg, &pmx->ac2.root_index, ofs,
		sizeof(pmx->ac2.root_index));
	STORE_FIELD(fn, arg, &pmx->ac2.shift1, ofs, sizeof(pmx->ac2.shift1));
	STORE_FIELD(fn, arg, &pmx->ac2.shift2, ofs, sizeof(pmx->ac2.shift2));
	STORE_FIELD(fn, arg, &pmx->ac2.shift3, ofs, sizeof(pmx->ac2.shift3));
		
	STORE_FIELD(fn, arg, pmx->ac2.mem, ofs, pmx->ac2.mem_sz);
	return (0);
}


int
ac_load(struct rte_pm_ctx *pmx, pm_load_fn_t *fn, void *arg) 
{
	uint8_t *ptr;
	uint64_t ofs;
	uint32_t type, version;
	char name[RTE_PM_NAMESIZE];

	ofs = 0;

	LOAD_FIELD(fn, arg, &type, ofs, sizeof(type));
	LOAD_FIELD(fn, arg, &version, ofs, sizeof(version));

	if (type != ac_serialize_type || version > ac_serialize_version) {
		RTE_LOG(ERR, PMAC,
			"%s: unsupported  type(%#x) or version(%#x)\n",
			__func__, type, version);
		return (-EINVAL);
	}

	LOAD_FIELD(fn, arg, name, ofs, sizeof(name));
	LOAD_FIELD(fn, arg, &pmx->search_fn, ofs, sizeof(pmx->search_fn));
	LOAD_FIELD(fn, arg, &pmx->bopt, ofs, sizeof(pmx->bopt));

	LOAD_FIELD(fn, arg, &pmx->ac2.mem_sz, ofs, sizeof(pmx->ac2.mem_sz));
	LOAD_FIELD(fn, arg, &pmx->ac2.ofs_match, ofs,
		sizeof(pmx->ac2.ofs_match));
	LOAD_FIELD(fn, arg, &pmx->ac2.ofs_l3, ofs, sizeof(pmx->ac2.ofs_l3));
	LOAD_FIELD(fn, arg, &pmx->ac2.ofs_trie32, ofs,
		sizeof(pmx->ac2.ofs_trie32));
	LOAD_FIELD(fn, arg, &pmx->ac2.ofs_bloom, ofs,
		sizeof(pmx->ac2.ofs_bloom));

	/* load runtime structures */
	LOAD_FIELD(fn, arg, &pmx->ac2.one_byte_low, ofs,
		sizeof(pmx->ac2.one_byte_low));
	LOAD_FIELD(fn, arg, &pmx->ac2.one_byte_high, ofs,
		sizeof(pmx->ac2.one_byte_high));
	LOAD_FIELD(fn, arg, &pmx->ac2.sse_mask, ofs,
		sizeof(pmx->ac2.sse_mask));
	LOAD_FIELD(fn, arg, &pmx->ac2.input_mask, ofs,
		sizeof(pmx->ac2.input_mask));
	LOAD_FIELD(fn, arg, &pmx->ac2.first_pattern, ofs,
		sizeof(pmx->ac2.first_pattern));
	LOAD_FIELD(fn, arg, &pmx->ac2.root_index, ofs,
		sizeof(pmx->ac2.root_index));
	LOAD_FIELD(fn, arg, &pmx->ac2.shift1, ofs, sizeof(pmx->ac2.shift1));
	LOAD_FIELD(fn, arg, &pmx->ac2.shift2, ofs, sizeof(pmx->ac2.shift2));
	LOAD_FIELD(fn, arg, &pmx->ac2.shift3, ofs, sizeof(pmx->ac2.shift3));

	RTE_LOG(DEBUG, PMAC, "%s(%s) requested %" PRIu64 " bytes for "
		"runtime memory structures\n",
		__func__, name, pmx->ac2.mem_sz);

	/* allocate memory for runtime structures. */
	if (pmx->ac2.mem_sz > SIZE_MAX ||
			(ptr = rte_zmalloc_socket(pmx->name, pmx->ac2.mem_sz,
			CACHE_LINE_SIZE, pmx->socket_id)) == NULL) {
		RTE_LOG(ERR, PMAC,
			"allocation of %" PRIu64 " bytes on socket %d "
			"for %s failed\n",
			pmx->ac2.mem_sz, pmx->socket_id, pmx->name);
		return (-ENOMEM);
	}

	/* setup pointers to runtime data structures. */
	pmx->ac2.match_index = (struct rte_ac2_match_data *)
		(ptr + pmx->ac2.ofs_match);
        pmx->ac2.bloom_l3 = (rte_ac2_index_t *)(ptr + pmx->ac2.ofs_l3);
        pmx->ac2.bloom_trie32 = (rte_ac2_index_t *)(ptr + pmx->ac2.ofs_trie32);
        pmx->ac2.bloom = ptr + pmx->ac2.ofs_bloom;
        pmx->ac2.mem = ptr;

	/* load runtime memory strucutres. */
	LOAD_FIELD(fn, arg, pmx->ac2.mem, ofs, pmx->ac2.mem_sz);
	return (0);
}

