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

#ifndef _RTE_PMAC_OSDEP_ALONE_H_
#define _RTE_PMAC_OSDEP_ALONE_H_

/**
 * @file
 *
 * RTE Pattern Match/ACL OS dependent file.
 * An example how to build/use PM/ACL library standalone
 * (without rest of DPDK).
 * Don't include that file on it's own, use <rte_pmac_osdep.h>.
 */

#ifdef __cplusplus
extern "C" {
#endif

#define	DUMMY_MACRO	do {} while (0)

/*
 * rte_common related.
 */
#define	__rte_unused	__attribute__((__unused__))

#define RTE_PTR_ADD(ptr, x)	((typeof(ptr))((uintptr_t)(ptr) + (x)))

#define	RTE_PTR_ALIGN_FLOOR(ptr, align) \
	(typeof(ptr))((uintptr_t)(ptr) & ~((uintptr_t)(align) - 1))

#define	RTE_PTR_ALIGN_CEIL(ptr, align) \
	RTE_PTR_ALIGN_FLOOR(RTE_PTR_ADD(ptr, (align) - 1), align)

#define	RTE_PTR_ALIGN(ptr, align)	RTE_PTR_ALIGN_CEIL(ptr, align)

#define	RTE_ALIGN_FLOOR(val, align) \
	(typeof(val))((val) & (~((typeof(val))((align) - 1))))

#define	RTE_ALIGN_CEIL(val, align) \
	RTE_ALIGN_FLOOR(((val) + ((typeof(val))(align) - 1)), align)

#define	RTE_ALIGN(ptr, align)	RTE_ALIGN_CEIL(ptr, align)

#define	RTE_MIN(a, b)	({ \
		typeof (a) _a = (a); \
		typeof (b) _b = (b); \
		_a < _b ? _a : _b;   \
	})

/**
 * Searches the input parameter for the least significant set bit
 * (starting from zero).
 * If a least significant 1 bit is found, its bit index is returned.
 * If the content of the input paramer is zero, then the content of the return
 * value is undefined.
 * @param v
 *     input parameter, should not be zero.
 * @return
 *     least significant set bit in the input parameter.
 */
static inline uint32_t
rte_bsf32(uint32_t v)
{
	asm("bsf %1,%0"
		: "=r" (v)
		: "rm" (v));
	return (v);
}

/*
 * rte_cycles related.
 */
#define rte_rdtsc	_rdtsc

/*
 * rte_lcore related.
 */
#define rte_lcore_id()	(0)

/*
 * rte_errno related.
 */
#define	rte_errno	errno
#define	E_RTE_NO_TAILQ	(-1)

/*
 * rte_rwlock related.
 */
#define	rte_rwlock_read_lock(x)		DUMMY_MACRO
#define	rte_rwlock_read_unlock(x)	DUMMY_MACRO
#define	rte_rwlock_write_lock(x)	DUMMY_MACRO
#define	rte_rwlock_write_unlock(x)	DUMMY_MACRO

/*
 * rte_memory related.
 */
#define	SOCKET_ID_ANY	-1                  /**< Any NUMA socket. */
#define	CACHE_LINE_SIZE	64                  /**< Cache line size. */
#define	CACHE_LINE_MASK	(CACHE_LINE_SIZE-1) /**< Cache line mask. */

/**
 * Force alignment to cache line.
 */
#define	__rte_cache_aligned	__attribute__((__aligned__(CACHE_LINE_SIZE)))


/*
 * rte_byteorder related.
 */
#define	rte_le_to_cpu_16(x)	(x)
#define	rte_le_to_cpu_32(x)	(x)

#define rte_cpu_to_be_16(x)	\
	(((x) & UINT8_MAX) << CHAR_BIT | ((x) >> CHAR_BIT & UINT8_MAX))
#define rte_cpu_to_be_32(x)	_bswap(x)

/*
 * rte_branch_prediction related.
 */
#ifndef	likely
#define	likely(x)	__builtin_expect((x),1)
#endif	/* likely */

#ifndef	unlikely
#define	unlikely(x)	__builtin_expect((x),0)
#endif	/* unlikely */


/*
 * rte_tailq related.
 */
static inline void *
rte_dummy_tailq(void)
{
	static __thread TAILQ_HEAD(rte_dummy_head, rte_dummy) dummy_head;
	TAILQ_INIT(&dummy_head);
	return (&dummy_head);
}

#define	RTE_TAILQ_LOOKUP_BY_IDX(idx, struct_name)	rte_dummy_tailq()	

#define RTE_EAL_TAILQ_REMOVE(idx, type, elm)	DUMMY_MACRO

/*
 * rte_string related
 */
#define	rte_snprintf(str, len, frmt, args...)	snprintf(str, len, frmt, ##args)

/*
 * rte_log related
 */
#define RTE_LOG(l, t, fmt, args...)	printf(fmt, ##args)

/*
 * rte_malloc related
 */
#define	rte_free(x)	free(x)

static inline void *
rte_zmalloc_socket(__rte_unused const char *type, size_t size, unsigned align,
	__rte_unused int socket)
{
	void *ptr;
	int rc;

	if ((rc = posix_memalign(&ptr, align, size)) != 0) {
		rte_errno = rc;
		return (NULL);
	}

	memset(ptr, 0, size);
	return (ptr);
}

/*
 * rte_debug related
 */
#define	rte_panic(fmt, args...)	do {         \
	RTE_LOG(CRIT, EAL, fmt, ##args);     \
	abort();                             \
} while (0)

#define	rte_exit(err, fmt, args...)	do { \
	RTE_LOG(CRIT, EAL, fmt, ##args);     \
	exit(err);                           \
} while (0)

#ifdef __cplusplus
}
#endif

#endif /* _RTE_PMAC_OSDEP_ALONE_H_ */
