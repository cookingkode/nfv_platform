/*
 *
 * Copyright 2014 Jyotiswarup Raiturkar
 * All rights reserved
 */

#ifndef _IPFRAG_H_
#define _IPFRAG_H_

#include <stdint.h>
#include <rte_common.h>
#include <rte_byteorder.h>
#include <rte_log.h>
#include <rte_memory.h>
#include <rte_memcpy.h>
#include <rte_tailq.h>
#include <rte_launch.h>
#include <rte_atomic.h>
#include <rte_cycles.h>
#include <rte_prefetch.h>
#include <rte_lcore.h>
#include <rte_branch_prediction.h>
#include <rte_interrupts.h>
#include <rte_random.h>
#include <rte_debug.h>
#include <rte_ring.h>
#include <rte_mempool.h>
#include <rte_mbuf.h>
#include <rte_malloc.h>
#include <rte_ip.h>
#include <rte_tcp.h>
#include <rte_udp.h>

enum {
        LAST_FRAG_IDX,
        FIRST_FRAG_IDX,
        MIN_FRAG_NUM,
        MAX_FRAG_NUM = 4,
};

struct ip_frag {
        uint16_t ofs;
        uint16_t len;
        struct rte_mbuf *mb;
};

/*
 * Use <src addr, dst_addr, id> to uniquely indetify fragmented datagram.
 */
struct ip_frag_key {
        uint64_t  src_dst;
        uint32_t  id;
};

#define IPV4_FRAG_KEY_INVALIDATE(k)     ((k)->src_dst = 0)
#define IPV4_FRAG_KEY_EMPTY(k)          ((k)->src_dst == 0)

#define IPV4_FRAG_KEY_CMP(k1, k2)       \
        (((k1)->src_dst ^ (k2)->src_dst) | ((k1)->id ^ (k2)->id))

/*
 * Fragmented packet to reassemble.
 * First two entries in the frags[] array are for the last and first fragments.
 */
struct frag_pkt {
        TAILQ_ENTRY(frag_pkt) lru;   /* LRU list */
        struct ip_frag_key    key;
        uint64_t              start;       /* creation timestamp */
        uint32_t              total_size;  /* expected reassembled size */
        uint32_t              frag_size;   /* size of fragments received */
        uint32_t              last_idx;    /* index of next entry to fill */
        struct ip_frag        frags[MAX_FRAG_NUM];
} __rte_cache_aligned;



TAILQ_HEAD(ipv4_pkt_list, frag_pkt);

/*
 * The basic idea is to use two hash functions and <bucket_entries>
 * associativity. This provides 2 * <bucket_entries> possible locations in
 * the hash table for each key. Sort of simplified Cuckoo hashing,
 * when the collision occurs and all 2 * <bucket_entries> are occupied,
 * instead of resinserting existing keys into alternative locations, we just
 * return a faiure.
 * Another thing timing: entries that resides in the table longer then
 * <max_cycles> are considered as invalid, and could be removed/replaced
 * byt the new ones.
 * <key, data> pair is stored together, all add/update/lookup opearions are not
 * MT safe.
 */


struct frag_tbl_stat {
        uint64_t find_num;      /* total # of find/insert attempts. */
        uint64_t add_num;       /* # of add ops. */
        uint64_t del_num;       /* # of del ops. */
        uint64_t reuse_num;     /* # of reuse (del/add) ops. */
        uint64_t fail_total;    /* total # of add failures. */
        uint64_t fail_nospace;  /* # of 'no space' add failures. */
} __rte_cache_aligned;

struct frag_tbl {
        uint64_t             max_cycles;      /* ttl for table entries. */
        uint32_t             entry_mask;      /* hash value mask. */
        uint32_t             max_entries;     /* max entries allowed. */
        uint32_t             use_entries;     /* entries in use. */
        uint32_t             bucket_entries;  /* hash assocaitivity. */
        uint32_t             nb_entries;      /* total size of the table. */
        uint32_t             nb_buckets;      /* num of associativity lines. */
        struct frag_pkt *last;           /* last used entry. */
        struct ipv4_pkt_list lru;             /* LRU list for table entries. */
        struct frag_tbl_stat stat;       /* statistics counters. */
        struct frag_pkt pkt[0];          /* hash table. */
};

#define MAX_PKT_BURST 32


/*
 * Create a new IPV4 Frag table.
 * @param bucket_num
 *  Number of buckets in the hash table.
 * @param bucket_entries
 *  Number of entries per bucket (e.g. hash associativity).
 *  Should be power of two.
 * @param max_entries
 *   Maximum number of entries that could be stored in the table.
 *   The value should be less or equal then bucket_num * bucket_entries.
 * @param max_cycles
 *   Maximum TTL in cycles for each fragmented packet.
 * @param socket_id
 *  The *socket_id* argument is the socket identifier in the case of
 *  NUMA. The value can be *SOCKET_ID_ANY* if there is no NUMA constraints.
 * @return
 *   The pointer to the new allocated mempool, on success. NULL on error.
 */

struct frag_tbl *
vb_ipv4_frag_tbl_create(uint32_t bucket_num, uint32_t bucket_entries,
                        uint32_t max_entries, uint64_t max_cycles, int socket_id);


/*
 * Easy API : Create a new IPV4 Frag table.
 * @param socket_id
 *  The *socket_id* argument is the socket identifier in the case of
 *  NUMA. The value can be *SOCKET_ID_ANY* if there is no NUMA constraints.
 * @return
 *   The pointer to the new allocated mempool, on success. NULL on error.
 */
static inline struct frag_tbl *
vb_ipv4_frag_tbl_create_easy(int socket_id) {
        uint64_t frag_cycles = (rte_get_tsc_hz() + MS_PER_S - 1);
        return vb_ipv4_frag_tbl_create(MS_PER_S , 2, MS_PER_S, frag_cycles,
                                       socket_id);

}



/*
 * Process new mbuf with fragment of IPV4 packet.
 * Incoming mbuf should have it's l2_len/l3_len fields setuped correclty.
 * @param tbl
 *   Table where to lookup/add the fragmented packet.
 * @param mb
 *   Incoming mbuf with IPV4 fragment.
 * @param tms
 *   Fragment arrival timestamp.
 * @param ip_hdr
 *   Pointer to the IPV4 header inside the fragment.
 * @param ip_ofs
 *   Fragment's offset (as extracted from the header).
 * @param ip_flag
 *   Fragment's MF flag.
 * @return
 *   Pointer to mbuf for reassebled packet, or NULL if:
 *   - an error occured.
 *   - not all fragments of the packet are collected yet.
 */
inline struct rte_mbuf *
vb_ipv4_frag_in(struct frag_tbl *tbl,
                struct rte_mbuf *mb, uint64_t tms, struct ipv4_hdr *ip_hdr,
                uint16_t ip_ofs, uint16_t ip_flag);
#endif /* _IPFRAG_H_ */
