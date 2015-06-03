/*
 *
 * Copyright 2014 Jyotiswarup Raiturkar
 * All rights reserved
 */

#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <sys/types.h>
#include <string.h>
#include <sys/queue.h>
#include <stdarg.h>
#include <errno.h>
#include <getopt.h>
#include <signal.h>

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
#include "ip_frag.h"
#include <rte_jhash.h>
#ifdef RTE_MACHINE_CPUFLAG_SSE4_2
#include <rte_hash_crc.h>
#endif /* RTE_MACHINE_CPUFLAG_SSE4_2 */





/* logging macros. */

#ifdef IP_FRAG_DEBUG
#define	VB_FRAG_LOG(lvl, fmt, args...)	RTE_LOG(lvl, USER1, fmt, ##args)
#else
#define	VB_FRAG_LOG(lvl, fmt, args...)	do {} while(0)
#endif /* IP_FRAG_DEBUG */

/*
 * Forward declarations
 */
static inline void
frag_reset(struct frag_pkt *fp, uint64_t tms);

/*
 * Fragment Table Start
 */

#define PRIME_VALUE     0xeaad8405


#define	IPV4_FRAG_TBL_POS(tbl, sig)	\
	((tbl)->pkt + ((sig) & (tbl)->entry_mask))

#define	IPV4_FRAG_HASH_FNUM	2

#ifdef IPV4_FRAG_TBL_STAT
#define	IPV4_FRAG_TBL_STAT_UPDATE(s, f, v)	((s)->f += (v))
#else
#define	IPV4_FRAG_TBL_STAT_UPDATE(s, f, v)	do {} while (0)
#endif /* IPV4_FRAG_TBL_STAT */

static inline void
frag_hash(const struct ip_frag_key *key, uint32_t *v1, uint32_t *v2)
{
        uint32_t v;
        const uint32_t *p;

        p = (const uint32_t *)&key->src_dst;

#ifdef RTE_MACHINE_CPUFLAG_SSE4_2
        v = rte_hash_crc_4byte(p[0], PRIME_VALUE);
        v = rte_hash_crc_4byte(p[1], v);
        v = rte_hash_crc_4byte(key->id, v);
#else

        v = rte_jhash_3words(p[0], p[1], key->id, PRIME_VALUE);
#endif /* RTE_MACHINE_CPUFLAG_SSE4_2 */

        *v1 =  v;
        *v2 = (v << 7) + (v >> 14);
}

/*
 * Update the table, after we finish processing it's entry.
 */
static inline void
frag_inuse(struct frag_tbl *tbl, const struct  frag_pkt *fp)
{
        if (IPV4_FRAG_KEY_EMPTY(&fp->key)) {
                TAILQ_REMOVE(&tbl->lru, fp, lru);
                tbl->use_entries--;
        }
}

static inline void
frag_free(struct frag_pkt *fp)
{
        uint32_t i;

        for (i = 0; i != fp->last_idx; i++) {
                if (fp->frags[i].mb != NULL) {
                        rte_prefetch0(fp->frags[i].mb);
                }
        }

        for (i = 0; i != fp->last_idx; i++) {
                if (fp->frags[i].mb != NULL) {
                        rte_pktmbuf_free(fp->frags[i].mb);
                        fp->frags[i].mb = NULL;
                }
        }
}


/*
 * For the given key, try to find an existing entry.
 * If such entry doesn't exist, will return free and/or timed-out entry,
 * that can be used for that key.
 */
static inline struct  frag_pkt *
frag_lookup(struct frag_tbl *tbl,
            const struct ip_frag_key *key, uint64_t tms,
            struct frag_pkt **free, struct frag_pkt **stale) {
        struct frag_pkt *p1, *p2;
        struct frag_pkt *empty, *old;
        uint64_t max_cycles;
        uint32_t i, assoc, sig1, sig2;

        empty = NULL;
        old = NULL;

        max_cycles = tbl->max_cycles;
        assoc = tbl->bucket_entries;

        if (tbl->last != NULL && IPV4_FRAG_KEY_CMP(&tbl->last->key, key) == 0)
                return (tbl->last);

        frag_hash(key, &sig1, &sig2);
        p1 = IPV4_FRAG_TBL_POS(tbl, sig1);
        p2 = IPV4_FRAG_TBL_POS(tbl, sig2);

        for (i = 0; i != assoc; i++) {

                VB_FRAG_LOG(DEBUG, "%s:%d:\n"
                            "tbl: %p, max_entries: %u, use_entries: %u\n"
                            "frag_pkt line0: %p, index: %u from %u\n"
                            "key: <%" PRIx64 ", %#x>, start: %" PRIu64 "\n",
                            __func__, __LINE__,
                            tbl, tbl->max_entries, tbl->use_entries,
                            p1, i, assoc,
                            p1[i].key.src_dst, p1[i].key.id, p1[i].start);

                if (IPV4_FRAG_KEY_CMP(&p1[i].key, key) == 0)
                        return (p1 + i);
                else if (IPV4_FRAG_KEY_EMPTY(&p1[i].key))
                        empty = (empty == NULL) ? (p1 + i) : empty;
                else if (max_cycles + p1[i].start < tms)
                        old = (old == NULL) ? (p1 + i) : old;

                VB_FRAG_LOG(DEBUG, "%s:%d:\n"
                            "tbl: %p, max_entries: %u, use_entries: %u\n"
                            "frag_pkt line1: %p, index: %u from %u\n"
                            "key: <%" PRIx64 ", %#x>, start: %" PRIu64 "\n",
                            __func__, __LINE__,
                            tbl, tbl->max_entries, tbl->use_entries,
                            p2, i, assoc,
                            p2[i].key.src_dst, p2[i].key.id, p2[i].start);

                if (IPV4_FRAG_KEY_CMP(&p2[i].key, key) == 0)
                        return (p2 + i);
                else if (IPV4_FRAG_KEY_EMPTY(&p2[i].key))
                        empty = (empty == NULL) ?( p2 + i) : empty;
                else if (max_cycles + p2[i].start < tms)
                        old = (old == NULL) ? (p2 + i) : old;
        }

        *free = empty;
        *stale = old;
        return (NULL);
}

static inline void
frag_tbl_del(struct frag_tbl *tbl,
             struct frag_pkt *fp)
{
        frag_free(fp);
        IPV4_FRAG_KEY_INVALIDATE(&fp->key);
        TAILQ_REMOVE(&tbl->lru, fp, lru);
        tbl->use_entries--;
        IPV4_FRAG_TBL_STAT_UPDATE(&tbl->stat, del_num, 1);
}

static inline void
frag_tbl_add(struct frag_tbl *tbl,  struct frag_pkt *fp,
             const struct ip_frag_key *key, uint64_t tms)
{
        fp->key = key[0];
        frag_reset(fp, tms);
        TAILQ_INSERT_TAIL(&tbl->lru, fp, lru);
        tbl->use_entries++;
        IPV4_FRAG_TBL_STAT_UPDATE(&tbl->stat, add_num, 1);
}

static inline void
frag_tbl_reuse(struct frag_tbl *tbl,
               struct frag_pkt *fp, uint64_t tms)
{
        frag_free(fp);
        frag_reset(fp, tms);
        TAILQ_REMOVE(&tbl->lru, fp, lru);
        TAILQ_INSERT_TAIL(&tbl->lru, fp, lru);
        IPV4_FRAG_TBL_STAT_UPDATE(&tbl->stat, reuse_num, 1);
}

/*
 * Find an entry in the table for the corresponding fragment.
 * If such entry is not present, then allocate a new one.
 * If the entry is stale, then free and reuse it.
 */
static inline struct frag_pkt *
frag_find(struct frag_tbl *tbl,
          const struct ip_frag_key *key, uint64_t tms) {
        struct frag_pkt *pkt, *free, *stale, *lru;
        uint64_t max_cycles;

        /*
         * Actually the two line below are totally redundant.
         * they are here, just to make gcc 4.6 happy.
         */
        free = NULL;
        stale = NULL;
        max_cycles = tbl->max_cycles;

        IPV4_FRAG_TBL_STAT_UPDATE(&tbl->stat, find_num, 1);

        if ((pkt = frag_lookup(tbl, key, tms, &free, &stale)) == NULL) {

                /*timed-out entry, free and invalidate it*/
                if (stale != NULL) {
                        frag_tbl_del(tbl,  stale);
                        free = stale;

                        /*
                         * we found a free entry, check if we can use it.
                         * If we run out of free entries in the table, then
                         * check if we have a timed out entry to delete.
                         */
                } else if (free != NULL &&
                           tbl->max_entries <= tbl->use_entries) {
                        lru = TAILQ_FIRST(&tbl->lru);
                        if (max_cycles + lru->start < tms) {
                                frag_tbl_del(tbl,  lru);
                        } else {
                                free = NULL;
                                IPV4_FRAG_TBL_STAT_UPDATE(&tbl->stat,
                                                          fail_nospace, 1);
                        }
                }

                /* found a free entry to reuse. */
                if (free != NULL) {
                        frag_tbl_add(tbl,  free, key, tms);
                        pkt = free;
                }

                /*
                 * we found the flow, but it is already timed out,
                 * so free associated resources, reposition it in the LRU list,
                 * and reuse it.
                 */
        } else if (max_cycles + pkt->start < tms) {
                frag_tbl_reuse(tbl,  pkt, tms);
        }

        IPV4_FRAG_TBL_STAT_UPDATE(&tbl->stat, fail_total, (pkt == NULL));

        tbl->last = pkt;
        return (pkt);
}

static inline void
frag_tbl_destroy( struct frag_tbl *tbl)
{
        rte_free(tbl);
}


#if 0
Not sure currently..
static void
frag_tbl_dump_stat(FILE *f, const struct frag_tbl *tbl)
{
        uint64_t fail_total, fail_nospace;

        fail_total = tbl->stat.fail_total;
        fail_nospace = tbl->stat.fail_nospace;

        fprintf(f, "max entries:\t%u;\n"
                "entries in use:\t%u;\n"
                "finds/inserts:\t%" PRIu64 ";\n"
                "entries added:\t%" PRIu64 ";\n"
                "entries deleted by timeout:\t%" PRIu64 ";\n"
                "entries reused by timeout:\t%" PRIu64 ";\n"
                "total add failures:\t%" PRIu64 ";\n"
                "add no-space failures:\t%" PRIu64 ";\n"
                "add hash-collisions failures:\t%" PRIu64 ";\n",
                tbl->max_entries,
                tbl->use_entries,
                tbl->stat.find_num,
                tbl->stat.add_num,
                tbl->stat.del_num,
                tbl->stat.reuse_num,
                fail_total,
                fail_nospace,
                fail_total - fail_nospace);
}
#endif

/*
 * Fragment Table End
 */


static inline void
frag_reset(struct frag_pkt *fp, uint64_t tms)
{
        static const struct ip_frag zero_frag = {
                .ofs = 0,
                .len = 0,
                .mb = NULL,
        };

        fp->start = tms;
        fp->total_size = UINT32_MAX;
        fp->frag_size = 0;
        fp->last_idx = MIN_FRAG_NUM;
        fp->frags[LAST_FRAG_IDX] = zero_frag;
        fp->frags[FIRST_FRAG_IDX] = zero_frag;
}


/*
 * Helper function.
 * Takes 2 mbufs that represents two framents of the same packet and
 * chains them into one mbuf.
 */
static inline void
frag_chain(struct rte_mbuf *mn, struct rte_mbuf *mp)
{
        struct rte_mbuf *ms;

        /* adjust start of the last fragment data. */
        rte_pktmbuf_adj(mp, (uint16_t)(mp->pkt.vlan_macip.f.l2_len +
                                       mp->pkt.vlan_macip.f.l3_len));

        /* chain two fragments. */
        ms = rte_pktmbuf_lastseg(mn);
        ms->pkt.next = mp;

        /* accumulate number of segments and total length. */
        mn->pkt.nb_segs = (uint8_t)(mn->pkt.nb_segs + mp->pkt.nb_segs);
        mn->pkt.pkt_len += mp->pkt.pkt_len;

        /* reset pkt_len and nb_segs for chained fragment. */
        mp->pkt.pkt_len = mp->pkt.data_len;
        mp->pkt.nb_segs = 1;
}

/*
 * Reassemble fragments into one packet.
 */
static inline struct rte_mbuf *
frag_reassemble(const struct frag_pkt *fp) {
        struct ipv4_hdr *ip_hdr;
        struct rte_mbuf *m, *prev;
        uint32_t i, n, ofs, first_len;

        first_len = fp->frags[FIRST_FRAG_IDX].len;
        n = fp->last_idx - 1;

        /*start from the last fragment. */
        m = fp->frags[LAST_FRAG_IDX].mb;
        ofs = fp->frags[LAST_FRAG_IDX].ofs;

        while (ofs != first_len) {

                prev = m;

                for (i = n; i != FIRST_FRAG_IDX && ofs != first_len; i--) {

                        /* previous fragment found. */
                        if(fp->frags[i].ofs + fp->frags[i].len == ofs) {

                                frag_chain(fp->frags[i].mb, m);

                                /* update our last fragment and offset. */
                                m = fp->frags[i].mb;
                                ofs = fp->frags[i].ofs;
                        }
                }

                /* error - hole in the packet. */
                if (m == prev) {
                        return (NULL);
                }
        }

        /* chain with the first fragment. */
        frag_chain(fp->frags[FIRST_FRAG_IDX].mb, m);
        m = fp->frags[FIRST_FRAG_IDX].mb;

        /* update mbuf fields for reassembled packet. */
        m->ol_flags |= PKT_TX_IP_CKSUM;

        /* update ipv4 header for the reassmebled packet */
        ip_hdr = (struct ipv4_hdr*)(rte_pktmbuf_mtod(m, uint8_t *) +
                                    m->pkt.vlan_macip.f.l2_len);

        ip_hdr->total_length = rte_cpu_to_be_16((uint16_t)(fp->total_size +
                                                m->pkt.vlan_macip.f.l3_len));
        ip_hdr->fragment_offset = (uint16_t)(ip_hdr->fragment_offset &
                                             rte_cpu_to_be_16(IPV4_HDR_DF_FLAG));
        ip_hdr->hdr_checksum = 0;

        return (m);
}

static inline struct rte_mbuf *
frag_process(struct frag_pkt *fp,
             struct rte_mbuf *mb, uint16_t ofs, uint16_t len, uint16_t more_frags) {
        uint32_t idx;

        fp->frag_size += len;

        /* this is the first fragment. */
        if (ofs == 0) {
                idx = (fp->frags[FIRST_FRAG_IDX].mb == NULL) ?
                      FIRST_FRAG_IDX : UINT32_MAX;

                /* this is the last fragment. */
        } else if (more_frags == 0) {
                fp->total_size = ofs + len;
                idx = (fp->frags[LAST_FRAG_IDX].mb == NULL) ?
                      LAST_FRAG_IDX : UINT32_MAX;

                /* this is the intermediate fragment. */
        } else if ((idx = fp->last_idx) <
                   sizeof (fp->frags) / sizeof (fp->frags[0])) {
                fp->last_idx++;
        }

        /*
         * errorneous packet: either exceeed max allowed number of fragments,
         * or duplicate first/last fragment encountered.
         */
        if (idx >= sizeof (fp->frags) / sizeof (fp->frags[0])) {

                /* report an error. */
                VB_FRAG_LOG(DEBUG, "%s:%d invalid fragmented packet:\n"
                            "frag_pkt: %p, key: <%" PRIx64 ", %#x>, "
                            "total_size: %u, frag_size: %u, last_idx: %u\n"
                            "first fragment: ofs: %u, len: %u\n"
                            "last fragment: ofs: %u, len: %u\n\n",
                            __func__, __LINE__,
                            fp, fp->key.src_dst, fp->key.id,
                            fp->total_size, fp->frag_size, fp->last_idx,
                            fp->frags[FIRST_FRAG_IDX].ofs,
                            fp->frags[FIRST_FRAG_IDX].len,
                            fp->frags[LAST_FRAG_IDX].ofs,
                            fp->frags[LAST_FRAG_IDX].len);

                /* free all fragments, invalidate the entry. */
                IPV4_FRAG_KEY_INVALIDATE(&fp->key);
                frag_free(fp);
                rte_pktmbuf_free(mb); /* freeing packet */
                return (NULL);
        }

        fp->frags[idx].ofs = ofs;
        fp->frags[idx].len = len;
        fp->frags[idx].mb = mb;

        mb = NULL;

        /* not all fragments are collected yet. */
        if (likely (fp->frag_size < fp->total_size)) {
                return (mb);

                /* if we collected all fragments, then try to reassemble. */
        } else if (fp->frag_size == fp->total_size &&
                   fp->frags[FIRST_FRAG_IDX].mb != NULL) {
                mb = frag_reassemble(fp);
        }

        /* errorenous set of fragments. */
        if (mb == NULL) {

                /* report an error. */
                VB_FRAG_LOG(DEBUG, "%s:%d invalid fragmented packet:\n"
                            "frag_pkt: %p, key: <%" PRIx64 ", %#x>, "
                            "total_size: %u, frag_size: %u, last_idx: %u\n"
                            "first fragment: ofs: %u, len: %u\n"
                            "last fragment: ofs: %u, len: %u\n\n",
                            __func__, __LINE__,
                            fp, fp->key.src_dst, fp->key.id,
                            fp->total_size, fp->frag_size, fp->last_idx,
                            fp->frags[FIRST_FRAG_IDX].ofs,
                            fp->frags[FIRST_FRAG_IDX].len,
                            fp->frags[LAST_FRAG_IDX].ofs,
                            fp->frags[LAST_FRAG_IDX].len);

                /* free associated resources. */
                frag_free(fp);
                rte_pktmbuf_free(mb); /* freeing packet */
        }

        /* we are done with that entry, invalidate it. */
        IPV4_FRAG_KEY_INVALIDATE(&fp->key);
        return (mb);
}

/*
 * External Functions
 *
 */


struct frag_tbl *
vb_ipv4_frag_tbl_create(uint32_t bucket_num, uint32_t bucket_entries,
                        uint32_t max_entries, uint64_t max_cycles, int socket_id) {
        struct frag_tbl *tbl;
        size_t sz;
        uint64_t nb_entries;

        nb_entries = rte_align32pow2(bucket_num);
        nb_entries *= bucket_entries;
        nb_entries *= IPV4_FRAG_HASH_FNUM;

        /* check input parameters. */
        if (rte_is_power_of_2(bucket_entries) == 0 ||
            nb_entries > UINT32_MAX || nb_entries == 0 ||
            nb_entries < max_entries) {
                RTE_LOG(ERR, USER1, "%s: invalid input parameter\n", __func__);
                return (NULL);
        }

        sz = sizeof (*tbl) + nb_entries * sizeof (tbl->pkt[0]);
        if ((tbl = rte_zmalloc_socket(__func__, sz, CACHE_LINE_SIZE,
                                      socket_id)) == NULL) {
                RTE_LOG(ERR, USER1,
                        "%s: allocation of %zu bytes at socket %d failed do\n",
                        __func__, sz, socket_id);
                return (NULL);
        }

        RTE_LOG(INFO, USER1, "%s: allocated of %zu bytes at socket %d\n",
                __func__, sz, socket_id);

        tbl->max_cycles = max_cycles;
        tbl->max_entries = max_entries;
        tbl->nb_entries = (uint32_t)nb_entries;
        tbl->nb_buckets = bucket_num;
        tbl->bucket_entries = bucket_entries;
        tbl->entry_mask = (tbl->nb_entries - 1) & ~(tbl->bucket_entries  - 1);

        TAILQ_INIT(&(tbl->lru));
        return (tbl);

}

struct rte_mbuf *
vb_ipv4_frag_in(struct frag_tbl *tbl,
                struct rte_mbuf *mb, uint64_t tms, struct ipv4_hdr *ip_hdr,
                uint16_t ip_ofs, uint16_t ip_flag) 
{
        struct frag_pkt *fp;
        struct ip_frag_key key;
        const uint64_t *psd;
        uint16_t ip_len;

        psd = (uint64_t *)&ip_hdr->src_addr;
        key.src_dst = psd[0];
        key.id = ip_hdr->packet_id;

        ip_ofs *= IPV4_HDR_OFFSET_UNITS;
        ip_len = (uint16_t)(rte_be_to_cpu_16(ip_hdr->total_length) -
                            mb->pkt.vlan_macip.f.l3_len);

        VB_FRAG_LOG(DEBUG, "%s:%d:\n"
                    "mbuf: %p, tms: %" PRIu64
                    ", key: <%" PRIx64 ", %#x>, ofs: %u, len: %u, flags: %#x\n"
                    "tbl: %p, max_cycles: %" PRIu64 ", entry_mask: %#x, "
                    "max_entries: %u, use_entries: %u\n\n",
                    __func__, __LINE__,
                    mb, tms, key.src_dst, key.id, ip_ofs, ip_len, ip_flag,
                    tbl, tbl->max_cycles, tbl->entry_mask, tbl->max_entries,
                    tbl->use_entries);

        /* try to find/add entry into the fragment's table. */
        if ((fp = frag_find(tbl,  &key, tms)) == NULL) {
                rte_prefetch0(mb);
                rte_pktmbuf_free(mb);
                return (NULL);
        }

        VB_FRAG_LOG(DEBUG, "%s:%d:\n"
                    "tbl: %p, max_entries: %u, use_entries: %u\n"
                    "frag_pkt: %p, key: <%" PRIx64 ", %#x>, start: %" PRIu64
                    ", total_size: %u, frag_size: %u, last_idx: %u\n\n",
                    __func__, __LINE__,
                    tbl, tbl->max_entries, tbl->use_entries,
                    fp, fp->key.src_dst, fp->key.id, fp->start,
                    fp->total_size, fp->frag_size, fp->last_idx);


        /* process the fragmented packet. */
        mb = frag_process(fp,  mb, ip_ofs, ip_len, ip_flag);
        frag_inuse(tbl, fp);

        VB_FRAG_LOG(DEBUG, "%s:%d:\n"
                    "mbuf: %p\n"
                    "tbl: %p, max_entries: %u, use_entries: %u\n"
                    "ipv4_frag_pkt: %p, key: <%" PRIx64 ", %#x>, start: %" PRIu64
                    ", total_size: %u, frag_size: %u, last_idx: %u\n\n",
                    __func__, __LINE__, mb,
                    tbl, tbl->max_entries, tbl->use_entries,
                    fp, fp->key.src_dst, fp->key.id, fp->start,
                    fp->total_size, fp->frag_size, fp->last_idx);

        return (mb);
}

