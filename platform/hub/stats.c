/*
 *
 * Copyright 201 Jyotiswarup Raiturkar
 * All rights reserved
 */
#include <stdio.h>
#include <rte_ether.h>
#include <rte_memzone.h>

#include "stats.h"
#include "init.h"
#include "vport.h" /* for MAX_VPORTS */

#define NO_FLAGS            0
#define MZ_STATS_INFO "MProc_stats_info"
#define VPORT_STATS_SIZE (sizeof(struct vport_statistics) * MAX_VPORTS +  \
                          sizeof(struct vswitch_statistics))
//#define STATS_DISABLE

/*
 * vport statistics structure, used by both clients and kni ports
 * to record traffic information
 */
struct vport_lcore_statistics {
    volatile uint64_t rx;
    volatile uint64_t tx;
    volatile uint64_t rx_drop;
    volatile uint64_t tx_drop;
    volatile uint64_t overrun;
} __rte_cache_aligned;

struct vport_statistics {
    struct vport_lcore_statistics stats[RTE_MAX_LCORE];
};

struct vswitch_lcore_statistics {
    uint64_t tx_drop;
    uint64_t rx_drop;
} __rte_cache_aligned;

struct vswitch_statistics {
    struct vswitch_lcore_statistics stats[RTE_MAX_LCORE];
};

static struct vport_statistics *vport_stats[MAX_VPORTS] = {NULL};
static struct vswitch_statistics *vswitch_stats = NULL;

void
stats_clear(void)
{
    stats_vswitch_clear();
    stats_vport_clear_all();
}

/*
 * Function to set vswitch statistic values to zero.
 */
void
stats_vswitch_clear(void)
{
    int i;

    for (i = 0; i < RTE_MAX_LCORE; i++) {
        vswitch_stats->stats[i].rx_drop = 0;
        vswitch_stats->stats[i].tx_drop = 0;
    }
}

/*
 * Function to set vport statistic values to zero.
 */
void
stats_vport_clear(unsigned vportid)
{
    int i;

    for (i = 0; i < RTE_MAX_LCORE; i++) {
        struct vport_lcore_statistics *s = &vport_stats[vportid]->stats[i];

        s->rx = 0;
        s->rx_drop = 0;
        s->tx = 0;
        s->tx_drop = 0;
        s->overrun = 0;
    }
}

/*
 * Function to set all vport statistics to zero
 */
void stats_vport_clear_all(void)
{
    unsigned vportid = 0;

    for (vportid = 0; vportid < MAX_VPORTS; vportid++) {
        stats_vport_clear(vportid);
    }
}

#ifdef STATS_DISABLE
void stats_vport_rx_increment(unsigned vportid, int inc)
{
}

void stats_vport_rx_drop_increment(unsigned vportid, int inc)
{
}

void stats_vport_tx_increment(unsigned vportid, int inc)
{
}

void stats_vport_tx_drop_increment(unsigned vportid, int inc)
{
}

void stats_vport_overrun_increment(unsigned vportid, int inc)
{
}

void stats_vswitch_rx_drop_increment(int inc)
{
}

void stats_vswitch_tx_drop_increment(int inc)
{
}

#else /* STATS_DISABLE */
inline void stats_vport_rx_increment(unsigned vportid, int inc)
{
    vport_stats[vportid]->stats[rte_lcore_id()].rx += inc;
}

inline void stats_vport_rx_drop_increment(unsigned vportid, int inc)
{
    vport_stats[vportid]->stats[rte_lcore_id()].rx_drop += inc;
}

inline void stats_vport_tx_increment(unsigned vportid, int inc)
{
    vport_stats[vportid]->stats[rte_lcore_id()].tx += inc;
}

inline void stats_vport_tx_drop_increment(unsigned vportid, int inc)
{
    vport_stats[vportid]->stats[rte_lcore_id()].tx_drop += inc;
}

inline void stats_vport_overrun_increment(unsigned vportid, int inc)
{
    vport_stats[vportid]->stats[rte_lcore_id()].overrun += inc;
}

inline void stats_vswitch_rx_drop_increment(int inc)
{
    vswitch_stats->stats[rte_lcore_id()].rx_drop += inc;
}

inline void stats_vswitch_tx_drop_increment(int inc)
{
    vswitch_stats->stats[rte_lcore_id()].tx_drop += inc;
}

#endif /* STATS_DISABLE */

inline uint64_t stats_vport_rx_get(unsigned vportid)
{
    uint64_t rx;
    int i;

    for (rx = 0, i = 0; i < RTE_MAX_LCORE; i++)
        rx += vport_stats[vportid]->stats[i].rx;

    return rx;
}

inline uint64_t stats_vport_rx_drop_get(unsigned vportid)
{
    uint64_t rx_drop;
    int i;

    for (rx_drop = 0, i = 0; i < RTE_MAX_LCORE; i++)
        rx_drop += vport_stats[vportid]->stats[i].rx_drop;

    return rx_drop;
}

inline uint64_t stats_vport_tx_get(unsigned vportid)
{
    uint64_t tx;
    int i;

    for (tx = 0, i = 0; i < RTE_MAX_LCORE; i++)
        tx += vport_stats[vportid]->stats[i].tx;

    return tx;
}

inline uint64_t stats_vport_tx_drop_get(unsigned vportid)
{
    uint64_t tx_drop;
    int i;

    for (tx_drop = 0, i = 0; i < RTE_MAX_LCORE; i++)
        tx_drop += vport_stats[vportid]->stats[i].tx_drop;

    return tx_drop;
}

inline uint64_t stats_vport_overrun_get(unsigned vportid)
{
    uint64_t overrun;
    int i;

    for (overrun = 0, i = 0; i < RTE_MAX_LCORE; i++)
        overrun += vport_stats[vportid]->stats[i].overrun;

    return overrun;
}

inline uint64_t stats_vswitch_rx_drop_get(void)
{
    uint64_t rx_drop;
    int i;

    for (rx_drop = 0, i = 0; i < RTE_MAX_LCORE; i++)
        rx_drop += vswitch_stats->stats[i].rx_drop;

    return rx_drop;
}

inline uint64_t stats_vswitch_tx_drop_get(void)
{
    uint64_t tx_drop;
    int i;

    for (tx_drop = 0, i = 0; i < RTE_MAX_LCORE; i++)
        tx_drop += vswitch_stats->stats[i].tx_drop;

    return tx_drop;
}

void
stats_init(void)
{
    const struct rte_memzone *mz = NULL;
    unsigned vportid = 0;
    /* set up array for statistics */
    mz = rte_memzone_reserve(MZ_STATS_INFO, VPORT_STATS_SIZE, rte_socket_id(), NO_FLAGS);
    if (mz == NULL)
        rte_exit(EXIT_FAILURE, "Cannot reserve memory zone for statistics\n");
    memset(mz->addr, 0, VPORT_STATS_SIZE);

    for (vportid = 0; vportid < MAX_VPORTS; vportid++) {
        vport_stats[vportid] = (void *)((char *)mz->addr +
                                        vportid * sizeof(struct vport_statistics));
    }

    vswitch_stats = (void *)((char *)mz->addr +
                             MAX_VPORTS * sizeof(struct vport_statistics));
}


