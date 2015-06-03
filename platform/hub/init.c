/*
 *
 * Copyright 2014 Jyotiswarup Raiturkar
 * All rights reserved
 */

#include <string.h>
#include <rte_string_fns.h>
#include <rte_malloc.h>
#include <rte_memzone.h>
#include <rte_ether.h>
#include <rte_ethdev.h>
#include "kni.h"
#include "init_drivers.h"
#include "flow.h"
#include "args.h"
#include "init.h"
#include "main.h"
#include "vport.h"
#include "datapath.h"
#include "stats.h"

#define RTE_LOGTYPE_APP RTE_LOGTYPE_USER1
#define NO_FLAGS 0

/* These are used to dimension the overall size of the mbuf mempool. They
 * are arbitrary values that have been determined by tuning */
#define MBUFS_PER_CLIENT  3072
#define MBUFS_PER_PORT    3072
#define MBUFS_PER_KNI     3072
#define MBUFS_PER_VETH    3072
#define MBUFS_PER_TAP     3072
#define MBUFS_PER_DAEMON  2048

#define PKTMBUF_POOL_NAME "MProc_pktmbuf_pool"

#define MBUF_CACHE_SIZE 32
#define MBUF_OVERHEAD (sizeof(struct rte_mbuf) + RTE_PKTMBUF_HEADROOM)
#define RX_MBUF_DATA_SIZE 2048
#define MBUF_SIZE (RX_MBUF_DATA_SIZE + MBUF_OVERHEAD)



/**
 * Initialise the mbuf pool
 */
static int
init_mbuf_pools(void)
{

    const unsigned num_mbufs = (num_clients * MBUFS_PER_CLIENT)
                               + (port_cfg.num_ports * MBUFS_PER_PORT)
                               + (num_kni * MBUFS_PER_KNI)
                               + (num_veth * MBUFS_PER_VETH)
                               + (num_tap * MBUFS_PER_TAP)
                               + MBUFS_PER_DAEMON;

    /* don't pass single-producer/single-consumer flags to mbuf create as it
     * seems faster to use a cache instead */
    printf("Creating mbuf pool '%s' [%u mbufs] ...\n",
           PKTMBUF_POOL_NAME, num_mbufs);

    if ((pktmbuf_pool = rte_mempool_lookup(PKTMBUF_POOL_NAME)) == NULL) {
        pktmbuf_pool = rte_mempool_create(PKTMBUF_POOL_NAME, num_mbufs,
                                          MBUF_SIZE, MBUF_CACHE_SIZE,
                                          sizeof(struct rte_pktmbuf_pool_private), rte_pktmbuf_pool_init,
                                          NULL, rte_pktmbuf_init, NULL, SOCKET0, NO_FLAGS );
    }

    return (pktmbuf_pool == NULL); /* 0  on success */
}

/**
 * Main init function for the multi-process server app,
 * calls subfunctions to do each stage of the initialisation.
 */
int
init(int argc, char *argv[])
{
    int retval;
    uint8_t total_ports = 0;

    /* init EAL, parsing EAL args */
    retval = rte_eal_init(argc, argv);
    if (retval < 0)
        return -1;
    argc -= retval;
    argv += retval;

    /*moved PCI init first */
    if (rte_eal_pci_probe())
        rte_panic("Cannot probe PCI\n");

    /* initialise the nic drivers */
    retval = init_drivers();
    if (retval != 0)
        rte_exit(EXIT_FAILURE, "Cannot initialise drivers\n");

    /* get total number of ports */
    total_ports = rte_eth_dev_count();
    /* parse additional, application arguments */
    retval = parse_app_args(total_ports, argc, argv);
    if (retval != 0) {
        return -1;
    }

    /* initialise mbuf pools */
    retval = init_mbuf_pools();
    if (retval != 0) {
        rte_exit(EXIT_FAILURE, "Cannot create needed mbuf pools\n");
    }

//    if (RTE_PROC_SECONDARY == rte_eal_process_type()) {
    {


        flow_table_init();
        datapath_init();
        // }

        vport_init();

        //if (RTE_PROC_SECONDARY == rte_eal_process_type()) {
        stats_init();
    }

    return 0;
}
