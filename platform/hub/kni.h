/*
 *
 * Copyright 2014 Jyotiswarup Raiturkar
 * All rights reserved
 */
#ifndef __KNI_H_
#define __KNI_H_


#include <string.h>


#include <rte_kni.h>
#include <exec-env/rte_kni_common.h>
#include <rte_spinlock.h>

#define MAX_KNI_PORTS          16
#define KNI_FIFO_COUNT_MAX     1024
#define KNI_FIFO_SIZE          ((KNI_FIFO_COUNT_MAX) * sizeof(void *) + \
                                sizeof(struct rte_kni_fifo))
#define FAIL_ON_MEMZONE_NULL(mz) \
	do { \
		if ((mz) == NULL) \
		{ rte_exit(EXIT_FAILURE, "FIFO initialisation failed.\n"); } \
	}while(0)
#define MZ_KNI_PORT_INFO "MProc_kni_port_info"


/**
 * KNI context
 */
struct rte_kni {
    char name[RTE_KNI_NAMESIZE];        /**< KNI interface name */
    uint16_t group_id;                   /**< Group ID of KNI devices  */
    struct rte_mempool *pktmbuf_pool;   /**< pkt mbuf mempool */
    unsigned mbuf_size;                 /**< mbuf size */

    struct rte_kni_fifo *tx_q;          /**< TX queue */
    struct rte_kni_fifo *rx_q;          /**< RX queue */
    struct rte_kni_fifo *alloc_q;       /**< Allocated mbufs queue */
    struct rte_kni_fifo *free_q;        /**< To be freed mbufs queue */

    /* For request & response */
    struct rte_kni_fifo *req_q;         /**< Request queue */
    struct rte_kni_fifo *resp_q;        /**< Response queue */
    void * sync_addr;                   /**< Req/Resp Mem address */

    struct rte_kni_ops ops;             /**< operations for request */
    uint8_t in_use : 1;             /**< kni creation flag */
};

struct rte_kni rte_kni_list[MAX_KNI_PORTS];
extern rte_spinlock_t rte_kni_locks[MAX_KNI_PORTS];

/* Reserves memory for MAX_KNI_PORTS number of KNI ports and initialises
 * the fifos
 */
void
init_kni(void);

#endif
