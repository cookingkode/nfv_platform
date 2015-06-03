/*
 *
 * Copyright 2014 Jyotiswarup Raiturkar
 * All rights reserved
 */
#ifndef _DPC_THREAD_H_
#define _DPC_THREAD_H_

#include <pthread.h>

#include <rte_memory.h>
#include <rte_mempool.h>
#include <rte_memcpy.h>
#include <rte_ring.h>

#define THREAD_NAME_LEN 256
#define MAX_NX_SLAVES	1
#define SLAVE_NAME_LEN  50
#define SLAVE_PKT_Q_LEN 4096
#define MAX_PKT_DQ_CNT  256
#define SLAVE_MSG_Q_LEN 1024
#define MAX_MSG_DQ_CNT  4

#define DPC_MAX_ARGS 8

typedef struct _op_item_arg_ {
    void *data;
    uint8_t free_data;
} op_item_arg_t;

/* DPC operation info */
typedef struct _op_item_ {
    uint8_t argc;
    op_item_arg_t argv[DPC_MAX_ARGS];
    void  (*func)( );
} op_item_t;

typedef struct _dpc_queue_stats_ {
    uint32_t nq;
    uint32_t dq;
    uint32_t nq_err;
    uint32_t dq_err;
} dpc_queue_stats_t;

typedef enum  _dpc_queue_id_ {
    DPC_QUEUE_PKT,
    DPC_QUEUE_MSG,
} dpc_queue_id_e;

#define DPC_NUMBER_ALLOCS 3
#define DPC_POOL_SIZE 512
#define DPC_POOL_CACHE_SIZE 64
struct dpc_thread_info {
    pthread_t thread_id;
    char name[THREAD_NAME_LEN];
    pthread_mutex_t queue_mutex;
    pthread_cond_t req_condvar;
    struct rte_ring *dpc_pkt_queue;
    struct rte_ring *dpc_msg_queue;
    dpc_queue_stats_t pkt_queue_stats;
    dpc_queue_stats_t msg_queue_stats;
    struct rte_mempool *objs[DPC_NUMBER_ALLOCS];
};

struct dpc_thread_config {
    unsigned pkt_queue_count;
    unsigned msg_queue_count;
    unsigned pkt_queue_flags;
    unsigned msg_queue_flags;
};

/* Submit a deferred job */
/* NOTES :
 *  - The function to be called must take pointers to the arguments. It is
 *    assumed the func knows thesize of the arguments
 *  - The arguments are deep copied, so that the caller of dpc_submit can free
 *    p1 and p2 after the call
 *
 */
int dpc_event_submit(struct dpc_thread_info *p_dpc_thread, dpc_queue_id_e queue_id,
                     void (* func)(), int argc, op_item_arg_t argv[]);

/* Initialize a DPC queue and thread
 *  dpc_thread        - handle for the queue
 *  work_queue_count -- how many items in the queue
 *  name             -  name for the DPC thread
 */
int dpc_init(struct dpc_thread_info *dpc_thread, char *name,
             struct dpc_thread_config *cfg);

#endif /* _DPC_THREAD_H_ */
