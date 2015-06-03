/*
 *
 * Copyright 2014 Jyotiswarup Raiturkar
 * All rights reserved
 */
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#include <rte_malloc.h>
#include <rte_ring.h>
#include <rte_string_fns.h>
#include <rte_errno.h>
#include <rte_debug.h>
#include <sys/syscall.h>

#include "dpc.h"

#define THREAD_NAME_LEN 256

#define NB_MAX_WORKER_THREAD 8
#define THREAD_STACK_SIZE  2116488
#define DPC_THREAD_IDENTIFIER 1

static int32_t
dpc_insert_queue(struct dpc_thread_info *p_dpc_thread,
                 struct rte_ring *queue, op_item_t *event)
{
    int need_to_send_signal = 0;

    if (unlikely(rte_ring_count(queue) <= 0)) {
        need_to_send_signal = 1;
    }

    while (rte_ring_enqueue(queue, event)) {
        // keep retrying
    }

    if (need_to_send_signal) {
        if (pthread_cond_signal(&(p_dpc_thread->req_condvar)) == -1) {
            // tbd - increment error count
        }
    }

    return 0;
}

/* param indicates which pool to get from */
__rte_unused static void *
dpc_malloc(struct rte_mempool **objs, size_t size , int param)
{
    char *p = NULL;
    char *malloc_p = NULL;



    if ( unlikely(!objs[param] || (objs[param]->size-__WORDSIZE) != size) ) {
        char name[50];
        rte_snprintf(name, 49, "DPC_MEMPOOL_%d", param );
        if(!objs[param] ) {/*  no pool */
            objs[param] = rte_mempool_create(name, DPC_POOL_SIZE,
                                             size+__WORDSIZE ,
                                             DPC_POOL_CACHE_SIZE,
                                             0,
                                             (rte_mempool_ctor_t *)NULL,
                                             NULL, NULL, NULL,
                                             SOCKET_ID_ANY, 0 );
            if(!objs[param]) {
                printf("\n DPC Mempool initialization failed %s\n",rte_strerror(rte_errno));
                goto alloc_outside_pool;
            }

            /*TODO replacement of mempool? */
        } else   { /*pool diff size */
            goto alloc_outside_pool;
        }
    }

    if ( rte_mempool_sc_get(objs[param], (void **)&p) ) {
        goto alloc_outside_pool;
    } else {
        *p = '1';
        return p+__WORDSIZE;
    }

alloc_outside_pool:
    malloc_p = rte_malloc(NULL, size+__WORDSIZE, 0);
    if (malloc_p) {
        *malloc_p = '0';
        return malloc_p+__WORDSIZE;
    } else {
        printf("\n DPC : out of memory ");
        return NULL;
    }
}

/* to be used with dpc_malloc ed things only! */
void
dpc_free(void *p)
{
    if(!p)
        return;

    void *ptr = p - __WORDSIZE;
    if(*((char *)ptr) == '0')
        rte_free(ptr);
    else
        rte_mempool_sp_put(
            (struct rte_mempool *)rte_mempool_from_obj(ptr),
            ptr);

}

static void
dpc_free_event(op_item_t *event)
{
    int i;
    if (event) {
        for (i=0; i<event->argc; i++) {
            if (event->argv[i].free_data) {
                rte_free(event->argv[i].data);
                event->argv[i].data = NULL;
                //printf("Freed arg %d\n", i+1);
            }
        }
        rte_free(event);
        event = NULL;
    }
}

void *
dpc_update_thread(void *arg)
{
    struct dpc_thread_info *p_info = (struct dpc_thread_info *) arg;
    op_item_t *packet_obj[MAX_PKT_DQ_CNT+1];
    op_item_t *msg_obj[MAX_MSG_DQ_CNT+1];
    int num_pkt_objs, num_msg_objs;
    int i;
//	pid_t tidt = syscall(SYS_gettid);

//	printf("####### Spawned a new pthread %d with start-routine dpc_update_thread\n", tidt);

    while (1) {
        pthread_mutex_lock(&(p_info->queue_mutex));
in_lock:
        num_pkt_objs = rte_ring_dequeue_burst(p_info->dpc_pkt_queue,
                                              (void *)packet_obj, MAX_PKT_DQ_CNT);

        num_msg_objs = rte_ring_dequeue_burst(p_info->dpc_msg_queue,
                                              (void *)msg_obj, MAX_MSG_DQ_CNT);
        if ((num_pkt_objs <= 0) && (num_msg_objs <= 0)) {
            pthread_cond_wait(&(p_info->req_condvar), &(p_info->queue_mutex));
            goto in_lock;
        }
        pthread_mutex_unlock(&(p_info->queue_mutex));

        for (i=0; i<num_pkt_objs; i++) {
            (*(packet_obj[i]->func))((packet_obj[i]->argc), (packet_obj[i]->argv));
            dpc_free_event(packet_obj[i]);
        }
        for (i=0; i<num_msg_objs; i++) {
            (*(msg_obj[i]->func))((msg_obj[i]->argc), (msg_obj[i]->argv));
            dpc_free_event(msg_obj[i]);
        }
    }
}

static int
init_thread_info(struct dpc_thread_info *p_mthr_info,
                 char name[THREAD_NAME_LEN],
                 struct dpc_thread_config *cfg)
{
    char queue_name[100];
    static int pkt_ring_num = 0;
    static int msg_ring_num = 0;

    if (!p_mthr_info) {
        return 1;
    }

    strncpy(p_mthr_info->name, name, THREAD_NAME_LEN);

    rte_snprintf(queue_name, sizeof(queue_name), "dpc_pkt_ring_%d", pkt_ring_num++);
    p_mthr_info->dpc_pkt_queue = rte_ring_create(queue_name,
                                 cfg->pkt_queue_count, SOCKET_ID_ANY, cfg->pkt_queue_flags);
    if (!p_mthr_info->dpc_pkt_queue) {
        rte_panic("DPC packet ring creation failed : %s\n", rte_strerror(rte_errno));
    }

    rte_snprintf(queue_name, sizeof(queue_name), "dpc_msg_ring_%d", msg_ring_num++);
    p_mthr_info->dpc_msg_queue = rte_ring_create(queue_name,
                                 cfg->msg_queue_count, SOCKET_ID_ANY, cfg->msg_queue_flags);
    if (!p_mthr_info->dpc_msg_queue) {
        rte_panic("DPC message ring creation failed : %s\n", rte_strerror(rte_errno));
    }

    if (pthread_cond_init(&p_mthr_info->req_condvar, NULL) != 0) {
        return -1;
    }

    return 0;
}

int
dpc_init(struct dpc_thread_info *p_dpc_thread, char *name,
         struct dpc_thread_config *cfg)
{
    int rc = 0;
    pthread_attr_t attr_thr;

    memset(p_dpc_thread, 0, sizeof(struct dpc_thread_info));

    pthread_attr_init(&attr_thr);
    pthread_attr_setscope(&attr_thr, PTHREAD_SCOPE_SYSTEM);
    pthread_attr_setdetachstate(&attr_thr, PTHREAD_CREATE_JOINABLE);
    pthread_attr_setstacksize(&attr_thr, THREAD_STACK_SIZE);

    rc = init_thread_info(p_dpc_thread, name, cfg);
    if (rc) {
        /* error in initializing DPC thread */
        exit(1); // for debug should be return  -1
    }

    if ((rc = pthread_create(&(p_dpc_thread->thread_id),
                             &attr_thr, dpc_update_thread,
                             (void *)&(p_dpc_thread->thread_id))) != 0) {
        /* error in creating the thread */
        exit(1);  // for debug should be return  -1
    }

    return rc;
}

int32_t
dpc_event_submit(struct dpc_thread_info *p_dpc_thread, dpc_queue_id_e queue_id,
                 void (* func)(), int argc, op_item_arg_t *argv)
{
    struct rte_ring *queue;
    dpc_queue_stats_t *stats;
    op_item_t *new_event;
    int32_t rc, i;

    if (likely(DPC_QUEUE_PKT == queue_id)) {
        queue = p_dpc_thread->dpc_pkt_queue;
        stats = &(p_dpc_thread->pkt_queue_stats);
    } else if (DPC_QUEUE_MSG == queue_id) {
        queue = p_dpc_thread->dpc_msg_queue;
        stats = &(p_dpc_thread->msg_queue_stats);
    } else {
        // tbd - increment error count
        return -1;
    }

    if (argc > DPC_MAX_ARGS) {
        stats->nq_err++;
        return -1;
    }

    new_event = rte_malloc("NULL", sizeof(op_item_t), 0);

    if (NULL == new_event) {
        stats->nq_err++;
        return -1;
    }

    new_event->argc = argc;
    new_event->func = func;
    for (i=0; i<argc; i++) {
        new_event->argv[i].data = argv[i].data;
        new_event->argv[i].free_data = argv[i].free_data;
    }

    rc = dpc_insert_queue(p_dpc_thread, queue, new_event);

    if (rc) {
        stats->nq_err++;
        dpc_free_event(new_event);
    }

    return rc;
}

