/*
 * Copyright 2014 Jyotiswarup Raiturkar
 * All rights reserved
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include "wrr.h"



static inline void * wrr_malloc(size_t s)
{
        return calloc(1, s);
}

static inline void  wrr_free(void *p)
{
        free(p);
}

static inline unsigned long
_gcd(unsigned long a, unsigned long b)
{
        unsigned long r;

        if (a < b) {
                unsigned long t = a;
                a = b;
                b = t;
        }

        if (!b)
                return a;

        while ((r = a % b) != 0) {
                a = b;
                b = r;
        }
        return b;
}

static inline void
wrr_params(struct wrr_pool *pool, unsigned int *gcd, unsigned long * period, unsigned int *max_w)
{
        int i;
        unsigned int g = 0;
        unsigned int p = 0;
        unsigned int m = 0;

        for (i = 0 ; i < pool->n_servers; i++) {
                if (pool->weights[i] > 0) {
                        g = (g > 0 ) ? _gcd(g, pool->weights[i]) : g ;
                        p += pool->weights[i];
                        if ( pool->weights[i] > m)
                                m = pool->weights[i];
                }
        }

        if(period)
                *period = p ;

        if(gcd)
                *gcd = g ? g: 1 ;

        if(max_w)
                *max_w = m ;
}

void
wrr_gen_seq(struct wrr_pool * pool)
{
        int i = 0;
        int s = -1;
        int cw = 0;


        //for(i = 0; i < pool->period; i++){
        while (1 ) {
                s = (s + 1) % pool->n_servers ;
                if (s == 0) {
                        cw = cw - pool->gcd;
                        if ( cw <= 0 ) {
                                cw = pool->max_weight ;
                        }
                }

                if( pool->weights[s] >= cw )
                        pool->seq[i++] = s ;
                if (i == pool->period)
                        break;
        }

}

struct wrr_pool *
wrr_create_pool(int n_servers, ... ) {
        va_list s_list;
        int i;
        struct wrr_pool *pool = wrr_malloc(sizeof(struct wrr_pool));
        pool->weights = wrr_malloc(n_servers * sizeof(unsigned int));
        va_start(s_list, n_servers);

        pool->n_servers = n_servers;

        for (i = 0; i< n_servers; i++) {
                pool->weights[i] = va_arg(s_list, unsigned int);
        }


        wrr_params(pool, &pool->gcd, &pool->period, &pool->max_weight);

        pool->seq = wrr_malloc(pool->period * sizeof(unsigned int));
        wrr_gen_seq(pool);

        return pool;
}

void
wrr_delete_pool(struct wrr_pool * pool)
{
        wrr_free(pool->weights);
        wrr_free(pool->seq);
        wrr_free(pool);

}

int
wrr_update_weight(struct wrr_pool *pool, int server, int new_weight)
{
        if(server<0 || server>(pool->n_servers-1))
                return EINVAL;

        pool->weights[server] = new_weight;
        wrr_params(pool, &pool->gcd, &pool->period, &pool->max_weight);
        wrr_free(pool->seq);
        pool->seq = wrr_malloc(pool->period * sizeof(unsigned int));
        wrr_gen_seq(pool);

        return 0;

}

#if LOCAL_TEST
int
main()
{
        int i;
        struct wrr_pool *p1 = wrr_create_pool(5, 8, 8, 8, 3, 3 );
        struct wrr_pool *p2 = wrr_create_pool(3, 4, 3, 2 );

        for (i = 0; i< p1->n_servers; i++) {
                printf("server %d weights %d \n", i, p1->weights[i]);
        }
        printf("\n\n");
        for (i = 0; i< p2->n_servers; i++) {
                printf("server %d weights %d \n", i, p2->weights[i]);
        }
        for (i = 0; i< p2->period; i++) {
                printf("%d ", p2->seq[i]);
        }

        wrr_update_weight(p2, 1, 4);
        wrr_update_weight(p2, 2, 4);
        printf("\n After Same Weights\n");
        for (i = 0; i< p2->period; i++) {
                printf("%d ", p2->seq[i]);
        }

        wrr_delete_pool(p1);

}
#endif
