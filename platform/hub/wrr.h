/*
 * Copyright 2014 Jyotiswarup Raiturkar
 * All rights reserved
 */
#ifndef _WRR_SCHEDULER_H
#define _WRR_SCHEDULER_H

#include <errno.h>
/*
   This module implements the WRR algorithm as described below :
          Supposing that there is a server set S = {S0, S1, …, Sn-1};
          W(Si) indicates the weight of Si;
          i indicates the server selected last time, and i is initialized with -1;
          cw is the current weight in scheduling, and cw is initialized with zero;
          max(S) is the maximum weight of all the servers in S;
          gcd(S) is the greatest common divisor of all server weights in S;

          while (true) {
              i = (i + 1) mod n;
              if (i == 0) {
                  cw = cw - gcd(S);
                  if (cw <= 0) {
                      cw = max(S);
                      if (cw == 0)
                      return NULL;
                  }
              }
              if (W(Si) >= cw)
                  return Si;
          }
   This is inspired by the WRR algorithm in the Linux IPVS server but
   generates the sequence beforehand as an optimization
*/

struct wrr_pool {
        int cs;         /*current server in sequence*/
        unsigned int *weights;   /* array of server weights indexed by server id */
        unsigned int n_servers;  /*number of servers */
        unsigned int gcd;        /* gcd of weights */
        unsigned int max_weight; /* max weight */
        unsigned int *seq;       /*wrr scheduling sequence; this is of size period */
        unsigned long period;    /* scheduling period; this keeps repeating */
};


/*
   According to the WRR alog, get the best server in the pool
   server ids start from 0 and go to n_servers-1
   returns EINVAL on error 0 on success
*/
unsigned int
wrr_schedule( struct wrr_pool *p);

/*
   creates a pool of servers, the weights should come after n_server
   returns NULL on error;
*/

struct wrr_pool *
wrr_create_pool(int n_servers, ... );


/*
   delete a pool of servers
*/
void
wrr_delete_pool(struct wrr_pool * pool);


/*
   updates weight of a server in the pool
   server ids start from 0 and go to n_servers-1
   returns EINVAL on error 0 on success
*/
int
wrr_update_weight(struct wrr_pool *p, int server, int new_weight);

#endif /* _WRR_SCHEDULER_H */

