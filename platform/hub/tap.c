/*
 *
 * Copyright 2014 Jyotiswarup Raiturkar
 * All rights reserved
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <linux/if_tun.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/time.h>
#include <errno.h>
#include <stdarg.h>

#include <rte_ethdev.h>
#include <rte_cycles.h>
#include <rte_memzone.h>
#include <rte_malloc.h>
#include <rte_log.h>
#include <rte_string_fns.h>

#include "tap.h"
#include "vport.h"
#include "init.h"


#define RTE_LOGTYPE_APP RTE_LOGTYPE_USER1

static int 
tap_alloc(char *dev)
{
    int flags;
    struct ifreq ifr;
    int fd, err;
    char *clonedev = "/dev/net/tun";

    if( (fd = open(clonedev , O_RDWR)) < 0 ) {
        perror("Opening /dev/net/tun");
        return fd;
    }

    memset(&ifr, 0, sizeof(ifr));

    ifr.ifr_flags =  IFF_TAP|IFF_NO_PI;

    if (*dev) {
        strncpy(ifr.ifr_name, dev, IFNAMSIZ);
    }

    if( (err = ioctl(fd, TUNSETIFF, (void *)&ifr)) < 0 ) {
        perror("ioctl(TUNSETIFF)");
        close(fd);
        return err;
    }

    strcpy(dev, ifr.ifr_name);

    flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    return fd;
}

static int __rte_unused
tap_read(int fd, char *buf, int n)  
{

    int nread;

    if((nread=read(fd, buf, n)) < 0) {
        perror("Reading data");
        exit(1);
    }
    return nread;
}

static int __rte_unused
tap_write(int fd, char *buf, int n)
{

    int nwrite;

    if((nwrite=write(fd, buf, n)) < 0) {
        perror("Writing data");
        exit(1);
    }
    return nwrite;
}

static int __rte_unused 
tap_read_n(int fd, char *buf, int n) 
{

    int nread, left = n;

    while(left > 0) {
        if ((nread = tap_read(fd, buf, left)) == 0) {
            return 0 ;
        } else {
            left -= nread;
            buf += nread;
        }
    }
    return n;
}


static char temp_buf[ETHER_MAX_LEN];
static int *tap_fds;
extern uint8_t num_tap;

int
init_taps()
{
    int i;
    char name[256];

    tap_fds = rte_malloc("TAP vport fd map", sizeof(int)*num_tap, 64);
    if (!tap_fds ){
        rte_exit(EXIT_FAILURE, " Unable to allocate memory for vport FD map !\n");
    }

    for(i=0; i<num_tap; i++ ){
        rte_snprintf(name, sizeof(name), "vTap%d",i);
        tap_fds[i]= tap_alloc(name);
    }    

    return 0;
}


uint16_t
receive_from_tap(uint8_t vportid, struct rte_mbuf **bufs)
{
    int fd = tap_fds[vportid - TAP0];
    int i, nread ;
    struct rte_mbuf * mbuf;
    
    for( i=0; i< PKT_BURST_SIZE; i++){
        nread=read(fd, temp_buf, ETHER_MAX_LEN); /* should give full frame */
        if(nread <0 )
            return i;

        mbuf = rte_pktmbuf_alloc(pktmbuf_pool);
        if (unlikely(mbuf == NULL)) {
            RTE_LOG(ERR, APP, "Failed to allocate memory for mbuf.\n");
            return i;
        }
    
        mbuf->pkt.data_len =  mbuf->pkt.pkt_len = nread;
        rte_memcpy((void*) mbuf->pkt.data,
                        (const void*) temp_buf, mbuf->pkt.data_len);

        bufs[i] = mbuf;
   } 

    return i;
}

int
send_to_tap(uint8_t vportid, struct rte_mbuf *buf)
{
    int fd = tap_fds[vportid - TAP0] ;
    
    char * data = rte_pktmbuf_mtod(buf, char *);
    if((write(fd, data , rte_pktmbuf_pkt_len(buf)) )< 0) {
        RTE_LOG(ERR, APP, "Failed to write on TAP!!\n");
        return -1;
    }

    return 0; 
}

#if 0

int main(int argc, char *argv[])
{

    int tap_fd, option;
    int flags = IFF_TUN;
    char if_name[IFNAMSIZ] = "";
    int maxfd;
    uint16_t nread, nwrite, plength;
    char buffer[BUFSIZE];
    int sock_fd, net_fd, optval = 1;
    socklen_t remotelen;


    /* Check command line options */
    while((option = getopt(argc, argv, "i:sc:p:uahd")) > 0) {
        switch(option) {
        case 'i':
            strncpy(if_name,optarg, IFNAMSIZ-1);
            break;
        default:
            printf("Unknown option %c\n", option);
            exit(1);
        }
    }

    if(*if_name == '\0') {
        printf("Must specify interface name!\n");
        exit(1);
    }


    /* initialize tun/tap interface */
    if ( (tap_fd = tap_alloc(if_name )) < 0 ) {
        printf("Error connecting to tun/tap interface %s!\n", if_name);
        exit(1);
    }

    printf("Successfully connected to interface %s\n", if_name);

    while (1) {

        nread = tap_read(tap_fd, buffer, BUFSIZE);
        printf("Read %d bytes from the tap interface\n",  nread);

        //    getchar();

        //nwrite = tap_write(net_fd, buffer, nread);
        //printf("Written %d bytes to the network\n",  nwrite);

    }

    return(0);

}
#endif 
