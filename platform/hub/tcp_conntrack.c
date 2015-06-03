/* 
 * Copyright 2014 Jyotiswarup Raiturkar
 * All rights reserved
 */

//#include <arpa/inet.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <rte_mbuf.h>
#include <rte_tcp.h>
#include <rte_ip.h>
#include <rte_rwlock.h>
#include <rte_ether.h>
#include <rte_byteorder.h>
#include "hmap.h"
#include "ip_utils.h"
#include "hash.h"
#include "tcp_conntrack.h"

#define DBG(a, ...) printf( __VA_ARGS__)

#define DBG_ERR( ...) printf( __VA_ARGS__)

static inline __attribute__ (( always_inline )) void *
tcp_session_alloc(size_t s)
{
        return  malloc(s);
}

static inline __attribute__ (( always_inline )) void
tcp_session_free(void * s)
{
        free(s);

}
/** TCP flags */
enum tcp_flags {
        /** TCP data transfer interface has been closed */
        TCP_XFER_CLOSED = 0x0001,
        /** TCP timestamps are enabled */
        TCP_TS_ENABLED = 0x0002,
        /** TCP acknowledgement is pending */
        TCP_ACK_PENDING = 0x0004,
};

/***************************************************************************
 *
 * Internal Functions
 *
 ***************************************************************************
 */

/**
 * Dump TCP flags
 *
 * @v flags		TCP flags
 */
static inline __attribute__ (( always_inline )) void
tcp_dump_flags ( unsigned int flags )
{
        if ( flags & TCP_RST )
                DBG ( tcp, " RST" );
        if ( flags & TCP_SYN )
                DBG ( tcp, " SYN" );
        if ( flags & TCP_PSH )
                DBG ( tcp, " PSH" );
        if ( flags & TCP_FIN )
                DBG ( tcp, " FIN" );
        if ( flags & TCP_ACK )
                DBG ( tcp, " ACK" );
}

struct session_key {
        uint32_t srv_addr;
        uint32_t cli_addr;
        uint16_t srv_port;
        uint16_t cli_port;

} __attribute__((__packed__));

struct tcp_session {
        /*state */
        unsigned int tcp_state;
        /* tuple */
        uint32_t srv_addr;
        uint32_t cli_addr;
        uint16_t srv_port;
        uint16_t cli_port;
        /* Sent sequence number */
        uint32_t snd_seq;
        /* chain on hmap nodes*/
        struct hmap_node node;
}  __attribute__((__packed__));

struct session_tbl_entry {
        rte_rwlock_t lock;   /* Lock to allow multiple readers and one writer */
        struct hmap hmap_sessions;
};

#define SESSION_TABLE_SIZE 1024
static struct session_tbl_entry session_table[SESSION_TABLE_SIZE];

static long
hash1(uint32_t a, uint32_t b, uint16_t c)
{
        uint64_t hash = 0;
        hash = a + (hash << 6) + (hash << 16) - hash;
        hash = b + (hash << 6) + (hash << 16) - hash;
        hash = c + (hash << 6) + (hash << 16) - hash;

        return hash;
}

static long
hash2(uint32_t a, uint32_t b, uint32_t c)
{
        /* Jerkin's hash lookup3 which is there with OVS*/
        return hash_3words(a, b, c );
}

static struct tcp_session*
tcp_session_lookup(  uint32_t srv_addr , uint32_t cli_addr,  uint16_t srv_port) {
        struct tcp_session *tcp = NULL ;
        int indx = hash1(srv_addr, cli_addr, srv_port) % SESSION_TABLE_SIZE;
        int key2 = hash2(srv_addr, cli_addr, srv_port);
        /* As we are reading from the table, acquire read lock */
        rte_rwlock_read_lock(&session_table[indx].lock);

        HMAP_FOR_EACH_WITH_HASH (tcp , node, key2 ,
                                 &session_table[indx].hmap_sessions) {
                if ( tcp->srv_addr==srv_addr &&
                     tcp->cli_addr==cli_addr &&
                     tcp->srv_port==srv_port   )
                        break;
        }

        /* release lock as we have everything we need */
        rte_rwlock_read_unlock(&session_table[indx].lock);

        return tcp;

}

static struct tcp_session*
tcp_session_insert(  uint32_t srv_addr , uint32_t cli_addr,  uint16_t srv_port) {
        struct tcp_session *tcp = tcp_session_alloc(sizeof(struct tcp_session *)) ;
        int indx = hash1(srv_addr, cli_addr, srv_port) % SESSION_TABLE_SIZE;
        int key2 = hash2(srv_addr, cli_addr, srv_port);
        rte_rwlock_read_lock(&session_table[indx].lock);
        hmap_insert(&session_table[indx].hmap_sessions, &tcp->node, key2);
        rte_rwlock_read_unlock(&session_table[indx].lock);

        return tcp;
}

static int
tcp_session_delete(  struct tcp_session* tcp)
{
        if (!tcp) return -1;

        int indx = hash1(tcp->srv_addr, tcp->cli_addr, tcp->srv_port);
        //int key2 = hash2(tcp->srv_addr, tcp->cli_addr, tcp->srv_port);
        rte_rwlock_read_lock(&session_table[indx].lock);
        hmap_remove(&session_table[indx].hmap_sessions, &tcp->node);
        rte_rwlock_read_unlock(&session_table[indx].lock);
        tcp_session_free(tcp);
        return 0;
}


static inline void
log_packet_drop( uint32_t src_ip, uint32_t dst_ip, uint16_t src_port,
                 uint16_t dst_port, char * reason)
{
        char s_addr[INET_ADDRSTRLEN];
        char d_addr[INET_ADDRSTRLEN];

        DBG_ERR("TCP CONNTRACK DROPPED | %s:%d -> %s:%d | %s \n",
                vb_inet_ntop(AF_INET, &src_ip,(char *) &s_addr,
                             sizeof(s_addr)), src_port,
                vb_inet_ntop(AF_INET, &dst_ip, (char *)&d_addr,
                             sizeof(d_addr)), dst_port,
                reason );

}

static int tcp_conntrack_max_interfaces = -1;
static unsigned int *inf_acls = NULL ; /* array to store allowed states for each interface */

/*
 * NOTE : I'm assuming that all access to inf_acls array elements is atomic.
 * even if there is a race condition, the final value of will be legal
 */

/***************************************************************************
 *
 * External Functions
 *
 ***************************************************************************
 */

int
vb_tcp_conntrack_init( int max_interfaces)
{
        /* TODO protect this fuction  with a lock */
        if (max_interfaces<1)
                return VB_TCP_CONTRACK_EINVAL;

        if(inf_acls)
                tcp_session_free(inf_acls);

        inf_acls = tcp_session_alloc(sizeof(int)*max_interfaces);

        memset(inf_acls, 0, sizeof(int)*max_interfaces);

        return VB_TCP_CONTRACK_OK;
}

int
vb_tcp_conntrack_allow_all( int inf_id)
{
        inf_acls[inf_id] = -1; /*set all bits to true */

        return VB_TCP_CONTRACK_OK;
}

int
vb_tcp_conntrack_set_acl ( int inf_id, unsigned int allowed_states )
{
        if (inf_id<0 || inf_id>tcp_conntrack_max_interfaces )
                return VB_TCP_CONTRACK_EINVAL;

        inf_acls[inf_id] = allowed_states;

        return VB_TCP_CONTRACK_OK;
}

int
vb_tcp_conntrack_block ( int inf_id)
{
        if (inf_id<0 || inf_id>tcp_conntrack_max_interfaces )
                return VB_TCP_CONTRACK_EINVAL;

        inf_acls[inf_id] = 0;

        return VB_TCP_CONTRACK_OK;
}

/**
 * Process received packet
 *
 */
int
vb_tcp_conntrack( struct rte_mbuf *pkt,
                  int inf_id )
{
        struct tcp_session *tcp;
        struct tcp_hdr *tcphdr;
        struct ipv4_hdr *ipv4hdr ;
        uint32_t seq;
        uint32_t ack;
        unsigned int flags;

        if (inf_id<0 || inf_id>tcp_conntrack_max_interfaces )
                return VB_TCP_CONTRACK_EINVAL;

        unsigned char *pkt_data = rte_pktmbuf_mtod(pkt, unsigned char *);
        unsigned int allowed_states = inf_acls[ inf_id];
        uint16_t ether_type = rte_be_to_cpu_16(((struct ether_hdr *)pkt_data)->ether_type);
        pkt_data += sizeof(struct ether_hdr);

        if (ether_type == ETHER_TYPE_VLAN) {
                ether_type = rte_be_to_cpu_16(((struct vlan_hdr *)pkt_data)->eth_proto);
                pkt_data += sizeof(struct vlan_hdr);
        }

        if (ether_type == ETHER_TYPE_IPv4) {
                ipv4hdr  = (struct ipv4_hdr *)pkt_data;
                pkt_data += sizeof(struct ipv4_hdr);
        } else {
                log_packet_drop(ipv4hdr->src_addr, ipv4hdr->dst_addr,
                                tcphdr->src_port, tcphdr->dst_port,
                                "non IPV4 packet");
                return VB_TCP_CONTRACK_DROP;
        }

        /* it's guaranteed that  ipv4_hdr->next_proto_id == IPPROTO_TCP */
        tcphdr = (struct tcp_hdr *)pkt_data;

        /* if the union of the possible states is not allowed drop the packet */
        if (!((allowed_states & VB_TCP_STATE_NEW)||
              (allowed_states & VB_TCP_STATE_ESTABLISHED))
           ) {
                log_packet_drop(ipv4hdr->src_addr, ipv4hdr->dst_addr,
                                tcphdr->src_port, tcphdr->dst_port,
                                "SYN from unauthorized client");
                return VB_TCP_CONTRACK_DROP;
        }

        /* Sanity check packet */
        if ( rte_pktmbuf_pkt_len(pkt) < sizeof ( *tcphdr ) ) {
                log_packet_drop(ipv4hdr->src_addr, ipv4hdr->dst_addr,
                                tcphdr->src_port, tcphdr->dst_port,
                                "TCP packet too short");
                return VB_TCP_CONTRACK_DROP;
        }

        /* TODO : check for large header size */

        /* TODO?  checksum match */

        seq =  rte_be_to_cpu_32( tcphdr->sent_seq );
        ack =  rte_be_to_cpu_32( tcphdr->recv_ack );
        flags = tcphdr->tcp_flags;

        /* TODO? care about TCP option  */

        /* Dump header */
        tcp_dump_flags (  tcphdr->tcp_flags );

        /* The following code is bit repetitive ; but intention is to avoid
           the session lookup as much as possible */

        /* SYN */
        if (unlikely(flags & TCP_SYN)) {
                if (!(flags & TCP_ACK) ) { /* SYN no ACK -> new connection */
                        if (!(allowed_states & VB_TCP_STATE_NEW)) {
                                log_packet_drop(ipv4hdr->src_addr, ipv4hdr->dst_addr,
                                                tcphdr->src_port, tcphdr->dst_port,
                                                "SYN from unauthorized client");
                                return VB_TCP_CONTRACK_DROP;
                        }

                        tcp = tcp_session_lookup(ipv4hdr->dst_addr, ipv4hdr->src_addr,
                                                 tcphdr->dst_port);
                        if(tcp)
                                return VB_TCP_CONTRACK_DROP; /* session exist! ducplicate SYN? what to do? */

                        tcp = tcp_session_insert(ipv4hdr->dst_addr, ipv4hdr->src_addr,
                                                 tcphdr->dst_port);
                        tcp->tcp_state = VB_TCP_STATE_NEW;
                        tcp->snd_seq = seq;
                        return VB_TCP_CONTRACK_OK;
                } else { /* we are moing to established */
                        if (!(allowed_states & VB_TCP_STATE_ESTABLISHED)) {
                                log_packet_drop(ipv4hdr->src_addr, ipv4hdr->dst_addr,
                                                tcphdr->src_port, tcphdr->dst_port,
                                                "SYN_ACK from unauthorized client");
                                return VB_TCP_CONTRACK_DROP;
                        }
                        tcp = tcp_session_lookup(ipv4hdr->dst_addr, ipv4hdr->src_addr,
                                                 tcphdr->dst_port);
                        if( ack == (tcp->snd_seq + 1)) {
                                tcp->tcp_state = VB_TCP_STATE_ESTABLISHED;
                        } else {
                                /* spurious SYN_ACK */
                                log_packet_drop(ipv4hdr->src_addr, ipv4hdr->dst_addr,
                                                tcphdr->src_port, tcphdr->dst_port,
                                                "Spurious SYN");
                                return VB_TCP_CONTRACK_DROP;
                        }

                }
        }

        if (unlikely(!(allowed_states & VB_TCP_STATE_ESTABLISHED))) {
                log_packet_drop(ipv4hdr->src_addr, ipv4hdr->dst_addr,
                                tcphdr->src_port, tcphdr->dst_port,
                                "ESTBLASHISHED TCP packet from unauthorized client");
                return VB_TCP_CONTRACK_DROP;

        }

        tcp = tcp_session_lookup(ipv4hdr->dst_addr, ipv4hdr->src_addr,
                                 tcphdr->dst_port);

        /* no connection was found
           TODO send RST ? */
        if(unlikely(!tcp)) {
                log_packet_drop(ipv4hdr->src_addr, ipv4hdr->dst_addr,
                                tcphdr->src_port, tcphdr->dst_port,
                                "SYN from unauthorized client");
                return VB_TCP_CONTRACK_DROP;
        }
        /* TODO check/handle spurious packets? */



        if ( unlikely(flags & TCP_FIN)) {
                if (!(flags & TCP_ACK) ) { /* SYN no ACK -> new connection */
                        tcp->snd_seq = seq;
                } else {
                        if( ack == (tcp->snd_seq + 1)) {
                                tcp_session_delete(tcp);
                                return VB_TCP_CONTRACK_OK;
                        } else {
                                /* spurious FIN_ACK */
                                log_packet_drop(ipv4hdr->src_addr, ipv4hdr->dst_addr,
                                                tcphdr->src_port, tcphdr->dst_port,
                                                "Spurious FIN");
                                return VB_TCP_CONTRACK_DROP;
                        }
                }
        }

        /* Handle RST, if present */
        if ( unlikely(flags & TCP_RST)  ) {
                tcp_session_delete(tcp);
                return VB_TCP_CONTRACK_OK;
        }


        return VB_TCP_CONTRACK_OK;
}
