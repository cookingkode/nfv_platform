/*
 *
 * Copyright 2014 Jyotiswarup Raiturkar
 * All rights reserved
 */
#ifndef _TCP_CONNTRAC_H
#define _TCP_CONNTRAC_H


/** End of TCP options list */
#define TCP_OPTION_END 0

/** TCP option pad */
#define TCP_OPTION_NOP 1

/** Generic TCP option */
struct tcp_option {
        uint8_t kind;
        uint8_t length;
} __attribute__ (( packed ));

/** TCP MSS option */
struct tcp_mss_option {
        uint8_t kind;
        uint8_t length;
        uint16_t mss;
} __attribute__ (( packed ));

/** Code for the TCP MSS option */
#define TCP_OPTION_MSS 2

/** TCP timestamp option */
struct tcp_timestamp_option {
        uint8_t kind;
        uint8_t length;
        uint32_t tsval;
        uint32_t tsecr;
} __attribute__ (( packed ));

/** Padded TCP timestamp option (used for sending) */
struct tcp_timestamp_padded_option {
        uint8_t nop[2];
        struct tcp_timestamp_option tsopt;
} __attribute__ (( packed ));

/** Code for the TCP timestamp option */
#define TCP_OPTION_TS 8

/** Parsed TCP options */
struct tcp_options {
        /** MSS option, if present */
        const struct tcp_mss_option *mssopt;
        /** Timestampe option, if present */
        const struct tcp_timestamp_option *tsopt;
};

/** @} */

/*
 * TCP flags
 */
#define TCP_CWR		0x80
#define TCP_ECE		0x40
#define TCP_URG		0x20
#define TCP_ACK		0x10
#define TCP_PSH		0x08
#define TCP_RST		0x04
#define TCP_SYN		0x02
#define TCP_FIN		0x01

/* TCP Conntrak Return Codes */
#define VB_TCP_CONTRACK_OK        0
#define VB_TCP_CONTRACK_EINVAL    -1
#define VB_TCP_CONTRACK_DROP      -100

/* TCP Connection States for Conntrack*/
/* While adding new states, please keep then bitwise exclusive so that's it's easy to a pass a set */
#define VB_TCP_STATE_NEW          0x1
#define VB_TCP_STATE_ESTABLISHED  0x2

/*
 * Init function for TCP connection tracking.
 * max_interfaces is the maximum number of interfaces in the system.
 *
 */
inline int
vb_tcp_conntrack_init( int max_interfaces);

/*
 * Main connection tracker used to process every TCP packet. The allowed_states
 * should be an OR mask of TCP Connection States.
 *
 * Function returns TCP Conntrak Return codes
 */
int
vb_tcp_conntrack_process ( struct rte_mbuf *pkt,
                           int inf_id ) ;


/*
 * Set the allowed states for an interface. The allowed_states
 * should be an OR mask of TCP Connection States.
 *
 * Function returns TCP Conntrak Return codes
 */

inline int
vb_tcp_conntrack_set_acl ( int inf_id,
                           unsigned int allowed_states );

/*
 * Clear the allowed states for an interface.
 *
 * Note : This will blackhole TCP for this interface!!
 *
 * Function returns TCP Conntrak Return codes
 */
inline int
vb_tcp_conntrack_block ( int inf_id);

/*
 * Allow all TCP traffic  for an interface.
 *
 * Function returns TCP Conntrak Return codes
 */
inline int
vb_tcp_conntrack_allow_all ( int inf_id);


#endif /* _TCP_CONNTRAC_H */
