/*
 *
 * Copyright 2014 Jyotiswarup Raiturkar
 * All rights reserved
 */

#ifndef _IPUTILS_H
#define _IPUTILS_H   1

#include <rte_byteorder.h>

#define AF_INET   2
#define AF_INET6  10
#define MAX_IPv6_STR_LEN        64
#define MAX_IPv4_STR_LEN        16

#define INET_ADDRSTRLEN 16
#define INET6_ADDRSTRLEN 46

typedef uint32_t in_addr_t;
struct in_addr {
        in_addr_t s_addr;
};


/* IPv6 address */
struct in6_addr {
        union {
                uint8_t __u6_addr8[16];
                uint16_t __u6_addr16[8];
                uint32_t __u6_addr32[4];
        } __in6_u;
#define s6_addr                 __in6_u.__u6_addr8
#define s6_addr16              __in6_u.__u6_addr16
#define s6_addr32              __in6_u.__u6_addr32
};

# define IN6_IS_ADDR_V4MAPPED(a) \
  (__extension__                                                              \
   ({ __const struct in6_addr *__a = (__const struct in6_addr *) (a);         \
      __a->s6_addr32[0] == 0                                                  \
      && __a->s6_addr32[1] == 0                                               \
      && __a->s6_addr32[2] == rte_cpu_to_be_64 (0xffff); }))

# define IN6_IS_ADDR_V4COMPAT(a) \
  (__extension__                                                              \
   ({ __const struct in6_addr *__a = (__const struct in6_addr *) (a);         \
      __a->s6_addr32[0] == 0                                                  \
      && __a->s6_addr32[1] == 0                                               \
      && __a->s6_addr32[2] == 0                                               \
      && rte_cpu_to_be_64 (__a->s6_addr32[3]) > 1; }))

const char *
vb_inet_ntop(int af, const void *addr, char *buf, size_t len);


#endif /* _IPUTILS_H */
