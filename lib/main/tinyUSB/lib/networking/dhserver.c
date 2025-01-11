/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 by Sergey Fetisov <fsenok@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "dhserver.h"

/* DHCP message type */
#define DHCP_DISCOVER       1
#define DHCP_OFFER          2
#define DHCP_REQUEST        3
#define DHCP_DECLINE        4
#define DHCP_ACK            5
#define DHCP_NAK            6
#define DHCP_RELEASE        7
#define DHCP_INFORM         8

/* DHCP options */
enum DHCP_OPTIONS
{
	DHCP_PAD                    = 0,
	DHCP_SUBNETMASK             = 1,
	DHCP_ROUTER                 = 3,
	DHCP_DNSSERVER              = 6,
	DHCP_HOSTNAME               = 12,
	DHCP_DNSDOMAIN              = 15,
	DHCP_MTU                    = 26,
	DHCP_BROADCAST              = 28,
	DHCP_PERFORMROUTERDISC      = 31,
	DHCP_STATICROUTE            = 33,
	DHCP_NISDOMAIN              = 40,
	DHCP_NISSERVER              = 41,
	DHCP_NTPSERVER              = 42,
	DHCP_VENDOR                 = 43,
	DHCP_IPADDRESS              = 50,
	DHCP_LEASETIME              = 51,
	DHCP_OPTIONSOVERLOADED      = 52,
	DHCP_MESSAGETYPE            = 53,
	DHCP_SERVERID               = 54,
	DHCP_PARAMETERREQUESTLIST   = 55,
	DHCP_MESSAGE                = 56,
	DHCP_MAXMESSAGESIZE         = 57,
	DHCP_RENEWALTIME            = 58,
	DHCP_REBINDTIME             = 59,
	DHCP_CLASSID                = 60,
	DHCP_CLIENTID               = 61,
	DHCP_USERCLASS              = 77,  /* RFC 3004 */
	DHCP_FQDN                   = 81,
	DHCP_DNSSEARCH              = 119, /* RFC 3397 */
	DHCP_CSR                    = 121, /* RFC 3442 */
	DHCP_MSCSR                  = 249, /* MS code for RFC 3442 */
	DHCP_END                    = 255
};

typedef struct
{
    uint8_t  dp_op;           /* packet opcode type */
    uint8_t  dp_htype;        /* hardware addr type */
    uint8_t  dp_hlen;         /* hardware addr length */
    uint8_t  dp_hops;         /* gateway hops */
    uint32_t dp_xid;          /* transaction ID */
    uint16_t dp_secs;         /* seconds since boot began */
    uint16_t dp_flags;
    uint8_t  dp_ciaddr[4];    /* client IP address */
    uint8_t  dp_yiaddr[4];    /* 'your' IP address */
    uint8_t  dp_siaddr[4];    /* server IP address */
    uint8_t  dp_giaddr[4];    /* gateway IP address */
    uint8_t  dp_chaddr[16];   /* client hardware address */
    uint8_t  dp_legacy[192];
    uint8_t  dp_magic[4];
    uint8_t  dp_options[275]; /* options area */
} DHCP_TYPE;

DHCP_TYPE dhcp_data;
static struct udp_pcb *pcb = NULL;
static const dhcp_config_t *config = NULL;

char magic_cookie[] = {0x63,0x82,0x53,0x63};

static ip4_addr_t get_ip(const uint8_t *pnt)
{
  ip4_addr_t result;
  memcpy(&result, pnt, sizeof(result));
  return result;
}

static void set_ip(uint8_t *pnt, ip4_addr_t value)
{
  memcpy(pnt, &value.addr, sizeof(value.addr));
}

static dhcp_entry_t *entry_by_ip(ip4_addr_t ip)
{
	int i;
	for (i = 0; i < config->num_entry; i++)
		if (config->entries[i].addr.addr == ip.addr)
			return &config->entries[i];
	return NULL;
}

static dhcp_entry_t *entry_by_mac(uint8_t *mac)
{
	int i;
	for (i = 0; i < config->num_entry; i++)
		if (memcmp(config->entries[i].mac, mac, 6) == 0)
			return &config->entries[i];
	return NULL;
}

static __inline bool is_vacant(dhcp_entry_t *entry)
{
	return memcmp("\0\0\0\0\0", entry->mac, 6) == 0;
}

static dhcp_entry_t *vacant_address(void)
{
	int i;
	for (i = 0; i < config->num_entry; i++)
		if (is_vacant(config->entries + i))
			return config->entries + i;
	return NULL;
}

static __inline void free_entry(dhcp_entry_t *entry)
{
	memset(entry->mac, 0, 6);
}

uint8_t *find_dhcp_option(uint8_t *attrs, int size, uint8_t attr)
{
	int i = 0;
	while ((i + 1) < size)
	{
		int next = i + attrs[i + 1] + 2;
		if (next > size) return NULL;
		if (attrs[i] == attr)
			return attrs + i;
		i = next;
	}
	return NULL;
}

int fill_options(void *dest,
	uint8_t msg_type,
	const char *domain,
	ip4_addr_t dns,
	int lease_time,
	ip4_addr_t serverid,
	ip4_addr_t router,
	ip4_addr_t subnet)
{
	uint8_t *ptr = (uint8_t *)dest;
	/* ACK message type */
	*ptr++ = 53;
	*ptr++ = 1;
	*ptr++ = msg_type;

	/* dhcp server identifier */
	*ptr++ = DHCP_SERVERID;
	*ptr++ = 4;
	set_ip(ptr, serverid);
	ptr += 4;

	/* lease time */
	*ptr++ = DHCP_LEASETIME;
	*ptr++ = 4;
	*ptr++ = (lease_time >> 24) & 0xFF;
	*ptr++ = (lease_time >> 16) & 0xFF;
	*ptr++ = (lease_time >> 8) & 0xFF;
	*ptr++ = (lease_time >> 0) & 0xFF;

	/* subnet mask */
	*ptr++ = DHCP_SUBNETMASK;
	*ptr++ = 4;
	set_ip(ptr, subnet);
	ptr += 4;

	/* router */
	if (router.addr != 0)
	{
		*ptr++ = DHCP_ROUTER;
		*ptr++ = 4;
		set_ip(ptr, router);
		ptr += 4;
	}

	/* domain name */
	if (domain != NULL)
	{
		int len = strlen(domain);
		*ptr++ = DHCP_DNSDOMAIN;
		*ptr++ = len;
		memcpy(ptr, domain, len);
		ptr += len;
	}

	/* domain name server (DNS) */
	if (dns.addr != 0)
	{
		*ptr++ = DHCP_DNSSERVER;
		*ptr++ = 4;
		set_ip(ptr, dns);
		ptr += 4;
	}

	/* end */
	*ptr++ = DHCP_END;
	return ptr - (uint8_t *)dest;
}

static void udp_recv_proc(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
	uint8_t *ptr;
	dhcp_entry_t *entry;
	struct pbuf *pp;
	struct netif *netif = netif_get_by_index(p->if_idx);

	(void)arg;
	(void)addr;

	unsigned n = p->len;
	if (n > sizeof(dhcp_data)) n = sizeof(dhcp_data);
	memcpy(&dhcp_data, p->payload, n);

	ptr = find_dhcp_option(dhcp_data.dp_options, sizeof(dhcp_data.dp_options), DHCP_MESSAGETYPE);
	if (ptr == NULL)
	{
		pbuf_free(p);
		return;
	}

	switch (ptr[2])
	{
		case DHCP_DISCOVER:
			entry = entry_by_mac(dhcp_data.dp_chaddr);
			if (entry == NULL) entry = vacant_address();
			if (entry == NULL) break;

			dhcp_data.dp_op = 2; /* reply */
			dhcp_data.dp_secs = 0;
			dhcp_data.dp_flags = 0;
			set_ip(dhcp_data.dp_yiaddr, entry->addr);
			memcpy(dhcp_data.dp_magic, magic_cookie, 4);

			memset(dhcp_data.dp_options, 0, sizeof(dhcp_data.dp_options));

			fill_options(dhcp_data.dp_options,
				DHCP_OFFER,
				config->domain,
				config->dns,
				entry->lease,
				*netif_ip4_addr(netif),
				config->router,
				*netif_ip4_netmask(netif));

			pp = pbuf_alloc(PBUF_TRANSPORT, sizeof(dhcp_data), PBUF_POOL);
			if (pp == NULL) break;
			memcpy(pp->payload, &dhcp_data, sizeof(dhcp_data));
			udp_sendto(upcb, pp, IP_ADDR_BROADCAST, port);
			pbuf_free(pp);
			break;

		case DHCP_REQUEST:
			/* 1. find requested ipaddr in option list */
			ptr = find_dhcp_option(dhcp_data.dp_options, sizeof(dhcp_data.dp_options), DHCP_IPADDRESS);
			if (ptr == NULL) break;
			if (ptr[1] != 4) break;
			ptr += 2;

			/* 2. does hw-address registered? */
			entry = entry_by_mac(dhcp_data.dp_chaddr);
			if (entry != NULL) free_entry(entry);

			/* 3. find requested ipaddr */
			entry = entry_by_ip(get_ip(ptr));
			if (entry == NULL) break;
			if (!is_vacant(entry)) break;

			/* 4. fill struct fields */
			memcpy(dhcp_data.dp_yiaddr, ptr, 4);
			dhcp_data.dp_op = 2; /* reply */
			dhcp_data.dp_secs = 0;
			dhcp_data.dp_flags = 0;
			memcpy(dhcp_data.dp_magic, magic_cookie, 4);

			/* 5. fill options */
			memset(dhcp_data.dp_options, 0, sizeof(dhcp_data.dp_options));

			fill_options(dhcp_data.dp_options,
				DHCP_ACK,
				config->domain,
				config->dns,
				entry->lease,
				*netif_ip4_addr(netif),
				config->router,
				*netif_ip4_netmask(netif));

			/* 6. send ACK */
			pp = pbuf_alloc(PBUF_TRANSPORT, sizeof(dhcp_data), PBUF_POOL);
			if (pp == NULL) break;
			memcpy(entry->mac, dhcp_data.dp_chaddr, 6);
			memcpy(pp->payload, &dhcp_data, sizeof(dhcp_data));
			udp_sendto(upcb, pp, IP_ADDR_BROADCAST, port);
			pbuf_free(pp);
			break;

		default:
				break;
	}
	pbuf_free(p);
}

err_t dhserv_init(const dhcp_config_t *c)
{
	err_t err;
	udp_init();
	dhserv_free();
	pcb = udp_new();
	if (pcb == NULL)
		return ERR_MEM;
	err = udp_bind(pcb, IP_ADDR_ANY, c->port);
	if (err != ERR_OK)
	{
		dhserv_free();
		return err;
	}
	udp_recv(pcb, udp_recv_proc, NULL);
	config = c;
	return ERR_OK;
}

void dhserv_free(void)
{
	if (pcb == NULL) return;
	udp_remove(pcb);
	pcb = NULL;
}
