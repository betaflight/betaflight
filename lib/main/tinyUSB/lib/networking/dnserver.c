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

/*
 * version: 1.0 demo (7.02.2015)
 * brief:   tiny dns ipv4 server using lwip (pcb)
 */

#include "dnserver.h"

#define DNS_MAX_HOST_NAME_LEN 128

static struct udp_pcb *pcb = NULL;
dns_query_proc_t query_proc = NULL;

#pragma pack(push, 1)
typedef struct
{
#if BYTE_ORDER == LITTLE_ENDIAN
	uint8_t rd: 1,     /* Recursion Desired */
	        tc: 1,     /* Truncation Flag */
	        aa: 1,     /* Authoritative Answer Flag */
	        opcode: 4, /* Operation code */
	        qr: 1;     /* Query/Response Flag */
	uint8_t rcode: 4,  /* Response Code */
	        z: 3,      /* Zero */
	        ra: 1;     /* Recursion Available */
#else
	uint8_t qr: 1,     /* Query/Response Flag */
	        opcode: 4, /* Operation code */
	        aa: 1,     /* Authoritative Answer Flag */
	        tc: 1,     /* Truncation Flag */
	        rd: 1;     /* Recursion Desired */
	uint8_t ra: 1,     /* Recursion Available */
	        z: 3,      /* Zero */
	        rcode: 4;  /* Response Code */
#endif
} dns_header_flags_t;

typedef struct
{
	uint16_t id;
	dns_header_flags_t flags;
	uint16_t n_record[4];
} dns_header_t;

typedef struct dns_answer
{
	uint16_t name;
	uint16_t type;
	uint16_t Class;
	uint32_t ttl;
	uint16_t len;
	uint32_t addr;
} dns_answer_t;
#pragma pack(pop)

typedef struct dns_query
{
	char name[DNS_MAX_HOST_NAME_LEN];
	uint16_t type;
	uint16_t Class;
} dns_query_t;

static uint16_t get_uint16(const uint8_t *pnt)
{
  uint16_t result;
  memcpy(&result, pnt, sizeof(result));
  return result;
}

static int parse_next_query(void *data, int size, dns_query_t *query)
{
	int len;
	int labels;
	uint8_t *ptr;

	len = 0;
	labels = 0;
	ptr = (uint8_t *)data;

	while (true)
	{
		uint8_t lable_len;
		if (size <= 0) return -1;
		lable_len = *ptr++;
		size--;
		if (lable_len == 0) break;
		if (labels > 0)
		{
			if (len == DNS_MAX_HOST_NAME_LEN) return -2;
			query->name[len++] = '.';
		}
		if (lable_len > size) return -1;
		if (len + lable_len >= DNS_MAX_HOST_NAME_LEN) return -2;
		memcpy(&query->name[len], ptr, lable_len);
		len += lable_len;
		ptr += lable_len;
		size -= lable_len;
		labels++;
	}

	if (size < 4) return -1;
	query->name[len] = 0;
	query->type = get_uint16(ptr);
	ptr += 2;
	query->Class = get_uint16(ptr);
	ptr += 2;
	return ptr - (uint8_t *)data;
}

static void udp_recv_proc(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
	int len;
	dns_header_t *header;
	static dns_query_t query;
	struct pbuf *out;
	ip4_addr_t host_addr;
	dns_answer_t *answer;

	(void)arg;

	if (p->len <= sizeof(dns_header_t)) goto error;
	header = (dns_header_t *)p->payload;
	if (header->flags.qr != 0) goto error;
	if (ntohs(header->n_record[0]) != 1) goto error;

	len = parse_next_query(header + 1, p->len - sizeof(dns_header_t), &query);
	if (len < 0) goto error;
	if (!query_proc(query.name, &host_addr)) goto error;

	len += sizeof(dns_header_t);
	out = pbuf_alloc(PBUF_TRANSPORT, len + 16, PBUF_POOL);
	if (out == NULL) goto error;

	memcpy(out->payload, p->payload, len);
	header = (dns_header_t *)out->payload;
	header->flags.qr = 1;
	header->n_record[1] = htons(1);
	answer = (struct dns_answer *)((uint8_t *)out->payload + len);
	answer->name = htons(0xC00C);
	answer->type = htons(1);
	answer->Class = htons(1);
	answer->ttl = htonl(32);
	answer->len = htons(4);
	answer->addr = host_addr.addr;

	udp_sendto(upcb, out, addr, port);
	pbuf_free(out);

error:
	pbuf_free(p);
}

err_t dnserv_init(const ip_addr_t *bind, uint16_t port, dns_query_proc_t qp)
{
	err_t err;
	udp_init();
	dnserv_free();
	pcb = udp_new();
	if (pcb == NULL)
		return ERR_MEM;
	err = udp_bind(pcb, bind, port);
	if (err != ERR_OK)
	{
		dnserv_free();
		return err;
	}
	udp_recv(pcb, udp_recv_proc, NULL);
	query_proc = qp;
	return ERR_OK;
}

void dnserv_free(void)
{
	if (pcb == NULL) return;
	udp_remove(pcb);
	pcb = NULL;
}
