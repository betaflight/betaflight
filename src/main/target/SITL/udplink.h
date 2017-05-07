/**
 * Copyright (c) 2017 cs8425
 *
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the MIT license.
 */

#ifndef __UDPLINK_H
#define __UDPLINK_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include <arpa/inet.h>
#include <netinet/in.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	int fd;
	struct sockaddr_in si;
	struct sockaddr_in recv;
	int port;
	char* addr;
	bool isServer;
} udpLink_t;

int udpInit(udpLink_t* link, const char* addr, int port, bool isServer);
int udpRecv(udpLink_t* link, void* data, size_t size, uint32_t timeout_ms);
int udpSend(udpLink_t* link, const void* data, size_t size);

#ifdef __cplusplus
} // extern "C"
#endif

#endif

