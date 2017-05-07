/**
 * Copyright (c) 2016 rxi
 *
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the MIT license. See LICENSE for details.
 */

#ifdef _WIN32
  #define _WIN32_WINNT 0x501
  #ifndef _CRT_SECURE_NO_WARNINGS
    #define _CRT_SECURE_NO_WARNINGS
  #endif
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #include <windows.h>
#else
  #define _POSIX_C_SOURCE 200809L
  #ifdef __APPLE__
    #define _DARWIN_UNLIMITED_SELECT
  #endif
  #include <unistd.h>
  #include <netdb.h>
  #include <fcntl.h>
  #include <sys/types.h>
  #include <sys/socket.h>
  #include <sys/time.h>
  #include <netinet/in.h>
  #include <netinet/tcp.h>
  #include <arpa/inet.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <signal.h>
#include <errno.h>
#include <limits.h>

#include "dyad.h"

#define DYAD_VERSION "0.2.1"


#ifdef _WIN32
  #define close(a) closesocket(a)
  #define getsockopt(a,b,c,d,e) getsockopt((a),(b),(c),(char*)(d),(e))
  #define setsockopt(a,b,c,d,e) setsockopt((a),(b),(c),(char*)(d),(e))
  #define select(a,b,c,d,e) select((int)(a),(b),(c),(d),(e))
  #define bind(a,b,c) bind((a),(b),(int)(c))
  #define connect(a,b,c) connect((a),(b),(int)(c))

  #undef  errno
  #define errno WSAGetLastError()

  #undef  EWOULDBLOCK
  #define EWOULDBLOCK WSAEWOULDBLOCK

  const char *inet_ntop(int af, const void *src, char *dst, socklen_t size) {
    union { struct sockaddr sa; struct sockaddr_in sai;
            struct sockaddr_in6 sai6; } addr;
    int res;
    memset(&addr, 0, sizeof(addr));
    addr.sa.sa_family = af;
    if (af == AF_INET6) {
      memcpy(&addr.sai6.sin6_addr, src, sizeof(addr.sai6.sin6_addr));
    } else {
      memcpy(&addr.sai.sin_addr, src, sizeof(addr.sai.sin_addr));
    }
    res = WSAAddressToStringA(&addr.sa, sizeof(addr), 0, dst, (LPDWORD) &size);
    if (res != 0) return NULL;
    return dst;
  }
#endif

#ifndef INVALID_SOCKET
  #define INVALID_SOCKET -1
#endif


/*===========================================================================*/
/* Memory                                                                    */
/*===========================================================================*/

static void panic(const char *fmt, ...);

static void *dyad_realloc(void *ptr, int n) {
  ptr = realloc(ptr, n);
  if (!ptr && n != 0) {
    panic("out of memory");
  }
  return ptr;
}


static void dyad_free(void *ptr) {
  free(ptr);
}


/*===========================================================================*/
/* Vec (dynamic array)                                                       */
/*===========================================================================*/

static void vec_expand(char **data, int *length, int *capacity, int memsz) {
  if (*length + 1 > *capacity) {
    if (*capacity == 0) {
      *capacity = 1;
    } else {
      *capacity <<= 1;
    }
    *data = dyad_realloc(*data, *capacity * memsz);
  }
}

static void vec_splice(
  char **data, int *length, int *capacity, int memsz, int start, int count
) {
  (void) capacity;
  memmove(*data + start * memsz,
          *data + (start + count) * memsz,
          (*length - start - count) * memsz);
}


#define Vec(T)\
  struct { T *data; int length, capacity; }


#define vec_unpack(v)\
  (char**)&(v)->data, &(v)->length, &(v)->capacity, sizeof(*(v)->data)


#define vec_init(v)\
  memset((v), 0, sizeof(*(v)))


#define vec_deinit(v)\
  dyad_free((v)->data)


#define vec_clear(v)\
  ((v)->length = 0)


#define vec_push(v, val)\
  ( vec_expand(vec_unpack(v)),\
    (v)->data[(v)->length++] = (val) )


#define vec_splice(v, start, count)\
  ( vec_splice(vec_unpack(v), start, count),\
    (v)->length -= (count) )



/*===========================================================================*/
/* SelectSet                                                                 */
/*===========================================================================*/

/* A wrapper around the three fd_sets used for select(). The fd_sets' allocated
 * memory is automatically expanded to accommodate fds as they are added.
 *
 * On Windows fd_sets are implemented as arrays; the FD_xxx macros are not used
 * by the wrapper and instead the fd_set struct is manipulated directly. The
 * wrapper should perform better than the normal FD_xxx macros, given that we
 * don't bother with the linear search which FD_SET would perform to check for
 * duplicates.
 *
 * On non-Windows platforms the sets are assumed to be bit arrays. The FD_xxx
 * macros are not used in case their implementation attempts to do bounds
 * checking; instead we manipulate the fd_sets' bits directly.
 */

enum {
  SELECT_READ,
  SELECT_WRITE,
  SELECT_EXCEPT,
  SELECT_MAX
};

typedef struct {
  int capacity;
  dyad_Socket maxfd;
  fd_set *fds[SELECT_MAX];
} SelectSet;

#define DYAD_UNSIGNED_BIT (sizeof(unsigned) * CHAR_BIT)


static void select_deinit(SelectSet *s) {
  int i;
  for (i = 0; i < SELECT_MAX; i++) {
    dyad_free(s->fds[i]);
    s->fds[i] = NULL;
  }
  s->capacity = 0;
}


static void select_grow(SelectSet *s) {
  int i;
  int oldCapacity = s->capacity;
  s->capacity = s->capacity ? s->capacity << 1 : 1;
  for (i = 0; i < SELECT_MAX; i++) {
    s->fds[i] = dyad_realloc(s->fds[i], s->capacity * sizeof(fd_set));
    memset(s->fds[i] + oldCapacity, 0,
           (s->capacity - oldCapacity) * sizeof(fd_set));
  }
}


static void select_zero(SelectSet *s) {
  int i;
  if (s->capacity == 0) return;
  s->maxfd = 0;
  for (i = 0; i < SELECT_MAX; i++) {
#if _WIN32
    s->fds[i]->fd_count = 0;
#else
    memset(s->fds[i], 0, s->capacity * sizeof(fd_set));
#endif
  }
}


static void select_add(SelectSet *s, int set, dyad_Socket fd) {
#ifdef _WIN32
  fd_set *f;
  if (s->capacity == 0) select_grow(s);
  while ((unsigned) (s->capacity * FD_SETSIZE) < s->fds[set]->fd_count + 1) {
    select_grow(s);
  }
  f = s->fds[set];
  f->fd_array[f->fd_count++] = fd;
#else
  unsigned *p;
  while (s->capacity * FD_SETSIZE < fd) {
    select_grow(s);
  }
  p = (unsigned*) s->fds[set];
  p[fd / DYAD_UNSIGNED_BIT] |= 1 << (fd % DYAD_UNSIGNED_BIT);
  if (fd > s->maxfd) s->maxfd = fd;
#endif
}


static int select_has(SelectSet *s, int set, dyad_Socket fd) {
#ifdef _WIN32
  unsigned i;
  fd_set *f;
  if (s->capacity == 0) return 0;
  f = s->fds[set];
  for (i = 0; i < f->fd_count; i++) {
    if (f->fd_array[i] == fd) {
      return 1;
    }
  }
  return 0;
#else
  unsigned *p;
  if (s->maxfd < fd) return 0;
  p = (unsigned*) s->fds[set];
  return p[fd / DYAD_UNSIGNED_BIT] & (1 << (fd % DYAD_UNSIGNED_BIT));
#endif
}


/*===========================================================================*/
/* Core                                                                      */
/*===========================================================================*/

typedef struct {
  int event;
  dyad_Callback callback;
  void *udata;
} Listener;


struct dyad_Stream {
  int state, flags;
  dyad_Socket sockfd;
  char *address;
  int port;
  int bytesSent, bytesReceived;
  double lastActivity, timeout;
  Vec(Listener) listeners;
  Vec(char) lineBuffer;
  Vec(char) writeBuffer;
  dyad_Stream *next;
};

#define DYAD_FLAG_READY   (1 << 0)
#define DYAD_FLAG_WRITTEN (1 << 1)


static dyad_Stream *dyad_streams;
static int dyad_streamCount;
static char dyad_panicMsgBuffer[128];
static dyad_PanicCallback panicCallback;
static SelectSet dyad_selectSet;
static double dyad_updateTimeout = 1;
static double dyad_tickInterval = 1;
static double dyad_lastTick = 0;


static void panic(const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  vsprintf(dyad_panicMsgBuffer, fmt, args);
  va_end(args);
  if (panicCallback) {
    panicCallback(dyad_panicMsgBuffer);
  } else {
    printf("dyad panic: %s\n", dyad_panicMsgBuffer);
  }
  exit(EXIT_FAILURE);
}


static dyad_Event createEvent(int type) {
  dyad_Event e;
  memset(&e, 0, sizeof(e));
  e.type = type;
  return e;
}


static void stream_destroy(dyad_Stream *stream);

static void destroyClosedStreams(void) {
  dyad_Stream *stream = dyad_streams;
  while (stream) {
    if (stream->state == DYAD_STATE_CLOSED) {
      dyad_Stream *next = stream->next;
      stream_destroy(stream);
      stream = next;
    } else {
      stream = stream->next;
    }
  }
}


static void stream_emitEvent(dyad_Stream *stream, dyad_Event *e);

static void updateTickTimer(void) {
  /* Update tick timer */
  if (dyad_lastTick == 0) {
    dyad_lastTick = dyad_getTime();
  }
  while (dyad_lastTick < dyad_getTime()) {
    /* Emit event on all streams */
    dyad_Stream *stream;
    dyad_Event e = createEvent(DYAD_EVENT_TICK);
    e.msg = "a tick has occured";
    stream = dyad_streams;
    while (stream) {
      stream_emitEvent(stream, &e);
      stream = stream->next;
    }
    dyad_lastTick += dyad_tickInterval;
  }
}


static void updateStreamTimeouts(void) {
  double currentTime = dyad_getTime();
  dyad_Stream *stream;
  dyad_Event e = createEvent(DYAD_EVENT_TIMEOUT);
  e.msg = "stream timed out";
  stream = dyad_streams;
  while (stream) {
    if (stream->timeout) {
      if (currentTime - stream->lastActivity > stream->timeout) {
        stream_emitEvent(stream, &e);
        dyad_close(stream);
      }
    }
    stream = stream->next;
  }
}



/*===========================================================================*/
/* Stream                                                                    */
/*===========================================================================*/

static void stream_destroy(dyad_Stream *stream) {
  dyad_Event e;
  dyad_Stream **next;
  /* Close socket */
  if (stream->sockfd != INVALID_SOCKET) {
    close(stream->sockfd);
  }
  /* Emit destroy event */
  e = createEvent(DYAD_EVENT_DESTROY);
  e.msg = "the stream has been destroyed";
  stream_emitEvent(stream, &e);
  /* Remove from list and decrement count */
  next = &dyad_streams;
  while (*next != stream) {
    next = &(*next)->next;
  }
  *next = stream->next;
  dyad_streamCount--;
  /* Destroy and free */
  vec_deinit(&stream->listeners);
  vec_deinit(&stream->lineBuffer);
  vec_deinit(&stream->writeBuffer);
  dyad_free(stream->address);
  dyad_free(stream);
}


static void stream_emitEvent(dyad_Stream *stream, dyad_Event *e) {
  int i;
  e->stream = stream;
  for (i = 0; i < stream->listeners.length; i++) {
    Listener *listener = &stream->listeners.data[i];
    if (listener->event == e->type) {
      e->udata = listener->udata;
      listener->callback(e);
    }
    /* Check to see if this listener was removed: If it was we decrement `i`
     * since the next listener will now be in this ones place */
    if (listener != &stream->listeners.data[i]) {
      i--;
    }
  }
}


static void stream_error(dyad_Stream *stream, const char *msg, int err) {
  char buf[256];
  dyad_Event e = createEvent(DYAD_EVENT_ERROR);
  if (err) {
    sprintf(buf, "%.160s (%.80s)", msg, strerror(err));
    e.msg = buf;
  } else {
    e.msg = msg;
  }
  stream_emitEvent(stream, &e);
  dyad_close(stream);
}


static void stream_initAddress(dyad_Stream *stream) {
  union { struct sockaddr sa; struct sockaddr_storage sas;
          struct sockaddr_in sai; struct sockaddr_in6 sai6; } addr;
  socklen_t size;
  memset(&addr, 0, sizeof(addr));
  size = sizeof(addr);
  dyad_free(stream->address);
  stream->address = NULL;
  if (getpeername(stream->sockfd, &addr.sa, &size) == -1) {
    if (getsockname(stream->sockfd, &addr.sa, &size) == -1) {
      return;
    }
  }
  if (addr.sas.ss_family == AF_INET6) {
    stream->address = dyad_realloc(NULL, INET6_ADDRSTRLEN);
    inet_ntop(AF_INET6, &addr.sai6.sin6_addr, stream->address,
              INET6_ADDRSTRLEN);
    stream->port = ntohs(addr.sai6.sin6_port);
  } else {
    stream->address = dyad_realloc(NULL, INET_ADDRSTRLEN);
    inet_ntop(AF_INET, &addr.sai.sin_addr, stream->address, INET_ADDRSTRLEN);
    stream->port = ntohs(addr.sai.sin_port);
  }
}


static void stream_setSocketNonBlocking(dyad_Stream *stream, int opt) {
#ifdef _WIN32
  u_long mode = opt;
  ioctlsocket(stream->sockfd, FIONBIO, &mode);
#else
  int flags = fcntl(stream->sockfd, F_GETFL);
  fcntl(stream->sockfd, F_SETFL,
        opt ? (flags | O_NONBLOCK) : (flags & ~O_NONBLOCK));
#endif
}


static void stream_setSocket(dyad_Stream *stream, dyad_Socket sockfd) {
  stream->sockfd = sockfd;
  stream_setSocketNonBlocking(stream, 1);
  stream_initAddress(stream);
}


static int stream_initSocket(
  dyad_Stream *stream, int domain, int type, int protocol
) {
  stream->sockfd = socket(domain, type, protocol);
  if (stream->sockfd == INVALID_SOCKET) {
    stream_error(stream, "could not create socket", errno);
    return -1;
  }
  stream_setSocket(stream, stream->sockfd);
  return 0;
}


static int stream_hasListenerForEvent(dyad_Stream *stream, int event) {
  int i;
  for (i = 0; i < stream->listeners.length; i++) {
    Listener *listener = &stream->listeners.data[i];
    if (listener->event == event) {
      return 1;
    }
  }
  return 0;
}


static void stream_handleReceivedData(dyad_Stream *stream) {
  for (;;) {
    /* Receive data */
    dyad_Event e;
    char data[8192];
    int size = recv(stream->sockfd, data, sizeof(data) - 1, 0);
    if (size <= 0) {
      if (size == 0 || errno != EWOULDBLOCK) {
        /* Handle disconnect */
        dyad_close(stream);
        return;
      } else {
        /* No more data */
        return;
      }
    }
    data[size] = 0;
    /* Update status */
    stream->bytesReceived += size;
    stream->lastActivity = dyad_getTime();
    /* Emit data event */
    e = createEvent(DYAD_EVENT_DATA);
    e.msg = "received data";
    e.data = data;
    e.size = size;
    stream_emitEvent(stream, &e);
    /* Check stream state in case it was closed during one of the data event
     * handlers. */
    if (stream->state != DYAD_STATE_CONNECTED) {
      return;
    }

    /* Handle line event */
    if (stream_hasListenerForEvent(stream, DYAD_EVENT_LINE)) {
      int i, start;
      char *buf;
      for (i = 0; i < size; i++) {
        vec_push(&stream->lineBuffer, data[i]);
      }
      start = 0;
      buf = stream->lineBuffer.data;
      for (i = 0; i < stream->lineBuffer.length; i++) {
        if (buf[i] == '\n') {
          dyad_Event e;
          buf[i] = '\0';
          e = createEvent(DYAD_EVENT_LINE);
          e.msg = "received line";
          e.data = &buf[start];
          e.size = i - start;
          /* Check and strip carriage return */
          if (e.size > 0 && e.data[e.size - 1] == '\r') {
            e.data[--e.size] = '\0';
          }
          stream_emitEvent(stream, &e);
          start = i + 1;
          /* Check stream state in case it was closed during one of the line
           * event handlers. */
          if (stream->state != DYAD_STATE_CONNECTED) {
            return;
          }
        }
      }
      if (start == stream->lineBuffer.length) {
        vec_clear(&stream->lineBuffer);
      } else {
        vec_splice(&stream->lineBuffer, 0, start);
      }
    }
  }
}


static void stream_acceptPendingConnections(dyad_Stream *stream) {
  for (;;) {
    dyad_Stream *remote;
    dyad_Event e;
    int err = 0;
    dyad_Socket sockfd = accept(stream->sockfd, NULL, NULL);
    if (sockfd == INVALID_SOCKET) {
      err = errno;
      if (err == EWOULDBLOCK) {
        /* No more waiting sockets */
        return;
      }
    }
    /* Create client stream */
    remote = dyad_newStream();
    remote->state = DYAD_STATE_CONNECTED;
    /* Set stream's socket */
    stream_setSocket(remote, sockfd);
    /* Emit accept event */
    e = createEvent(DYAD_EVENT_ACCEPT);
    e.msg = "accepted connection";
    e.remote = remote;
    stream_emitEvent(stream, &e);
    /* Handle invalid socket -- the stream is still made and the ACCEPT event
     * is still emitted, but its shut immediately with an error */
    if (remote->sockfd == INVALID_SOCKET) {
      stream_error(remote, "failed to create socket on accept", err);
      return;
    }
  }
}


static int stream_flushWriteBuffer(dyad_Stream *stream) {
  stream->flags &= ~DYAD_FLAG_WRITTEN;
  if (stream->writeBuffer.length > 0) {
    /* Send data */
    int size = send(stream->sockfd, stream->writeBuffer.data,
                    stream->writeBuffer.length, 0);
    if (size <= 0) {
      if (errno == EWOULDBLOCK) {
        /* No more data can be written */
        return 0;
      } else {
        /* Handle disconnect */
        dyad_close(stream);
        return 0;
      }
    }
    if (size == stream->writeBuffer.length) {
      vec_clear(&stream->writeBuffer);
    } else {
      vec_splice(&stream->writeBuffer, 0, size);
    }
    /* Update status */
    stream->bytesSent += size;
    stream->lastActivity = dyad_getTime();
  }

  if (stream->writeBuffer.length == 0) {
    dyad_Event e;
    /* If this is a 'closing' stream we can properly close it now */
    if (stream->state == DYAD_STATE_CLOSING) {
      dyad_close(stream);
      return 0;
    }
    /* Set ready flag and emit 'ready for data' event */
    stream->flags |= DYAD_FLAG_READY;
    e = createEvent(DYAD_EVENT_READY);
    e.msg = "stream is ready for more data";
    stream_emitEvent(stream, &e);
  }
  /* Return 1 to indicate that more data can immediately be written to the
   * stream's socket */
  return 1;
}



/*===========================================================================*/
/* API                                                                       */
/*===========================================================================*/

/*---------------------------------------------------------------------------*/
/* Core                                                                      */
/*---------------------------------------------------------------------------*/

void dyad_update(void) {
  dyad_Stream *stream;
  struct timeval tv;

  destroyClosedStreams();
  updateTickTimer();
  updateStreamTimeouts();

  /* Create fd sets for select() */
  select_zero(&dyad_selectSet);

  stream = dyad_streams;
  while (stream) {
    switch (stream->state) {
      case DYAD_STATE_CONNECTED:
        select_add(&dyad_selectSet, SELECT_READ, stream->sockfd);
        if (!(stream->flags & DYAD_FLAG_READY) ||
            stream->writeBuffer.length != 0
        ) {
          select_add(&dyad_selectSet, SELECT_WRITE, stream->sockfd);
        }
        break;
      case DYAD_STATE_CLOSING:
        select_add(&dyad_selectSet, SELECT_WRITE, stream->sockfd);
        break;
      case DYAD_STATE_CONNECTING:
        select_add(&dyad_selectSet, SELECT_WRITE, stream->sockfd);
        select_add(&dyad_selectSet, SELECT_EXCEPT, stream->sockfd);
        break;
      case DYAD_STATE_LISTENING:
        select_add(&dyad_selectSet, SELECT_READ, stream->sockfd);
        break;
    }
    stream = stream->next;
  }

  /* Init timeout value and do select */
  #ifdef _MSC_VER
    #pragma warning(push)
    /* Disable double to long implicit conversion warning,
     * because the type of timeval's fields don't agree across platforms */
    #pragma warning(disable: 4244)
  #endif
  tv.tv_sec = dyad_updateTimeout;
  tv.tv_usec = (dyad_updateTimeout - tv.tv_sec) * 1e6;
  #ifdef _MSC_VER
    #pragma warning(pop)
  #endif

  select(dyad_selectSet.maxfd + 1,
         dyad_selectSet.fds[SELECT_READ],
         dyad_selectSet.fds[SELECT_WRITE],
         dyad_selectSet.fds[SELECT_EXCEPT],
         &tv);

  /* Handle streams */
  stream = dyad_streams;
  while (stream) {
    switch (stream->state) {

      case DYAD_STATE_CONNECTED:
        if (select_has(&dyad_selectSet, SELECT_READ, stream->sockfd)) {
          stream_handleReceivedData(stream);
          if (stream->state == DYAD_STATE_CLOSED) {
            break;
          }
        }
        /* Fall through */

      case DYAD_STATE_CLOSING:
        if (select_has(&dyad_selectSet, SELECT_WRITE, stream->sockfd)) {
          stream_flushWriteBuffer(stream);
        }
        break;

      case DYAD_STATE_CONNECTING:
        if (select_has(&dyad_selectSet, SELECT_WRITE, stream->sockfd)) {
          /* Check socket for error */
          int optval = 0;
          socklen_t optlen = sizeof(optval);
          dyad_Event e;
          getsockopt(stream->sockfd, SOL_SOCKET, SO_ERROR, &optval, &optlen);
          if (optval != 0) goto connectFailed;
          /* Handle succeselful connection */
          stream->state = DYAD_STATE_CONNECTED;
          stream->lastActivity = dyad_getTime();
          stream_initAddress(stream);
          /* Emit connect event */
          e = createEvent(DYAD_EVENT_CONNECT);
          e.msg = "connected to server";
          stream_emitEvent(stream, &e);
        } else if (
          select_has(&dyad_selectSet, SELECT_EXCEPT, stream->sockfd)
        ) {
          /* Handle failed connection */
connectFailed:
          stream_error(stream, "could not connect to server", 0);
        }
        break;

      case DYAD_STATE_LISTENING:
        if (select_has(&dyad_selectSet, SELECT_READ, stream->sockfd)) {
          stream_acceptPendingConnections(stream);
        }
        break;
    }

    /* If data was just now written to the stream we should immediately try to
     * send it */
    if (
      stream->flags & DYAD_FLAG_WRITTEN &&
      stream->state != DYAD_STATE_CLOSED
    ) {
      stream_flushWriteBuffer(stream);
    }

    stream = stream->next;
  }
}


void dyad_init(void) {
#ifdef _WIN32
  WSADATA dat;
  int err = WSAStartup(MAKEWORD(2, 2), &dat);
  if (err != 0) {
    panic("WSAStartup failed (%d)", err);
  }
#else
  /* Stops the SIGPIPE signal being raised when writing to a closed socket */
  signal(SIGPIPE, SIG_IGN);
#endif
}


void dyad_shutdown(void) {
  /* Close and destroy all the streams */
  while (dyad_streams) {
    dyad_close(dyad_streams);
    stream_destroy(dyad_streams);
  }
  /* Clear up everything */
  select_deinit(&dyad_selectSet);
#ifdef _WIN32
  WSACleanup();
#endif
}


const char *dyad_getVersion(void) {
  return DYAD_VERSION;
}


double dyad_getTime(void) {
#ifdef _WIN32
  FILETIME ft;
  GetSystemTimeAsFileTime(&ft);
  return (ft.dwHighDateTime * 4294967296.0 / 1e7) + ft.dwLowDateTime / 1e7;
#else
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec + tv.tv_usec / 1e6;
#endif
}


int dyad_getStreamCount(void) {
  return dyad_streamCount;
}


void dyad_setTickInterval(double seconds) {
  dyad_tickInterval = seconds;
}


void dyad_setUpdateTimeout(double seconds) {
  dyad_updateTimeout = seconds;
}


dyad_PanicCallback dyad_atPanic(dyad_PanicCallback func) {
  dyad_PanicCallback old = panicCallback;
  panicCallback = func;
  return old;
}


/*---------------------------------------------------------------------------*/
/* Stream                                                                    */
/*---------------------------------------------------------------------------*/

dyad_Stream *dyad_newStream(void) {
  dyad_Stream *stream = dyad_realloc(NULL, sizeof(*stream));
  memset(stream, 0, sizeof(*stream));
  stream->state = DYAD_STATE_CLOSED;
  stream->sockfd = INVALID_SOCKET;
  stream->lastActivity = dyad_getTime();
  /* Add to list and increment count */
  stream->next = dyad_streams;
  dyad_streams = stream;
  dyad_streamCount++;
  return stream;
}


void dyad_addListener(
  dyad_Stream *stream, int event, dyad_Callback callback, void *udata
) {
  Listener listener;
  listener.event = event;
  listener.callback = callback;
  listener.udata = udata;
  vec_push(&stream->listeners, listener);
}


void dyad_removeListener(
  dyad_Stream *stream, int event, dyad_Callback callback, void *udata
) {
  int i = stream->listeners.length;
  while (i--) {
    Listener *x = &stream->listeners.data[i];
    if (x->event == event && x->callback == callback && x->udata == udata) {
      vec_splice(&stream->listeners, i, 1);
    }
  }
}


void dyad_removeAllListeners(dyad_Stream *stream, int event) {
  if (event == DYAD_EVENT_NULL) {
    vec_clear(&stream->listeners);
  } else {
    int i = stream->listeners.length;
    while (i--) {
      if (stream->listeners.data[i].event == event) {
        vec_splice(&stream->listeners, i, 1);
      }
    }
  }
}


void dyad_close(dyad_Stream *stream) {
  dyad_Event e;
  if (stream->state == DYAD_STATE_CLOSED) return;
  stream->state = DYAD_STATE_CLOSED;
  /* Close socket */
  if (stream->sockfd != INVALID_SOCKET) {
    close(stream->sockfd);
    stream->sockfd = INVALID_SOCKET;
  }
  /* Emit event */
  e = createEvent(DYAD_EVENT_CLOSE);
  e.msg = "stream closed";
  stream_emitEvent(stream, &e);
  /* Clear buffers */
  vec_clear(&stream->lineBuffer);
  vec_clear(&stream->writeBuffer);
}


void dyad_end(dyad_Stream *stream) {
  if (stream->state == DYAD_STATE_CLOSED) return;
  if (stream->writeBuffer.length > 0) {
    stream->state = DYAD_STATE_CLOSING;
  } else {
    dyad_close(stream);
  }
}


int dyad_listenEx(
  dyad_Stream *stream, const char *host, int port, int backlog
) {
  struct addrinfo hints, *ai = NULL;
  int err, optval;
  char buf[64];
  dyad_Event e;

  /* Get addrinfo */
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags = AI_PASSIVE;
  sprintf(buf, "%d", port);
  err = getaddrinfo(host, buf, &hints, &ai);
  if (err) {
    stream_error(stream, "could not get addrinfo", errno);
    goto fail;
  }
  /* Init socket */
  err = stream_initSocket(stream, ai->ai_family, ai->ai_socktype,
                          ai->ai_protocol);
  if (err) goto fail;
  /* Set SO_REUSEADDR so that the socket can be immediately bound without
   * having to wait for any closed socket on the same port to timeout */
  optval = 1;
  setsockopt(stream->sockfd, SOL_SOCKET, SO_REUSEADDR,
             &optval, sizeof(optval));
  /* Bind and listen */
  err = bind(stream->sockfd, ai->ai_addr, ai->ai_addrlen);
  if (err) {
    stream_error(stream, "could not bind socket", errno);
    goto fail;
  }
  err = listen(stream->sockfd, backlog);
  if (err) {
    stream_error(stream, "socket failed on listen", errno);
    goto fail;
  }
  stream->state = DYAD_STATE_LISTENING;
  stream->port = port;
  stream_initAddress(stream);
  /* Emit listening event */
  e = createEvent(DYAD_EVENT_LISTEN);
  e.msg = "socket is listening";
  stream_emitEvent(stream, &e);
  freeaddrinfo(ai);
  return 0;
  fail:
  if (ai) freeaddrinfo(ai);
  return -1;
}


int dyad_listen(dyad_Stream *stream, int port) {
  return dyad_listenEx(stream, NULL, port, 511);
}


int dyad_connect(dyad_Stream *stream, const char *host, int port) {
  struct addrinfo hints, *ai = NULL;
  int err;
  char buf[64];

  /* Resolve host */
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  sprintf(buf, "%d", port);
  err = getaddrinfo(host, buf, &hints, &ai);
  if (err) {
    stream_error(stream, "could not resolve host", 0);
    goto fail;
  }
  /* Start connecting */
  err = stream_initSocket(stream, ai->ai_family, ai->ai_socktype,
                          ai->ai_protocol);
  if (err) goto fail;
  connect(stream->sockfd, ai->ai_addr, ai->ai_addrlen);
  stream->state = DYAD_STATE_CONNECTING;
  freeaddrinfo(ai);
  return 0;
fail:
  if (ai) freeaddrinfo(ai);
  return -1;
}


void dyad_write(dyad_Stream *stream, const void *data, int size) {
  const char *p = data;
  while (size--) {
    vec_push(&stream->writeBuffer, *p++);
  }
  stream->flags |= DYAD_FLAG_WRITTEN;
}


void dyad_vwritef(dyad_Stream *stream, const char *fmt, va_list args) {
  char buf[512];
  char *str;
  char f[] = "%_";
  FILE *fp;
  int c;
  while (*fmt) {
    if (*fmt == '%') {
      fmt++;
      switch (*fmt) {
        case 'r':
          fp = va_arg(args, FILE*);
          if (fp == NULL) {
            str = "(null)";
            goto writeStr;
          }
          while ((c = fgetc(fp)) != EOF) {
            vec_push(&stream->writeBuffer, c);
          }
          break;
        case 'c':
          vec_push(&stream->writeBuffer, va_arg(args, int));
          break;
        case 's':
          str = va_arg(args, char*);
          if (str == NULL) str = "(null)";
          writeStr:
          while (*str) {
            vec_push(&stream->writeBuffer, *str++);
          }
          break;
        case 'b':
          str = va_arg(args, char*);
          c = va_arg(args, int);
          while (c--) {
            vec_push(&stream->writeBuffer, *str++);
          }
          break;
        default:
          f[1] = *fmt;
          switch (*fmt) {
            case 'f':
            case 'g': sprintf(buf, f, va_arg(args, double));    break;
            case 'd':
            case 'i': sprintf(buf, f, va_arg(args, int));       break;
            case 'x':
            case 'X': sprintf(buf, f, va_arg(args, unsigned));  break;
            case 'p': sprintf(buf, f, va_arg(args, void*));     break;
            default : buf[0] = *fmt; buf[1] = '\0';
          }
          str = buf;
          goto writeStr;
      }
    } else {
      vec_push(&stream->writeBuffer, *fmt);
    }
    fmt++;
  }
  stream->flags |= DYAD_FLAG_WRITTEN;
}


void dyad_writef(dyad_Stream *stream, const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  dyad_vwritef(stream, fmt, args);
  va_end(args);
}


void dyad_setTimeout(dyad_Stream *stream, double seconds) {
  stream->timeout = seconds;
}


void dyad_setNoDelay(dyad_Stream *stream, int opt) {
  opt = !!opt;
  setsockopt(stream->sockfd, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));
}


int dyad_getState(dyad_Stream *stream) {
  return stream->state;
}


const char *dyad_getAddress(dyad_Stream *stream) {
  return stream->address ? stream->address : "";
}


int dyad_getPort(dyad_Stream *stream) {
  return stream->port;
}


int dyad_getBytesSent(dyad_Stream *stream) {
  return stream->bytesSent;
}


int dyad_getBytesReceived(dyad_Stream *stream) {
  return stream->bytesReceived;
}


dyad_Socket dyad_getSocket(dyad_Stream *stream) {
  return stream->sockfd;
}
