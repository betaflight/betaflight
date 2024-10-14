/*
 * TODO...
 */
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "platform.h"

#include "build/build_config.h"

#include "common/utils.h"

#include "io/serial.h"
#include "serial_ws.h"

#if !defined (LWS_PLUGIN_STATIC)
#define LWS_DLL
#define LWS_INTERNAL
#include <libwebsockets.h>
#endif

#define BASE_PORT 5760

static const struct serialPortVTable wsVTable;  // Forward
static wsPort_t wsSerialPorts[SERIAL_PORT_COUNT];
static bool wsPortInitialized[SERIAL_PORT_COUNT];

//protocol definition

/* one of these is created for each vhost our protocol is used with */
struct per_vhost_data__minimal {
	struct lws_context *context;
	struct lws_vhost *vhost;
	const struct lws_protocols *protocol;
};

static int
callback_minimal(struct lws *wsi, enum lws_callback_reasons reason,
			void *user, void *in, size_t len)
{
  (void) user;

  const struct lws_protocols * proto = lws_get_protocol(wsi);

  if(!proto){
    return 0;
  }

  if(strcmp(proto->name,"wsSerial") != 0){
    return 0;
  }

  wsPort_t *wsPort = (wsPort_t *)proto->user;

  //fprintf(stderr, "callback on UART%u, %d reason: %d\n", wsPort->id + 1, wsPort->clientCount, reason);

	struct per_vhost_data__minimal *vhd =
			(struct per_vhost_data__minimal *)
			lws_protocol_vh_priv_get(lws_get_vhost(wsi),
					lws_get_protocol(wsi));
	int m;

	switch (reason) {
	case LWS_CALLBACK_PROTOCOL_INIT:
    fprintf(stderr, "LWS_CALLBACK_PROTOCOL_INIT\n");
		vhd = lws_protocol_vh_priv_zalloc(lws_get_vhost(wsi),
				lws_get_protocol(wsi),
				sizeof(struct per_vhost_data__minimal));
		vhd->context = lws_get_context(wsi);
		vhd->protocol = lws_get_protocol(wsi);
		vhd->vhost = lws_get_vhost(wsi);
		break;

	case LWS_CALLBACK_ESTABLISHED:
    fprintf(stderr, "LWS_CALLBACK_ESTABLISHED\n");
    fprintf(stderr, "New connection on UART%u, %d\n", wsPort->id + 1, wsPort->clientCount);
    wsPort->wsi = wsi;
    wsPort->connected = true;
    if (wsPort->clientCount > 0) {
      break;
    }
    wsPort->clientCount++;
    fprintf(stderr, "[NEW]UART%u: %d,%d\n", wsPort->id + 1, wsPort->connected, wsPort->clientCount);
		break;

	case LWS_CALLBACK_CLOSED:
    fprintf(stderr, "LWS_CALLBACK_CLOSED\n");
    wsPort->clientCount--;
    wsPort->wsi = NULL;
    fprintf(stderr, "[CLS]UART%u: %d,%d\n", wsPort->id + 1, wsPort->connected, wsPort->clientCount);
    if (wsPort->clientCount == 0) {
      wsPort->connected = false;
    }
		break;

	case LWS_CALLBACK_SERVER_WRITEABLE:
    //fprintf(stderr, "LWS_CALLBACK_SERVER_WRITEABLE\n");
    if (wsPort->wsi == NULL) {
      fprintf(stderr, "wsPort->wsi == NULL\n");
      break;
    }
    if (wsPort->wsi != wsi){
      fprintf(stderr, "wsPort->wsi != wsi\n");
      break; // different client than last connected?
    }

    if (wsPort->port.txBufferHead < wsPort->port.txBufferTail) {
        // send data till end of buffer
        int chunk = wsPort->port.txBufferSize - wsPort->port.txBufferTail;
        m = lws_write(wsi, (unsigned char *)&wsPort->port.txBuffer[wsPort->port.txBufferTail], chunk, LWS_WRITE_BINARY);
        if (m < 0) {
          lwsl_err("ERROR %d writing to ws\n", m);
          break;
        }
        
        //advance tail by written bytes
        wsPort->port.txBufferTail += m;
        if(chunk == m){
          // if remainder of buffer got written reset tail to start of ring buffer
          wsPort->port.txBufferTail = 0;
        }
        //fprintf(stderr, "Written %d bytes\n", m);
    }
    int chunk = wsPort->port.txBufferHead - wsPort->port.txBufferTail;
    if (chunk){
      m = lws_write(wsi, (unsigned char  *)&wsPort->port.txBuffer[wsPort->port.txBufferTail], chunk, LWS_WRITE_BINARY);
      if (m < 0) {
        lwsl_err("ERROR %d writing to ws\n", m);
        break;
      }
      wsPort->port.txBufferTail += m;
      //fprintf(stderr, "Written %d bytes\n", m);
    }
		break;

	case LWS_CALLBACK_RECEIVE:
    //(stderr, "LWS_CALLBACK_RECEIVE\n");
    if (wsPort->wsi == NULL) {
      fprintf(stderr, "wsPort->wsi == NULL\n");
      break;
    }
    if (wsPort->wsi != wsi){
      fprintf(stderr, "wsPort->wsi != wsi\n");
      break; // different client than last connected?
    }

    uint8_t * data = (uint8_t*)in;
    int size = len;
    while (size--) {
      //printf("%c", *ch);
      //printf("%02X", *ch);
      wsPort->port.rxBuffer[wsPort->port.rxBufferHead] = *(data++);
      if (wsPort->port.rxBufferHead + 1 >= wsPort->port.rxBufferSize) {
          wsPort->port.rxBufferHead = 0;
      } else {
          wsPort->port.rxBufferHead++;
      }
    }
    //fprintf(stderr, "Read %d bytes\n", len);

		break;

	default:
		break;
	}

	return 0;
}

#define LWS_PLUGIN_PROTOCOL_SERIAL \
	{ \
		"wsSerial", \
		callback_minimal, \
		0, \
		512, \
		0, NULL, 512 \
	}

static struct lws_protocols protocols[] = {
	LWS_PLUGIN_PROTOCOL_SERIAL,
	LWS_PROTOCOL_LIST_TERM
};


static wsPort_t *wsReconfigure(wsPort_t *s, int id) {
  if (wsPortInitialized[id]) {
    fprintf(stderr, "port is already initialized!\n");
    return s;
  }

  wsPortInitialized[id] = true;

  s->connected = false;
  s->clientCount = 0;
  s->id = id;

  struct lws_context_creation_info info;
  struct lws_context *context;

  lws_set_log_level(LLL_USER | LLL_ERR | LLL_WARN | LLL_NOTICE, NULL);
  memset(&info, 0, sizeof info); /* otherwise uninitialized garbage */
	info.port = BASE_PORT + id + 1;
	info.mounts = NULL;

  protocols[0].user = (void*)s;

	info.protocols = protocols;
	info.vhost_name = "localhost";
  info.options = LWS_SERVER_OPTION_HTTP_HEADERS_SECURITY_BEST_PRACTICES_ENFORCE;
  context = lws_create_context(&info);
  if (!context) {
		lwsl_err("lws init failed\n");
    fprintf(stderr,
            "init port %u for UART%u failed!!\n",
            (unsigned)BASE_PORT + id + 1,
            (unsigned)id + 1);
	}
  fprintf(stderr,
          "init port %u for UART%u\n",
          (unsigned)BASE_PORT + id + 1,
          (unsigned)id + 1);

  s->context = context;



  return s;
}

serialPort_t *serialWsOpen(int id,
                        serialReceiveCallbackPtr rxCallback,
                        void *rxCallbackData,
                        uint32_t baudRate,
                        portMode_e mode,
                        portOptions_e options) {
    wsPort_t *s = NULL;

#if defined(USE_UART1) || defined(USE_UART2) || defined(USE_UART3) || \
  defined(USE_UART4) || defined(USE_UART5) || defined(USE_UART6) ||   \
  defined(USE_UART7) || defined(USE_UART8)
    if (id >= 0 && id < SERIAL_PORT_COUNT) {
        s = wsReconfigure(&wsSerialPorts[id], id);
    }
#endif
    if (!s) return NULL;

    s->port.vTable = &wsVTable;

    // common serial initialisation code should move to serialPort::init()
    s->port.rxBufferHead = s->port.rxBufferTail = 0;
    s->port.txBufferHead = s->port.txBufferTail = 0;
    s->port.rxBufferSize = RX_BUFFER_SIZE;
    s->port.txBufferSize = TX_BUFFER_SIZE;
    s->port.rxBuffer = s->rxBuffer;
    s->port.txBuffer = s->txBuffer;

    // callback works for IRQ-based RX ONLY
    s->port.rxCallback = rxCallback;
    s->port.rxCallbackData = rxCallbackData;
    s->port.mode = mode;
    s->port.baudRate = baudRate;
    s->port.options = options;

    return (serialPort_t *)s;
}

uint32_t wsTotalRxBytesWaiting(const serialPort_t *instance) {
    wsPort_t *s = (wsPort_t *)instance;
    uint32_t count;

    if (s->port.rxBufferHead >= s->port.rxBufferTail) {
        count = s->port.rxBufferHead - s->port.rxBufferTail;
    } else {
        count =
          s->port.rxBufferSize + s->port.rxBufferHead - s->port.rxBufferTail;
    }

    return count;
}

uint32_t wsTotalTxBytesFree(const serialPort_t *instance) {
    wsPort_t *s = (wsPort_t *)instance;
    uint32_t bytesUsed;

    if (s->port.txBufferHead >= s->port.txBufferTail) {
        bytesUsed = s->port.txBufferHead - s->port.txBufferTail;
    } else {
        bytesUsed =
          s->port.txBufferSize + s->port.txBufferHead - s->port.txBufferTail;
    }
    uint32_t bytesFree = (s->port.txBufferSize - 1) - bytesUsed;

    return bytesFree;
}

bool isWsTransmitBufferEmpty(const serialPort_t *instance) {
    wsPort_t *s = (wsPort_t *)instance;

    bool isEmpty = s->port.txBufferTail == s->port.txBufferHead;

    return isEmpty;
}

uint8_t wsRead(serialPort_t *instance) {
    uint8_t ch;
    wsPort_t *s = (wsPort_t *)instance;

    ch = s->port.rxBuffer[s->port.rxBufferTail];
    if (s->port.rxBufferTail + 1 >= s->port.rxBufferSize) {
        s->port.rxBufferTail = 0;
    } else {
        s->port.rxBufferTail++;
    }

    //fprintf(stderr, "%c", ch);

    return ch;
}

void wsWrite(serialPort_t *instance, uint8_t ch) {
    wsPort_t *s = (wsPort_t *)instance;

    //fprintf(stderr, "%c", ch);

    //TODO: lock/thread sync ?

    s->port.txBuffer[s->port.txBufferHead] = ch;
    if (s->port.txBufferHead + 1 >= s->port.txBufferSize) {
        s->port.txBufferHead = 0;
    } else {
        s->port.txBufferHead++;
    }

}

void wsUpdate(void){
  for(int i = 0; i < SERIAL_PORT_COUNT; i++){
    if(wsPortInitialized[i]){
      wsPort_t* ws = &wsSerialPorts[i];
      int n = lws_service(ws->context, 1000);
      if(n<0){
         fprintf(stderr,
            "timeout port %u for UART%u failed!!\n",
            (unsigned)BASE_PORT + ws->id + 1,
            (unsigned)ws->id + 1);
      }

      /*
      * let everybody know we want to write something on them
      * as soon as they are ready
      */
      int maxLoops = 2;
      while(ws->wsi && !isWsTransmitBufferEmpty((serialPort_t *)&wsSerialPorts[i]) && lws_callback_on_writable(ws->wsi)){
        if(maxLoops <= 0){
          break;
        }
        maxLoops--;
      }
    }
  }
}

static const struct serialPortVTable wsVTable = {
  .serialWrite = wsWrite,
  .serialTotalRxWaiting = wsTotalRxBytesWaiting,
  .serialTotalTxFree = wsTotalTxBytesFree,
  .serialRead = wsRead,
  .serialSetBaudRate = NULL,
  .isSerialTransmitBufferEmpty = isWsTransmitBufferEmpty,
  .setMode = NULL,
  .setCtrlLineStateCb = NULL,
  .setBaudRateCb = NULL,
  .writeBuf = NULL,
  .beginWrite = NULL,
  .endWrite = NULL,
};
