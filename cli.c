#include "board.h"
#include "mw.h"

// we unset this on 'exit'
extern uint8_t cliMode;
static void cliExit(char *cmdline);
static void cliHelp(char *cmdline);
static void cliVersion(char *cmdline);

// buffer
char cliBuffer[32];
uint8_t bufferIndex = 0;

typedef struct {
    char *name;
    char *param;
    void (*func)(char *cmdline);
} cliCmd;

// should be sorted a..z for bsearch()
const cliCmd cmdTable[] = {
    { "exit", "", cliExit },
    { "help", "", cliHelp },
    { "version", "", cliVersion },
};
#define CMD_COUNT (sizeof cmdTable / sizeof cmdTable[0])

static void cliPrompt(void)
{
    uartPrint("\r\n# ");
    memset(cliBuffer, 0, sizeof(cliBuffer));
    bufferIndex = 0;
}

static int cliCompare(const void *a, const void *b)
{
    const cliCmd *ca = a, *cb = b;
    return strncasecmp(ca->name, cb->name, strlen(cb->name));
}

static void cliExit(char *cmdline)
{
    uartPrint("Leaving CLI mode...\r\n");
    memset(cliBuffer, 0, sizeof(cliBuffer));
    bufferIndex = 0;
    cliMode = 0;
}

static void cliHelp(char *cmdline)
{
    uint8_t i = 0;

    uartPrint("Available commands:\r\n");    

    for (i = 0; i < CMD_COUNT; i++) {
    	uartPrint((uint8_t *)cmdTable[i].name);
    	uartWrite(' ');
    	uartPrint((uint8_t *)cmdTable[i].param);
    	uartPrint("\r\n");
    }
}

static void cliVersion(char *cmdline)
{
    uartPrint("Afro32 CLI version 2.0-pre1");
}

void cliProcess(void)
{
    while (uartAvailable()) {
        uint8_t c = uartRead();

    	cliBuffer[bufferIndex++] = c;
        if (bufferIndex == sizeof(cliBuffer)) {
            bufferIndex--;
            c = '\n';
        }

    	if (bufferIndex && (c == '\n' || c == '\r')) {
            // enter pressed
            cliCmd *cmd = NULL;
            cliCmd target;
    		uartPrint("\r\n");
            cliBuffer[bufferIndex] = 0; // null terminate
            
            target.name = cliBuffer;
            target.param = NULL;
            
            cmd = bsearch(&target, cmdTable, CMD_COUNT, sizeof cmdTable[0], cliCompare);
            if (cmd)
                cmd->func(cliBuffer + strlen(cmd->name));
            else
                uartPrint("ERR: Unknown command, try 'HELP'");

            // 'exit' will reset this flag, so we don't need to print prompt again
            if (cliMode)
    	        cliPrompt();

        } else if (c == 127) {
            // backspace
            if (bufferIndex > 1) {
        		cliBuffer[bufferIndex - 2] = 0;
        		uartPrint("\r# ");
        		uartPrint((uint8_t *)cliBuffer);
        		uartWrite(' ');
        		uartPrint("\r# ");
        		uartPrint((uint8_t *)cliBuffer);
        		bufferIndex -= 2;
            }
        } else if (c < 32 || c > 126) {
            // non-printable ascii
            bufferIndex--;
        } else {
            uartWrite(c);
        }
    }
}
