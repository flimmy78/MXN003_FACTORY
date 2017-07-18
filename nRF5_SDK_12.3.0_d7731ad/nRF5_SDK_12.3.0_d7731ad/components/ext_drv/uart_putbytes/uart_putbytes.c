#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>

#include "app_uart.h"

static void U_PutUARTBytes(uint8_t *data,uint16_t len)
{
   uint16_t index;

   for(index = 0; index < len; index++)
			while(app_uart_put(*(data+index)) != NRF_SUCCESS);
}

static void U_PutUARTByte(char * fmt, int size){
	U_PutUARTBytes((uint8_t*)fmt, size);
	U_PutUARTBytes((uint8_t*)"\r\n", 2);
}


void PutUARTBytes(const char *fmt, ...)
{
    static char logCbuf[1024];
    va_list args;
    char *p;
    int n, m;

    memset(logCbuf, 0, 1024);
    p = logCbuf;
    m = 1020;
    va_start(args, fmt);
    n = vsnprintf(p, m, fmt, args);
    va_end(args);

    U_PutUARTByte(logCbuf, n);
}

