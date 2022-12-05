#include "gtrack.h"
#include <malloc.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

//#define DEBUG_ON

void *gtrack_alloc(unsigned int numElements, unsigned int sizeInBytes)
{
    #ifdef DEBUG_ON
        void * addr = malloc(numElements*sizeInBytes);
        memset(addr, 0x00, numElements*sizeInBytes);
        printf("##gtrack_alloc(%d,%d) -> (%p,%lld)\r\n", numElements, sizeInBytes, addr, (uint64_t)addr);
        return addr;
    #else
        void * addr = malloc(numElements*sizeInBytes);
        memset(addr, 0x00, numElements*sizeInBytes);
        return addr;
    #endif
}

void gtrack_free(void *pFree, unsigned int sizeInBytes)
{
    #ifdef DEBUG_ON
        printf("##gtrack_free(%p,%d)\r\n", pFree, sizeInBytes);
	    free(pFree);
    #else
	    free(pFree);
    #endif
}

void gtrack_log(GTRACK_VERBOSE_TYPE level, const char *format, ...)
{
	va_list args;
    va_start(args, format);
	vprintf(format, args);
	va_end(args);
}
