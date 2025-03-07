#ifndef  __PUB_H__
#define __PUB_H__

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int   u32;

extern uint8_t DebugOn;
#define DEBUG_CODE    if(DebugOn)

extern long CurrentTime;
extern long MainLoop;
long UpdateCurrentTime(void);

#endif

