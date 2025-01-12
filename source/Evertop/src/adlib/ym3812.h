// taken from new codes
// https://raw.githubusercontent.com/ljbade/wolf4sdl/master/mame/fmopl.cpp
// https://raw.githubusercontent.com/ljbade/wolf4sdl/master/mame/fmopl.h
// http://198.100.149.83/ser/roms1/src/emu/sound/3812intf.c
// old codes
// https://github.com/ValleyBell/libvgm/blob/master/emu/cores/fmopl.c
// https://github.com/ValleyBell/libvgm/blob/master/emu/cores/fmopl.h
// see also 
// https://github.com/xobs/chumby-mame/blob/master/src/sndhrdw/ym3812.cpp 
// https://github.com/bji/libmame/blob/master/old/src/emu/sound/3812intf.c
// modify by MD

#ifndef __FMOPL_H_
#define __FMOPL_H_

// #include "freertos/FreeRTOS.h"
// #include <esp_system.h>
//#include "Arduino.h"

#define HAS_YM3812 1

/* select output bits size of output : 8 or 16 */
//#define OPL_SAMPLE_BITS 16
#define OPL_SAMPLE_BITS 8

//extern uint8_t *adlibMemory;
//#pragma message "declaring adlibMemory"
// uint8_t *adlibMemory;

typedef unsigned char	UINT8;   /* unsigned  8bit */
typedef unsigned short	UINT16;  /* unsigned 16bit */
typedef unsigned int	UINT32;  /* unsigned 32bit */
typedef signed char		INT8;    /* signed  8bit   */
typedef signed short	INT16;   /* signed 16bit   */
typedef signed int		INT32;   /* signed 32bit   */

//typedef int BOOL;
typedef bool BOOL;

#if (OPL_SAMPLE_BITS==16)
typedef INT16 OPLSAMPLE;
#endif
#if (OPL_SAMPLE_BITS==8)
typedef INT8 OPLSAMPLE;
#endif

typedef void(*OPL_TIMERHANDLER)(int channel, double interval_Sec);
typedef void(*OPL_IRQHANDLER)(int param, int irq);
typedef void(*OPL_UPDATEHANDLER)(int param, int min_interval_us);
typedef void(*OPL_PORTHANDLER_W)(int param, unsigned char data);
typedef unsigned char(*OPL_PORTHANDLER_R)(int param);

int  YM3812Init(int num, int clock, int rate);
void YM3812Shutdown(void);
void YM3812ResetChip(int which);
int  YM3812Write(int which, int a, int v);
unsigned char YM3812Read(int which, int a);
void YM3812Mute(int which, int channel, BOOL mute);
int  YM3812TimerOver(int which, int c);
//void YM3812UpdateOne(int which, INT16 *buffer, int length);
void YM3812UpdateOne(int which, OPLSAMPLE *buffer, int length);
unsigned char YM3812GetStatus(int which);

void YM3812SetTimerHandler(int which, OPL_TIMERHANDLER TimerHandler, int channelOffset);
void YM3812SetIRQHandler(int which, OPL_IRQHANDLER IRQHandler, int param);
void YM3812SetUpdateHandler(int which, OPL_UPDATEHANDLER UpdateHandler, int param);

void YM3812setAdlibMemoryLocation(uint8_t *adlibMemoryLocation);

#endif /* __FMOPL_H_ */