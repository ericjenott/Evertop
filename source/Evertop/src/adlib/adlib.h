#pragma once

#include "Arduino.h"

#include "ym3812.h"

//#include "esp32_sound_cfg.h"

#define OPL_TIMER_1 0
#define OPL_TIMER_2 1

#define TIMER1_DEFAULT_INTERVAL_MS 21
#define TIMER2_DEFAULT_INTERVAL_MS 82

#define LPT_SOUND_TYPE_DISABLED 0
#define LPT_SOUND_TYPE_COVOX 1
#define LPT_SOUND_TYPE_DISNEY 2

//extern uint8_t *adlibMemory;
//uint8_t *adlibMemory;

void initAdlib(UINT32 samplingRate);
void closeAdlib();
void resetAdlib();
void writeAdlib(UINT8 address, UINT8 val);
UINT8 readAdlibReg(UINT8 address);
//void updateAdlibFrame(UINT32 length, INT16 *buffer);
void updateAdlibFrame(UINT32 length, OPLSAMPLE *buffer);
UINT8 adlibTimerOverflow(UINT8 timerId);
UINT8 adlibGetStatus();
void setAdlibTimerHandler(OPL_TIMERHANDLER TimerHandler);
void setAdlibIRQHandler(OPL_IRQHANDLER IRQHandler);
void setAdlibUpdateHandler(OPL_UPDATEHANDLER UpdateHandler);
int getAdlibEnabled();
void setAdlibEnabled(bool enabled);
void adlibSetMemoryLocation(uint8_t *adlibMem);