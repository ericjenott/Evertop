#include "stdio.h" 
#include "string.h" 
#include "adlib.h" 

#define YM3812_CLOCK 3580000
#define YM3812_NUM_CHIPS 1
#define YM3812_CHIP_ID 0

bool AdlibEnabled = false;
int getAdlibEnabled()
{
  //printf("returning AdlibEnabled = %d\n", AdlibEnabled);
	return AdlibEnabled;
}

void setAdlibEnabled(bool enabled)
{
	AdlibEnabled = enabled;
  printf("set  AdlibEnabled = %d\n", AdlibEnabled);
}

void initAdlib(UINT32 samplingRate) 
{
	YM3812Init(YM3812_NUM_CHIPS, YM3812_CLOCK, samplingRate);
}

void closeAdlib() 
{
	YM3812Shutdown();
}

void resetAdlib() 
{
	YM3812ResetChip(YM3812_CHIP_ID);
}

void writeAdlib(UINT8 address, UINT8 val) 
{
	YM3812Write(YM3812_CHIP_ID, address, val);
}

UINT8 readAdlibReg(UINT8 address) 
{
	return YM3812Read(YM3812_CHIP_ID, address);
}

//void updateAdlibFrame(UINT32 length, INT16 *buffer) {
void updateAdlibFrame(UINT32 length, OPLSAMPLE *buffer) 
{
	YM3812UpdateOne(YM3812_CHIP_ID, buffer, length);  //this is the one that really does everything.
}

UINT8 adlibTimerOverflow(UINT8 timerId)   // timers usually only used for detecting presense of Adlib card, if detectable, shouldn't need to worry about how to use this.
{
	return YM3812TimerOver(YM3812_CHIP_ID, timerId);
}

UINT8 adlibGetStatus()   // returns status byte, of which only 3 bits matter
{
	return YM3812GetStatus(YM3812_CHIP_ID);
}

void setAdlibTimerHandler(OPL_TIMERHANDLER TimerHandler)  // timers usually only used for detecting presense of Adlib card, if detectable, shouldn't need to worry about how to use this.
{
	YM3812SetTimerHandler(YM3812_CHIP_ID, TimerHandler, 0);
}

void setAdlibIRQHandler(OPL_IRQHANDLER IRQHandler)   // Interrupt/IRQ normally never used, maybe don't have to worry about how to use this.
{
	YM3812SetIRQHandler(YM3812_CHIP_ID, IRQHandler, 0);
}

void setAdlibUpdateHandler(OPL_UPDATEHANDLER UpdateHandler)  // this gets set to "UpdateHandler_3812", the code in which is all commented out, so probably don't need to worry about how to use this.
{
	YM3812SetUpdateHandler(YM3812_CHIP_ID, UpdateHandler, 0);
}

void adlibSetMemoryLocation(uint8_t *adlibMemoryLocation)
{
  //adlibMemory = adlibMemoryLocation;
  YM3812setAdlibMemoryLocation(adlibMemoryLocation);
}

