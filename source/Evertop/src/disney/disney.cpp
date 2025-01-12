#include <stdio.h>
#include <stdlib.h>
#include "Arduino.h"
#include "disney.h"

#ifndef min
#define min(a,b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef max
#define max(a,b) (((a) > (b)) ? (a) : (b))
#endif

//unsigned char disneySndBuf[DISNEY_BUF_MAX];
unsigned char *disneySndBuf;
unsigned int disneyBufRear = DISNEY_BUF_MAX - 1;   // Initally assumed that rear is at end
unsigned int disneyBufFront = 0;

unsigned int  lastDisneyAudioOut = 0;
unsigned int unreadStatusByteCount = 0;
unsigned int curDisneyBufSize = 0;

//unsigned char curByte = 0;
unsigned char lastByte = 0;
unsigned int evenNess = 0;
uint8_t moder = 2;

void emptyDisneyBuffer();
bool isDisneyBufEmpty();

int LptSndType = 2;

void disneyInit()
{
  disneySndBuf = (unsigned char *)malloc(DISNEY_BUF_MAX);
}

unsigned int getLastAudioDisneyOut(){return lastDisneyAudioOut;}
void setLastDisneyAudioOut(unsigned int ms){lastDisneyAudioOut = ms;}

unsigned int getUnreadStatusByteCount(){return unreadStatusByteCount;}
void setUnreadStatusByteCount(unsigned int count){unreadStatusByteCount = count;}

unsigned int getCurDisneyBufSize()
{
  //Serial.println("getCurDisneyBufSize()");
  return curDisneyBufSize;
}

void setCurDisneyBufSize(unsigned int bufsize){curDisneyBufSize = bufsize;}

int getLptSndType()
{
	return LptSndType;
}
void setLptSndType(int sndType)
{
	LptSndType = sndType;
}

int AudioSampleRate = 48000;

// Disney Audio Buffer Queue
// https://codeforwin.org/2018/08/queue-implementation-using-array-in-c.html



void setAudioSampleRate(int rate)
{
  AudioSampleRate = rate;
}

bool isDisneyBufFull()
{
	return (curDisneyBufSize == DISNEY_BUF_MAX);
}

void emptyDisneyBuffer()
{
	curDisneyBufSize = 0;
}

bool isDisneyBufEmpty()
{
	return (curDisneyBufSize == 0);
}

// unsigned char getCurByte()
// {
  // return curByte;
// }

bool addByteToDisneyBuf(unsigned char data)
{
  // curByte = data;
	// Queue is full throw Queue out of capacity error.
	if (isDisneyBufFull())
	{
		//Serial.println("Disney buffer is full\n");
    //delay(10);
		return false;
	}

	// Ensure rear never crosses array bounds
	disneyBufRear = (disneyBufRear + 1) % DISNEY_BUF_MAX;

	// Increment queue size
	curDisneyBufSize++;
  //Serial.printf("curDisneyBufSize = %d\n", curDisneyBufSize);

	// Enqueue new element to queue
	disneySndBuf[disneyBufRear] = data;

	// Successfully enqueued element to queue
	return true;
}

unsigned char getDisneyBufByte()
{
	unsigned char data = 0;
	// Queue is empty, throw Queue underflow error
	if (isDisneyBufEmpty())
	{
		//printf("Disney buffer is empty\n");
    // if queue is empty, give the last byte we had
    data = lastByte;
	}
	else 
  {
    if (evenNess % 2 == 0) // only give next byte every 2 reads (sounds like it should be about 4.5), to try to upsample to FabGL's SoundGen's higher sampling rate
    {
      // Dequeue element from queue
      data = disneySndBuf[disneyBufFront];

      // Ensure front never crosses array bounds
      disneyBufFront = (disneyBufFront + 1) % DISNEY_BUF_MAX;

      // Decrease queue size
      curDisneyBufSize--;
      lastByte = data;
    }
    else
    {
      // if this is not the 2nd of every 2 reads, give a copy of the last byte
      data = lastByte;   
    }
	}
  evenNess++;
	return data;
}

// int outputDisneyFrame() 
// {
	// unsigned int currentTime = millis(); // in ms since Windows starts
	// unsigned int timeLapsed = currentTime - lastDisneyAudioOut;

	// if (timeLapsed > DISNEY_AUD_BUF_INT && curDisneyBufSize > 0)
	// {
		// if (curDisneyBufSize > 2 * DISNEY_BUF_MAX / 3)
		// {
			// printf("Skipping previous audio frames due to huge buffer size: %d\n", curDisneyBufSize);
			// emptyDisneyBuffer();
		// }
		// else 
    // {
			// unsigned int audBufSize = 0;
			// double samplingRate = 0;
			// double multiplier = 0;
			// unsigned int bytesToPlay = 0;

			// if (getLptSndType() == LPT_SOUND_TYPE_DISNEY)
			// {
				// // Disney sound source has an internal buffer
				// // and always plays audio at a fixed rate of 7kHz
				// samplingRate = 7000;
				// bytesToPlay = min(timeLapsed * samplingRate / 1000, curDisneyBufSize);
			// }
			// else 
      // {
				// // For Covox Speech Thing, we output whatever in audio buffer 
				// // Try to calculate the raw sampling rate of the original audio (e.g. 8kHz) and upscale the samples
				// // to match the sound card sampling rate (e.g. 44.1kHz)
				// samplingRate = (double)curDisneyBufSize * 1000 / (double)timeLapsed;

				// // for small set of samples, estimated sampling rate is not accurate. We assume lowest
				// samplingRate = max(samplingRate, 8000);				

				// bytesToPlay = curDisneyBufSize;
			// }

			// // samplingRate = samplingRate * 1.1;

			// multiplier = (double)AudioSampleRate / (double)samplingRate;			
			// audBufSize = multiplier * bytesToPlay;

			// // crude proportional up-sample algorithm
			// // not accurate but it's ok since the original Covox does not have very good audio quality
			// // better option: http://www.mega-nerd.com/SRC/ 
			// int8_t * orgAudioBuf = (int8_t*)malloc(curDisneyBufSize);
			// int8_t* newAudioBuf = (int8_t*)malloc(audBufSize);
			// unsigned int start = 0;
			// while (!isDisneyBufEmpty())
			// {
				// unsigned char bufOut = getDisneyBufByte();
				// orgAudioBuf[start] = bufOut;
				// start++;
			// }

			// for (unsigned int i = 0; i < audBufSize; i++)
			// {
				// unsigned int oldInd = (unsigned int)(i / multiplier);
				// newAudioBuf[i] = orgAudioBuf[oldInd];
			// }

			// printf("lapsed: %d. audBufSize: %d, rawRate: %.0fHz, multiplier: %.4f, remainBytes: %d\n", timeLapsed, audBufSize, samplingRate, multiplier, curDisneyBufSize);

			// //WaveOutDisney->Write((PBYTE)newAudioBuf, audBufSize);
			// free(orgAudioBuf);
			// free(newAudioBuf);
      // //return sample;
		// }
		// lastDisneyAudioOut = currentTime;
    // return 127;
	// }
// }