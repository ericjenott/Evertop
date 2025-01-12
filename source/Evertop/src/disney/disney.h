#pragma once

#ifndef DISNEY_H
#define DISNEY_H

//#define DISNEY_BUF_MAX 4096
#define DISNEY_BUF_MAX 512
#define DISNEY_AUD_BUF_INT 5
#define LPT_SOUND_TYPE_DISABLED 0
#define LPT_SOUND_TYPE_COVOX 1
#define LPT_SOUND_TYPE_DISNEY 2


void disneyInit();

void setAudioSampleRate(int rate);
int getLptSndType();
void setLptSndType(int sndType);
bool isDisneyBufFull();
bool addByteToDisneyBuf(unsigned char data);
int outputDisneyFrame();


unsigned int getLastAudioDisneyOut();
void setLastDisneyAudioOut(unsigned int ms);

unsigned int getUnreadStatusByteCount();
void setUnreadStatusByteCount(unsigned int count);

unsigned int getCurDisneyBufSize();
void setCurDisneyBufSize(unsigned int bufsize);

unsigned char getDisneyBufByte();
//unsigned char getCurByte();
#endif //ifndef DISNEY_H
