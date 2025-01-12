/*
  Created by Fabrizio Di Vittorio (fdivitto2013@gmail.com) - <http://www.fabgl.com>
  Copyright (c) 2019-2022 Fabrizio Di Vittorio.
  All rights reserved.


* Please contact fdivitto2013@gmail.com if you need a commercial license.


* This library and related software is available under GPL v3.

  FabGL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  FabGL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with FabGL.  If not, see <http://www.gnu.org/licenses/>.
 */




#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#include <string.h>
#include <ctype.h>
#include <math.h>

#include "driver/dac.h"
#include "soc/i2s_reg.h"
#include "driver/periph_ctrl.h"
#include "soc/rtc.h"
#include <soc/sens_reg.h>
#include "esp_log.h"
#include "driver/sigmadelta.h"


#include "soundgen.h"
#include "../../../adlib/adlib.h"
#include "../../../adlib/ym3812.h"

#include "../../../midi/ESP32Synth.h"

#include "../../../disney/disney.h"

#pragma GCC optimize ("O2")

#define minimum(a,b) ((a) < (b) ? (a) : (b))

namespace fabgl {



////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SineWaveformGenerator


static const int8_t sinTable[257] = {
   0,    3,    6,    9,   12,   16,   19,   22,   25,   28,   31,   34,   37,   40,   43,   46,
  49,   51,   54,   57,   60,   63,   65,   68,   71,   73,   76,   78,   81,   83,   85,   88,
  90,   92,   94,   96,   98,  100,  102,  104,  106,  107,  109,  111,  112,  113,  115,  116,
 117,  118,  120,  121,  122,  122,  123,  124,  125,  125,  126,  126,  126,  127,  127,  127,
 127,  127,  127,  127,  126,  126,  126,  125,  125,  124,  123,  122,  122,  121,  120,  118,
 117,  116,  115,  113,  112,  111,  109,  107,  106,  104,  102,  100,   98,   96,   94,   92,
  90,   88,   85,   83,   81,   78,   76,   73,   71,   68,   65,   63,   60,   57,   54,   51,
  49,   46,   43,   40,   37,   34,   31,   28,   25,   22,   19,   16,   12,    9,    6,    3,
   0,   -3,   -6,   -9,  -12,  -16,  -19,  -22,  -25,  -28,  -31,  -34,  -37,  -40,  -43,  -46,
 -49,  -51,  -54,  -57,  -60,  -63,  -65,  -68,  -71,  -73,  -76,  -78,  -81,  -83,  -85,  -88,
 -90,  -92,  -94,  -96,  -98, -100, -102, -104, -106, -107, -109, -111, -112, -113, -115, -116,
-117, -118, -120, -121, -122, -122, -123, -124, -125, -125, -126, -126, -126, -127, -127, -127,
-127, -127, -127, -127, -126, -126, -126, -125, -125, -124, -123, -122, -122, -121, -120, -118,
-117, -116, -115, -113, -112, -111, -109, -107, -106, -104, -102, -100,  -98,  -96,  -94,  -92,
 -90,  -88,  -85,  -83,  -81,  -78,  -76,  -73,  -71,  -68,  -65,  -63,  -60,  -57,  -54,  -51,
 -49,  -46,  -43,  -40,  -37,  -34,  -31,  -28,  -25,  -22,  -19,  -16,  -12,   -9,   -6,   -3,
   0,
};


SineWaveformGenerator::SineWaveformGenerator()
 : m_phaseInc(0),
   m_phaseAcc(0),
   m_frequency(0),
   m_lastSample(0)
{
}


void SineWaveformGenerator::setFrequency(int value) {
  if (m_frequency != value) {
    m_frequency = value;
    m_phaseInc = (((uint32_t)m_frequency * 256) << 11) / sampleRate();
  }
}


int SineWaveformGenerator::getSample() 
{
  if (m_frequency == 0 || duration() == 0) 
  {
    if (m_lastSample > 0)
    {
      --m_lastSample;
    }
    else if (m_lastSample < 0)
    {
      ++m_lastSample;
    }
    else
    {
      m_phaseAcc = 0;
    }
    return m_lastSample;
  }

  // get sample  (-128...+127)
  uint32_t index = m_phaseAcc >> 11;
  int sample = sinTable[index];

  // process volume
  sample = sample * volume() / 127;

  m_lastSample = sample;

  m_phaseAcc = (m_phaseAcc + m_phaseInc) & 0x7ffff;

  decDuration();

  return sample;
}


// SineWaveformGenerator
////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SquareWaveformGenerator


SquareWaveformGenerator::SquareWaveformGenerator()
  : m_phaseInc(0),
    m_phaseAcc(0),
    m_frequency(0),
    m_lastSample(0),
    m_dutyCycle(127)
{
}


void SquareWaveformGenerator::setFrequency(int value) {
  if (m_frequency != value) {
    m_frequency = value;
    m_phaseInc = (((uint32_t)m_frequency * 256) << 11) / sampleRate();
  }
}


// dutyCycle: 0..255 (255=100%)
void SquareWaveformGenerator::setDutyCycle(int dutyCycle)
{
  m_dutyCycle = dutyCycle;
}


int SquareWaveformGenerator::getSample() 
{
  if (m_frequency == 0 || duration() == 0) 
  {
    if (m_lastSample > 0)
    {
      --m_lastSample;
    }
    else if (m_lastSample < 0)
    {
      ++m_lastSample;
    }
    else
    {
      m_phaseAcc = 0;
    }
    return m_lastSample;
  }

  uint32_t index = m_phaseAcc >> 11;
  int sample = (index <= m_dutyCycle ? 127 : -127);

  // process volume
  sample = sample * volume() / 127;

  m_lastSample = sample;

  m_phaseAcc = (m_phaseAcc + m_phaseInc) & 0x7ffff;

  decDuration();

  return sample;
}

// SquareWaveformGenerator
////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TriangleWaveformGenerator


TriangleWaveformGenerator::TriangleWaveformGenerator()
  : m_phaseInc(0),
    m_phaseAcc(0),
    m_frequency(0),
    m_lastSample(0)
{
}


void TriangleWaveformGenerator::setFrequency(int value) {
  if (m_frequency != value) {
    m_frequency = value;
    m_phaseInc = (((uint32_t)m_frequency * 256) << 11) / sampleRate();
  }
}


int TriangleWaveformGenerator::getSample() {
  if (m_frequency == 0 || duration() == 0) {
    if (m_lastSample > 0)
      --m_lastSample;
    else if (m_lastSample < 0)
      ++m_lastSample;
    else
      m_phaseAcc = 0;
    return m_lastSample;
  }

  uint32_t index = m_phaseAcc >> 11;
  int sample = (index & 0x80 ? -1 : 1) * ((index & 0x3F) * 2 - (index & 0x40 ? 0 : 127));

  // process volume
  sample = sample * volume() / 127;

  m_lastSample = sample;

  m_phaseAcc = (m_phaseAcc + m_phaseInc) & 0x7ffff;

  decDuration();

  return sample;
}

// TriangleWaveformGenerator
////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SawtoothWaveformGenerator


SawtoothWaveformGenerator::SawtoothWaveformGenerator()
  : m_phaseInc(0),
    m_phaseAcc(0),
    m_frequency(0),
    m_lastSample(0)
{
}


void SawtoothWaveformGenerator::setFrequency(int value) 
{
  if (m_frequency != value) 
  {
    m_frequency = value;
    m_phaseInc = (((uint32_t)m_frequency * 256) << 11) / sampleRate();
  }
}


int SawtoothWaveformGenerator::getSample() 
{
  if (m_frequency == 0 || duration() == 0) 
  {
    if (m_lastSample > 0)
      --m_lastSample;
    else if (m_lastSample < 0)
      ++m_lastSample;
    else
      m_phaseAcc = 0;
    return m_lastSample;
  }

  uint32_t index = m_phaseAcc >> 11;
  int sample = index - 128;

  // process volume
  sample = sample * volume() / 127;

  m_lastSample = sample;

  m_phaseAcc = (m_phaseAcc + m_phaseInc) & 0x7ffff;

  decDuration();

  return sample;
}

// TriangleWaveformGenerator
////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////
// NoiseWaveformGenerator


NoiseWaveformGenerator::NoiseWaveformGenerator()
  : m_noise(0xFAB7)
{
}


void NoiseWaveformGenerator::setFrequency(int value)
{
}


int NoiseWaveformGenerator::getSample()
{
  if (duration() == 0) {
    return 0;
  }

  // noise generator based on Galois LFSR
  m_noise = (m_noise >> 1) ^ (-(m_noise & 1) & 0xB400u);
  int sample = 127 - (m_noise >> 8);

  // process volume
  sample = sample * volume() / 127;

  decDuration();

  return sample;
}


// NoiseWaveformGenerator
////////////////////////////////////////////////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////////////////////////////////
// VICNoiseGenerator
// "tries" to emulate VIC6561 noise generator
// derived from a reverse engineering VHDL code: http://sleepingelephant.com/ipw-web/bulletin/bb/viewtopic.php?f=11&t=8733&fbclid=IwAR16x6OMMb670bA2ZtYcbB0Zat_X-oHNB0NxxYigPffXa8G_InSIoNAEiPU


VICNoiseGenerator::VICNoiseGenerator()
  : m_frequency(0),
    m_counter(0),
    m_LFSR(LFSRINIT),
    m_outSR(0)
{
}


void VICNoiseGenerator::setFrequency(int value)
{
  if (m_frequency != value) {
    m_frequency = value >= 127 ? 0 : value;
    m_LFSR      = LFSRINIT;
    m_counter   = 0;
    m_outSR     = 0;
  }
}


int VICNoiseGenerator::getSample()
{
  if (duration() == 0) {
    return 0;
  }

  const int reduc = CLK / 8 / sampleRate(); // resample to sampleRate() (ie 16000Hz)

  int sample = 0;

  for (int i = 0; i < reduc; ++i) {

    if (m_counter >= 127) {

      // reset counter
      m_counter = m_frequency;

      if (m_LFSR & 1)
        m_outSR = ((m_outSR << 1) | ~(m_outSR >> 7));

      m_LFSR <<= 1;
      int bit3  = (m_LFSR >> 3) & 1;
      int bit12 = (m_LFSR >> 12) & 1;
      int bit14 = (m_LFSR >> 14) & 1;
      int bit15 = (m_LFSR >> 15) & 1;
      m_LFSR |= (bit3 ^ bit12) ^ (bit14 ^ bit15);
    } else
      ++m_counter;

    sample += m_outSR & 1 ? 127 : -128;
  }

  // simple mean of all samples

  sample = sample / reduc;

  // process volume
  sample = sample * volume() / 127;

  decDuration();

  return sample;
}


// VICNoiseGenerator
/////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AdlibSoundGenerator


//AdlibSoundGenerator::AdlibSoundGenerator(int8_t const * data, int length)
AdlibSoundGenerator::AdlibSoundGenerator()
{
}

static DWORD m_timer1expiry;
static DWORD m_timer2expiry;
static double m_eps;
static bool m_shouldSendIRQ;

void AdlibSoundGenerator::init()
{
  //// in milliseconds since Windows starts
  //// 0 means timer not running
  // m_timer1expiry = 0;
  // m_timer2expiry = 0;
  // m_eps = 1e-05;
  // m_shouldSendIRQ = false;
  m_lastAdlibReadVal = 0;
  m_lastAdlibAudioOut = 0;
  m_Int8Pending = 0; // for timer software interrupt
  
  // maybe the best combination I've found yet is m_AudioSampleRate = 32768, FABGL_SOUNDGEN_DEFAULT_SAMPLE_RATE 16384
  m_AudioSampleRate = 16000; // original setting from windows version of 8086tiny
  //m_AudioSampleRate = 16384;
  //m_AudioSampleRate = 8192;
  //m_AudioSampleRate = 32768;
  //m_AudioSampleRate = 24000;
  //m_AudioSampleRate = 44100;
  //m_AudioSampleRate = 32000;
  //m_AudioSampleRate = 12288;
  //m_AudioSampleRate = 65535;
  
 setAdlibEnabled(true);  //merely sets adlibEnabled = true
 initAdlib(m_AudioSampleRate); // creates a pointer to an FM_OPL struct
 setAdlibTimerHandler(TimerHandler_3812);   //timer stuff, unneeded, ignore for now
 setAdlibIRQHandler(IRQHandler_3812);     //IRQHandler_3812 is an empty, so probably abandoned, function.  Ignore.
 setAdlibUpdateHandler(UpdateHandler_3812);    // UpdateHandler_3812 is an empty function, so probably abandoned, function.  Ignore.
 m_lastAdlibAudioOut = millis();  // set to zero instead? milliseconds elapsed since last time we output audio sample to the speaker circuitry.
}

UINT8 AdlibSoundGenerator::getAdlibStatus()
{
  UINT8 astat = adlibGetStatus();
  return astat;
}

void AdlibSoundGenerator::writeAdlibRegister(UINT8 address, UINT8 val) 
{
  //printf("Writing adlib %d, %d\n", address, val);
  writeAdlib(address, val);
}

void AdlibSoundGenerator::updateAdlibTimer()
{
 // according to cosmodoc.org, these two timers are never or almost never used by any PC applications, except in detecting the presence of an Adlib card.  So I probably don't need to change anything here.
 DWORD currentTime = millis(); // in ms since esp32 starts

 if (m_timer1expiry > 0 && m_timer1expiry <= currentTime)
 {
   // printf("Current time is %d - Timer 1 overflow\n", currentTime);
   adlibTimerOverflow(OPL_TIMER_1);

   if (m_shouldSendIRQ)
   {
     m_Int8Pending++;
     //printf("m_Int8Pending(timer1) = %d\n", m_Int8Pending);
   }
 }

 if (m_timer2expiry > 0 && m_timer2expiry <= currentTime)
 {
   // printf("Current time is %d - Timer 2 overflow\n", currentTime);
   adlibTimerOverflow(OPL_TIMER_2);

   if (m_shouldSendIRQ)
   {
     m_Int8Pending++;
     //printf("m_Int8Pending(timer2) = %d\n", m_Int8Pending);
   }
 }
}

void AdlibSoundGenerator::TimerHandler_3812(int channel, double interval_Sec) 
//void TimerHandler_3812(int channel, double interval_Sec) 
{
 DWORD currentTime = millis();
 //printf("TimerHandler_3812 channel=%d interval_Sec=%.20f currentTime=%d\n", channel, interval_Sec, currentTime);

 if (channel == 0)
 {
   if (abs(interval_Sec) > m_eps)
   {
     m_timer1expiry = currentTime + interval_Sec * 1000;
   }
   else
   {
     m_timer1expiry = 0;
   }
 }
 else
 {
   if (abs(interval_Sec) > m_eps)
   {
     m_timer2expiry = currentTime + interval_Sec * 1000;
   }
   else
   {
     m_timer2expiry = 0;
   }
 }
}

void AdlibSoundGenerator::IRQHandler_3812(int param, int irq) 
//void IRQHandler_3812(int param, int irq) 
{
 //printf("IRQHandler_3812 param=%d irq=%d\n", param, irq);
 m_shouldSendIRQ = irq; // 0 to disable IRQ, 1 to enable IRQ
}

void AdlibSoundGenerator::UpdateHandler_3812(int param, int min_interval_us) 
//void UpdateHandler_3812(int param, int min_interval_us) 
{
 //printf("UpdateHandler_3812 param=%d min_interval_us=%d\n", param, min_interval_us);
 // updateAdlibFrame(0, buffer);  
}

int AdlibSoundGenerator::isAdlibEnabled()
{
  return getAdlibEnabled();
}

void AdlibSoundGenerator::setFrequency(int value)
{
}

void AdlibSoundGenerator::setAdlibMemory(uint8_t *adlibMemoryLocation)
{
  m_AdlibMemory = adlibMemoryLocation;
  adlibSetMemoryLocation(m_AdlibMemory);
}

// void AdlibSoundGenerator::getAdlibMemory(int value)
// {
// }

// updateAdlibTimer
// getAdlibSamples
// TimerHandler_3812
// IRQHandler_3812
// UpdateHandler_3812
//  setAdlibEnabled(true);  //merely sets adlibEnabled = true
//  initAdlib(AudioSampleRate); // creates a pointer to an FM_OPL struct
//  setAdlibTimerHandler(TimerHandler_3812);   //timer stuff, unneeded, ignore for now
//  setAdlibIRQHandler(IRQHandler_3812);     //IRQHandler_3812 is an empty, so probably abandoned, function.  Ignore.
//  setAdlibUpdateHandler(UpdateHandler_3812);    // UpdateHandler_3812 is an empty, so probably abandoned, function.  Ignore.
//  lastAdlibAudioOut = millis();  // milliseconds elapsed since last time we output audio sample to the speaker circuitry.

//OPLSAMPLE adlibOutputBuf[50 * (48000 / 1000) * 2];  //50 * (48000 / 1000)  = 2400, maximum value of numOfSamples below
//int8_t outbuf2[50 * (48000 / 1000) * 2];
// void AdlibSoundGenerator::outputAdlibFrame(void *arg) 
// {
  // DWORD currentTime = millis(); // in ms since Windows starts
  // unsigned int timeLapsed = currentTime - m_lastAdlibAudioOut;
  // static uint32_t count = 0;
 
  // if (timeLapsed > 0)
  // {
    // // generate only the most recent samples, otherwise if 8088 execution pauses, for example because of an opened menu, 
    // // the time lapsed will be huge causing the number of sample to grow very large and cWaveOut->Write to take a long time to write
    // // the effect is cascading due to the way the adlib timer is implemented (based on system clock, not CPU clock)
    // // the best way to avoid this is to implement the Adlib timer based on CPU clock, which will cause static in the audio output
    // // since there are gaps in the emulated CPU running sequence (for the code to process other stuff).
    // // This can be observed in the existing PC speaker emulation code by playing a continuous sound - there will be gaps in the tone
    // // However, because PC speaker quality is already poor, the result is still acceptable, which is definitely not the case with Adlib emulation.
    // // The value chosen should be much larger than the number of actual samples generated between two successive 8088 timer ticks
    // // otherwise the audio will still be choppy   
    // timeLapsed = minimum(timeLapsed, 50); // play only the last few ms to prevent buffer overflow


    // unsigned int numOfSamples = timeLapsed * m_AudioSampleRate / 1000;
    // // m_lastAdlibAudioOut = currentTime;

    // // printf("Samples needed: %d.\n", numOfSamples);

    // //INT16* buf = (INT16*)malloc(numOfSamples * 2);   // need to get rid of this malloc
    // //OPLSAMPLE* adlibOutputBuf = (OPLSAMPLE*)malloc(numOfSamples * 2);   // need to get rid of this malloc
    // //int8_t* outbuf2 = (int8_t *)malloc(numOfSamples);
    // // OPLSAMPLE adlibOutputBuf[50 * (48000 / 1000) * 2];  //50 * (48000 / 1000)  = 2400, maximum value of numOfSamples below
    // // int8_t outbuf2[50 * (48000 / 1000) * 2];
    // updateAdlibFrame(numOfSamples, adlibOutputBuf); 
    // auto soundGenerator = (SoundGenerator *) arg;
    // for (int i = 0; i < numOfSamples; ++i)
    // {
      // outbuf2[i] = (int8_t)((adlibOutputBuf[i] >> 7) & 0x00ff);   
    // }
    // soundGenerator->playSamples(outbuf2, numOfSamples);
    // m_lastAdlibAudioOut = currentTime;
    // //uint32_t samplesSum = 0;
    // //bool hasSound = false;
    // //bool printed = false;
    
    // if(count % 1600 == 0)  //dump buffer to serial port every 20000 cycles, for debugging
    // {
      // for (int i = 0; i < numOfSamples; ++i)
      // {
        // {
           // printf("%03d ", outbuf2[i]);
        // }
      // }
      // printf("\n");   
    // }
    // count++;
    
    
    
    // // for (int n = 0; n < numOfSamples; n++)
    // // {
     // // samplesSum += buf[n];
     // // buf2[n] = (uint8_t)((buf[n] >> 8)); // + 128);
     // // if (buf2[n] != 0)
     // // {
       // // hasSound = true;
     // // }
     // // //printf("%04x ", buf[n]);
    // // }
    // //printf("\nsamplesSum = %d\n\n", samplesSum);
    // //WaveOutAdlib->Write((PBYTE)buf, numOfSamples * 2); // I think this is where the sample needs to be played through the physical system
    // //https://www.instructables.com/ESP-32-Based-Audio-Player/ see how to output wav file from esp32 to GPIO25
    // //size_t bytes_written;
    // //int BYTES_TO_WRITE = numOfSamples * 2;
    // //int BYTES_TO_WRITE = numOfSamples;

    // //if (samplesSum > 0)
    // // if (hasSound == true)
    // // {
     // // hasSound = false;
     // // printf("timeLapsed = %d\n", timeLapsed);
     // // for (int n = 0; n < numOfSamples; n++)
     // // {
       // // printf("%02hhx ", buf[n]);
     // // }
     // // printf("\nBTW=%d, samplesSum = %d, sizeof(OPLSAMPLE) = %d\n", BYTES_TO_WRITE, samplesSum, sizeof(OPLSAMPLE));
     // // //i2s_write( I2S_NUM_0, buf2, BYTES_TO_WRITE, &bytes_written, portMAX_DELAY);
    // // }

    // //free(adlibOutputBuf);
    // //free(outbuf2);
  // }
// }

int AdlibSoundGenerator::getSample() 
{
  // here I need to change the code to call outputAdlibFrame() to get the sample bytes
  //Serial.printf("duration = %d\n", duration());
  
  //updateAdlibTimer();
  
  if (duration() == 0) 
  {
    return 0;
  }

  // int sample = m_data[m_index++];
  

  // if (m_index == m_length)
  // {
    // m_index = 0;
  // }
  //int sample;
  
  
  
  OPLSAMPLE sample;
  updateAdlibFrame(1, &sample);  //need to implement this!

  decDuration();
#if (OPL_SAMPLE_BITS==16)  
  int8_t sample2 = (uint8_t) ((sample >> 8) & 0xff);
  sample2 = sample2 * volume() / 127;
  return sample2;
#else
  sample = sample * volume() / 127;
  return sample;
#endif // (OPL_SAMPLE_BITS==16)   


  // int sampleint = sample;  // leave only least significant byte, set all higher bytes to 0
  // sampleint &= 0xff;
  
  //uint8_t sample2 = sample + 128;
  //int sampleint = (int)sample2;  // leave only least significant byte, set all higher bytes to 0
  //sampleint &= 0xff;
  //sample = sample * volume() / 127;
  //sampleint = sampleint * volume() / 127;

  // decDuration();
  //int8_t sample2 = (int8_t)(sample >> 8);
  //return (int)sample2;
  //return sample2;
  
  //return sampleint;
}


// AdlibSoundGenerator
////////////////////////////////////////////////////////////////////////////////////////////////////////////





////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MidiSoundGenerator

MidiSoundGenerator::MidiSoundGenerator()
{
}

int midiport = 0;
void MidiSoundGenerator::init()
{
	if (midiport < 0)
	{
		printf("MIDI is disabled\r\n");
		return;
	}

  printf("MidiSoundGenerator::init()\n");
  setupMidiOutput();
}

// select which MIDI output port to open. 0 to use default
int MidiSoundGenerator::getMidiPort()
{
  //printf("getMidiPort returning midiport = %d\n", midiport);
	return midiport;
}

//void MidiSoundGenerator::sendMidiMsg(unsigned long word) 
void MidiSoundGenerator::sendMidiMsg(unsigned char *data) 
{
  //return;
	if (midiport < 0)
	{
		printf("MIDI is disabled\r\n");
		return;
	}
  //printf("midiMessage: %02x %02x %02x %02x\n", midiMessage.data[0], midiMessage.data[1], midiMessage.data[2], midiMessage.data[3]);
  for (int n = 0; n < 4; n++)
  {
    //printf("0x%02x ", data[n]);
    writeMidiByte(data[n]);
  }
  //printf("\n");
}

void MidiSoundGenerator::sendMidiLongMsg(unsigned char bytes[], int length) 
{
	if (midiport < 0)
	{
		printf("MIDI is disabled\r\n");
		return;
	}
  printf("sendMidiLongMsg() len=%d\n", length);
  for (int n = 0; n <= length; n++)
  {
    printf("%02x ", bytes[n]);
    writeMidiByte(bytes[n]);
  }
  printf("\n");
}

unsigned char MidiSoundGenerator::getExpectedParamCount(unsigned char midiCmd) 
{
	// get the expected parameter count for a MIDI command byte
	// return -1 if midiCmd is not a valid command byte
	// see http://fmslogo.sourceforge.net/manual/midi-table.html for details
	if (midiCmd < 0b10000000)
	{
		return 0; // MIDI data byte, invalid param
	}
	else if (midiCmd < 192) {
		return 2;
	}
	else if (midiCmd < 224) {
		return 1;
	}
	else if (midiCmd < 240) {
		return 2;
	}
	else if (midiCmd == 241) {
		return 0; 	// SysEx message handled separately, not included here
	}
	else if (midiCmd == 242) {
		return 2;
	}
	else if (midiCmd == 243) {
		return 1;
	}
	else { // >=244
		return 0;
	}
}

void MidiSoundGenerator::setFrequency(int value)
{
}

//int8_t sample = -127;
int MidiSoundGenerator::getSample() 
{  
  //printf("MidiSoundGenerator::getSample()\n");
  if (duration() == 0) 
  {
    return 0;
  }

  int8_t sample = synthInterrupt();
  decDuration();

  sample = sample * volume() / 127;
  return sample;
}

// MidiSoundGenerator
////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DisneySoundGenerator

DisneySoundGenerator::DisneySoundGenerator()
{
}

void DisneySoundGenerator::init()
{
	// if (midiport < 0)
	// {
		// printf("MIDI is disabled\r\n");
		// return;
	// }

  printf("DisneySoundGenerator::init()\n");



  setLastDisneyAudioOut(0);
  setUnreadStatusByteCount(0);
  setCurDisneyBufSize(0);
  /* 
    unsigned int getLastDisneyOut();
    void setLastDisneyOut(unsigned int ms)
    unsigned int getUnreadByteCount();
    void setUnreadByteCount(unsigned int count);
    unsigned int getDisneyBufSize();
    void setDisneyBufSize(unsigned int bufsize);  
  */
  setAudioSampleRate(48000);
  disneyInit();
}

unsigned int DisneySoundGenerator::getLastDisneyOut(){return getLastAudioDisneyOut();}
void DisneySoundGenerator::setLastDisneyOut(unsigned int ms){setLastDisneyAudioOut(ms);}
unsigned int DisneySoundGenerator::getUnreadByteCount(){return getUnreadStatusByteCount();}
void DisneySoundGenerator::setUnreadByteCount(unsigned int count){setUnreadStatusByteCount(count);}
unsigned int DisneySoundGenerator::getDisneyBufSize(){return getCurDisneyBufSize();}
void DisneySoundGenerator::setDisneyBufSize(unsigned int bufsize){setCurDisneyBufSize(bufsize);}

void DisneySoundGenerator::setFrequency(int value)
{
}

int DisneySoundGenerator::getSample() 
{  
  // Disney Sound Source produces audio at 7KHz.  That's a bit more than half of my 16384.  I need to compensate for this.  
  // Maybe return the last value every other time rather than reading DSS's buffer?
  if (duration() == 0) 
  {
    return 0;
  }
  int8_t sample = (int8_t)getDisneyBufByte();
  //int8_t sample = getCurByte();
  sample = sample + 127;
  //int8_t sample = 0;
  decDuration();

  sample = sample * volume() / 127;
  return sample;
}

int DisneySoundGenerator::checkLptSndType()
{
  return getLptSndType();
}

bool DisneySoundGenerator::writeByteToDisneyBuf(unsigned char data)
{
  return addByteToDisneyBuf(data);
}


// DisneySoundGenerator
////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CovoxSoundGenerator

CovoxSoundGenerator::CovoxSoundGenerator()
{
}

void CovoxSoundGenerator::init()
{
	// if (midiport < 0)
	// {
		// printf("MIDI is disabled\r\n");
		// return;
	// }

  printf("CovoxSoundGenerator::init()\n");
  //setupMidiOutput();
}

void CovoxSoundGenerator::setFrequency(int value)
{
}

int CovoxSoundGenerator::getSample() 
{  
  //printf("CovoxSoundGenerator::getSample()\n");
  if (duration() == 0) 
  {
    return 0;
  }

  //int8_t sample = synthInterrupt();
  int8_t sample = 127;
  decDuration();

  sample = sample * volume() / 127;
  return sample;
}

// CovoxSoundGenerator
////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SamplesGenerator


SamplesGenerator::SamplesGenerator(int8_t const * data, int length)
  : m_data(data),
    m_length(length),
    m_index(0)
{
}


void SamplesGenerator::setFrequency(int value)
{
}


int SamplesGenerator::getSample() {

  if (duration() == 0) {
    return 0;
  }

  int sample = m_data[m_index++];

  if (m_index == m_length)
    m_index = 0;

  // process volume
  sample = sample * volume() / 127;

  decDuration();

  return sample;
}


// SamplesGenerator
////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SoundGenerator


SoundGenerator::SoundGenerator(int sampleRate, gpio_num_t gpio, SoundGenMethod genMethod)
  : m_channels(nullptr),
    m_sampleBuffer{0},
    m_volume(127),
    m_sampleRate(sampleRate),
    m_play(false),
    m_gpio(gpio),
    m_isr_handle(nullptr),
    m_DMAChain(nullptr),
    m_genMethod(genMethod),
    m_initDone(false),
    m_timerHandle(nullptr)
{
}


SoundGenerator::~SoundGenerator()
{
  clear();
    
  if (m_isr_handle) {
    // cleanup DAC mode
    periph_module_disable(PERIPH_I2S0_MODULE);
    esp_intr_free(m_isr_handle);
    for (int i = 0; i < 2; ++i)
      heap_caps_free(m_sampleBuffer[i]);
    heap_caps_free((void*)m_DMAChain);
  }
  
  if (m_timerHandle) {
    // cleanup sigmadelta mode
    esp_timer_stop(m_timerHandle);
    esp_timer_delete(m_timerHandle);
    m_timerHandle = nullptr;
  }
  
  #ifdef FABGL_EMULATED
  SDL_CloseAudioDevice(m_device);
  #endif
    
}


void SoundGenerator::clear()
{
  play(false);
  m_channels = nullptr;
}


void SoundGenerator::setDMANode(int index, volatile uint16_t * buf, int len)
{
  m_DMAChain[index].eof          = 1; // always generate interrupt
  m_DMAChain[index].sosf         = 0;
  m_DMAChain[index].owner        = 1;
  m_DMAChain[index].qe.stqe_next = (lldesc_t *) (m_DMAChain + index + 1);
  m_DMAChain[index].offset       = 0;
  m_DMAChain[index].size         = len * sizeof(uint16_t);
  m_DMAChain[index].length       = len * sizeof(uint16_t);
  m_DMAChain[index].buf          = (uint8_t*) buf;
}

// void SoundGenerator::setDMANode(int index, volatile uint8_t * buf, int len)
// {
  // m_DMAChain[index].eof          = 1; // always generate interrupt
  // m_DMAChain[index].sosf         = 0;
  // m_DMAChain[index].owner        = 1;
  // m_DMAChain[index].qe.stqe_next = (lldesc_t *) (m_DMAChain + index + 1);
  // m_DMAChain[index].offset       = 0;
  // m_DMAChain[index].size         = len * sizeof(uint8_t);
  // m_DMAChain[index].length       = len * sizeof(uint8_t);
  // m_DMAChain[index].buf          = (uint8_t*) buf;
// }




void SoundGenerator::dac_init()
{
  m_DMAChain = (volatile lldesc_t *) heap_caps_malloc(2 * sizeof(lldesc_t), MALLOC_CAP_DMA);
  
  for (int i = 0; i < 2; ++i) 
  {
    m_sampleBuffer[i] = (uint16_t *) heap_caps_malloc(FABGL_SOUNDGEN_SAMPLE_BUFFER_SIZE * sizeof(uint16_t), MALLOC_CAP_DMA);
    for (int j = 0; j < FABGL_SOUNDGEN_SAMPLE_BUFFER_SIZE; ++j)
    {
      m_sampleBuffer[i][j] = 0x7f00;
    }
    setDMANode(i, m_sampleBuffer[i], FABGL_SOUNDGEN_SAMPLE_BUFFER_SIZE);
  }
  m_DMAChain[1].sosf = 1;
  m_DMAChain[1].qe.stqe_next = (lldesc_t *) m_DMAChain; // closes DMA chain
    
  periph_module_enable(PERIPH_I2S0_MODULE);

  // Initialize I2S device
  I2S0.conf.tx_reset                     = 1;
  I2S0.conf.tx_reset                     = 0;

  // Reset DMA
  I2S0.lc_conf.in_rst                    = 1;
  I2S0.lc_conf.in_rst                    = 0;

  // Reset FIFO
  I2S0.conf.rx_fifo_reset                = 1;
  I2S0.conf.rx_fifo_reset                = 0;
  
  I2S0.conf_chan.tx_chan_mod = (m_gpio == GPIO_NUM_25 ? 3 : 4);

  I2S0.fifo_conf.tx_fifo_mod_force_en    = 1;
  I2S0.fifo_conf.tx_fifo_mod             = 1;
  I2S0.fifo_conf.dscr_en                 = 1;

  I2S0.conf.tx_mono                      = 1;
  I2S0.conf.tx_start                     = 0;
  I2S0.conf.tx_msb_right                 = 1;
  I2S0.conf.tx_right_first               = 1;
  I2S0.conf.tx_slave_mod                 = 0;
  I2S0.conf.tx_short_sync                = 0;
  I2S0.conf.tx_msb_shift                 = 0;
  
  I2S0.conf2.lcd_en                      = 1;
  I2S0.conf2.camera_en                   = 0;

  int a, b, num, m;
  m_sampleRate = calcI2STimingParams(m_sampleRate, &a, &b, &num, &m);
  I2S0.clkm_conf.clka_en                 = 0;
  I2S0.clkm_conf.clkm_div_a              = a;
  I2S0.clkm_conf.clkm_div_b              = b;
  I2S0.clkm_conf.clkm_div_num            = num;
  I2S0.sample_rate_conf.tx_bck_div_num   = m;

  I2S0.sample_rate_conf.tx_bits_mod      = 16;

  if (m_isr_handle == nullptr) 
  {
    esp_intr_alloc_pinnedToCore(ETS_I2S0_INTR_SOURCE, ESP_INTR_FLAG_LEVEL1, ISRHandler, this, &m_isr_handle, CoreUsage::quietCore());
    //esp_intr_alloc_pinnedToCore(ETS_I2S0_INTR_SOURCE, ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM, ISRHandler, this, &m_isr_handle, CoreUsage::quietCore());
    I2S0.int_clr.val     = 0xFFFFFFFF;
    I2S0.int_ena.out_eof = 1;
  }
  
  I2S0.out_link.addr  = (uintptr_t) m_DMAChain;
  I2S0.out_link.start = 1;
  I2S0.conf.tx_start  = 1;

  dac_i2s_enable();
  dac_output_enable(m_gpio == GPIO_NUM_25 ? DAC_CHANNEL_1 : DAC_CHANNEL_2);
}

// void SoundGenerator::dac_init()
// {
  // m_DMAChain = (volatile lldesc_t *) heap_caps_malloc(2 * sizeof(lldesc_t), MALLOC_CAP_DMA);
  
  // for (int i = 0; i < 2; ++i) 
  // {
    // m_sampleBuffer[i] = (uint8_t *) heap_caps_malloc(FABGL_SOUNDGEN_SAMPLE_BUFFER_SIZE * sizeof(uint8_t), MALLOC_CAP_DMA);
    // for (int j = 0; j < FABGL_SOUNDGEN_SAMPLE_BUFFER_SIZE; ++j)
    // {
      // m_sampleBuffer[i][j] = 0x7f;
    // }
    // setDMANode(i, m_sampleBuffer[i], FABGL_SOUNDGEN_SAMPLE_BUFFER_SIZE);
  // }
  // m_DMAChain[1].sosf = 1;
  // m_DMAChain[1].qe.stqe_next = (lldesc_t *) m_DMAChain; // closes DMA chain
    
  // periph_module_enable(PERIPH_I2S0_MODULE);

  // // there are some explanations of these settings at https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/I2S.cpp

  // // Initialize I2S device
  // I2S0.conf.tx_reset                     = 1;
  // I2S0.conf.tx_reset                     = 0;

  // // Reset DMA
  // I2S0.lc_conf.in_rst                    = 1;
  // I2S0.lc_conf.in_rst                    = 0;

  // // Reset FIFO
  // I2S0.conf.rx_fifo_reset                = 1;
  // I2S0.conf.rx_fifo_reset                = 0;
  
  // I2S0.conf_chan.tx_chan_mod = (m_gpio == GPIO_NUM_25 ? 3 : 4);

  // I2S0.fifo_conf.tx_fifo_mod_force_en    = 1;
  // I2S0.fifo_conf.tx_fifo_mod             = 1;
  // I2S0.fifo_conf.dscr_en                 = 1;

  // I2S0.conf.tx_mono                      = 1;
  // I2S0.conf.tx_start                     = 0;
  // I2S0.conf.tx_msb_right                 = 1;
  // I2S0.conf.tx_right_first               = 1;
  // I2S0.conf.tx_slave_mod                 = 0;
  // I2S0.conf.tx_short_sync                = 0;
  // I2S0.conf.tx_msb_shift                 = 0;
  
  // I2S0.conf2.lcd_en                      = 1;
  // I2S0.conf2.camera_en                   = 0;

  // int a, b, num, m;
  // printf("m_sampleRate = %d\n", m_sampleRate);
  // m_sampleRate = calcI2STimingParams(m_sampleRate, &a, &b, &num, &m);
  // printf("m_sampleRate = %d, a = %d, b = %d, num = %d, m = %d\n", m_sampleRate, a, b, num, m);
  // // above line output: m_sampleRate = 16384, a = 57, b = 8, num = 244, m = 20

  // I2S0.clkm_conf.clka_en                 = 0;
  // I2S0.clkm_conf.clkm_div_a              = a;
  // I2S0.clkm_conf.clkm_div_b              = b;
  // I2S0.clkm_conf.clkm_div_num            = num;
  // I2S0.sample_rate_conf.tx_bck_div_num   = m;

  // //I2S0.sample_rate_conf.tx_bits_mod      = 16;
  // I2S0.sample_rate_conf.tx_bits_mod      = 8;

  // if (m_isr_handle == nullptr) 
  // {
    // //esp_intr_alloc_pinnedToCore(ETS_I2S0_INTR_SOURCE, ESP_INTR_FLAG_LEVEL1, ISRHandler, this, &m_isr_handle, CoreUsage::quietCore());
    // printf("pinning i2s interrupt to core %d\n", CoreUsage::quietCore());
    // esp_intr_alloc_pinnedToCore(ETS_I2S0_INTR_SOURCE, ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM, ISRHandler, this, &m_isr_handle, CoreUsage::quietCore());
    // //esp_intr_alloc_pinnedToCore(ETS_I2S0_INTR_SOURCE, ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM, ISRHandler, this, &m_isr_handle, CoreUsage::quietCore() ^ 1);
    // I2S0.int_clr.val     = 0xFFFFFFFF;
    // I2S0.int_ena.out_eof = 1;
  // }
  
  // I2S0.out_link.addr  = (uintptr_t) m_DMAChain;
  // I2S0.out_link.start = 1;
  // I2S0.conf.tx_start  = 1;

  // dac_i2s_enable();
  // dac_output_enable(m_gpio == GPIO_NUM_25 ? DAC_CHANNEL_1 : DAC_CHANNEL_2);
// }



void SoundGenerator::sigmadelta_init()
{
  sigmadelta_config_t sigmadelta_cfg;
  sigmadelta_cfg.channel             = SIGMADELTA_CHANNEL_0;
  sigmadelta_cfg.sigmadelta_prescale = 10;
  sigmadelta_cfg.sigmadelta_duty     = 0;
  sigmadelta_cfg.sigmadelta_gpio     = m_gpio;
  sigmadelta_config(&sigmadelta_cfg);

  esp_timer_create_args_t args = { };
  args.callback        = timerHandler;
  args.arg             = this;
  args.dispatch_method = ESP_TIMER_TASK;
  esp_timer_create(&args, &m_timerHandle);
}


void SoundGenerator::init()
{
  if (!m_initDone) 
  {
    // handle automatic paramters
    printf("SoundGenerator::init():\n");
    if (m_genMethod == SoundGenMethod::Auto)
    {
      if (CurrentVideoMode::get() == VideoMode::CVBS)
      {     
        m_genMethod = SoundGenMethod::SigmaDelta;
      }
      else
      {
        printf("setting m_genMethod = SoundGenMethod::DAC\n");        
        m_genMethod = SoundGenMethod::DAC; //epd version chooses this
      }
      //m_genMethod = CurrentVideoMode::get() == VideoMode::CVBS ? SoundGenMethod::SigmaDelta : SoundGenMethod::DAC;
    }
    if (m_gpio == GPIO_AUTO)
    {  
      if (m_genMethod == SoundGenMethod::DAC)
      {
        printf("using SoundGenMethod::DAC, setting m_gpio = GPIO_NUM_25\n");        
        m_gpio = GPIO_NUM_25;      //epd version chooses this
      }
      else
      {
        printf("%d\n", __LINE__);        
        m_gpio = GPIO_NUM_23;
      }
      //m_gpio = m_genMethod == SoundGenMethod::DAC ? GPIO_NUM_25 : GPIO_NUM_23;
    }
    
    // actual init
    if (m_genMethod == SoundGenMethod::DAC)
    {
      printf("using SoundGenMethod::DAC, calling dac_init()\n");      
      dac_init();  // epd version chooses this
    }
    else
    {
      printf("%d\n", __LINE__);      
      sigmadelta_init();
    }
      
    m_initDone = true;
  }
}


bool SoundGenerator::play(bool value)
{
  if (value != m_play) 
  {
    init();
    
    if (m_genMethod == SoundGenMethod::DAC) 
    {
      I2S0.conf.tx_start = value;  //epd version chooses this
    }
    else
    {
      if (value)
      {
        esp_timer_start_periodic(m_timerHandle, 1000000 / m_sampleRate);
      }
      else
      {
        esp_timer_stop(m_timerHandle);
      }
    }    
    m_play = value;
    return !value;
  }
  else
  {
    return value;
  }
}


SamplesGenerator * SoundGenerator::playSamples(int8_t const * data, int length, int volume, int durationMS)
{
  auto sgen = new SamplesGenerator(data, length);
  attach(sgen);
  sgen->setAutoDestroy(true);
  if (durationMS > -1)
    sgen->setDuration(durationMS > 0 ? (m_sampleRate / 1000 * durationMS) : length );
  sgen->setVolume(volume);
  sgen->enable(true);
  play(true);
  return sgen;
}


// does NOT take ownership of the waveform generator
void SoundGenerator::attach(WaveformGenerator * value)
{
  bool isPlaying = play(false);

  value->setSampleRate(m_sampleRate);
  
  //printf("m_channels originally is %p\n", m_channels);
  value->next = m_channels;
  //printf("value->next set to %p\n", value->next);
  m_channels = value;
  //printf("m_channels set to %p\n", m_channels);

  play(isPlaying);
}


void SoundGenerator::detach(WaveformGenerator * value)
{
  if (!value)
    return;

  bool isPlaying = play(false);
  detachNoSuspend(value);
  play(isPlaying);
}


void SoundGenerator::detachNoSuspend(WaveformGenerator * value)
{
  for (WaveformGenerator * c = m_channels, * prev = nullptr; c; prev = c, c = c->next) {
    if (c == value) {
      if (prev)
        prev->next = c->next;
      else
        m_channels = c->next;
      if (value->autoDestroy())
        delete value;
      break;
    }
  }
}


//int IRAM_ATTR SoundGenerator::getSample()
int SoundGenerator::getSample()
{
  //printf("in SoundGenerator::getSample()\n");
  int sample = 0, tvol = 0;
  for (auto g = m_channels; g; ) 
  {
    if (!g)
    {
      continue;
    }
    
    if (g->enabled()) 
    {
      
      //Serial.printf("calling g->getSample()\n");
      int tSample = g->getSample();
      //sample += g->getSample(); // calls (for example) AdlibSoundGenerator::getSample()
      sample += tSample;
      if (tSample != 0)
      {
        tvol += g->volume();
      }
    }
    else if (g->duration() == 0 && g->autoDetach()) 
    {
      auto curr = g;
      g = g->next;  // setup next item before detaching this one
      detachNoSuspend(curr);
      continue; // bypass "g = g->next;"
    }
    g = g->next;  //commented out for testing.  should probably be enabled.
  }
  
  
  
  //temporary testing, simplified version
  // auto g = m_channels;
  // if(g)
  // {
    // if (g->enabled()) 
    // {
      // //Serial.printf("calling g->getSample()\n");
      // sample += g->getSample();   //getSample() seems to crash
      // tvol += g->volume();
      // g = g->next;
    // }
  // }
    
  int avol = tvol ? imin(127, 127 * 127 / tvol) : 127;
  sample = sample * avol / 127;
  sample = sample * volume() / 127;
  
  return sample;
}


// // used by DAC generator
// void IRAM_ATTR SoundGenerator::ISRHandler(void * arg)
// {
  
  // if (I2S0.int_st.out_eof) 
  // {
    // auto soundGenerator = (SoundGenerator *) arg;
    // auto desc = (volatile lldesc_t*) I2S0.out_eof_des_addr;
    
    // auto buf = (uint16_t *) soundGenerator->m_sampleBuffer[desc->sosf];
    
    // for (int i = 0; i < FABGL_SOUNDGEN_SAMPLE_BUFFER_SIZE; ++i)
    // //for (int i = 0; i < FABGL_SOUNDGEN_SAMPLE_BUFFER_SIZE / 2; ++i)
    // {
      // buf[i ^ 1] = (soundGenerator->getSample() + 127) << 8;   //original
      // //buf[i] = (soundGenerator->getSample() + 127) << 8;
      // //buf[i*2] = buf[i*2+1] = (soundGenerator->getSample() + 127) << 8;
    // }
    
  // }
  // I2S0.int_clr.val = I2S0.int_st.val;
// }

// used by DAC generator
//void IRAM_ATTR SoundGenerator::ISRHandler(void * arg)
void SoundGenerator::ISRHandler(void * arg)
{
  
  if (I2S0.int_st.out_eof) 
  {
    auto soundGenerator = (SoundGenerator *) arg;
    auto desc = (volatile lldesc_t*) I2S0.out_eof_des_addr;
    
    auto buf = (uint16_t *) soundGenerator->m_sampleBuffer[desc->sosf];
    //auto buf = (uint8_t *) soundGenerator->m_sampleBuffer[desc->sosf];
    
    for (int i = 0; i < FABGL_SOUNDGEN_SAMPLE_BUFFER_SIZE; ++i)
    //for (int i = 0; i < FABGL_SOUNDGEN_SAMPLE_BUFFER_SIZE / 2; ++i)
    {
      buf[i ^ 1] = (soundGenerator->getSample() + 127) << 8;   //original
      //buf[i] = (soundGenerator->getSample() + 127);
      //buf[i] = (soundGenerator->getSample() + 127) << 8;
      //buf[i*2] = buf[i*2+1] = (soundGenerator->getSample() + 127) << 8;
    }
  }
  I2S0.int_clr.val = I2S0.int_st.val;
}


// used by sigma-delta generator
void SoundGenerator::timerHandler(void * args)
{
  auto soundGenerator = (SoundGenerator *) args;

  sigmadelta_set_duty(SIGMADELTA_CHANNEL_0, soundGenerator->getSample());
}


void SoundGenerator::dumpBuffer(void * arg)
{
  if (I2S0.int_st.out_eof) 
  {
    auto soundGenerator = (SoundGenerator *) arg;
    auto desc = (volatile lldesc_t*) I2S0.out_eof_des_addr;
    
    auto buf = (uint8_t *) soundGenerator->m_sampleBuffer[desc->sosf];
    
    for (int i = 0; i < FABGL_SOUNDGEN_SAMPLE_BUFFER_SIZE; ++i)
    {
      //buf[i ^ 1] = (soundGenerator->getSample() + 127) << 8;   //original
      //printf("[%d] 0x%02x ", i, buf[i]);
      printf("%03d ", buf[i]);
      if ((i + 1) % 8 == 0)
      {
        printf("\n");
      }
    }
    printf("\n");
    
  }
}


uint8_t lastbyte = 0;
void SoundGenerator::dumpBuffer2(void * arg)
{
  if (I2S0.int_st.out_eof) 
  {
    auto soundGenerator = (SoundGenerator *) arg;
    auto desc = (volatile lldesc_t*) I2S0.out_eof_des_addr;
    
    auto buf = (uint8_t *) soundGenerator->m_sampleBuffer[desc->sosf];
    bool printed = false;
    int c = 0;
    for (int i = 0; i < FABGL_SOUNDGEN_SAMPLE_BUFFER_SIZE; ++i)
    {
      //buf[i ^ 1] = (soundGenerator->getSample() + 127) << 8;   //original
      //printf("[%d] 0x%02x ", i, buf[i]);
      if (buf[i] != lastbyte)
      {
        printf("%03d ", buf[i]);
        printed = true;
      }
      // if (printed)
      // {
        // if ((c + 1) % 8 == 0)
        // {
          // printf("\n");
        // }
      // }
      //printed = false;
      lastbyte = buf[i];
      c++;
    }
    if (printed)
    {
      printf("\n");
    }
    
  }
}

// SoundGenerator
////////////////////////////////////////////////////////////////////////////////////////////////////////////



/*

// note (C,D,E,F,G,A,B) + [#,b] + octave (2..7) + space + tempo (99..1)
// pause (P) + space + tempo (99.1)
char const * noteToFreq(char const * note, int * freq)
{
  uint16_t NIDX2FREQ[][12] = { {   66,   70,   74,   78,   83,   88,   93,   98,  104,  110,  117,  124 }, // 2
                               {  131,  139,  147,  156,  165,  175,  185,  196,  208,  220,  233,  247 }, // 3
                               {  262,  277,  294,  311,  330,  349,  370,  392,  415,  440,  466,  494 }, // 4
                               {  523,  554,  587,  622,  659,  698,  740,  784,  831,  880,  932,  988 }, // 5
                               { 1046, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 1760, 1865, 1976 }, // 6
                               { 2093, 2217, 2349, 2489, 2637, 2794, 2960, 3136, 3322, 3520, 3729, 3951 }, // 7
                             };
  uint8_t NNAME2NIDX[] = {9, 11, 0, 2, 4, 5, 7};  // A, B, C, D, E, F, G
  *freq = 0;
  while (*note && *note == ' ')
    ++note;
  if (*note == 0)
    return note;
  int noteIndex = (*note >= 'A' && *note <= 'G' ? NNAME2NIDX[*note - 'A'] : -2); // -2 = pause
  ++note;
  if (*note == '#') {
    ++noteIndex;
    ++note;
  } else if (*note == 'b') {
    --noteIndex;
    ++note;
  }
  int octave = *note - '0';
  ++note;
  if (noteIndex == -1) {
    noteIndex = 11;
    --octave;
  } else if (noteIndex == 12) {
    noteIndex = 0;
    ++octave;
  }
  if (noteIndex >= 0 && noteIndex <= 11 && octave >= 2 && octave <= 7)
    *freq = NIDX2FREQ[octave - 2][noteIndex];
  return note;
}


char const * noteToDelay(char const * note, int * delayMS)
{
  *delayMS = 0;
  while (*note && *note == ' ')
    ++note;
  if (*note == 0)
    return note;
  int val = atoi(note);
  if (val > 0)
    *delayMS = 1000 / val;
  return note + (val > 9 ? 2 : 1);
}



void play_task(void*arg)
{
  const char * music = "A4 4 A4 4 A#4 4 C5 4 C5 4 A#4 4 A4 4 G4 4 F4 4 F4 4 G4 4 A4 4 A4 2 G4 16 G4 2 P 8 "
                       "A4 4 A4 4 A#4 4 C5 4 C5 4 A#4 4 A4 4 G4 4 F4 4 F4 4 G4 4 A4 4 G4 2 F4 16 F4 2 P 8";
  while (true) {
    if (Play) {
      const char * m = music;
      while (*m && Play) {
        int freq, delms;
        m = noteToFreq(m, &freq);
        m = noteToDelay(m, &delms);
        frequency = freq;
        delay(delms);
        frequency = 0;
        delay(25);
      }
      Play = false;
    } else
      delay(100);
  }
}


*/








} // end of namespace

