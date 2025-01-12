/**
 * ESP32Synth
 * 
 * TODO:
 * create exponential scale for adsr knob values (only calculated when knob is rotated, so no lookup tables needed!)
 * implement all missing features like FM, LFO, Reverb and Low/Hi pass
 */

// ADSR stands for Attack, Decay, Sustain, Release
/*
 * INCLUDES
 */
#include "Arduino.h"

#include "ESP32Synth.h"

// Note ID to frequency table
#include "MidiTable.h"  

// ADSR exponential tables (length and max value of 4096)
#include "ExpTable.h"

// Wave tables (amplitude of +-2048)
#include "Sinus2048Int.h"
#include "Saw2048Int.h"
#include "Tri2048Int.h"


/*
 * DEFINITIONS AND VARIABLES
 */

//----------- WAVE TABLES -------------
// length of wavetables (must be a power of 2)
#define WAVETABLELENGTH 2048

// wave selection IDs
#define SQR  0    // square
#define SAW  1    // sawtooth
#define TRI  2    // triangle
#define SIN  3    // sinus
#define FM  4     // FM waves (unimplemented)
#define WMAX  5   // number of waves

uint32_t waveForm = SAW;          // currently selected waveform
//uint32_t waveForm = TRI;          // currently selected waveform
uint32_t pwmWidth = 1023;         // width of square wave (0-2047)


//----------- PINS ---------------
#define DAC_OUT     25            // Audio out using 8 bit internal DAC
// MIDI input is on serial port 0 (RX). This means that USB serial should also work


//--------- INTERRUPT ---------------
//#define SAMPLERATE  40000         // samples per second (40KHz)
#define SAMPLERATE  16000         // samples per second (16KHz goes well with SoundGen 16K).  Same as SoundGen sample rate

// ISRMS is not used
//#define ISRMS       25            // ms delay between each interrupt, should be synced with SAMPLERATE (40KHz)
#define ISRMS       61            // ms delay between each interrupt, should be synced with SAMPLERATE (16KHz)
//#define ISRMS       25            // ms delay between each interrupt, should be synced with SAMPLERATE (16KHz)

uint32_t mil        = 0;          // replaces millis() and is calculated during interrupt
uint32_t globalTic  = 0;          // ticks at SAMPLERATE, used for counting to 1ms

// ESP32 timer for interrupt
hw_timer_t * timer      = NULL;
portMUX_TYPE timerMux   = portMUX_INITIALIZER_UNLOCKED;


//------------ ADSR --------------
// ADSR state codes
#define ADSR_IDLE      0
#define ADSR_ATTACK    1
#define ADSR_DECAY     2
#define ADSR_SUSTAIN   3
#define ADSR_RELEASE   4

// Max time for attack, decay and release in seconds
#define ADSR_MAXATTACKTIME    3
#define ADSR_MAXRELEASETIME   6
#define ADSR_MAXDECAYTIME     8

uint32_t AmaxCount  = 0;    // number of counts within attack state (duration in samples)
uint32_t DmaxCount  = 0;    // number of counts within decay state (duration in samples)
uint32_t RmaxCount  = 0;    // number of counts within release state (duration in samples)
uint32_t Sval       = 4095; // sustain value

// ADSR data structure
typedef struct {
  uint32_t state;           // current ADSR state
  uint32_t counter;         // counter for calculations within current ADSR state
  uint32_t output;          // output of ADSR, has range of 0-4095
  uint32_t lastOutput;      // previous output value (for release value calculation)
} ADSR;


// ------------ VOICE --------------
#define MAXVOICES   20      // how many voices are allowed to be active at the same time

// Voice data structure
typedef struct {
  ADSR      adsr;           // ADSR values (each voice has its own set of ADSR values)
  uint32_t  wave;           // selected wave (currently unused. only global waveForm variable is used)
  uint32_t  note;           // note ID of voice
  uint32_t  velocity;       // velocity of current note (currently unused, should be relatively easy to implement)
  uint32_t  indexStep;      // by how much we increase the wave table index for each sample (changes with frequency)
  uint32_t  tableIndex;     // current index of wave table
  int32_t   output;         // integer output of voice, has range of +-2047
  uint32_t  activationTime; // millis when the voice was activated
} Voice;

Voice voices[MAXVOICES];    // all voices in one array

// menu pages
#define PAGES       9
#define PAGE_WAVE   0
#define PAGE_ADSR   1
#define PAGE_FM     2
#define PAGE_LFO    3
#define PAGE_REVERB 4
#define PAGE_PWM    5
#define PAGE_PASS   6
#define PAGE_VOICES 7
#define PAGE_SCOPE  8

uint32_t  page = PAGE_WAVE;           // the current menu page




//------------- MIDI ---------------
#define MIDICONTROLMAXVAL 127 // maximum value of a MIDI control byte

uint8_t mBuffer[3]  = {0,0,0};  // MIDI argument buffer of three bytes
uint8_t mIdx        = 0;        // current index of MIDI buffer
uint8_t cmdLen      = 2;        // number of arguments of current midi command
uint8_t command     = 0;        // midi command type


/*
 * FUNCTIONS
 */

// What to do when a note is pressed
// Looks for an idle voice slot to assign it to
// If it cannot find an idle slot, it will look for a voice slot with the same note ID 
// If it cannot find slot with the same note ID, it will look for the slot with the lowest amplitude in release state
// If a slot is still not found, it will select the oldest slot
//
// Args:
//  - channel:  MIDI channel of pressed note
//  - noteID:   note ID of pressed note
//  - velocity: velocity of pressed note
void handleNoteOn(uint8_t channel, uint8_t noteID, uint8_t velocity) 
{
  int32_t slot = -1;  // a slot of -1 means that no slot has been selected
  
  // looks for an idle slot
  for (uint32_t n=0; n < MAXVOICES; n++) 
  {
    if(voices[n].adsr.state == ADSR_IDLE)
    {
      slot = n;
      break;
    }
  }

  // if no slot was found, look for a slot with the same note ID 
  if (slot == -1) 
  {
    for (uint32_t n=0; n < MAXVOICES; n++) 
    {
      if(voices[n].note == noteID)
      {
        slot = n;
        break;
      }
    }
  }

  // if no slot was found, look for a slot with the lowest amplitude in release state
  if (slot == -1) 
  {
    uint32_t lowestRelease = 0xFFFFFFFF; // initialize to max uint32_t value
    for (uint32_t n=0; n < MAXVOICES; n++) 
    {
      // if the voice is in release state
      if(voices[n].adsr.state == ADSR_RELEASE)
      {
        // if the adsr output is lower than the previously found lowest output
        if (voices[n].adsr.output < lowestRelease)
        {
          lowestRelease = voices[n].adsr.output;  // update the lowestRelease
          slot = n;                               // update the current found slot
        }
      }
    }
  }

  // if still no slot was found, select the oldest slot
  if (slot == -1) 
  {
    slot = 0;                                   // to make sure slot will always be a valid value
    uint32_t lowestTime = 0xFFFFFFFF;           // initialize to max uint32_t value
    for (uint32_t n=0; n < MAXVOICES; n++) 
    {
      if (voices[n].activationTime < lowestTime)
      {
        lowestTime = voices[n].activationTime;  // update the lowestTime
        slot = n;                               // update the current found slot
      }
    }
  }

  // when we arrive here, we have found a slot
  voices[slot].note = noteID;         // set the note ID
  voices[slot].activationTime = mil;  // set the activation time
  uint32_t f = getFreq(noteID);       // get freqency of note ID
  setVoiceFreqency(f, slot);          // set the freqency
  setGateOn(slot);                    // notify the ADSR that the note was pressed
}


// What to do when a note is released
// Looks for all voices with the same note ID as the released note ID
// Also checks if the voice is not idle and already in release state, since we want to ignore those
// Then tells the ADSR to release those voices
//
// Args:
//  - channel:  MIDI channel of released note
//  - noteID:   note ID of released note
//  - velocity: velocity of released note
void handleNoteOff(uint8_t channel, uint8_t noteID, uint8_t velocity) 
{
  for (int n=0; n < MAXVOICES; n++) 
  {
    if(voices[n].note == noteID && voices[n].adsr.state != ADSR_IDLE && voices[n].adsr.state != ADSR_RELEASE)
    {
      setGateOff(n);
    }
  }
}


// Returns the current audio sample by doing the following steps:
//  - update the states of the active voices and ADSRs
//  - apply ADSR to the outputs of all active voices
//  - mix those outputs together
//  - scale output to range of DAC
//  - apply clipping where needed
//  - convert signed sample to unsigned sample
//  - return unsigned sample
//uint32_t produceSample() 
int8_t produceSample() 
{
  // update the states of the active voices and ADSRs
  for (uint32_t n = 0; n < MAXVOICES; n++) 
  {
    if ( voices[n].adsr.state != ADSR_IDLE) 
    {
      updateVoice(n);
      updateADSR(n);  
    }
  }

  // apply ADSR to the outputs of all active voices and add them together
  int32_t sum = 0;  
  for (uint32_t n = 0; n < MAXVOICES; n++) 
  {
    if (voices[n].adsr.state != ADSR_IDLE) 
    {
      // voice output is in range of +-2047
      // ADSR output is in range of 0-4095
      // multiply both outputs and divide by 4096 to get an output of +-2047
      int32_t signedADSR = voices[n].adsr.output;   // convert uint32_t to int_32t
      int32_t ADSRappliedOutput = (signedADSR * voices[n].output) >> 12;
      sum = sum + ADSRappliedOutput;             
    } 
  }
       
  // scale output to range of DAC
  sum = sum >> 7; // divide by 8 to allow for 8 peaks at the same time, then divide by 16 to reduce amplitude from +-2047 to +-128.
  
  // apply hard clipping if needed
  if (sum > 127)
    sum = 127;
  if (sum < -128)
    sum = -128;

  //int8_t sum2 = sum & 0xff;
  return sum;

  // convert signed sample to unsigned sample
  //uint32_t unsignedSample = 128 + sum;  // add 128 since the DAC has a range of 0-255
    
  //return unsignedSample;
}


// // Interrupt routine of synthesizer (40KHz)
// // Increments counters
// // Produces and outputs a sample
// void synthInterrupt() 
// {
  // portENTER_CRITICAL_ISR(&timerMux);
  
  // globalTic++;          // counter at 40KHz
  // if (globalTic >= 40)  // 40 tics at 40KHz is 1ms
  // {
    // globalTic = 0;      // reset counter
    // mil++;              // increase millis counter
  // }
  
  // // generate and output a sample
  // dacWrite(DAC_OUT, produceSample());
  
  // portEXIT_CRITICAL_ISR(&timerMux);
// }


// Interrupt routine of synthesizer (40KHz)
// Increments counters
// Produces and outputs a sample
//void synthInterrupt() 
int8_t synthInterrupt()
{
  //portENTER_CRITICAL_ISR(&timerMux);
  
  globalTic++;          // counter at 40KHz
  if (globalTic >= 40)  // 40 tics at 40KHz is 1ms
  {
    globalTic = 0;      // reset counter
    mil++;              // increase millis counter
  }
  
  // generate and output a sample
  //dacWrite(DAC_OUT, produceSample());
  return produceSample();
  //portEXIT_CRITICAL_ISR(&timerMux);
}

// Things to do at bootup
void setupMidiOutput() 
{
  //Serial.begin(115200); // setup serial for MIDI and PC communication
  
  initVoices();
  initADSR();

  // setup timer for interrupt
  //timer = timerBegin(0, 80, true);
  //timerAttachInterrupt(timer, synthInterrupt, true);
  //timerAlarmWrite(timer, ISRMS, true);
  //timerAlarmEnable(timer);
}


// // Should probably be called from somewhere inside FabGL's SoundGenerator
// // Maybe set up an XQueue to replace the MIDI over serial connection?
// // First I'll just try to get it working with the main CPU task running this code on the writePort() to 0x330
// void receiveMidiData() 
// {
  // checkMIDI();        // check for MIDI commands
// }


















/*
 * ADSR
 * Contains ADSR related functions
 * // ADSR stands for Attack, Decay, Sustain, Release
 */



// Initialize the ADSR
// by setting initial values for ADSR
void initADSR()
{
  setADSRattack     (8);
  
  setADSRdecay      (10);
  
  setADSRsustain    (127);
  
  setADSRrelease    (13);
}


// Tell ADSR that the note is pressed
void setGateOn(uint32_t n) 
{
  voices[n].adsr.counter  = 0;
  voices[n].adsr.state    = ADSR_ATTACK;
}


// Tell ADSR that the note is released
void setGateOff(uint32_t n) 
{
  voices[n].adsr.lastOutput   = voices[n].adsr.output;
  voices[n].adsr.counter      = 0;
  voices[n].adsr.state        = ADSR_RELEASE;
}


// Sets AmaxCount value based on the given MIDI value (0-127)
// We can use floats here, since this is only calculated when the ADSR settings are changed
// Currently a linear scale, should make this exponential in the future
void setADSRattack(uint32_t MIDIvalue) 
{
  float f = float(SAMPLERATE) * (float(MIDIvalue) * float(ADSR_MAXATTACKTIME) / float(MIDICONTROLMAXVAL));
  AmaxCount = uint32_t(f);
}


// Sets DmaxCount value based on the given MIDI value (0-127)
// We can use floats here, since this is only calculated when the ADSR settings are changed
// Currently a linear scale, should make this exponential in the future
void setADSRdecay(uint32_t MIDIvalue) 
{
  float f = float(SAMPLERATE) * (float(MIDIvalue) * float(ADSR_MAXDECAYTIME) / float(MIDICONTROLMAXVAL));
  DmaxCount = uint32_t(f);
}


// Sets sustain value by mapping the given MIDI value to 0-4095
// We can use floats here, since this is only calculated when the ADSR settings are changed
// Which is the valid range for sustain values
// Currently is a linear function, should make this exponential in the future
void setADSRsustain(uint32_t MIDIvalue)
{
  Sval = map(MIDIvalue, 0, 127, 0, 4095);
}


// Sets RmaxCount value based on the given MIDI value (0-127)
// We can use floats here, since this is only calculated when the ADSR settings are changed
// Currently a linear scale, should make this exponential in the future
void setADSRrelease(uint32_t MIDIvalue) 
{
  float f = float(SAMPLERATE) * (float(MIDIvalue) * float(ADSR_MAXRELEASETIME) / float(MIDICONTROLMAXVAL));
  RmaxCount = uint32_t(f);
}


// Updates the ADSR state, counter and output value for the given voice
// Is called for every sample
void updateADSR(uint32_t n) 
{
  // do different things for different states
  switch (voices[n].adsr.state) 
  {
    case ADSR_ATTACK:
        voices[n].adsr.counter += 1; 
        // if we reached the end of attack
        if (voices[n].adsr.counter >= AmaxCount) 
        {
          voices[n].adsr.state    = ADSR_DECAY;
          voices[n].adsr.counter  = 0;
        }
        else 
        {
          // calculate ADSR table index based on current counter and maxCount
          uint32_t maxCountDiv4096 = AmaxCount >> 12;
          if (maxCountDiv4096 == 0) // prevent division by 0
            maxCountDiv4096 = 1;
          uint32_t index = voices[n].adsr.counter / maxCountDiv4096;
          if (index > 4095)
            index = 4095;
          
          voices[n].adsr.output = getAttack(index);
        }
      break;
    case ADSR_DECAY:
        voices[n].adsr.counter += 1; 
        // if we reached the end of decay
        if ( voices[n].adsr.counter >= DmaxCount) 
        {
          voices[n].adsr.state = ADSR_SUSTAIN;
          voices[n].adsr.counter = 0;
        }
        else 
        {
          // calculate ADSR table index based on current counter and maxCount
          uint32_t maxCountDiv4096 = DmaxCount >> 12;
          if (maxCountDiv4096 == 0) // prevent division by 0
            maxCountDiv4096 = 1;
          uint32_t index = voices[n].adsr.counter / maxCountDiv4096;
          if (index > 4095)
            index = 4095;

          uint32_t v = getDecay(index);
          // scale decay to sustain difference
          // TODO test this
          v = (v * (4095-Sval)) >> 12;
          v = v + Sval;
          
          voices[n].adsr.output = v;
        }
      break;
    case ADSR_SUSTAIN:
      voices[n].adsr.output = Sval;
      break;
    case ADSR_RELEASE:
        voices[n].adsr.counter += 1; 
        // if we reached the end of release
        if ( voices[n].adsr.counter >= RmaxCount) 
        {
          voices[n].adsr.counter  = 0;
          voices[n].adsr.state    = ADSR_IDLE;
          voices[n].adsr.output   = 0;
          //voices[n].note          = 0;          // reset note ID
        }
        else 
        {
          // calculate ADSR table index based on current counter and maxCount
          uint32_t maxCountDiv4096 = RmaxCount >> 12;
          if (maxCountDiv4096 == 0)
            maxCountDiv4096 = 1;
          uint32_t index = voices[n].adsr.counter / maxCountDiv4096;
          if (index > 4095)
            index = 4095;
          
          uint32_t v = getDecay(index);

          // scale output height by lastOutput
          v = (v * voices[n].adsr.lastOutput) >> 12;
          voices[n].adsr.output = v;
        }
      break;
  }
}














/*
 * MIDI
 * Contains all MIDI related functions
 * Please note that this is all designed for an Acorn Masterkey (61) keyboard
 * So if you have another keyboard, you should update the executeMidi function with your own MIDI codes
 */


// Execute the received MIDI command
void executeMidi() 
{
  // note on
  if (command >= 0x90 && command <= 0x9f) 
  {
    handleNoteOn((command & 0x0f), mBuffer[0], mBuffer[1]); // channel, note, velocity
  }
  
  // note off
  else if (command >= 0x80 && command <= 0x8f) 
  {
    handleNoteOff((command & 0x0f), mBuffer[0], 0); // channel, note, velocity
  }
  
  // control change
  else if (command >= 0xb0 && command <= 0xbf) 
  {
    // modulation wheel
    if ( mBuffer[0] == 0x01) 
    {
      if (page == PAGE_WAVE) 
      {
        // set waveform
        waveForm = map(mBuffer[1], 0, 127, 0, WMAX-1);
        //sendFrameBuffer = true;
      }
      else if (page == PAGE_PWM) 
      {
        // set PWM width
        pwmWidth = map(mBuffer[1], 0, 127, 0, 2047);
        //sendFrameBuffer = true;
      }
      else if (page == PAGE_SCOPE)
      {
        // set scope delay
        //scopeDelay = mBuffer[1] << 2;
      }
    }
    
    // volume slider
    else if ( mBuffer[0] == 0x07) 
    {
      // currently set PWM width, should become something else in the future
      pwmWidth = map(mBuffer[1], 0, 127, 0, 2047);
    }

    // C1 knob
    else if ( mBuffer[0] == 0x4a) 
    {
      // set attack
      setADSRattack     (mBuffer[1]);
    }

    // C2 knob
    else if ( mBuffer[0] == 0x47) 
    {
      // set decay
      setADSRdecay      (mBuffer[1]);
    }

    // C3 knob
    else if ( mBuffer[0] == 0x49) 
    {
      // set sustain
      setADSRsustain     (mBuffer[1]);
    }

    // C4 knob
    else if ( mBuffer[0] == 0x48) 
    {
      // set release
      setADSRrelease     (mBuffer[1]);
    }
  }

  // pitch bend
//  else if (command >= 0xe0 && command <= 0xef) 
//  {
//    // change menu page when pich bend is at top or bottom
//    if (mBuffer[1] == 127)
//    {
//      page++;
//      if (page >= PAGES)
//        page = 0;
//      sendFrameBuffer = true;
//    }
//    else if (mBuffer[1] == 0)
//    {
//      if (page == 0)
//        page = PAGES-1;
//      else
//        page--;
//      sendFrameBuffer = true;
//    }
//  }
}


// Checks if MIDI byte b is a command
// Returns the number of expected arguments if b is a command
// Else it returns 0
uint32_t isCommand(uint8_t b ) 
{
  uint32_t l = 0;
  if (b >= 0x80 && b <= 0x8f) 
  { 
    // Note off
    l = 2; 
    command = b;
  }
  else if (b >= 0x90 && b <= 0x9f) 
  { 
    // Note on
    l = 2;
    command = b;
  }
  else if (b >= 0xa0 && b <= 0xaf) 
  {
    // aftertouch, poly
    l = 2;
    command = b;
  }
  else if (b >= 0xb0 && b <= 0xbf) 
  {
    // control change
    l = 2;
    command = b;
  }
  else if (b >= 0xc0 && b <= 0xcf) 
  {
    // program change
    l = 1;
    command = b;
  }
  else if (b >= 0xd0 && b <= 0xdf) 
  {
    // aftertouch, channel
    l = 1;
    command = b;
  }
  else if (b >= 0xe0 && b <= 0xef) 
  {
    // Pitch wheel
    l = 2;
    command = b;
  }
  else if (b >= 0xf0 && b <= 0xff) 
  {
    // System exclusive
    l = 1;
    command = b;
  }
  return l;
}


// Reads and parses MIDI byte from serial buffer
// Executes the MIDI command when all arguments are received
void writeMidiByte(uint8_t b)
{
  //uint8_t b = Serial.read();
  
  uint32_t len = isCommand(b);
  if (len > 0)
  {
    cmdLen = len;
    mIdx = 0;
  }
  else 
  {
    mBuffer[mIdx++] = b;
    cmdLen--;
    if (cmdLen == 0) 
    {
      executeMidi();
    }
  }
}


// // Checks if MIDI bytes are avaiable and reads/executes them
// void checkMIDI()
// {
  // while (Serial.available())
    // doRead();
// }






















/*
 * VOICE
 * Contains all voice related functions
 */


// Initialize all voices
void initVoices() 
{
  for (uint32_t n = 0; n < MAXVOICES; n++) 
  {
    voices[n].activationTime  = 0;
    voices[n].indexStep       = 0;
    voices[n].tableIndex      = 0;
    voices[n].adsr.state      = ADSR_IDLE;
    voices[n].adsr.output     = 0;
  }
}


// Set frequency of voice n by setting the index step based on the given frequency
// Index step is multiplied by 256 to preserve precision
void setVoiceFreqency(uint32_t f, uint32_t n) 
{
  voices[n].indexStep = ((WAVETABLELENGTH  * f) <<8) / SAMPLERATE; // multiply by 256 to prevent losing precision
  voices[n].tableIndex = 0;
}


// Updates the state of the voice by incrementing counters and traversing trough the wave tables
void updateVoice(uint32_t n) 
{
  // step through the wavetable, the larger the step value, the higher the frequency
  voices[n].tableIndex += voices[n].indexStep;  

  // wrap around table index (requires WAVETABLELENGTH to be a power of 2)
  // also note that we divide here by 256, since the index step is multiplied by 256 (for precision)
  uint32_t tIdx = (voices[n].tableIndex >> 8) & (WAVETABLELENGTH-1);   

  int32_t amplitude = 0;
  // which table to use:
  if(waveForm == SAW)
    amplitude = getSawInt(tIdx);
  else if(waveForm == SIN)
    amplitude = getSinInt(tIdx);  
  else if(waveForm == TRI)
    amplitude = getTriInt(tIdx); 
  else if(waveForm == SQR)
    amplitude = (tIdx < pwmWidth)?-2047:2047; 

  // set output value
  voices[n].output = amplitude; //  +- 2047
}

















