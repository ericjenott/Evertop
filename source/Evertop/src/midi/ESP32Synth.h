#include "Arduino.h"

void writeMidiByte(uint8_t b);
void setupMidiOutput();
int8_t synthInterrupt();

// ADSR
void initADSR();
void setGateOn(uint32_t n);
void setGateOff(uint32_t n);
void setADSRattack(uint32_t MIDIvalue);
void setADSRdecay(uint32_t MIDIvalue);
void setADSRsustain(uint32_t MIDIvalue);
void setADSRrelease(uint32_t MIDIvalue);
void updateADSR(uint32_t n);

// //MIDI
// void handleNoteOn(uint8_t channel, uint8_t noteID, uint8_t velocity);
void checkMIDI();


//VOICE
void setVoiceFreqency(uint32_t f, uint32_t n);
void updateVoice(uint32_t n);
void initVoices() ;



