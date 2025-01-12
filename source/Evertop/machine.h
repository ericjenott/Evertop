/*
  Created by Fabrizio Di Vittorio (fdivitto2013@ENABLE_CH376gmail.com) - <http://www.fabgl.com>
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


#pragma once


//#include "src/FabGL/src/fabgl.h"
//#include "src/FabGL/src/emudevs/graphicsadapter.h"
#include "src/FabGL/src/emudevs/i8086.h"
#include "src/FabGL/src/emudevs/PIC8259.h"
#include "src/FabGL/src/emudevs/PIT8253.h"
#include "src/FabGL/src/emudevs/i8042.h"
#include "src/FabGL/src/emudevs/MC146818.h"
#include "src/FabGL/src/emudevs/PC8250.h"
//#include "src/FabGL/src/emudevs/fake8250.h"
//#include "src/FabGL/src/devdrivers/MCP23S17.h"

#include "bios.h"

// sound devices enable/disable
#define ENABLE_SOUNDGEN 1
#define ENABLE_ADLIB 1  // set to zero to turn off Adlib card
#define ENABLE_MIDI 1 // set to zero to turn off MIDI output device
#define ENABLE_PC_SPEAKER 1   // set to zero to turn off pc speaker
#define ENABLE_DISNEY 1 // set to zero to turn off Disney Sound System
#define ENABLE_COVOX 0  // set to zero to turn off COVX Sound System 

// CH376 USB drive interface enable/disable
#define ENABLE_CH376 1

// ethernet enable/disable
#define ENABLE_ETHERNET 1

//#define RAM_SIZE             1048576    // 1024 * 1024 must correspond to bios MEMSIZE
#define RAM_SIZE 0x10FFF0  //1114096 (1048576 + 65536)
#define VIDEOMEMSIZE         65536
//#define VIDEOMEMSIZE         68000 // make room for 4000 bytes of textmode video data + 64000 for MCGA320x200

#define NETWORKMODE_NONE 0
#define NETWORKMODE_WIFI 1
#define NETWORKMODE_ETH  2

#define PWR_SAVE_OFF 0
#define PWR_SAVE_FULL 1
#define PWR_SAVE_HALF 2

// maximum number of serial ports
#define SERIALPORTS          4

#define HIRES_FONT_HEIGHT 16
#define HIRES_FONT_WIDTH 8
//#define HIRES_FONT_HEIGHT 9
//#define HIRES_FONT_WIDTH 6
#define HIRES_FONT_BASELINE 12

#define LORES_FONT_HEIGHT 16
#define LORES_FONT_WIDTH 16
//#define LORES_FONT_HEIGHT 9
//#define LORES_FONT_WIDTH 6
#define LORES_FONT_BASELINE 12

#define SCREEN_PIXEL_WIDTH 648
#define SCREEN_PIXEL_HEIGHT 480

#define SCREEN80X25_COLS 80
#define SCREEN80X25_ROWS 25
#define SCREEN40X25_COLS 40
#define SCREEN40X25_ROWS 25

#define CGA_40X25_GRAY 0x00
#define CGA_40X25_COLOR 0x01
#define CGA_80X25_GRAY  0x02
#define CGA_80X25_COLOR  0x03
#define MDA_80X25_MONO  0x07

// graphics video modes
#define CGA_320X200_COLOR 0x04
#define CGA_320X200_GRAY 0x05
#define CGA_640X200_MONO 0x06
#define HGC_720X348_MONO 0x08
#define EGA_320X200_COLOR 0x0d
#define EGA_640X200_COLOR 0x0e
#define EGA_640X350_MONO 0x0f
#define MCGA_640X480_MONO 0x11
#define VGA_640X480_COLOR 0x12
#define MCGA_320X200_COLOR 0x13


#define DEFAULT_FG_COLOR GxEPD_BLACK
#define DEFAULT_BG_COLOR GxEPD_WHITE


//using fabgl::GraphicsAdapter;
using fabgl::PIC8259;
using fabgl::PIT8253;
using fabgl::i8042;
using fabgl::MC146818;
//using fabgl::MCP23S17;  //i2c bus, not needed in old XT, enable if want to expand hardware IO capabilites
using fabgl::SerialPort;
//using fabgl::FakeSerialPort;
using fabgl::PC8250;
//using fabgl::fake8250;
// using fabgl::AdlibSoundGenerator;

typedef void (*SysReqCallback)();



class Machine {

public:
  Machine();
  ~Machine();
  
  void setBaseDirectory(char const * value)    { m_baseDir = value; }

  void setDriveImage(int drive, char const * filename, int cylinders = 0, int heads = 0, int sectors = 0);
  
  bool diskChanged(int drive)                  { return m_diskChanged[drive]; }
  void resetDiskChanged(int drive)             { m_diskChanged[drive] = false; }

  void setBootDrive(int drive)                 { m_bootDrive = drive; }

  void setSysReqCallback(SysReqCallback value) { m_sysReqCallback = value; }
  void setSysReqCallback2(SysReqCallback value) { m_sysReqCallback2 = value; }
  void setSysReqCallback3(SysReqCallback value) { m_sysReqCallback3 = value; }
  void setSysReqCallback4(SysReqCallback value) { m_sysReqCallback4 = value; }
  void clearLastFrameBuffer();
  void invertColors();

  void redrawScreen();
  void drawBorders();
  //void refreshScreen();
  bool checkHibernateFile();
  uint8_t powersaveEnabled;
  
  void setCOM1(SerialPort * serialPort);

  void setCOM2(SerialPort * serialPort);

//  void setCOM3(FakeSerialPort * serialPort);

//  fake8250 getCOM3(){return m_COM3;}

  void run();

  void trigReset()                     { m_reset = true; }

  uint32_t ticksCounter()              { return m_ticksCounter; }

  i8042 * getI8042()                   { return &m_i8042; }
  void  resetKeyboard();
  
  MC146818 * getMC146818()             { return &m_MC146818; }

  uint8_t * memory()                   { return s_memory; }

  uint8_t * videoMemory()              { return s_videoMemory; }

  uint8_t * frameBuffer()              { return m_frameBuffer; }

//  GraphicsAdapter * graphicsAdapter()  { return &m_graphicsAdapter; }

  FILE * disk(int index)               { return m_disk[index]; }
  char const * diskFilename(int index) { return m_diskFilename[index]; }
  uint64_t diskSize(int index)         { return m_diskSize[index]; }
  uint16_t diskCylinders(int index)    { return m_diskCylinders[index]; }
  uint8_t diskHeads(int index)         { return m_diskHeads[index]; }
  uint8_t diskSectors(int index)       { return m_diskSectors[index]; }

  void dumpMemory(char const * filename);
  void dumpInfo(char const * filename);
  void printDumpInfo();
  //void testpresskey(char testkey);

  //uint32_t getAdlibSamples(int8_t *samples);

  static void writeVideoMemory8(void * context, int address, uint8_t value);
  static void writeVideoMemory16(void * context, int address, uint16_t value);
  static uint8_t readVideoMemory8(void * context, int address);
  static uint16_t readVideoMemory16(void * context, int address);

  void setVideoMode(int video_mode);
  void setPixel(int x, int y, uint8_t color);

  void sound(int freq, uint32_t duration);  //sound speaker at 917hz for ms (blocking, whole system stops)
  
  uint32_t                 last_checkscreen_time;
  bool                     need_unhibernate = false;
  uint32_t                 machine_idx;
  bool                     testmode = false;
  bool epd_busy(void);
  uint8_t *                m_frameBuffer;
  static void printCharAttr(char ascii, unsigned char attribute, int charLocation, bool inverse = false);
  static uint8_t *         s_videoMemory;



// speaker/audio
#if ENABLE_SOUNDGEN  
  bool                     m_speakerDataEnable;
  SoundGenerator           m_soundGen;
#if ENABLE_ADLIB
  AdlibSoundGenerator   m_adlibSoundGenerator;
#endif //ENABLE_ADLIB
#if ENABLE_PC_SPEAKER
  //SineWaveformGenerator    m_sinWaveGen;
  SquareWaveformGenerator    m_squareWaveGen;
#endif //ENABLE_PC_SPEAKER
#if ENABLE_MIDI
  MidiSoundGenerator   m_midiSoundGenerator;
#endif //ENABLE_MIDI
#if ENABLE_DISNEY
  DisneySoundGenerator m_disneySoundGenerator;
#endif //ENABLE_DISNEY
#if ENABLE_COVOX
  CovoxSoundGenerator m_covoxSoundGenerator;
#endif //ENABLE_COVOX
#endif //ENABLE_SOUNDGEN

#if ENABLE_ETHERNET
  void prepareNetworkSetup();
  static void NE2000IRQHandler(void *context);
#endif // ENABLE_ETHERNET  

  uint16_t                 m_CGAMemoryOffset;
  uint16_t                 m_MCGAMemoryOffset;
  uint16_t                 m_HGCMemoryOffset;

  void hibernate();
  void lightSleep();
  void unhibernate(bool need_reset = false);

  void powerOff();
  
private:


  static void runTask(void * pvParameters);

  void init();
  void reset();

  void tick();
  //static void printCharAttr(char ascii, unsigned char attribute, int charLocation, bool inverse = false);

  //void drawCursor();
  //void set320x200pixel(int x, int y, uint8_t color);
  //void set640x200pixel(int x, int y, uint8_t color);
  //void set720x348pixel(int x, int y, uint8_t color);
  
  void setCGAMode();
  void setMCGAMode();
  void setCGA6845Register(uint8_t value);
  void setMCGARegister(uint8_t value);

  void setHGCMode();
  void setHGC6845Register(uint8_t value);

  //static void writePort(void * context, int address, uint8_t value);
  //static void writePort(void * context, int address, uint8_t value);
  static void writePort(void * context, int address, uint8_t value, bool isWord, bool is2ndByte = false);
  static uint8_t readPort(void * context, int address, bool isWord = false, bool is2ndByte = false);

//  static void writeVideoMemory8(void * context, int address, uint8_t value);
//  static void writeVideoMemory16(void * context, int address, uint16_t value);
//  static uint8_t readVideoMemory8(void * context, int address);
//  static uint16_t readVideoMemory16(void * context, int address);

  static bool interrupt(void * context, int num);

  static void incHltCount();
  static void incMemsCount();

  static void PITChangeOut(void * context, int timerIndex);

  static bool MC146818Interrupt(void * context);
  
  static bool COM1Interrupt(PC8250 * source, void * context);
  static bool COM2Interrupt(PC8250 * source, void * context);
//  static bool COM3Interrupt(fake8250 * source, void * context);

  static bool keyboardInterrupt(void * context);
  static bool mouseInterrupt(void * context);
  static bool resetMachine(void * context);
  static bool sysReq(void * context);
  static bool sysReq2(void * context);
  static bool sysReq3(void * context);
  static bool sysReq4(void * context);
  static bool sysReq5(void * context);
  static bool sysReq6(void * context);
  


  static bool intPending(int &IntNumber);
  
  void speakerSetFreq();
  void speakerEnableDisable();

  void autoDetectDriveGeometry(int drive);

  
  bool                     m_reset;

//  GraphicsAdapter          m_graphicsAdapter;

  BIOS                     m_BIOS;

  // 0, 1 = floppy
  // >= 2 = hard disk
  char *                   m_diskFilename[DISKCOUNT];
  bool                     m_diskChanged[DISKCOUNT];
  FILE *                   m_disk[DISKCOUNT];
  //uint64_t                 m_diskSize[DISKCOUNT];
  uint32_t                 m_diskSize[DISKCOUNT];
  uint16_t                 m_diskCylinders[DISKCOUNT];
  uint8_t                  m_diskHeads[DISKCOUNT];
  uint8_t                  m_diskSectors[DISKCOUNT];

  static uint8_t *         s_memory;
  
  //uint8_t *                m_frameBuffer;
  
//  uint8_t *                last_frameBuffer;
  static uint8_t *         MCGA_videoMemory;
  //uint8_t *                char_refreshes;
  //uint32_t                 last_key_time;

  // 8259 Programmable Interrupt Controllers
  PIC8259                  m_PIC8259A;  // master
  PIC8259                  m_PIC8259B;  // slave

  // 8253 Programmable Interval Timers
  // pin connections of PIT8253 on the IBM XT:
  //    gate-0 = gate-1 = +5V
  //    gate-2 = TIM2GATESPK
  //    clk-0  = clk-1  = clk-2 = 1193182 Hz
  //    out-0  = IRQ0
  //    out-1  = RAM refresh
  //    out-2  = speaker
  PIT8253                  m_PIT8253;

  // 8042 PS/2 Keyboard Controller
  i8042                    m_i8042;

  TaskHandle_t             m_taskHandle;

  uint32_t                 m_ticksCounter;

  // CGA
  uint8_t                  m_CGA6845SelectRegister;
  uint8_t                  m_MCGASelectRegister;
  uint8_t                  m_CGA6845[18];
  uint8_t                  m_MCGA[18];
//  uint16_t                 m_CGAMemoryOffset;
//  uint16_t                 m_MCGAMemoryOffset;
  uint8_t                  m_CGAModeReg;
  uint8_t                  m_CGAColorReg;
  uint16_t                 m_CGAVSyncQuery;

  // Hercules
  uint8_t                  m_HGC6845SelectRegister;
  uint8_t                  m_HGC6845[18];
//  uint16_t                 m_HGCMemoryOffset;
  uint8_t                  m_HGCModeReg;
  uint8_t                  m_HGCSwitchReg;
  uint16_t                 m_HGCVSyncQuery;

//  int                     cursor_pos; 
//  int                     last_cursor_pos;
//  bool                    cursor_visible;
//  int                     cursor_start_scanline;
//  int                     cursor_end_scanline;




  // CMOS & RTC
  MC146818                 m_MC146818;

  // extended I/O (MCP23S17)
  //MCP23S17                 m_MCP23S17;
  //uint8_t                  m_MCP23S17Sel;

  uint8_t                  m_bootDrive;

  SysReqCallback           m_sysReqCallback;
  SysReqCallback           m_sysReqCallback2;
  SysReqCallback           m_sysReqCallback3;
  SysReqCallback           m_sysReqCallback4;
  
  char const *             m_baseDir;
  
  // serial ports
  PC8250                   m_COM1;
  PC8250                   m_COM2;
//  fake8250                 m_COM3;

};
