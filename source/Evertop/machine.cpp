/*
  Created by Fabrizio Di Vittorio (fdivitto2013@gmail.com) - <http://www.fabgl.com>
  Copyright (c) 2019-2022 Fabrizio Di Vittorio.
  All rights reserved.a


0x10FFF0

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
#define minimum(a,b) ((a) < (b) ? (a) : (b))

//typedef unsigned long DWORD;
typedef unsigned int DWORD;

#include "machine.h"

#include "src/GxEPD2/src/GxEPD2_BW.h"
#include "fonts/VGA8x16.h"
#include "fonts/BIOS8x8.h"
#include "fonts/BIOS8x8Bold.h"
#include "fonts/VGA8x16Bold.h"
#include "src/FabGL/src/fabutils.h"
#include "src/FabGL/src/devdrivers/kbdlayouts.h"

#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <time.h>
#include <math.h>

//for COVOX DAC audio
//#include <driver/dac.h>

//ULP stuff
#include "esp32/ulp.h"
//#include "driver/rtc_io.h"
//#include "soc/rtc.h"
//#include "soc/soc_caps.h"

// Adlib stuff
//#include "src/FabGL/src/devdrivers/adlib/adlib.h"
//#include "driver/i2s.h"
//#include "esp_err.h"
//#include "esp_log.h"

//MIDI stuff
//#incude "src/midi/ESP32Synth.h"

// Disney sound stuff
//#include "src/disney/disney.h"

// E-ink display stuff
// old GxEPD library includes 
//#include "src/GxEPD/src/GxEPD.h"
//#include "src/GxEPD/src/GxIO/GxIO_SPI/GxIO_SPI.h"
//#include "src/GxEPD/src/GxIO/GxIO.h"
//#include "src/GxEPD/src/GxGDEP015OC1/GxGDEP015OC1.h"    // 1.54" b/w
//#include "src/GxEPD/src/GxGDEH029A1/GxGDEH029A1.h" // 2.9" b/w
//#include "src/GxEPD/src/GxDEPG0750BN/GxDEPG0750BN.h" // 7.5" b/w

// base class GxEPD2_GFX can be used to pass references or pointers to the display instance as parameter, uses ~1.2k more code
// enable or disable GxEPD2_GFX base class
#define ENABLE_GxEPD2_GFX 0
// new GxEPD2 library includes
#include "src/GxEPD2/src/GxEPD2_BW.h"

// Networking mode (none, wifi, or ethernet)
extern int networkMode;
extern uint8_t ethMacAddr[];
uint8_t ethBuf[1500];
bool ethSendbufFree = true;
bool ethSleep = false;
uint32_t lastEthSendTime = 0;
uint32_t lastEthRecvTime = 0;
void ethWakeUp();
void ethGoToSleep();

extern bool colorDifferentiation;

//#include "SPI.h"
#include "Esp.h"
#include "board_def.h"
//extern GxIO_Class io;
//extern GxEPD_Class display;
extern GxEPD2_BW<GxEPD2_583_T8, GxEPD2_583_T8::HEIGHT> display;
extern int refreshes;
extern void refreshScreen();

extern void EPD_display_init();
extern void EPD_sleep();
extern void EPD_init_Fast();
extern void PIC_display_Clear();

extern bool videoTaskSuspended;
extern bool machineSuspended;
//static uint32_t idle_cnt = 0;
//uint32_t last_idle_cnt = 0;
//static void idle_task(void *parm);


//ch376 stuff
uint8_t ch375Command = 0x00;
uint8_t ch375PreviousCommand = 0x00;
uint8_t ch375Data = 0x00;
bool ch375WriteCommandMode = false;
bool ch375IntCleared = true;
uint32_t usbSpeed = 9600;
uint8_t serial1Mode = 0; //0 = USB, 1 = LoRa
#pragma GCC optimize ("O3")




// CGA Craphics Card Ports Bits

#define CGA_MODECONTROLREG_TEXT80         0x01   // 0 = 40x25, 1 = 80x25
#define CGA_MODECONTROLREG_GRAPHICS       0x02   // 0 = text,  1 = graphics
#define CGA_MODECONTROLREG_COLOR          0x04   // 0 = color, 1 = monochrome
#define CGA_MODECONTROLREG_ENABLED        0x08   // 0 = video off, 1 = video on
#define CGA_MODECONTROLREG_GRAPH640       0x10   // 0 = 320x200 graphics, 1 = 640x200 graphics
#define CGA_MODECONTROLREG_BIT7BLINK      0x20   // 0 = text mode bit 7 controls background, 1 = text mode bit 7 controls blinking

#define CGA_COLORCONTROLREG_BACKCOLR_MASK 0x0f   // mask for 320x200 background color index (on 640x200 is the foreground)
#define CGA_COLORCONTROLREG_HIGHINTENSITY 0x10   // select high intensity colors
#define CGA_COLORCONTROLREG_PALETTESEL    0x20   // 0 is Green, red and brown, 1 is Cyan, magenta and white


// Hercules (HGC) Ports Bits

#define HGC_MODECONTROLREG_GRAPHICS       0x02   // 0 = text mode, 1 = graphics mode
#define HGC_MODECONTROLREG_ENABLED        0x08   // 0 = video off, 1 = video on
#define HGC_MODECONTROLREG_BIT7BLINK      0x20   // 0 = text mode bit 7 controls background, 1 = text mode bit 7 controls blinking
#define HGC_MODECONTROLREG_GRAPHICSPAGE   0x80   // 0 = graphics mapped on page 0 (0xB0000), 1 = graphics mapped on page 1 (0xB8000)

#define HGC_CONFSWITCH_ALLOWGRAPHICSMODE  0x01   // 0 = prevents graphics mode, 1 = allows graphics mode
#define HGC_CONFSWITCH_ALLOWPAGE1         0x02   // 0 = prevents access to page 1, 1 = allows access to page 1


// I/O expander (based on MCP23S17) ports

#define EXTIO_CONFIG                    0x00e0   // configuration port (see EXTIO_CONFIG_.... flags)
// whole 8 bit ports handling
#define EXTIO_DIRA                      0x00e1   // port A direction (0 = input, 1 = output)
#define EXTIO_DIRB                      0x00e2   // port B direction (0 = input, 1 = output)
#define EXTIO_PULLUPA                   0x00e3   // port A pullup enable (0 = disabled, 1 = enabled)
#define EXTIO_PULLUPB                   0x00e4   // port B pullup enable (0 = disabled, 1 = enabled)
#define EXTIO_PORTA                     0x00e5   // port A read/write
#define EXTIO_PORTB                     0x00e6   // port B read/write
// single GPIO handling
#define EXTIO_GPIOSEL                   0x00e7   // GPIO selection (0..7 = PA0..PA7, 8..15 = PB0..PB8)
#define EXTIO_GPIOCONF                  0x00e8   // selected GPIO direction and pullup (0 = input, 1 = output, 2 = input with pullup)
#define EXTIO_GPIO                      0x00e9   // selected GPIO read or write (0 = low, 1 = high)

// I/O expander configuration bits
#define EXTIO_CONFIG_AVAILABLE            0x01   // 1 = external IO available, 0 = not available
#define EXTIO_CONFIG_INT_POLARITY         0x02   // 1 = positive polarity, 0 = negative polarity (default)


#define SQ_REG_COUNT 5

//sound stuff
//#define TONE_PWM_CHANNEL 0
#define TONE_OUTPUT_PIN 25
//#define TONE_PWM_RESOLUTION 16

static unsigned char SQIndex = 0;
//static const unsigned char DefSQRegisters[SQ_REG_COUNT] =
//{
//  0x00, 0x01, 0x03, 0x00, 0x07
//};

static unsigned char SQRegisters[SQ_REG_COUNT] =
{
  0x00, 0x01, 0x03, 0x00, 0x07
};

uint8_t bgColor[16][16] = 
{
  // these 8 background colors must use white foreground color (text color) (foreground color = 0)
  {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}, //black *
  {0x55, 0xff, 0x55, 0xff, 0x55, 0xff, 0x55, 0xff, 0x55, 0xff, 0x55, 0xff, 0x55, 0xff, 0x55, 0xff}, //DARK BLUE *
  {0x77, 0xee, 0xdd, 0xbb, 0x77, 0xee, 0xdd, 0xbb, 0x77, 0xee, 0xdd, 0xbb, 0x77, 0xee, 0xdd, 0xbb}, //DARK GREEN
  {0xbb, 0xee, 0xbb, 0xee, 0xbb, 0xee, 0xbb, 0xee, 0xbb, 0xee, 0xbb, 0xee, 0xbb, 0xee, 0xbb, 0xee}, //DARK CYAN
  {0xee, 0x77, 0xbb, 0xdd, 0xee, 0x77, 0xbb, 0xdd, 0xee, 0x77, 0xbb, 0xdd, 0xee, 0x77, 0xbb, 0xdd}, //DARK RED
  {0x55, 0xff, 0xaa, 0xff, 0x55, 0xff, 0xaa, 0xff, 0x55, 0xff, 0xaa, 0xff, 0x55, 0xff, 0xaa, 0xff}, //DARK MAGENTA
  {0x77, 0xff, 0xdd, 0xff, 0x77, 0xff, 0xdd, 0xff, 0x77, 0xff, 0xdd, 0xff, 0x77, 0xff, 0xdd, 0xff}, //BROWN 
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, //DIM WHITE (not as bright as 0x15)

  // this bg color can use either white or black text color, not need to force modify.
  {0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa}, //GRAY *
  
  // these 8 background colors must use black foreground color (text color) (foreground color = 1)
  {0xaa, 0x00, 0xaa, 0x00, 0xaa, 0x00, 0xaa, 0x00, 0xaa, 0x00, 0xaa, 0x00, 0xaa, 0x00, 0xaa, 0x00}, //bright blue *
  {0x88, 0x11, 0x22, 0x44, 0x88, 0x11, 0x22, 0x44, 0x88, 0x11, 0x22, 0x44, 0x88, 0x11, 0x22, 0x44}, //bright green
  {0x44, 0x11, 0x44, 0x11, 0x44, 0x11, 0x44, 0x11, 0x44, 0x11, 0x44, 0x11, 0x44, 0x11, 0x44, 0x11}, //bright cyan
  {0x11, 0x88, 0x44, 0x22, 0x11, 0x88, 0x44, 0x22, 0x11, 0x88, 0x44, 0x22, 0x11, 0x88, 0x44, 0x22}, //bright red
  {0xaa, 0x00, 0x55, 0x00, 0xaa, 0x00, 0x55, 0x00, 0xaa, 0x00, 0x55, 0x00, 0xaa, 0x00, 0x55, 0x00}, //bright magenta
  {0x88, 0x00, 0x22, 0x00, 0x88, 0x00, 0x22, 0x00, 0x88, 0x00, 0x22, 0x00, 0x88, 0x00, 0x22, 0x00}, //yellow
  {0x88, 0x00, 0x00, 0x00, 0x88, 0x00, 0x00, 0x00, 0x88, 0x00, 0x00, 0x00, 0x88, 0x00, 0x00, 0x00}, //bright white *
};




// Timer0 Configuration Pointer (Handle)
//hw_timer_t *Timer0_Cfg = NULL;

// The Timer0 ISR Function (Executes Every Timer0 Interrupt Interval, for generating audio)
//void IRAM_ATTR Timer0_ISR() 
//{
//  static uint8_t pos = 1;       
//  digitalWrite(TONE_OUTPUT_PIN, pos = !pos);
//}
//////////////////////////////////////////////////////////////////////////////////////
// Machine


using fabgl::i8086;


uint8_t *         Machine::s_memory;
uint8_t *         Machine::s_videoMemory;
uint8_t *         Machine::MCGA_videoMemory;

uint32_t                 last_key_time = 0;
uint32_t                 last_last_key_time = 0;
int                      keyspressed = 0;
uint32_t                 last_fullscreen_refresh = 0;
//bool powersaveEnabled = true;
bool  need_fullscreen_refresh = false;
bool global_need_refresh = false;
bool start_refresh = true;
uint32_t                 last_screen_change_time = 0;
bool waiting_for_epd_clean = false;
bool waiting_for_initPartialUpdate = false;
bool screenDirty = true;
bool videoMemoryUpdated = true;
uint32_t lastVideoWrite = 0;
uint32_t lastVideoHandler = 0;
uint32_t lastDiskHandler = 0;
uint32_t lastHelpers = 0;
uint32_t lastVideoMemoryWrite = 0;
uint32_t lastVideoMemoryRead = 0;
bool need_cursor_update = false;
uint32_t sinceLastBorders = 0;
int                     cursor_pos = -1;
int                     last_cursor_pos = -1;
bool                    cursor_visible = false;
int                     cursor_start_scanline = 0;
int                     cursor_end_scanline = 0;
bool enableVideo = true;
//int smallX = HIRES_FONT_WIDTH * SCREEN80X25_COLS;
//int smallY = HIRES_FONT_HEIGHT * SCREEN80X25_ROWS;
int smallX = 640;
int smallY = 480;
int largeX = 0;
int largeY = 0;
uint8_t *fBuffer;
uint32_t changedPixels = 0;
uint32_t cumulativeChangedPixels = 0;
uint32_t lastChangedPixels = 0;
int CPT = 40000;
bool refreshNow = false;
uint32_t refreshNowTime = 0;
uint32_t lastDisplayFastTime = 0xffffffff;
uint32_t lastInstantRefreshTime = 0xffffffff;
uint32_t lastSetPixelMsgTime = 0;
uint32_t hlts = 0;
uint32_t mems = 0;
uint32_t vidmems = 0;
uint32_t memsarr[64];
uint32_t memsTotal = 0;
uint32_t memsIdx = 0;
uint32_t memsAvgArr[16];
uint32_t memsAvgTotal = 0;
uint32_t memsAvgIdx = 0;
uint32_t helpers = 0;
uint32_t zeroHelpers = 0;
uint32_t int2fs = 0;
uint32_t int28s = 0;
uint32_t readkeys = 0;
uint32_t read8042s = 0;
uint32_t stepsCount = 0;


uint8_t video_mode = CGA_80X25_GRAY;  // set to something impossible in order to force change at startup.  This way we get the wild characters screen on startup.
//int last_video_mode = CGA_80X25_GRAY;
uint8_t last_video_mode = 0xff; //non-existent video mode, so will force video mode change update
bool graphics_mode = false;
int font_width = HIRES_FONT_WIDTH;
int font_height = HIRES_FONT_HEIGHT;
int font_baseline = HIRES_FONT_BASELINE;

//foreground and background colors default values
uint16_t fg_color = DEFAULT_FG_COLOR;
uint16_t bg_color = DEFAULT_BG_COLOR;
uint8_t egaWriteMode = 0;

// custom Hercules resolutions:
uint16_t HGC_width = 720;
uint16_t HGC_height = 348;
int8_t HGC_scanlines = 4;

bool ethInterrupt = false;
void ethRecvISR();
void writeToEPD8(uint32_t addr, uint8_t value);
void setMCGA320x200pixel(int x, int y, uint8_t color, bool deb = false);
void set320x200pixel(int x, int y, uint8_t color);
void set640x200pixel(int x, int y, uint8_t color);
void set640x480pixel(int x, int y, uint8_t color);

static void video_task(void *context);
//static void adlib_task(void *context);

bool screenMutex = false;
uint8_t *bitTable;
uint8_t *iBoxMemory;
uint8_t *adlibMem;

TaskHandle_t videoTaskHandle;
//TaskHandle_t adlibTaskHandle;
TaskHandle_t m_taskHandle;

bool need_change_video_mode = false;


int speakerFreq = 0;
//bool speakerEnabled = false;
//GFXcanvas1 canvas(8, 14); // temporary 8x14 character editing canvas scratchspace


#if ENABLE_ETHERNET
//ethernet adapter ne2000 stuff:
// NE2000 Emulation
#include "src/ne2000/ne2000.h"
//#include "src/ne2000/pcap_helper.h"
#include <WiFi.h>
#include "esp_private/wifi.h"

#define MAC_ADDRESS_LEN 6
//uint8_t mac_addr[MAC_ADDRESS_LEN];
uint8_t mac_addr[MAC_ADDRESS_LEN]; // = {0x08, 0xf9, 0xe0, 0xf3, 0x5e, 0x40};  //should allow user to set this in ibox menu, (or choose to randomly generate?)  Or maybe we have to set it to the fixed hardware MAC address of the ESP32's wifi interface?
bool NE2000Enabled = true;
bool isMacProvided = true;   //set to false to randomly generate.  true if mac_addr is already defined with 6 bytes of mac address data.
NE2000State* NE2000Adapter;
unsigned char SelectedNetworkDeviceIndex = 0;

//HANDLE hThreadReceiveNetworkBytes = INVALID_HANDLE_VALUE;
TaskHandle_t hThreadReceiveNetworkBytes;
DWORD threadIDReceiveNetworkBytes = 0;

// in 16-bit mode, in ax, dx / out dx, ax will translate to 2 successive calls to ReadPort/WritePort
// we need to emulate the behaviour appropriately
uint16_t lastNE2000WordRead = 0;
unsigned char lastNE2000ByteWritten = 0;
static int NE2000IRQPending = 0; // for NE2000
#endif ENABLE_ETHERNET




//void IRAM_ATTR COM3Recv(void * context)
//{
//  auto m = (Machine*)context;
//  printf(".");
//  //auto ser = (FakeSerialPort*) m->m_COM3.getSerialPort();
//  //ser->m_rxCallback(ser->m_callbackArgs, 'A', true);
//  //ser->insertRX('A');
//  byte randomValue = random(0, 37);
//  char letter = randomValue + 'a';
//  if(randomValue > 26)
//    letter = (randomValue - 26) + '0';
//  m->getCOM3().getSerialPort()->insertRX(letter);
//}


#if ENABLE_ETHERNET

void NE2000ModeChangeHandler(bool isIn16Bit) {
}

void Machine::NE2000IRQHandler(void *context)
{
  auto m = (Machine*)context;
  //printf("inside NE2000IRQHandler\n");
  //NE2000IRQPending++;
  //m->m_PIC8259A.signalInterrupt(????);
  m->m_PIC8259A.signalInterrupt(7); // IRQ 7 on PIC8259A(master PIC) translates to int 0x0f (int 15)
  /*
  7 - 0x0f
  6 - 0x0e
  5 - 0x0d
  4 - 0x0c
  3 - 0x0b
  2 - 0x0a
  */
}

void NE2000SendHandler(unsigned char* data, uint16_t count)
{
  if (networkMode == NETWORKMODE_WIFI)
  {
    int res = esp_wifi_internal_tx(WIFI_IF_STA, data, count);
    if (res != ESP_OK) 
    {
      printf("WiFi send packet failed: %d\n", res);
    }
  }
  else if(networkMode == NETWORKMODE_ETH)
  {
    // first check if ethernet adapter is asleep.  If so, wake it up 
    if (ethSleep)
    {
      ethWakeUp();
//      printf("Waking up ETH...\n");    
//      ethSleep = false;
//      ethSendbufFree = true;
//      lastEthSendTime = millis();
//      lastEthRecvTime = millis();
    }

    
    // first check to make sure outgoing buffer is empty:
    if(ethSendbufFree)  // this is lazy.  should keep trying to send until ethSendBufFree = true, not just check once, otherwise packet might not be sent and system will never know.
    {
      Serial2.write(0x57); // command prefix
      Serial2.write(0xAB); // command prefix
      Serial2.write(0x30); // CMD_GET_INT_STATUS_SN
      Serial2.write(0x00); // socket number 0x00
      while(!Serial2.available())
      {
        delay(1);
      }
      uint8_t tmpval = Serial2.read();
      //printf("Before sending data, CMD_GET_INT_STATUS_SN received 0x%02X from ethernet port!\n", tmpval);
//    if ((tmpval & 0x01) == 0x01) // bit 1 of return value set to 1 indicates that send buffer is free
//    {
      //printf("count = %d\n", count);
      uint8_t byteCountLow = (uint8_t)(count & 0xFF);
      uint8_t byteCountHigh = (uint8_t)(count >> 8);;
      //printf("networkMode == NETWORKMODE_ETH in NE2000SendHandler()\n");
      //printf("byteCountLow = 0x%02X\n", byteCountLow);
      //printf("byteCountHigh = 0x%02X\n", byteCountHigh);
      Serial2.write(0x57);
      Serial2.write(0xAB);
      Serial2.write(0x39); // CMD_WRITE_SEND_BUF_SN
      Serial2.write(0x00); // Socket 0x00
      Serial2.write(byteCountLow); // data length low byte
      Serial2.write(byteCountHigh); // data length high byte
      for(int n = 0; n < count; n++)
      {
        Serial2.write((uint8_t)(data[n]));
      }
      Serial2.flush();
      ethSendbufFree = false;


      Serial2.write(0x57); // command prefix
      Serial2.write(0xAB); // command prefix
      Serial2.write(0x30); // CMD_GET_INT_STATUS_SN
      Serial2.write(0x00); // socket number 0x00
      while(!Serial2.available())
      {
        delay(1);
      }
      uint8_t tmpval2 = Serial2.read();
      //printf("After sending data, CMD_GET_INT_STATUS_SN received 0x%02X from ethernet port!\n", tmpval2);
      if ((tmpval2 & 0x01) == 0x01) // bit 1 of return value set to 1 indicates that send buffer is free
      {      
        //printf("sendBuf free immediately after sending\n");
        ethSendbufFree = true;
      }     
    }
    lastEthSendTime = millis(); 
//    }
    
//    Serial2.write(0x57); // command prefix
//    Serial2.write(0xAB); // command prefix
//    Serial2.write(0x30); // CMD_GET_INT_STATUS_SN
//    Serial2.write(0x00); // socket number 0x00
//    while(!Serial2.available())
//    {
//      delay(1);
//    }
//    uint8_t tmpval = Serial2.read();
//    printf("After sending data, CMD_GET_INT_STATUS_SN received 0x%02X from ethernet port!\n", tmpval);

//    if(tmpval != 0x00)
//    {
//      printf("CMD_GET_INT_STATUS_SN received 0x%02X from ethernet port!\n", tmpval);
//      printf("0x%02X & 0x04 = 0x%02X\n", tmpval, tmpval & 0x04);
//    }
    
//    if ((tmpval & 0x04) == 0x04) // bit 3 of return value set to 1 indicates there is unread data from the network waiting to be read
//    {
//      //printf("CMD_GET_INT_STATUS_SN received 0x%02X from ethernet port!\n", tmpval); 
//      // send command to retrieve number of bytes available
//      Serial2.write(0x57); // command prefix
//      Serial2.write(0xAB); // command prefix
//      Serial2.write(0x3B); // CMD_GET_RECV_LEN_SN
//      Serial2.write(0x00); // socket number 0x00
//      while(!Serial2.available())
//      {
//        delay(1);
//      }
//      uint8_t dataLenLow = Serial2.read();
//      uint8_t dataLenHigh = Serial2.read();
//      uint16_t numBytes = dataLenHigh;
//      numBytes = numBytes << 8;
//      numBytes += dataLenLow;
//      if(numBytes > 1500)
//      {
//        numBytes = 1500;
//        dataLenHigh = 0x05;
//        dataLenLow = 0xDC;
//      }      
//      printf("%d bytes (%d %d) available to read from ethernet port\n", numBytes, dataLenLow, dataLenHigh);
//
//      // send command to receive bytes waiting in ethernet port incoming buffer
//      Serial2.write(0x57); // command prefix
//      Serial2.write(0xAB); // command prefix
//      Serial2.write(0x3C); // CMD_READ_RECV_BUF_SN
//      Serial2.write(0x00); // socket number 0x00
//      Serial2.write(dataLenLow); // data length low byte
//      Serial2.write(dataLenHigh); // data length high byte
//      while(!Serial2.available())
//      {
//        delay(1);
//      }
//      
//      int counts = 0;
//      for(int n = 0; n < numBytes; n++)
//      {
//        int count = 0;
//        if(!Serial2.available())
//        {
//          counts++;
//          if (counts > 1000)
//          {
//            break;
//          }
//          delay(1);
//        }
//        ethBuf[n] = Serial2.read();
//        if (n % 8 == 0)
//        {
//          printf("  ");
//        }
//        if (n % 16 == 0)
//        {
//          printf("\n");
//        }
//        printf("%02X ", ethBuf[n]);
//      }
//      printf("\n");
//      
//      if (NE2000Adapter)
//      {
//        ne2000_receive(NE2000Adapter, ethBuf, numBytes);
//      }
//    }
  }
}

void PCapDataReceive(const unsigned char* data, unsigned int count)
{
  if (NE2000Adapter)
  {
    ne2000_receive(NE2000Adapter, data, count);
  }
}

void Machine::prepareNetworkSetup() //void *context) 
{
  //auto m = (Machine*)context;
  if (networkMode == NETWORKMODE_WIFI)
  {
    esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, mac_addr);
  }
  else if (networkMode == NETWORKMODE_ETH)
  {
    memcpy(mac_addr, ethMacAddr, 6);
    printf("ethMacAddr = mac_addr = ");
    for(int n = 0; n < 6; n++)
    {
      printf("0x%02X ", mac_addr[n]);
    }
    printf("\n");
  }
  NE2000Adapter = new NE2000State();   // this line causes "abort() was called at PC 0x402269bb on core 1".  This is because an instance of NE2000State() tries to allocate 49152 bytes for its "mem" array.  Should be able to fix by using PSRAM.
  NE2000Adapter->mem = (uint8_t*)(SOC_EXTRAM_DATA_LOW + 2 * 1024 * 1024) + RAM_SIZE + 65536 + 86690 + 65536;
  if (!isMacProvided)
  {
    printf("Using randomized MAC address\n");
    NE2000Adapter->macaddr[0] = 0x00;
    srand(millis());
    for (int i = 0; i < 6; i++)
    {
      NE2000Adapter->macaddr[i] = rand() % 256; // beteen 00 and FF
    }

    // https://www.geeksforgeeks.org/introduction-of-mac-address-in-computer-network/
    // for byte 0 of Mac address
    // bit 0: 0 = unicast, 1 = nulticast
    // bit 1: 0 = globally unique, 1 = locally administrator
    // We set to 00 (unicast, locally admin)
    NE2000Adapter->macaddr[0] = NE2000Adapter->macaddr[0] & 0b11111100;
  }
  else 
  {
    memcpy(NE2000Adapter->macaddr, mac_addr, MAC_ADDRESS_LEN);
  }

  printf("NE2000 MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
  NE2000Adapter->macaddr[0],
  NE2000Adapter->macaddr[1],
  NE2000Adapter->macaddr[2],
  NE2000Adapter->macaddr[3],
  NE2000Adapter->macaddr[4],
  NE2000Adapter->macaddr[5]);
  
  ne2000_reset(NE2000Adapter);
  NE2000SetIRQHandler(this, NE2000IRQHandler);
  NE2000SetSendHandler(NE2000SendHandler);
  NE2000SetModeChangeHandler(NE2000ModeChangeHandler);
}

#endif // ENABLE_ETHERNET


void Machine::hibernate()
{
  /* To do:
   *  1. Cursor position and shape
   *  2. When using LSHIFT+ALT+PRINTSCRN to hibernate, LSHIFT is still "depressed" after unhibernate until key is pressed and unpressed again.
   *  3. Hibernate autoamatically after idle time (5 min?), when rebooting default to unhibernate if .hib file exists
   *  4. Allow cancelling unhibernate by pressing ESC before unhibernate.  If cancelled, delete .hib file.
   *  5. After unhibernate, delete .hib file to prevent possible new unhibernate on old .hib file which could cause disk corruption.
   *  6. Unhibernate should do some kind of size and CRC check on .hib file to make sure it is complete and correct, using corrupt file might cause disk corruption.
   *  7. resetKeyboard() at end of unhibernate() is a temporary bandaid and should be fixed.  This might break some programs which have re-programmed the i8042
  
   */
  
  printf(String("writing /" + String(machine_idx) + ".hib\n").c_str());
  constexpr int BLOCKLEN = 1024;
  auto file = FileBrowser(m_baseDir).openFile(String("/" + String(machine_idx) + ".hib").c_str(), "wb");
  if (file) 
  {
    printf("File created successfully\n");
    display.fillRect(230, 180, 164, 104, GxEPD_WHITE);
    screenMutex = true;
    display.fillRect(250, 200, 124, 64, GxEPD_BLACK);
    display.setCursor(268, 224);
    display.setTextColor(GxEPD_WHITE);
    display.print("Hibernating");
    display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT);   
    PIC_display_Clear();
    display.displayStayOn(false);
    
    resetKeyboard();  //can this fix "shift key is stuck on after shift+alt+printscreen hibernate" problem?

    fseek(file, 0, SEEK_SET);

    //RAM
    for (int i = 0; i < RAM_SIZE; i += BLOCKLEN)
    {
      fwrite(s_memory + i, 1, BLOCKLEN, file);
    }

    //VIDEO RAM
    for (int i = 0; i < VIDEOMEMSIZE; i += BLOCKLEN)
    {
      fwrite(s_videoMemory + i, 1, BLOCKLEN, file);
    }

    //CPU
    fwrite(i8086::get_regs(), 1, 48, file); // save CPU registers (48 bytes)
    fwrite(i8086::get_flags(), 1, 10, file); // save CPU flags (10 bytes)
    fwrite(i8086::get_reg_ip(), 1, 2, file); // save CPU IP (instruction pointer) (2 bytes)
    fwrite(i8086::get_seg_override_en(), 1, 1, file); // save CPU's seg_override_en (1 byte)
    fwrite(i8086::get_rep_override_en(), 1, 1, file); // save CPU's rep_override_en (1 byte)

    //VIDEO
    fwrite(&video_mode, 1, 1, file); // save current video mode (1 byte)
    fwrite(&graphics_mode, 1, 1, file); // save graphics mode boolean flag (1 byte)    
    fwrite(&fBuffer, 1, 4, file); // save fBuffer pointer (4 bytes)
    fwrite(&m_frameBuffer, 1, 4, file); // save m_frameBuffer pointer (4 bytes)
    fwrite(&egaWriteMode, 1, 1, file); // save egaWriteMode (1 byte)
    fwrite(&m_CGAMemoryOffset, 1, 2, file); // save m_CGAMemoryOffset (2 bytes)
    fwrite(&m_MCGAMemoryOffset, 1, 2, file); // save m_MCGAMemoryOffset (2 bytes)
    fwrite(&m_HGCMemoryOffset, 1, 2, file); // save m_HGCMemoryOffset (2 bytes)
    fwrite(SQRegisters, 1, 5, file); // save SQRegisters array (5 bytes)

    fwrite(&m_CGA6845SelectRegister, 1, 1, file); // save m_CGA6845SelectRegister (1 bytes)
    fwrite(&m_MCGASelectRegister, 1, 1, file); // save m_MCGASelectRegister (1 bytes)
    fwrite(m_CGA6845, 1, 18, file); // save m_CGA6845 array (18 bytes)
    fwrite(m_MCGA, 1, 18, file); // save m_MCGA array (18 bytes)
    fwrite(&m_CGAModeReg, 1, 1, file); // save m_CGAModeReg (1 bytes)
    fwrite(&m_CGAColorReg, 1, 1, file); // save m_CGAColorReg (1 bytes)
    fwrite(&m_CGAVSyncQuery, 1, 2, file); // save m_CGAVSyncQuery (2 bytes)
    fwrite(&MCGA_videoMemory, 1, 4, file); // save MCGA_videoMemory pointer (4 bytes)
    fwrite(&SQIndex, 1, 1, file); // save SQIndex pointer (1 byte)
    
    fwrite(&m_HGC6845SelectRegister, 1, 1, file); // save m_HGC6845SelectRegister (1 bytes)
    fwrite(&m_HGC6845SelectRegister, 1, 1, file); // save m_HGC6845SelectRegister (1 bytes)
    fwrite(m_HGC6845, 1, 18, file); // save m_HGC6845 array (18 bytes)
    fwrite(&m_HGCModeReg, 1, 1, file); // save m_HGCModeReg (1 bytes)
    fwrite(&m_HGCSwitchReg, 1, 1, file); // save m_HGCSwitchReg (1 bytes)
    fwrite(&m_HGCVSyncQuery, 1, 2, file); // save m_HGCVSyncQuery (2 bytes)

    //PIT8253 TIMER 
    fwrite(m_PIT8253.get_m_timer(), 1, 90, file); // save PIT8235 timer settings/status (should be 30 * 3 bytes?)
    fwrite(m_PIT8253.get_m_lastTickTime(), 1, 4, file); // save PIT8235 m_lastTickTime (4 bytes)
    fwrite(m_PIT8253.get_m_acc(), 1, 4, file); // save PIT8235 m_acc (4 bytes)

    //i8042 keyboard controller
    fwrite(m_i8042.get_m_STATUS(), 1, 1, file); // save m_i8042 m_STATUS (1 bytes)
    fwrite(m_i8042.get_m_DBBOUT(), 1, 1, file); // save m_i8042 m_DBBOUT (1 bytes)
    fwrite(m_i8042.get_m_DBBIN(), 1, 1, file); // save m_i8042 m_DBBIN (1 bytes)
    fwrite(m_i8042.get_m_commandByte(), 1, 1, file); // save m_i8042 m_commandByte (1 bytes)
    fwrite(m_i8042.get_m_writeToMouse(), 1, 1, file); // save m_i8042 m_writeToMouse (1 bytes)

    //PIC8259A (m_PIC8259A)
    fwrite(m_PIC8259A.get_m_state(), 1, 1, file); // save m_PIC8259A m_state (1 bytes)
    fwrite(m_PIC8259A.get_m_baseVector(), 1, 1, file); // save m_PIC8259A m_baseVector (1 bytes)
    fwrite(m_PIC8259A.get_m_autoEOI(), 1, 1, file); // save m_PIC8259A m_autoEOI (1 bytes)
    fwrite(m_PIC8259A.get_m_IRR(), 1, 1, file); // save m_PIC8259A m_IRR (1 bytes)
    fwrite(m_PIC8259A.get_m_ISR(), 1, 1, file); // save m_PIC8259A m_ISR (1 bytes)
    fwrite(m_PIC8259A.get_m_IMR(), 1, 1, file); // save m_PIC8259A m_IMR (1 bytes)
    fwrite(m_PIC8259A.get_m_readISR(), 1, 1, file); // save m_PIC8259A m_readISR (1 bytes)
    fwrite(m_PIC8259A.get_m_pendingInterrupt(), 1, 1, file); // save m_PIC8259A m_pendingInterrupt (1 bytes)
    fwrite(m_PIC8259A.get_m_pendingIR(), 1, 1, file); // save m_PIC8259A m_pendingIR (1 bytes)

    //PIC8259B (m_PIC8259B)
    fwrite(m_PIC8259B.get_m_state(), 1, 1, file); // save m_PIC8259B m_state (1 bytes)
    fwrite(m_PIC8259B.get_m_baseVector(), 1, 1, file); // save m_PIC8259B m_baseVector (1 bytes)
    fwrite(m_PIC8259B.get_m_autoEOI(), 1, 1, file); // save m_PIC8259B m_autoEOI (1 bytes)
    fwrite(m_PIC8259B.get_m_IRR(), 1, 1, file); // save m_PIC8259B m_IRR (1 bytes)
    fwrite(m_PIC8259B.get_m_ISR(), 1, 1, file); // save m_PIC8259B m_ISR (1 bytes)
    fwrite(m_PIC8259B.get_m_IMR(), 1, 1, file); // save m_PIC8259B m_IMR (1 bytes)
    fwrite(m_PIC8259B.get_m_readISR(), 1, 1, file); // save m_PIC8259B m_readISR (1 bytes)
    fwrite(m_PIC8259B.get_m_pendingInterrupt(), 1, 1, file); // save m_PIC8259B m_pendingInterrupt (1 bytes)
    fwrite(m_PIC8259B.get_m_pendingIR(), 1, 1, file); // save m_PIC8259B m_pendingIR (1 bytes)  

    // display foreground and background color status
    fwrite(&bg_color, 1, 2, file);
    fwrite(&fg_color, 1, 2, file);

    //should be total of 1114171(wrong) bytes file size
    fclose(file);
  }
  else
  {
    printf("file creation failed\n");
  }
  printDumpInfo();
//  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
//  //esp_sleep_enable_gpio_wakeup();
//  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
//  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
//  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
//  esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF);
//  //esp_sleep_pd_config(ESP_PD_DOMAIN_RC_FAST, ESP_PD_OPTION_OFF);
//  //esp_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO, ESP_PD_OPTION_OFF);
//  //esp_sleep_pd_config(ESP_PD_DOMAIN_MODEM, ESP_PD_OPTION_OFF);
//  esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);


  
  display.fillRect(230, 180, 164, 104, GxEPD_WHITE);
  screenMutex = true;
  display.fillRect(250, 200, 124, 64, GxEPD_BLACK);
  display.setCursor(268, 224);
  display.setTextColor(GxEPD_WHITE);
  display.print("Hibernated");
  display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT);   
  delay(1000);
  //PIC_display_Clear();
  //display.displayStayOn(false);
  while(epd_busy());
  display.powerOff();
  delay(50);
  pinMode(21, OUTPUT);
  digitalWrite(21, LOW);
  delay(2000);
  // if hardware power off succeeds, below code should never execute
  printf("AUTO POWER OFF FAILED!  DEEP SLEEPING INSTEAD!\n");
  display.fillRect(200, 168, 240, 150, GxEPD_WHITE);
  display.drawRect(200, 168, 240, 150, GxEPD_BLACK);
  display.fillRect(220, 188, 200, 110, GxEPD_BLACK);
  display.setCursor(238, 207);
  display.setTextColor(GxEPD_WHITE);
  display.print("Power off failed!");
  display.setCursor(238, 222);
  display.print("Deep sleep instead!");
  display.setCursor(238, 237);
  display.print("PLEASE MANUALLY TURN");
  display.setCursor(238, 253);
  display.print("OFF POWER SWITCH!");
  display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT);   
  PIC_display_Clear();
  display.displayStayOn(false);
  while(epd_busy());
  display.powerOff();
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
  esp_deep_sleep_start();


  
  //m_i8042.disableKeyboard();   // this currently causes "Guru Meditation Error: Core  1 panic'ed (IllegalInstruction)."
  printf("done\nentering deep sleep...\n");
  delay(1000); 
  esp_deep_sleep_start();
}

bool Machine::checkHibernateFile()
{
  printf(String("checking for /" + String(machine_idx) + ".hib\n").c_str());
  auto file = FileBrowser(m_baseDir).openFile(String("/" + String(machine_idx) + ".hib").c_str(), "r");
  if (file)
  {
    fclose(file);
    return true;  
  }
  fclose(file);
  return false;
}

void Machine::unhibernate(bool need_reset)
{
  printf(String("reading /" + String(machine_idx) + ".hib\n").c_str());
  auto file = FileBrowser(m_baseDir).openFile(String("/" + String(machine_idx) + ".hib").c_str(), "r");
  if (file) 
  {
    printf("File opened successfully\n");
    if (need_reset)
    {
      reset();  
    }
    
    fseek(file, 0, SEEK_SET);

    //RAM
    constexpr int BLOCKLEN = 1024;
    for (int i = 0; i < RAM_SIZE; i += BLOCKLEN)
    {
      fread(s_memory + i, 1, BLOCKLEN, file);
    }

    //VIDEO RAM
    for (int i = 0; i < VIDEOMEMSIZE; i += BLOCKLEN)
    {
      fread(s_videoMemory + i, 1, BLOCKLEN, file);
    }

    //CPU
    fread(i8086::get_regs(), 1, 48, file); // save CPU registers (48 bytes)
    fread(i8086::get_flags(), 1, 10, file); // save CPU flags (10 bytes)
    fread(i8086::get_reg_ip(), 1, 2, file); // save CPU IP (instruction pointer) (2 bytes)
    fread(i8086::get_seg_override_en(), 1, 1, file); // save CPU's seg_override_en (1 byte)
    fread(i8086::get_rep_override_en(), 1, 1, file); // save CPU's rep_override_en (1 byte)

    //VIDEO
    fread(&video_mode, 1, 1, file); // save current video mode (1 byte)
    fread(&graphics_mode, 1, 1, file); // save graphics mode boolean flag (1 byte)
    fread(&fBuffer, 1, 4, file); // save fBuffer pointer (4 bytes)
    fread(&m_frameBuffer, 1, 4, file); // save m_frameBuffer pointer (4 bytes)
    fread(&egaWriteMode, 1, 1, file); // save egaWriteMode pointer (1 byte)
    fread(&m_CGAMemoryOffset, 1, 2, file); // save m_CGAMemoryOffset pointer (2 bytes)
    fread(&m_MCGAMemoryOffset, 1, 2, file); // save m_MCGAMemoryOffset pointer (2 bytes)
    fread(&m_HGCMemoryOffset, 1, 2, file); // save m_HGCMemoryOffset pointer (2 bytes)
    fread(SQRegisters, 1, 5, file); // save SQRegisters array (5 bytes)
    
    fread(&m_CGA6845SelectRegister, 1, 1, file); // save m_CGA6845SelectRegister (1 bytes)
    fread(&m_MCGASelectRegister, 1, 1, file); // save m_MCGASelectRegister (1 bytes)
    fread(m_CGA6845, 1, 18, file); // save m_CGA6845 array (18 bytes)
    fread(m_MCGA, 1, 18, file); // save m_MCGA array (18 bytes)
    fread(&m_CGAModeReg, 1, 1, file); // save m_CGAModeReg (1 bytes)
    fread(&m_CGAColorReg, 1, 1, file); // save m_CGAColorReg (1 bytes)
    fread(&m_CGAVSyncQuery, 1, 2, file); // save m_CGAVSyncQuery (2 bytes)
    fread(&MCGA_videoMemory, 1, 4, file); // save MCGA_videoMemory pointer (4 bytes)
    fread(&SQIndex, 1, 1, file); // save SQIndex pointer (1 byte)
    
    fread(&m_HGC6845SelectRegister, 1, 1, file); // save m_HGC6845SelectRegister (1 bytes)
    fread(&m_HGC6845SelectRegister, 1, 1, file); // save m_HGC6845SelectRegister (1 bytes)
    fread(m_HGC6845, 1, 18, file); // save m_HGC6845 array (18 bytes)
    fread(&m_HGCModeReg, 1, 1, file); // save m_HGCModeReg (1 bytes)
    fread(&m_HGCSwitchReg, 1, 1, file); // save m_HGCSwitchReg (1 bytes)
    fread(&m_HGCVSyncQuery, 1, 2, file); // save m_HGCVSyncQuery (2 bytes)

    // PIT8253 TIMER
    fread(m_PIT8253.get_m_timer(), 1, 90, file); // save PIT8235 timer settings/status (should be 30 * 3 bytes?)
    fread(m_PIT8253.get_m_lastTickTime(), 1, 4, file); // save PIT8235 m_lastTickTime (4 bytes)
    fread(m_PIT8253.get_m_acc(), 1, 4, file); // save PIT8235 m_acc (4 bytes)

    //i8042 keyboard controller
    fread(m_i8042.get_m_STATUS(), 1, 1, file); // save m_i8042 m_STATUS (1 bytes)
    fread(m_i8042.get_m_DBBOUT(), 1, 1, file); // save m_i8042 m_DBBOUT (1 bytes)
    fread(m_i8042.get_m_DBBIN(), 1, 1, file); // save m_i8042 m_DBBIN (1 bytes)
    fread(m_i8042.get_m_commandByte(), 1, 1, file); // save m_i8042 m_commandByte (1 bytes)
    fread(m_i8042.get_m_writeToMouse(), 1, 1, file); // save m_i8042 m_writeToMouse (1 bytes)

    //PIC8259A (m_PIC8259A) interrupt controller
    fread(m_PIC8259A.get_m_state(), 1, 1, file); // save m_PIC8259A m_state (1 bytes)
    fread(m_PIC8259A.get_m_baseVector(), 1, 1, file); // save m_PIC8259A m_baseVector (1 bytes)
    fread(m_PIC8259A.get_m_autoEOI(), 1, 1, file); // save m_PIC8259A m_autoEOI (1 bytes)
    fread(m_PIC8259A.get_m_IRR(), 1, 1, file); // save m_PIC8259A m_IRR (1 bytes)
    fread(m_PIC8259A.get_m_ISR(), 1, 1, file); // save m_PIC8259A m_ISR (1 bytes)
    fread(m_PIC8259A.get_m_IMR(), 1, 1, file); // save m_PIC8259A m_IMR (1 bytes)
    fread(m_PIC8259A.get_m_readISR(), 1, 1, file); // save m_PIC8259A m_readISR (1 bytes)
    fread(m_PIC8259A.get_m_pendingInterrupt(), 1, 1, file); // save m_PIC8259A m_pendingInterrupt (1 bytes)
    fread(m_PIC8259A.get_m_pendingIR(), 1, 1, file); // save m_PIC8259A m_pendingIR (1 bytes)

    //PIC8259B (m_PIC8259B) interrupt controller
    fread(m_PIC8259B.get_m_state(), 1, 1, file); // save m_PIC8259B m_state (1 bytes)
    fread(m_PIC8259B.get_m_baseVector(), 1, 1, file); // save m_PIC8259B m_baseVector (1 bytes)
    fread(m_PIC8259B.get_m_autoEOI(), 1, 1, file); // save m_PIC8259B m_autoEOI (1 bytes)
    fread(m_PIC8259B.get_m_IRR(), 1, 1, file); // save m_PIC8259B m_IRR (1 bytes)
    fread(m_PIC8259B.get_m_ISR(), 1, 1, file); // save m_PIC8259B m_ISR (1 bytes)
    fread(m_PIC8259B.get_m_IMR(), 1, 1, file); // save m_PIC8259B m_IMR (1 bytes)
    fread(m_PIC8259B.get_m_readISR(), 1, 1, file); // save m_PIC8259B m_readISR (1 bytes)
    fread(m_PIC8259B.get_m_pendingInterrupt(), 1, 1, file); // save m_PIC8259B m_pendingInterrupt (1 bytes)
    fread(m_PIC8259B.get_m_pendingIR(), 1, 1, file); // save m_PIC8259B m_pendingIR (1 bytes)   

    // display foreground and background color status
    fread(&bg_color, 1, 2, file);
    fread(&fg_color, 1, 2, file);

    
    //should be total of 1114171(wrong) bytes file size
    fclose(file);
    printf("Removing /%d.hib\n", machine_idx);
    FileBrowser(m_baseDir).remove(String("/" + String(machine_idx) + ".hib").c_str());
    printDumpInfo();
    m_sysReqCallback2();  // full screen refresh
//    screenDirty = true; //redrawScreen();
//    screenDirty = true; //display.display(false);
    need_change_video_mode = true;
    resetKeyboard(); // this is a temporary bandaid and should be fixed.  This might break some programs which have re-programmed the i8042
  }
  else
  {
    printf("file open failed\n");
    return;
  }
  printf("done\n");
  m_reset = false;
}

void Machine::powerOff()
{
  printf("Powering off power circuit because 0xff was written to port 0x22bc!\n");
  delay(1);
  while(epd_busy());
  display.powerOff();
  delay(10);
  pinMode(21, OUTPUT);
  digitalWrite(21, LOW);
  delay(2000);   //should be powered off within a millisecond, and this command should never finish, but lets give it 2 seconds anyway.  After this, if power is still on, at least hibernate to save some power.
  printf("AUTO POWER OFF FAILED!  DEEP SLEEPING INSTEAD!\n");
  display.fillRect(200, 168, 240, 150, GxEPD_WHITE);
  display.drawRect(200, 168, 240, 150, GxEPD_BLACK);
  display.fillRect(220, 188, 200, 110, GxEPD_BLACK);
  display.setCursor(238, 207);
  display.setTextColor(GxEPD_WHITE);
  display.print("Power off failed!");
  display.setCursor(238, 222);
  display.print("Deep sleep instead!");
  display.setCursor(238, 237);
  display.print("PLEASE MANUALLY TURN");
  display.setCursor(238, 253);
  display.print("OFF POWER SWITCH!");
  display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT);   
  PIC_display_Clear();
  display.displayStayOn(false);
  while(epd_busy());
  display.powerOff();
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
  esp_deep_sleep_start();
}

Machine::Machine() :
  //powersaveEnabled(PWR_SAVE_HALF),
  //powersaveEnabled(PWR_SAVE_OFF),
  last_checkscreen_time(0),
  m_diskFilename(),
  m_diskChanged(),
  m_disk(),
  m_frameBuffer(nullptr),
  //last_frameBuffer(nullptr),
  //char_refreshes(nullptr),
  m_bootDrive(0),
  m_sysReqCallback(nullptr),
  m_sysReqCallback2(nullptr),
  m_sysReqCallback3(nullptr),
  m_sysReqCallback4(nullptr),
  m_baseDir(nullptr)
{
}


Machine::~Machine()
{
  for (int i = 0; i < DISKCOUNT; ++i) {
    free(m_diskFilename[i]);
    if (m_disk[i])
      fclose(m_disk[i]);
  }
  vTaskDelete(m_taskHandle);
  free(s_videoMemory);
}

void Machine::init()
{
//
  printf("Machine::init() running on core ");
  printf("%d\n", xPortGetCoreID());



#if ENABLE_CH376
  // Serial1 for USB flash drive module
  pinMode(4, INPUT);
  //Serial1.setRxBufferSize(10000);
  Serial1.begin(9600, SERIAL_8N1, 39, 0);
  serial1Mode = 0;
  usbSpeed = 9600;
  // should try "pinging" ch375, if get an incorrect response, switch to 1000000 baud and try again before giving up, since ch375 will still be set to 1000000 baud after a reset, unless power is interrupted
  delay(10);
  Serial1.write(0x06);
  delay(10);
  Serial1.write(0xAA);
  delay(10);
  uint8_t res = Serial1.read();
  printf("Serial1.read() returned 0x%02x\n", res);
  if (res != 0x55)
  {
    printf("ch375 not responding properly at 9600 baud, switching to 1000000 baud\n");
    Serial1.begin(1000000, SERIAL_8N1, 39, 0);
    usbSpeed = 1000000;
  }
//  delay(10);
#endif  // ENABLE_CH376


//  printf("line %d\n", __LINE__);
//  pinMode(TONE_OUTPUT_PIN, OUTPUT);
    pinMode(25, OUTPUT);   // should remove this?
//  // Configure Timer0 Interrupt
//  Timer0_Cfg = timerBegin(0, 80, true);
//  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
//  printf("line %d\n", __LINE__);
  


  
  srand((uint32_t)time(NULL));
  //powersaveEnabled = PWR_SAVE_HALF;
  //powersaveEnabled = PWR_SAVE_OFF;
  for (int n = 0; n < 63; n++)
  {
    memsarr[n] = 0;
    if (n < 16)
    {
      memsAvgArr[n] = 0;
    }
  }
  // to avoid PSRAM bug without -mfix-esp32-psram-cache-issue
  // core 0 can only work reliably with the lower 2 MB and core 1 only with the higher 2 MB.
  //printf("SOC_EXTRAM_DATA_LOW = 0x%08x\n", SOC_EXTRAM_DATA_LOW);
  s_memory = (uint8_t*)(SOC_EXTRAM_DATA_LOW + (xPortGetCoreID() == 1 ? 2 * 1024 * 1024 : 0));  //we run on core 1, so start at offset 2 * 1024 * 1024
  printf("s_memory = %p\n", s_memory);  //should be zero since we are using core 0
  //s_videoMemory = (uint8_t*)heap_caps_malloc(VIDEOMEMSIZE, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
  //s_videoMemory = s_memory + 2000000;
  // iBoxMemory is at offset (2 * 1024 * 1024) + RAM_SIZE + 65536;
  s_videoMemory = (uint8_t*)(SOC_EXTRAM_DATA_LOW + 2 * 1024 * 1024) + /* IBM XT's RAM */ RAM_SIZE + /* bitTable */ 65536 + /* iBoxMemory + ADLIBMEMORY */ 86690;
  //printf("(m)allocated %d bytes of for s_videoMemory at %p\n", VIDEOMEMSIZE, s_videoMemory);
  printf("s_videoMemory at %p\n", s_videoMemory);
  //s_videoMemory = (uint8_t*)malloc(4000);  //(VIDEOMEMSIZE)
  //memset(s_videoMemory, 0, VIDEOMEMSIZE);
  memset(s_memory, 0, RAM_SIZE);

  //last_frameBuffer = (uint8_t *)malloc(16000);
  //last_frameBuffer = (uint8_t*)heap_caps_malloc(16000, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
  //last_frameBuffer = (uint8_t*)(SOC_EXTRAM_DATA_LOW + (xPortGetCoreID() == 1 ? (2 * 1024 * 1024) + (1024 * 1024) : 0  + (1024 * 1024)));
  //MCGA_videoMemory = (uint8_t*)(SOC_EXTRAM_DATA_LOW + (xPortGetCoreID() == 1 ? (2 * 1024 * 1024) + (1024 * 1024) : 0  + (1024 * 1024)));
  MCGA_videoMemory = s_videoMemory;
  
  //printf("allocated last_framebuffer in PSRAM at %p\n", last_frameBuffer);
  //printf("allocated 16000 bytes for last_framebuffer at %p\n", last_frameBuffer);
  //char_refreshes = (uint8_t *)malloc(2000);
  //cursor_pos = -1;
  //last_cursor_pos = -1;
  //cursor_visible = false;
  //cursor_start_scanline = 0;
  //cursor_end_scanline = 0;
  
#if ENABLE_SOUNDGEN  
  m_soundGen.play(true);
#if ENABLE_ADLIB  
  //printf("adlibMem = %p\n", adlibMem);
  adlibMem = iBoxMemory + 45729;
  //printf("adlibMem = %p\n", adlibMem);
  m_adlibSoundGenerator.setAdlibMemory(adlibMem);
  //printf("sound volume = %d\n", m_soundGen.volume());
  m_adlibSoundGenerator.init();
  m_adlibSoundGenerator.setVolume(127);
  //printf("&m_adlibSoundGenerator = %p\n", &m_adlibSoundGenerator); 
  //m_soundGen.attach(&m_adlibSoundGenerator);
  //m_adlibSoundGenerator.enable(true);
#endif //ENABLE_ADLIB

#if ENABLE_MIDI  
  m_midiSoundGenerator.init();
  m_midiSoundGenerator.setVolume(127);
  //printf("&m_midiSoundGenerator = %p\n", &m_midiSoundGenerator); 
  //m_soundGen.attach(&m_midiSoundGenerator);
  //m_midiSoundGenerator.enable(true);
#endif //ENABLE_MIDI

#if ENABLE_DISNEY 
  m_disneySoundGenerator.init();
  m_disneySoundGenerator.setVolume(127);
  //printf("&m_disneySoundGenerator = %p\n", &m_disneySoundGenerator); 
  //m_soundGen.attach(&m_disneySoundGenerator);
  //m_disneySoundGenerator.enable(true);
#endif //ENABLE_DISNEY

#if ENABLE_COVOX
  m_covoxSoundGenerator.init();
  m_covoxSoundGenerator.setVolume(127);
  //printf("&m_covoxSoundGenerator = %p\n", &m_covoxSoundGenerator); 
  //m_covoxGen.attach(&m_covoxSoundGenerator);
  //m_covoxSoundGenerator.enable(true);
#endif //ENABLE_COVOX

#if ENABLE_PC_SPEAKER
  m_squareWaveGen.setVolume(127);
  //m_squareWaveGen.setDutyCycle(127);
  m_soundGen.attach(&m_squareWaveGen);
  m_squareWaveGen.enable(true);
#endif //ENABLE_PC_SPEAKER

  m_soundGen.setVolume(127);
#endif // ENABLE_SOUNDGEN

#if ENABLE_ETHERNET
  if(networkMode == NETWORKMODE_ETH)
  {
    // reset ch395 via RSTI pin on GPIO23
    pinMode(23, OUTPUT);
    digitalWrite(23, LOW);
    delay(100);
    digitalWrite(23, HIGH);
    delay(100);
    
    Serial2.setRxBufferSize(1500);
    
    Serial2.begin(9600, SERIAL_8N1, 36, 22);
    printf("executed Serial2.begin(9600, SERIAL_8N1, %d, %d)\n", 36, 22);
    pinMode(34, INPUT_PULLUP); // receive interrupt signals from ethernet port
    // now we should be able to communicate with ethernet port via Serial2

    //ethWakeUp();

  

    // send test command and data byte, return byte should be bitwise NOT of data byte
    //57 AB 06 55; 
    Serial2.write(0x57);
    Serial2.write(0xAB);
    Serial2.write(0x06); // CMD_CHECK_EXIST command
    Serial2.write(0x55); // 55 is data byte
    printf("Sent test byte sequence 57  AB  06  55 to ethernet port\n");
    delay(10);
    uint32_t startTime = millis();
    bool dataAvailable = true;
    while(!Serial2.available())
    {
      delay(10);
      if (millis() - startTime > 2000)
      {
        printf("got no response when probling ethernet adapter, giving up\n");
        dataAvailable = false;
        break;
      }
    }
    if(dataAvailable)
    {
      uint8_t tmpval = Serial2.read();
      printf("First CMD_CHECK_EXIST received 0x%02X from ethernet port!\n", tmpval);  // if received 0xAA, proves commo is working
      if(tmpval == 0xAA)
      {
        // enter low power mode
        Serial2.write(0x57);
        Serial2.write(0xAB);
        Serial2.write(0x55); // CMD_SET_FUN_PARA command
        Serial2.write(0x14); // binary b0010100. Bit 2 is FUN_PARA_FLAG_LOW_PWR flag, bit 4 is FUN_PARA_FLAG_DISABLE_SEND_OK flag
        //Serial2.write(0x04); // binary b0000100. Bit 2 is FUN_PARA_FLAG_LOW_PWR flag
        Serial2.write(0x00); // bits 5-31 reserved
        Serial2.write(0x00); // bits 5-31 reserved
        Serial2.write(0x00); // bits 5-31 reserved     
        //printf("Set ETH to lower power mode"); // and turn off SEND OK interrupts\n");
        printf("Set ETH to lower power mode and turn off SEND OK interrupts\n");
        delay(100);

        
        //set rx buffer size
        //SET_RECV_BUF  parameters are socket, start buffer, num buffers
        //SET_RECV_BUF  0, 0, 44
        Serial2.write(0x57);
        Serial2.write(0xAB);
        Serial2.write(0x52); // CMD_SET_RECV_BUF command
        Serial2.write(0x00); // socket index
        Serial2.write(0x00); // buffer start block address
        Serial2.write(0x2C); // number of blocks
        printf("Set ETH receive buffer to 44x512 = 22528 bytes\n");
    
        //set tx buffer size
        //SET_SEND_BUF  parameters are socket, start buffer, num buffers
        //SET_SEND_BUF 0, 45, 4
        Serial2.write(0x57);
        Serial2.write(0xAB);
        Serial2.write(0x53); // CMD_SET_SEND_BUF command
        Serial2.write(0x00); // socket index
        Serial2.write(0x2C); // buffer start block address
        Serial2.write(0x04); // number of blocks
        printf("Set ETH send buffer to 4x512 = 2048 bytes\n");
        
        // Switch to high baud rate
        Serial2.write(0x57); // command prefix
        Serial2.write(0xAB); // command prefix
        Serial2.write(0x02); // CMD_SET_BAUDRATE
        /*
         * Calculation formula:
         * BaudRate = (Baud rate coefficient 2 <<16) + (Baud rate coefficient 1 << 8) + Baud rate coefficient 0
         */
        Serial2.write(0xc0); // coefficient 1
        Serial2.write(0xc6); // coefficient 2
        Serial2.write(0x2d); // coefficient 3
        delay(100);
        Serial2.begin(3000000, SERIAL_8N1, 36, 22);
    //    Serial2.begin(2999757, SERIAL_8N1, 36, 22);
        
    //    Serial2.write(0x00); // coefficient 1
    //    Serial2.write(0x08); // coefficient 2
    //    Serial2.write(0x07); // coefficient 3
    //    delay(100);
    //    Serial2.begin(460800, SERIAL_8N1, 36, 22);
    
    //    Serial2.write(0x40); // coefficient 1
    //    Serial2.write(0x42); // coefficient 2
    //    Serial2.write(0x0f); // coefficient 3
    //    delay(100);
    //    Serial2.begin(2000000, SERIAL_8N1, 36, 22);
    
    //    Serial2.write(0xA0); // coefficient 1
    //    Serial2.write(0x86); // coefficient 2
    //    Serial2.write(0x01); // coefficient 3
    //    delay(100);
    //    Serial2.begin(100000, SERIAL_8N1, 36, 22);
    
        // test again
        // send test command and data byte, return byte should be bitwise NOT of data byte
        //57 AB 06 55; 
        Serial2.write(0x57);
        Serial2.write(0xAB);
        Serial2.write(0x06); // CMD_CHECK_EXIST command
        Serial2.write(0x55); // 55 is data byte
        printf("Sent test byte sequence 57  AB  06  55 to ethernet port\n");
        delay(10);
        while(!Serial2.available())
        {
          delay(1);
        }
        tmpval = Serial2.read();
        printf("Second CMD_CHECK_EXIST received 0x%02X from ethernet port!\n", tmpval);  // if received 0xAA, proves commo is working
    
        // get chip version
        Serial2.write(0x57);
        Serial2.write(0xAB);
        Serial2.write(0x01); // CMD_GET_IC_VER command
        delay(1);
        while(!Serial2.available())
        {
          delay(1);
        }
        tmpval = Serial2.read();
        printf("CMD_GET_IC_VER received 0x%02X from ethernet port!\n", tmpval);  // if received 0xAA, proves commo is working
    
        
//    //  SEND CMD_INIT_CH395 command
//        printf("Sending CMD_INIT_CH395 command\n");
//        // 57 AB 27
//        Serial2.write(0x57); // command prefix
//        Serial2.write(0xAB); // command prefix
//        Serial2.write(0x27); // CMD_INIT_CH395
//        delay(700); // datasheet says 350ms, try twice as much to be safe
//        // 57 AB 2C
//        Serial2.write(0x57); // command prefix
//        Serial2.write(0xAB); // command prefix
//        Serial2.write(0x2C); // CMD_GET_CMD_STATUS    
//        while(!Serial2.available())
//        {
//          delay(10);
//        }
//        tmpval = Serial2.read();
//        printf("CMD_GET_CMD_STATUS received 0x%02X from ethernet port!\n", tmpval);  
//    
//        // PHY is 10M half duplex when the connection mode code is 04H;
//        printf("Setting ethernet port to 10Mbps half duplex mode\n");
//        Serial2.write(0x57); // command prefix
//        Serial2.write(0xAB); // command prefix
//        Serial2.write(0x20); // CMD_SET_PHY
//        Serial2.write(0x04); // 10M half duplex mode
        
        // GET MAC ADDRESS
        printf("getting MAC address\n");
        // 57 AB 40
        //uint8_t macbuf[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        Serial2.write(0x57); // command prefix
        Serial2.write(0xAB); // command prefix
        Serial2.write(0x40); // CMD_GET_MAC_ADDR
        while(!Serial2.available())
        {
          delay(1);
        }
        int count = 0;
        while(Serial2.available())
        {
          tmpval = Serial2.read();
          printf("received: 0x%02X\n", tmpval);
          ethMacAddr[count] = tmpval;
          delay(5);
          count++;
        }
        
//        printf("Setting ethernet port to MAC RAW mode\n");
//        // set ethernet port to MAC RAW working mode
//        // 57 AB 34 00 01
//        Serial2.write(0x57); // command prefix
//        Serial2.write(0xAB); // command prefix
//        Serial2.write(0x34); // CMD_SET_PROTO_TYPE_SN
//        Serial2.write(0x00); // Socket 0x00
//        Serial2.write(0x01); // mode 0x01 (MAC RAW)
//    
//        // send CMD_OPEN_SOCKET_SN command to open socket 0x01
//        // 57 AB 35 00
//        Serial2.write(0x57); // command prefix
//        Serial2.write(0xAB); // command prefix
//        Serial2.write(0x35); // CMD_OPEN_SOCKET_SN
//        Serial2.write(0x00); // Socket 0x00
//    
//        delay(100);
//    
//        // send CMD_GET_CMD_STATUS to get result of operation
//        // 57 AB 2C
//        Serial2.write(0x57); // command prefix
//        Serial2.write(0xAB); // command prefix
//        Serial2.write(0x2C); // CMD_GET_CMD_STATUS
//        delay(10);
//        while(!Serial2.available())
//        {
//          delay(10);
//        }
//        tmpval = Serial2.read();
//        printf("CMD_GET_CMD_STATUS received 0x%02X from ethernet port!\n", tmpval); 
//    
//        // clear current receive buffer
//        Serial2.write(0x57); // command prefix
//        Serial2.write(0xAB); // command prefix
//        Serial2.write(0x2E); // CMD_CLEAR_RECV_BUF_SN
//        Serial2.write(0x00); // Socket 
    
        // setup INT pin interrupt and ISR routine:
        attachInterrupt(34, ethRecvISR, FALLING);

        // put eth adapter into sleep mode to save power after boot, can wake up later when needed.
        Serial2.write(0x57); // command prefix
        Serial2.write(0xAB); // command prefix
        Serial2.write(0x03); // CMD_ENTER_SLEEP
        printf("ETH entered sleep mode\n");
        ethSleep = true;
      }
      else
      {
        printf("Received wrong value when probling ethernet adapter.  Not initializing!\n");
      }
    }
  }

  if (NE2000Enabled)
  {    
    prepareNetworkSetup();
  }
  else 
  {
    printf("NE2000 emulation disabled\n");
  }
#endif //ENABLE_ETHERNET

  //printf("calling m_i8042.init()\n");
  m_i8042.init(); //keyboard controller
  m_i8042.setCallbacks(this, keyboardInterrupt, mouseInterrupt, resetMachine, sysReq, sysReq2, sysReq3, sysReq4, sysReq5, sysReq6);

//  printf("pausing 5s before m_PIT8253.setCallbacks(this, PITChangeOut);\n");
//  delay(5000);
//  



  m_PIT8253.setCallbacks(this, PITChangeOut);  // programmable interval timer
  m_PIT8253.reset();

//  printf("pausing 5s after m_PIT8253.setCallbacks(this, PITChangeOut);\n");
//  delay(5000);

  m_MC146818.init("PCEmulator");  //RTC (real time clock)
  m_MC146818.setCallbacks(this, MC146818Interrupt);

  //m_MCP23S17.begin();  //16-Bit SPI I/O Expander with Serial Interface
  //m_MCP23S17Sel = 0;

  m_BIOS.init(this);  

  i8086::setCallbacks(this, readPort, writePort, writeVideoMemory8, writeVideoMemory16, readVideoMemory8, readVideoMemory16, interrupt, incHltCount, incMemsCount, intPending);
  // I can probably do something in i8086 to monitor s_memory access in order to determine if I should enter light sleep.
  i8086::setMemory(s_memory);


//  i2s_config_t i2s_config = {
//    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
//    .sample_rate =  8000,   //original setting in issue_7252B.ino
//    .bits_per_sample = (i2s_bits_per_sample_t)16,
//    .channel_format = (i2s_channel_fmt_t)I2S_COMM_FORMAT_STAND_I2S,
//    .communication_format = I2S_COMM_FORMAT_STAND_MSB,
//    .intr_alloc_flags = 0,
//    .dma_buf_count = 6,
//    .dma_buf_len = 256,
//    .use_apll = 1
//  };

//  if(ESP_OK != i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL)) //install and start i2s driver
//  { 
//    printf("i2s_driver_install FAILED!");
//    //while(1){vTaskDelay(100);}
//  }
//  i2s_set_dac_mode(I2S_DAC_CHANNEL_RIGHT_EN); // GPIO 25
//  
//#if ENABLE_SOUNDGEN
//#if ENABLE_ADLIB
//  printf("adlibMem = %p\n", adlibMem);
//  adlibMem = iBoxMemory + 45729;
//  printf("adlibMem = %p\n", adlibMem);
//  m_adlibSoundGenerator.setAdlibMemory(adlibMem);
//#endif //ENABLE_ADLIB
//#endif //ENABLE_SOUNDGEN

//  m_reset = true;
//  xTaskCreatePinnedToCore(
//      idle_task, /* Function to implement the task */
//      "idle_task", /* Name of the task */
//      1024 * 2,  /* Stack size in words */
//      NULL,  /* Task input parameter */
//      0,  /* Priority of the task */
//      //&Task1,  /* Task handle. */
//      NULL,
//      0); /* Core where the task should run */
  //xTaskCreate(idle_task, "idle_task", 1024 * 2, NULL,  0, NULL);


//  xTaskCreatePinnedToCore(
//      speaker_task, /* Function to implement the task */
//      "speaker_task", /* Name of the task */
//      1024 * 2,  /* Stack size in words */
//      NULL,  /* Task input parameter */
//      0,  /* Priority of the task */
//      //&Task1,  /* Task handle. */
//      NULL,
//      1); /* Core where the task should run */

  xTaskCreatePinnedToCore(
      video_task, /* Function to implement the task */
      "video_task", /* Name of the task */
      1024 * 2,  /* Stack size in words */
      this,  /* Task input parameter */
      //23,  /* Priority of the task */
      1, /* Priority of the task */
      &videoTaskHandle,  /* Task handle. */
      //NULL,
      CoreUsage::quietCore());
      //xPortGetCoreID() ^ 1); /* Core where the task should run,whichever one isn't running Machine::init() */ 

//  xTaskCreatePinnedToCore(
//      adlib_task, /* Function to implement the task */
//      "adlib_task", /* Name of the task */
//      1024 * 2,  /* Stack size in words */
//      this,  /* Task input parameter */
//      1,  /* Priority of the task */
//      &adlibTaskHandle,  /* Task handle. */
//      //NULL,
//      CoreUsage::quietCore());
//      //xPortGetCoreID() ^ 1); /* Core where the task should run,whichever one isn't running Machine::init() */ 


//  ledcSetup(TONE_PWM_CHANNEL, speakerFreq, TONE_PWM_RESOLUTION);
//  ledcAttachPin(TONE_OUTPUT_PIN, 0);


  printf("FABGLIB_VIDEO_CPUINTENSIVE_TASKS_CORE = %d\n", FABGLIB_VIDEO_CPUINTENSIVE_TASKS_CORE);
  printf("WIFI_TASK_CORE_ID = %d\n", WIFI_TASK_CORE_ID);
  m_frameBuffer = s_videoMemory + 0x8000;  // initialize to CGA 80 column text mode (BIOS mode 3)


//  hw_timer_t *Timer0_Cfg = NULL;
//  Timer0_Cfg = timerBegin(0, 80, true);
//  timerAttachInterrupt(Timer0_Cfg, COM3Recv, true);
//  timerAlarmWrite(Timer0_Cfg, 5000, true);
//  timerAlarmEnable(Timer0_Cfg);
  need_change_video_mode = true;
  m_reset = true;
}

void ethWakeUp()
{
  printf("Waking up ETH...\n");    
  uint8_t tmpval;

  // get chip version -- need to run this to wake chip up
  Serial2.write(0x57);
  Serial2.write(0xAB);
  Serial2.write(0x01); // CMD_GET_IC_VER command
//  delay(1);
//  while(!Serial2.available())
//  {
//    delay(1);
//  }
//  tmpval = Serial2.read();
//  printf("CMD_GET_IC_VER received 0x%02X from ethernet port!\n", tmpval);  // if received 0xAA, proves commo is working

  delay(25); // datasheet says 15ms at most.
  
  // enter low power mode
//  Serial2.write(0x57);
//  Serial2.write(0xAB);
//  Serial2.write(0x55); // CMD_SET_FUN_PARA command
//  Serial2.write(0x14); // binary b0010100. Bit 2 is FUN_PARA_FLAG_LOW_PWR flag, bit 4 is FUN_PARA_FLAG_DISABLE_SEND_OK flag
//  //Serial2.write(0x04); // binary b0000100. Bit 2 is FUN_PARA_FLAG_LOW_PWR flag
//  Serial2.write(0x00); // bits 5-31 reserved
//  Serial2.write(0x00); // bits 5-31 reserved
//  Serial2.write(0x00); // bits 5-31 reserved     
//  //printf("Set ETH to lower power mode"); // and turn off SEND OK interrupts\n");
//  printf("Set ETH to lower power mode and turn off SEND OK interrupts\n");
//  delay(10);
 
  //  SEND CMD_INIT_CH395 command
  printf("Sending CMD_INIT_CH395 command\n");
  // 57 AB 27
  Serial2.write(0x57); // command prefix
  Serial2.write(0xAB); // command prefix
  Serial2.write(0x27); // CMD_INIT_CH395
  delay(350); // datasheet says 350ms, try twice as much to be safe
  uint32_t startWaitTime = millis();
  while (1)
  {
    // 57 AB 2C
    Serial2.write(0x57); // command prefix
    Serial2.write(0xAB); // command prefix
    Serial2.write(0x2C); // CMD_GET_CMD_STATUS    
    while(!Serial2.available())
    {
      delay(1);
    }
    tmpval = Serial2.read();
    printf("CMD_GET_CMD_STATUS received 0x%02X from ethernet port!\n", tmpval); 
    if (tmpval == 0x00)
    {
      // success, can continue
      break;
    }
    if (millis() - startWaitTime > 2000)
    {
      // still not responding after 2000ms, give up and put it back to sleep;
      ethGoToSleep();
      return;
    }
    delay(10);
  }

  //delay(250);
  // PHY is 10M half duplex when the connection mode code is 04H;
  printf("Setting ethernet port to 10Mbps half duplex mode\n");
  Serial2.write(0x57); // command prefix
  Serial2.write(0xAB); // command prefix
  Serial2.write(0x20); // CMD_SET_PHY
  Serial2.write(0x04); // 10M half duplex mode

  //delay(250);
  printf("Setting ethernet port to MAC RAW mode\n");
  // set ethernet port to MAC RAW working mode
  // 57 AB 34 00 01
  Serial2.write(0x57); // command prefix
  Serial2.write(0xAB); // command prefix
  Serial2.write(0x34); // CMD_SET_PROTO_TYPE_SN
  Serial2.write(0x00); // Socket 0x00
  Serial2.write(0x01); // mode 0x01 (MAC RAW)

  delay(250);
  // send CMD_OPEN_SOCKET_SN command to open socket 0x01
  // 57 AB 35 00
  Serial2.write(0x57); // command prefix
  Serial2.write(0xAB); // command prefix
  Serial2.write(0x35); // CMD_OPEN_SOCKET_SN
  Serial2.write(0x00); // Socket 0x00
  
  delay(1);

  //delay(250);
  // send CMD_GET_CMD_STATUS to get result of operation
  // 57 AB 2C
  Serial2.write(0x57); // command prefix
  Serial2.write(0xAB); // command prefix
  Serial2.write(0x2C); // CMD_GET_CMD_STATUS
  while(!Serial2.available())
  {
    delay(1);
  }
  tmpval = Serial2.read();
  printf("CMD_GET_CMD_STATUS received 0x%02X from ethernet port!\n", tmpval); 

  //delay(250);
  // clear current receive buffer
  Serial2.write(0x57); // command prefix
  Serial2.write(0xAB); // command prefix
  Serial2.write(0x2E); // CMD_CLEAR_RECV_BUF_SN
  Serial2.write(0x00); // Socket  

  ethSleep = false;
  ethSendbufFree = true;
  lastEthSendTime = millis();
  lastEthRecvTime = millis();
}

void ethGoToSleep()
{
  // put eth adapter into sleep mode to save power, can wake up later when needed.
  Serial2.write(0x57); // command prefix
  Serial2.write(0xAB); // command prefix
  Serial2.write(0x03); // CMD_ENTER_SLEEP
  printf("ETH entered sleep mode\n");
  ethSleep = true;
}

bool Machine::epd_busy()
{
  return !digitalRead(ELINK_BUSY); //return 1 if busy, 0 if not busy;
}

void Machine::setVideoMode(int mode)
{
  printf("Setting video mode to 0x%02x\n", mode);
  video_mode = mode;
  lastVideoWrite = millis();
  graphics_mode = true;
  {
    last_video_mode = video_mode;
    m_frameBuffer = MCGA_videoMemory;
    fBuffer = m_frameBuffer;
    cursor_visible = false;
    //last_screen_change_time = millis();       
    // display.anything() needs to move to video_task
    need_change_video_mode = true;
    //screenDirty = true; //display.fillScreen(fg_color); // fill with fg_color because we swap fg/bg in graphics modes
    
    
    while(epd_busy());
    // display.anything() needs to move to video_task
    //screenMutex = false;
    //screenDirty = true; //display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT);
    //screenMutex = false;
    changedPixels = 0;
    //waiting_for_initPartialUpdate = true;
    //need_fullscreen_refresh = true;         
  }
  enableVideo = true;
}

void Machine::setPixel(int x, int y, uint8_t color)
{
  // need to add writing pixel to video memory, so I can't remove these from here and put exclusively in video_task yet
  /* need to add modes 
    MCGA_640X480_MONO 0x11
    EGA_320X200_COLOR 0x0d
    EGA_640X200_COLOR 0x0e
    EGA_640X350_MONO 0x0f
    MCGA_640X480_MONO 0x11
    VGA_640X480_COLOR 0x12
  */
  
  if (millis() - lastSetPixelMsgTime > 1000)
  {
    printf("setPixel(%d,%d,0x%02x)\n", x, y, color);
    lastSetPixelMsgTime = millis();
  }
  if (video_mode == EGA_320X200_COLOR)
  {
    // temporary bandaid.  need to change to actual EGA pixel write.
    setMCGA320x200pixel(x, y, color);
    // still need to write pixel to video RAM
  } 
  else if (video_mode == MCGA_320X200_COLOR)
  {
    setMCGA320x200pixel(x, y, color);
    // still need to write pixel to video RAM
  } 
  else if (video_mode == CGA_320X200_COLOR || video_mode == CGA_320X200_GRAY)
  {
    set320x200pixel(x, y, color);
    // still need to write pixel to video RAM
  }
  else if (video_mode == CGA_640X200_MONO)
  {
    set640x200pixel(x, y, color);
    // still need to write pixel to video RAM
  }
  else if (video_mode == MCGA_640X480_MONO)
  {
    set640x480pixel(x, y, color);
    // still need to write pixel to video RAM
  }
  screenDirty = true;
  lastVideoWrite = millis();
  lastVideoHandler = millis();
  last_screen_change_time = millis();
  global_need_refresh = true;
}

void Machine::resetKeyboard()
{
  m_i8042.reset();
  m_i8042.init(); //keyboard controller
  m_i8042.setCallbacks(this, keyboardInterrupt, mouseInterrupt, resetMachine, sysReq, sysReq2, sysReq3, sysReq4, sysReq5, sysReq6);
}

void Machine::setDriveImage(int drive, char const * filename, int cylinders, int heads, int sectors)
{
  if (m_disk[drive]) 
  {
    fclose(m_disk[drive]);
    m_disk[drive] = nullptr;
  }

  if (m_diskFilename[drive]) 
  {
    free(m_diskFilename[drive]);
    m_diskFilename[drive] = nullptr;
  }

  m_BIOS.setDriveMediaType(drive, mediaUnknown);

  m_diskCylinders[drive] = cylinders;
  m_diskHeads[drive]     = heads;
  m_diskSectors[drive]   = sectors;
  
  m_diskChanged[drive]   = true;

  if (filename) 
  {
    m_diskFilename[drive] = strdup(filename);
    printf("Loading %s on drive %d\n", m_diskFilename[drive], drive);
    m_disk[drive] = FileBrowser(m_baseDir).openFile(filename, "r+b");
    if (m_disk[drive]) 
    {
      printf("File opened successfully\n");
      
      // get image file size
      fseek(m_disk[drive], 0L, SEEK_END);
      m_diskSize[drive] = ftell(m_disk[drive]);
      printf("m_diskSize[%d] = %u\n", drive, m_diskSize[drive]);

      // need to detect geometry?
      if (cylinders == 0 || heads == 0 || sectors == 0)
      {
        printf("calling autoDetectDriveGeometry(drive)\n");
        autoDetectDriveGeometry(drive);
      }
    }
    else
    {
      printf("File open failed\n");
    }
  }
}


void Machine::autoDetectDriveGeometry(int drive)
{
  // well known floppy formats
  static const struct {
    uint16_t tracks;
    uint8_t  sectors;
    uint8_t  heads;
  } FLOPPYFORMATS[] = {
    { 40,  8, 1 },   //  163840 bytes (160K, 5.25 inch)
    { 40,  9, 1 },   //  184320 bytes (180K, 5.25 inch)
    { 40,  8, 2 },   //  327680 bytes (320K, 5.25 inch)
    { 40,  9, 2 },   //  368640 bytes (360K, 5.25 inch)
    { 80,  9, 2 },   //  737280 bytes (720K, 3.5 inch)
    { 80, 15, 2 },   // 1228800 bytes (1200K, 5.25 inch)
    { 80, 18, 2 },   // 1474560 bytes (1440K, 3.5 inch)
    { 80, 36, 2 },   // 2949120 bytes (2880K, 3.5 inch)
  };

  // look for well known floppy formats
  for (auto const & ff : FLOPPYFORMATS) {
    if (512 * ff.tracks * ff.sectors * ff.heads == m_diskSize[drive]) {
      m_diskCylinders[drive] = ff.tracks;
      m_diskHeads[drive]     = ff.heads;
      m_diskSectors[drive]   = ff.sectors;
      printf("autoDetectDriveGeometry, found floppy: t=%d s=%d h=%d\n", ff.tracks, ff.sectors, ff.heads);
      return;
    }
  }

  // maybe an hard disk, try to calculate geometry (max 528MB, common lower end for BIOS and MSDOS: https://tldp.org/HOWTO/Large-Disk-HOWTO-4.html)
  constexpr int MAXCYLINDERS = 1024;  // Cylinders : 1...1024
  constexpr int MAXHEADS     = 255;    // Heads     : 1...16 (actual limit is 256)
  constexpr int MAXSECTORS   = 63;    // Sectors   : 1...63
  uint32_t bytesPerSector = 512;
  uint32_t c = 1;
  uint32_t h = 1;
  printf("pass 0: m_diskSize[%d] = %u\n", drive, m_diskSize[drive]);
  uint32_t s = (m_diskSize[drive] / bytesPerSector);    //s = 2476736512 / 512 = 4837376
  printf("pass 0.1: m_diskSize[%d] = %u, s = %u\n", drive, m_diskSize[drive], s);
  if (s > MAXSECTORS) // true
  {
    h = s / MAXSECTORS; // h = 4837376 / 63 = 76783
    s = MAXSECTORS;    // s = 63
    printf("pass 1: h = %d, s = %d\n", h, s);
  }
  if (h > MAXHEADS) //true
  {
    c = h / MAXHEADS; // c = 76783 / 255 = 301
    h = MAXHEADS;     // h = 255
    printf("pass 2: c = %d, h = %d\n", c, h);
  }
  if (c > MAXCYLINDERS) // false
  {
    c = MAXCYLINDERS;
    printf("pass 3: c = %d\n", c);
  }
  m_diskCylinders[drive] = c;
  m_diskHeads[drive]     = h;
  m_diskSectors[drive]   = s;
  printf("autoDetectDriveGeometry, found HD: c=%d h=%d s=%d (tot=%u filesz=%u)\n", m_diskCylinders[drive], m_diskHeads[drive], m_diskSectors[drive], (uint32_t)(512 * m_diskCylinders[drive] * m_diskHeads[drive] * m_diskSectors[drive]), (uint32_t)m_diskSize[drive]);

/*
Loading 2_5GB-1.img on drive 3
File opened successfully
m_diskSize[3] = 2476736512
calling autoDetectDriveGeometry(drive)
autoDetectDriveGeometry, found HD: c=1024 h=255 s=63 (tot=4127719424 filesz=2476736512)
*/
}


void Machine::setCOM1(SerialPort * serialPort)
{
  m_COM1.setCallbacks(this, COM1Interrupt);
  m_COM1.setSerialPort(serialPort);
}


void Machine::setCOM2(SerialPort * serialPort)
{
  if (networkMode != NETWORKMODE_ETH)
  {
    m_COM2.setCallbacks(this, COM2Interrupt);
    m_COM2.setSerialPort(serialPort);
  }
}

//void Machine::setCOM3(FakeSerialPort * serialPort)
//{
//  m_COM3.setCallbacks(this, COM3Interrupt);
//  m_COM3.setSerialPort(serialPort);
//}

void Machine::reset()
{
  m_reset = false;

  m_ticksCounter = 0;

  m_CGAMemoryOffset = 0;
  m_CGAModeReg      = 0;
  m_CGAColorReg     = 0;
  m_CGAVSyncQuery   = 0;

  m_HGCMemoryOffset = 0;
  m_HGCModeReg      = 0;
  m_HGCSwitchReg    = 0;
  m_HGCVSyncQuery   = 0;
  
#if ENABLE_SOUNDGEN
#if ENABLE_PC_SPEAKER
  m_speakerDataEnable = false;
#endif //ENABLE_PC_SPEAKER
#endif //ENABLE_SOUNDGEN
  m_i8042.reset();

  m_PIC8259A.reset();
  m_PIC8259B.reset();

  m_PIT8253.reset();
  m_PIT8253.setGate(0, true);
  //m_PIT8253.setGate(1, true); // @TODO: timer 1 used for DRAM refresh, required to run?

  m_MC146818.reset();
  
  m_COM1.reset();
  if (networkMode != NETWORKMODE_ETH)
  {
    m_COM2.reset();
  }
  //m_COM3.reset();

  memset(m_CGA6845, 0, sizeof(m_CGA6845));
  memset(m_HGC6845, 0, sizeof(m_HGC6845));
//  printf("Line %d\n", __LINE__);

  //memset(s_videoMemory, 0, 65536); // this line is breaking adlib output. why?
  //memset(s_videoMemory, 0, 32768); // this doesn't break adlib output. why?
  memset(s_videoMemory, 0, 57344); // this doesn't break adlib output. why?
  //memset(s_videoMemory, 0, 61440); // this line is breaking adlib output. why?  Doesn't break Disney DSS
  
  memset(bitTable, 1, 65536);
//  printf("Line %d\n", __LINE__);
//  redrawScreen();
//  printf("Line %d\n", __LINE__);
//  while(epd_busy());
//  display.displayFast(smallX, smallY, largeX - smallX, largeY - smallY); // changed portion of screen partial update
//  printf("Line %d\n", __LINE__);



//  videoTaskSuspended = true;
//  machineSuspended = true;
//  invertColors();
//  redrawScreen();
//  drawBorders();
//  helpers = 0;
//  changedPixels = 9999999;
//  screenDirty = true;
//  need_fullscreen_refresh = false;
//  last_fullscreen_refresh = millis();    //comment out for debugging, normally use   
//  last_screen_change_time = millis();
//  helpers = 0; 
//  need_fullscreen_refresh = false;
//  last_fullscreen_refresh = millis();    //comment out for debugging, normally use   
//  last_screen_change_time = millis(); 
//  videoTaskSuspended = false;
//  machineSuspended = false;




  
  m_BIOS.reset();

  i8086::reset();

  // set boot drive (0, 1, 0x80, 0x81)
  i8086::setDL((m_bootDrive & 1) | (m_bootDrive > 1 ? 0x80 : 0x00));
}


void Machine::run()
{
  //BaseType_t xTaskCreatePinnedToCore(TaskFunction_t pvTaskCode, const char *const pcName, const uint32_t usStackDepth, void *const pvParameters, UBaseType_t uxPriority, TaskHandle_t *const pvCreatedTask, const BaseType_t xCoreID)
  printf("Line %d\n", __LINE__);
  xTaskCreatePinnedToCore(&runTask, "", 4000, this, 5, &m_taskHandle, CoreUsage::quietCore() ^ 1);
  printf("Line %d\n", __LINE__);
  //xTaskCreatePinnedToCore(&runTask, "", 4000, this, 5, &m_taskHandle, CoreUsage::quietCore());  // at least once pcplus's sound problem occurred with this setting.  After resetting with the reset button, problem went away
}


void Machine::runTask(void * pvParameters)
{
  Serial.println(__LINE__);
  vTaskDelay(1);
  auto m = (Machine*)pvParameters;
  Serial.println(__LINE__);
  m->init();
  Serial.println( __LINE__);
  if (m->need_unhibernate)
  {
    m->unhibernate(true);
  }
  Serial.println(__LINE__);
  
  while (true) 
  {
    if (m->m_reset)
    {
      printf("calling reset() within main loop\n");
      m->reset();
      printf("dumpinfo after reset in main loop:\n");
      m->printDumpInfo();
    }


    //printf("step()\n");
    i8086::step();
    stepsCount++;
    //
    //printf("tick()\n");
    m->tick();
    while(machineSuspended)
    {
      vTaskDelay(50);
    }
    //
//    if (stepsCount >> 20 == 1)
//    {
//      stepsCount = 0;
//      //printf("vTaskDelay(1)\n");
//      //vTaskDelay(1);
//      printf("sending data to serial3\n");
//      vTaskDelay(5);
//      //m_COM3.rxCallBack(ser->m_callbackArgs, r, true);
//      auto ser = (FakeSerialPort*) m->m_COM3.getSerialPort();
//      //ser->m_rxCallback(ser->m_callbackArgs, 'A', true);
//      ser->insertRX('A');
//    }
  }
}

const char *bit_rep[16] = {
    [ 0] = "0000", [ 1] = "0001", [ 2] = "0010", [ 3] = "0011",
    [ 4] = "0100", [ 5] = "0101", [ 6] = "0110", [ 7] = "0111",
    [ 8] = "1000", [ 9] = "1001", [10] = "1010", [11] = "1011",
    [12] = "1100", [13] = "1101", [14] = "1110", [15] = "1111",
};

void print_byte(uint8_t byte)
{
    printf("%s%s", bit_rep[byte >> 4], bit_rep[byte & 0x0F]);
}

void Machine::printCharAttr(char ascii, unsigned char attribute, int charLocation, bool inverse)
{
  if (video_mode == MDA_80X25_MONO)
  {
    //MDA monochrome 80x25 mode, no color, just underline, inverse, foreground intensity, and background intensity attribute flags
    /*
     * The attribute bytes mostly behave like a bitmap:
  
      Bits 0-2: 1 => underline, other values => no underline. 
      Bit 3: High intensity. b00001000 (0x08) 
      Bit 7: Blink b10000000 (0x80) or bg high intensity
  
      but there are eight exceptions:
      Attribute 0Fh displays as high intensity (same as 07h?)
      Attributes 00h, 08h, 80h and 88h display as black space.
      Attribute 70h displays as black on green.
      Attribute 78h displays as dark green on green. In fact, depending on timing and on the design of the monitor, it may have a bright green 'halo' where the dark green and bright green bits meet.
      Attribute F0h displays as a blinking version of 70h (if blinking is enabled); as black on bright green otherwise.
      Attribute F8h displays as a blinking version of 78h (if blinking is enabled); as dark green on bright green otherwise.
  
     */
     
     /*
      * display.setTextColor(GxEPD_BLACK);
      * #define GxEPD_BLACK     0x0000
        #define GxEPD_DARKGREY  0x7BEF      // 128, 128, 128
        #define GxEPD_LIGHTGREY 0xC618      // 192, 192, 192
        #define GxEPD_WHITE     0xFFFF
        #define GxEPD_RED       0xF800      // 255,   0,   0
      */
  
    #define FONT_UNDERLINE 0x07 //bits 0,1,2
    #define FONT_INTENSITY 0x08 // bit 3
    #define FONT_DISPLAY_MODE 0x70 //bits 4,5,6
    #define FONT_BG_INTENSITY 0x80 //bit 7
    
    
    /*
    Bits 0,1,2: Only bit 0 set to 1 = underline; any other setting don't underline
    Bit3 1 = bright, 0 = normal intensity
    Bits 4,5,6 all set = inverse video; all unset = no display; other settings = normal display
    Bit 7 = blinking; if set, blinking if blinking enabled.  Otherwise background is bright (if reverse text)
    */
    int charX = ((charLocation) % 80) * font_width;
    int charY = ((charLocation) / 80) * font_height;
    if (smallX > charX)
    {
      smallX = charX;
    }
    if (smallY > charY)
    {
      smallY = charY;
    }
    if (largeX < charX + 8)
    {
      largeX = charX + 8;
    }
    if (largeY < charY + 16)
    {
      largeY = charY + 16;
    }
    //printf("change area1: %d, %d, %d, %d\n", smallX, smallY, largeX, largeY);    
    display.setCursor(charX, charY);
    // underlined?
    int underline = FONT_UNDERLINE & attribute;
    if (underline != 1)
    {
      underline = 0;
    }
  
    // bright?
    int fg_bright = FONT_INTENSITY & attribute;

    // display mode (normal, off, or inverse)
    int display_mode = FONT_DISPLAY_MODE & attribute;
    bool display_off = false;
    bool display_inverse = false;
    //bool display_normal = false;
    if (display_mode == 0x00 && (attribute == 0x00 || attribute == 0x08 || attribute == 0x80 || attribute == 0x88))
    {
      // bits 4,5,6 all bits off and all other bits off except 3 and 7
      display_off = true;
    }
    else if (display_mode == 0x70)
    {
      // all bits on, inverse display
      display_inverse = true;
    }
//    else
//    {
//      display_normal = true;
//    }
  
    // background is intense (only for inverse text)
    int bg_bright = FONT_BG_INTENSITY & attribute;
  
    //first decide foreground and background colors based on inversion state
    uint16_t this_fg_color = fg_color;
    uint16_t this_bg_color = bg_color;
    
    if (inverse)
    {
      // manually specified inverse display when calling this function
      uint16_t this_old_fg_color = this_fg_color;
      this_fg_color = this_bg_color;
      this_bg_color = this_old_fg_color;    
    }
    if (display_inverse)
    {
      //attribute specifies inverse display
      uint16_t this_old_fg_color = this_fg_color;
      this_fg_color = this_bg_color;
      this_bg_color = this_old_fg_color;
    }
  
    if (fg_bright || bg_bright) // attribe says display with high intensity foreground or background
    {
      // We can't control intensity, so use a bold font instead
      display.setFont(&VGA8x16Bold);
    }
    else
    {
      //do not display high intensity
      // use standard font to display non-high intensity text
      display.setFont(&VGA8x16);
    }
  
    //printf("cL = %d, c_p = %d\n", charLocation, cursor_pos);
//    if (charLocation == cursor_pos && cursor_visible)
//    {
//      //can't blink cursor, so invert current inversion state to make cursor highly visible
//      // maybe I shoud draw a box instead, or invert the bottom half of the character only...
//      uint16_t this_old_fg_color = this_fg_color;
//      this_fg_color = this_bg_color;
//      this_bg_color = this_old_fg_color;
//      //printf("inverting cursor\n");
//    }
    
    //fill the background for this character location
    display.fillRect(charX, charY, font_width, font_height, this_bg_color);
    
    //set the text color for this character
    display.setTextColor(this_fg_color, this_bg_color);

    // only display something if display_off is false
    if (display_off)
    {
      //printf("Off:%d,%d\n", charX, charY);
    }
    else
    {
      //underline if needed, but only in video mode 7, since color and grayscale modes will generate ugly underlines because of color attributes
      if (underline)
      {
        //printf("U%d,%d\n", charX, charY);
        display.drawLine(charX, charY + HIRES_FONT_BASELINE, charX + font_width, charY + HIRES_FONT_BASELINE, this_fg_color);
      }
    
      // set background brightness. Only works in inverse mode.  Since we don't have shades of black and white, do nothing
      if (bg_bright && display_inverse)
      {
        // maybe I can create another font with super thin lines or tiny characters to represent "bright" backgrounds
        //display.drawLine(charX, charY + HIRES_FONT_BASELINE, charX + font_width, charY + HIRES_FONT_BASELINE, this_fg_color);
      }
  
      //I may be able to differentiate between the 8 "dim" colors and the 8 "bright" colors and choose normal or bold font respectively when displaying
      // color text modes.
      
      //draw the character
      display.print(ascii);
      changedPixels += 128;
    }
  }
  else if (video_mode == CGA_40X25_GRAY || video_mode == CGA_40X25_COLOR || video_mode == CGA_80X25_GRAY || video_mode == CGA_80X25_COLOR)
  {
    // color or grayscale modes
    /* ATTRIBUTE BYTE BITMAPPING: 
     *  BIT 7: background intensity
     *  BIT 6: background RED
     *  BIT 5: background GREEN
     *  BIT 4: background BLUE
     *  BIT 3: foreground intensity
     *  BIT 2: foreground RED
     *  BIT 1: foreground GREEN
     *  BIT 0: foreground BLUE
     */
    int charX = 0;
    int charY = 0;
    if (video_mode == CGA_80X25_GRAY || video_mode == CGA_80X25_COLOR)
    {
      charX = ((charLocation) % 80) * font_width;
      charY = ((charLocation) / 80) * font_height;
    }
    else if (video_mode == CGA_40X25_GRAY || video_mode == CGA_40X25_COLOR)
    {
      charX = ((charLocation) % 40) * font_width;
      charY = ((charLocation) / 40) * font_height;
    }
    if (smallX > charX)
    {
      smallX = charX;
    }
    if (smallY > charY)
    {
      smallY = charY;
    }
    if (largeX < charX + 8)
    {
      largeX = charX + 8;
    }
    if (largeY < charY + 16)
    {
      largeY = charY + 16;
    }
    //printf("change area1: %d, %d, %d, %d\n", smallX, smallY, largeX, largeY);
    display.setCursor(charX, charY);
    uint16_t back_color = (attribute & 0x70) >> 4;
    uint16_t full_back_color = attribute >> 4; // high four bits of attribute are background color
    uint16_t fore_color = attribute & 0x07;
    uint16_t full_fore_color = attribute & 0x0f; // low four bits of attribute are foreground (text) color
    
    // foreground bright?
    int fg_bright = (FONT_INTENSITY & attribute) >> 3;
    // background bright?
    int bg_bright = (FONT_BG_INTENSITY & attribute) >> 7;
    
    //printf("%d,%d:'%c',%02x,bg=%d,fg=%d,bgb=%d,fgb=%d\n", charX / font_width, charY / font_height, (char)ascii, ascii, back_color, fore_color, bg_bright, fg_bright);
    bool fill_bg_color = false;
    bool fill_fg_color = false;
    bool display_inverse = false;

    
    //if (back_color > fore_color)
    if (
      //(fore_color == 0 && back_color != 0)
      (back_color != 0)
      || (bg_bright && !fg_bright)
      )
    {
      // if background color value is larger than foreground color value, display as inverted text, otherwise normal
      // the above method is illogical as these are color bit fields and only the high bit has intensity value
      // better:
      // 1: if foreground is black (n000), invert
      // 2: if background is intense (1nnn) and foreground is not intense (0nnn), then invert
      display_inverse = true;
    }
    else if (back_color == 0 && fore_color == 0 && bg_bright == fg_bright)
    {
      //bg and fg colors are both zero, intensities are same 
      fill_bg_color = true;
    }
    else if(back_color == fore_color && bg_bright == fg_bright)
    {
      //bg and fg colors are same, intensities are same 
      fill_fg_color = true;
    }
    //bg and fg colors are same, fg intensity brighter than bg intensity
    //bg and fg colors are same, bg intensity brighter than fg intensity

    uint16_t this_bg_color = bg_color;
    uint16_t this_fg_color = fg_color;
    
    //printf("this_fg_color = 0x%02x\n", this_fg_color);
    //printf("this_bg_color = 0x%02x\n", this_bg_color);
    if (inverse)
    {
      // manually specified inverse display when calling this function
      uint16_t this_old_fg_color = this_fg_color;
      this_fg_color = this_bg_color;
      this_bg_color = this_old_fg_color;    
    }
    
    if (display_inverse)
    {
      //chosen because background color value is larger than foreground color value
      uint16_t this_old_fg_color = this_fg_color;
      this_fg_color = this_bg_color;
      this_bg_color = this_old_fg_color;
    }

    if (fill_bg_color)
    {
      this_fg_color = this_bg_color;
    }

    if (fill_fg_color)
    {
      this_bg_color = this_fg_color;
    }
    
    if (fg_bright) // attribe says display fg with high intensite foreground
    {
      if (video_mode == CGA_80X25_GRAY || video_mode == CGA_80X25_COLOR)
      {
        // We can't control intensity, so use a bold font instead
        display.setFont(&VGA8x16Bold);
      }
      else if (video_mode == CGA_40X25_GRAY || video_mode == CGA_40X25_COLOR)
      {
        // no bold font for 40x25 mode yet, just use normal font for now
        display.setFont(&BIOS8x8Bold);
      }
    }
    else
    {
      //do not display high intensity
      if (video_mode == CGA_80X25_GRAY || video_mode == CGA_80X25_COLOR)
      {
        // use standard font to display non-high intensity text
        display.setFont(&VGA8x16);
      }
      else if (video_mode == CGA_40X25_GRAY || video_mode == CGA_40X25_COLOR)
      {
        display.setFont(&BIOS8x8);
      }
    }

    //fill the background for this character location
    //printf("fillRect() bg filling\n");


    
  if (!colorDifferentiation)
  {
      display.fillRect(charX, charY, font_width, font_height, this_bg_color);
  }
  else
  {
    /* //Simple, functional, ugly algorithmic method:   
    int v = (back_color & 0x0c) >> 2;  // 2 high bits of background color 
    int h = back_color & 0x03;  // 2 low bits of background color
    //test: fill bg with pattern
    uint16_t c = this_bg_color;
    for (int n = 0; n < 8*16; n++)
    {
      if (((n / 8) % 4) == v && (n % 4) == h)
      {
        c = this_fg_color;
      }
      else
      {
        c = this_bg_color;
      }
      display.drawPixel(charX + n % 8, charY + n / 8, c);
    }
    */

    // Using pre-defined "bgColor" array:
    for (int row = 0; row < 16; row++)
    {
      // get byte that defines the 8 pixels for this row of the bit pattern that represents this background color
      uint8_t colorByte = bgColor[full_back_color][row];
      for (int col = 0; col < 8; col++)
      {
        // extract the bit for this pixel location:
        uint8_t pixel = (colorByte >> col) & 0x01;
        display.drawPixel(charX + col, charY + row, bg_color == GxEPD_WHITE ? pixel : 1 - pixel);
      }
    }
    if (full_back_color < 7)
    {
      this_fg_color = fg_color;
    }
    if (full_back_color == 7)
    {
      this_fg_color = bg_color;
    }
    //else if (full_back_color == 8)
    //{
      //no need to change
    //}
    else if (full_back_color > 8)
    {
      this_fg_color = bg_color;
    }
  }
  
    //set the text color for this character
    display.setTextColor(this_fg_color, this_bg_color);
//    if (charLocation == cursor_pos && cursor_visible)
//    {
//      //can't blink cursor, so invert current inversion state to make cursor highly visible
//      // maybe I shoud draw a box instead, or invert the bottom half of the character only...
//      
//      int quarter_font_height = font_height / 4;
//      display.fillRect(charX, charY + (3 * quarter_font_height), font_width, quarter_font_height, this_fg_color);
//      //printf("inverting cursor\n");
//    }

    // set background brightness. Only works in inverse mode.  Since we don't have shades of black and white, do nothing
    if (bg_bright && display_inverse)
    {
      // maybe I can create another font with super thin lines or tiny characters to represent "bright" backgrounds
      //display.drawLine(charX, charY + HIRES_FONT_BASELINE, charX + font_width, charY + HIRES_FONT_BASELINE, this_fg_color);
    }

    //I may be able to differentiate between the 8 "dim" colors and the 8 "bright" colors and choose normal or bold font respectively when displaying
    // color text modes.
    
    //draw the character
    //printf("display.print(ascii);\n");
    //printf("%d,%d:%c\n", charX, charY, ascii);
    screenMutex = true;

    // some limited foreground color differentiation
    if (colorDifferentiation && full_fore_color != 0x00 && full_fore_color != 0x07 && full_fore_color != 0x0f && (full_back_color == 0 || full_back_color == 0x06  || full_back_color == 0x07 || full_back_color == 0x0e || full_back_color == 0x0f)) // if fore color is not black, white, or bright white, and background is solid black or white, or brown, or yellow, display as shaded in colorDifferentiation mode (other combinations are too ugly and/or hard to read)
    {
      display.drawChar(charX, charY, ascii, this_fg_color, this_fg_color, 1, 1, true);
    }
    else
    {
      display.print(ascii);
    }
    screenMutex = false;
    changedPixels += 64;
  }
}





void Machine::invertColors()
{
  uint16_t old_bg_color = bg_color;
  bg_color = fg_color;
  fg_color = old_bg_color; 
  screenDirty = true; //display.fillScreen(bg_color);
}

//void Machine::clearLastFrameBuffer()
//{
//  int numbytes = 3999; // 80x25 text modes
//  if (video_mode == CGA_40X25_GRAY || video_mode == CGA_40X25_COLOR)
//  {
//    numbytes = 1999;
//  } 
//  if (video_mode == CGA_320X200_COLOR || video_mode == CGA_320X200_GRAY || video_mode == CGA_640X200_MONO)
//  {
//    numbytes = 16384;
//  }
//  if (video_mode == HGC_720X348_MONO)
//  {
//    numbytes = 65535;  
//  }
//  for (int i = 0; i <= numbytes; i++)
//  {
//    last_frameBuffer[i] = 0x00;
//  }  
//}



//void Machine::set720x348pixel(int x, int y, uint8_t color)
void set720x348pixel(int x, int y, uint8_t color)
{
  // set a 640x200 screen format pixel.  On the physical 648x480 EPD display, this can be done
  // using a 1x2 physical pixel "pixel", which would allow for 4 different patterns, but since only 
  // background (default black) and foreground color are allowed, no need, just use both off or both on, 1-bit color

  // convert to physical pixels - not needed for 720x348
  int px = x;
  int py = y;
  int cfg = 0;
  int cbg = 0;
  switch (color)
  {
    case 0:  // set one pixel to foreground color, because we default to use black background white foreground in graphics modes (pixel off/color zero)
      display.drawPixel(px, py, fg_color);
      cfg = 1;
      //display.drawPixel(px, py + 1, fg_color);
      break;
    case 1: // set one pixel to background color, because we default to use black background white foreground in graphics modes (pixel on/color one)
      screenMutex = true;
      display.drawPixel(px, py, bg_color);
      screenMutex = false;
      cbg = 1;
      //display.drawPixel(px, py + 1, bg_color);
      break;
    default:
      break;
  }
  if (smallX > px)
  {
    smallX = px;
  }
  if (smallY > py)
  {
    smallY = py;
  }
  if (largeX < px)
  {
    largeX = px;
  }
  if (largeY < py)
  {
    largeY = py;
  }
  //printf("change area1: %d, %d, %d, %d\n", smallX, smallY, largeX, largeY);
  
//  if (fg_color == GxEPD_BLACK)
//  {
//    changedPixels += cfg;
//  }
//  else if (bg_color == GxEPD_BLACK)
//  {
//    changedPixels += cbg;
//  }
  changedPixels += cfg + cbg;
}

//void Machine::set640x200pixel(int x, int y, uint8_t color)
void set640x200pixel(int x, int y, uint8_t color)
{
  // set a 640x200 screen format pixel.  On the physical 648x480 EPD display, this can be done
  // using a 1x2 physical pixel "pixel", which would allow for 4 different patterns, but since only 
  // background (default black) and foreground color are allowed, no need, just use both off or both on, 1-bit color

  // convert to physical pixels
  int px = x;
  int py = y * 2;
  int cfg = 0;
  int cbg = 0;
  switch (color)
  {
    case 0:  // set two pixels to foreground color, because we default to use black background white foreground in graphics modes (pixel off/color zero)
      screenMutex = true;
      display.drawPixel(px, py, fg_color);
      display.drawPixel(px, py + 1, fg_color);
      screenMutex = false;
      cfg = 2;
      break;
    case 1: // set two pixels to background color, because we default to use black background white foreground in graphics modes (pixel on/color one)
      screenMutex = true;
      display.drawPixel(px, py, bg_color);
      display.drawPixel(px, py + 1, bg_color);
      screenMutex = false;
      cbg = 2;
      break;
    default:
      break;
  }
  if (smallX > px)
  {
    smallX = px;
  }
  if (smallY > py)
  {
    smallY = py;
  }
  if (largeX < px)
  {
    largeX = px;
  }
  if (largeY < py)
  {
    largeY = py;
  }
  //printf("change area1: %d, %d, %d, %d\n", smallX, smallY, largeX, largeY);  
//  if (fg_color == GxEPD_BLACK)
//  {
//    changedPixels += cfg;
//  }
//  else if (bg_color == GxEPD_BLACK)
//  {
//    changedPixels += cbg;
//  }
  changedPixels += cfg + cbg;
}

void set640x480pixel(int x, int y, uint8_t color)
{
  // set a 640x480 screen format pixel.
  int cfg = 0;
  int cbg = 0;
  switch (color)
  {
    case 0:  // set pixel to foreground color, because we default to use black background white foreground in graphics modes (pixel off/color zero)
      screenMutex = true;
      display.drawPixel(x, y, fg_color);
      screenMutex = false;
      cfg = 1;
      break;
    case 1: // set pixel to background color, because we default to use black background white foreground in graphics modes (pixel on/color one)
      screenMutex = true;
      display.drawPixel(x, y, bg_color);
      screenMutex = false;
      cbg = 1;
      break;
    default:
      break;
  }
  if (smallX > x)
  {
    smallX = x;
  }
  if (smallY > y)
  {
    smallY = y;
  }
  if (largeX < x)
  {
    largeX = x;
  }
  if (largeY < y)
  {
    largeY = y;
  }
  //printf("change area1: %d, %d, %d, %d\n", smallX, smallY, largeX, largeY);  
//  if (fg_color == GxEPD_BLACK)
//  {
//    changedPixels += cfg;
//  }
//  else if (bg_color == GxEPD_BLACK)
//  {
//    changedPixels += cbg;
//  }
  changedPixels += cfg + cbg;
}

//void Machine::set320x200pixel(int x, int y, uint8_t color)
void set320x200pixel(int x, int y, uint8_t color)
{
  // set a 320x200 screen format pixel.  On the physical 648x480 EPD display, this can be done
  // using a 2x2 physical pixel "pixel", and vary the 2x2 pixel pattern to emulate the 4 possible CGA "colors"

  // convert to physical pixels
  //printf("%d,%d:%d\n", x, y, color);
  int px = x * 2;
  int py = y * 2;
  int cfg = 0;
  int cbg = 0;
  switch (color)
  {
    // by default use black for background and white for foreground, the inverse of text modes, otherwise graphics are noticeabley weird
    case 0:  // set four pixels to background color (pixel off/color zero)
      screenMutex = true;
      display.drawPixel(px, py, fg_color);
      display.drawPixel(px + 1, py, fg_color);
      display.drawPixel(px, py + 1, fg_color);
      display.drawPixel(px + 1, py + 1, fg_color);
      screenMutex = false;
      cfg = 4;
      break;
    case 1: // set a solid pixel for color one
      screenMutex = true;
      display.drawPixel(px, py, bg_color);
      display.drawPixel(px + 1, py, bg_color);
      display.drawPixel(px, py + 1, bg_color);
      display.drawPixel(px + 1, py + 1, bg_color);
      screenMutex = false;
      cbg = 4;
      break;
//    case 2: // set a vertical right line for color two
//      display.drawPixel(px, py, bg_color);
//      display.drawPixel(px + 1, py, fg_color);
//      display.drawPixel(px, py + 1, bg_color);
//      display.drawPixel(px + 1, py + 1, fg_color);
//      break;
//    case 2: // set a horizontal bottom line for color two
//      display.drawPixel(px, py, bg_color);
//      display.drawPixel(px + 1, py, bg_color);
//      display.drawPixel(px, py + 1, fg_color);
//      display.drawPixel(px + 1, py + 1, fg_color);
//      break;
    case 2: // set a upper right pixel for color two
      screenMutex = true;
      display.drawPixel(px, py, fg_color);
      display.drawPixel(px + 1, py, bg_color);
      display.drawPixel(px, py + 1, fg_color);
      display.drawPixel(px + 1, py + 1, fg_color);
      screenMutex = false;
      cfg = 3;
      cbg = 1;
      break;      
    case 3: // set a backward slanting line for color two
      screenMutex = true;
      display.drawPixel(px, py, bg_color);
      display.drawPixel(px + 1, py, fg_color);
      display.drawPixel(px, py + 1, fg_color);
      display.drawPixel(px + 1, py + 1, bg_color);
      screenMutex = false;
      cfg = 2;
      cbg = 2;
      break;
    default:
      break;
  }
//  if (fg_color == GxEPD_BLACK)
//  {
//    changedPixels += cfg;
//  }
//  else if (bg_color == GxEPD_BLACK)
//  {
//    changedPixels += cbg;
//  }
  if (smallX > px)
  {
    smallX = px;
  }
  if (smallY > py)
  {
    smallY = py;
  }
  if (largeX < px)
  {
    largeX = px;
  }
  if (largeY < py)
  {
    largeY = py;
  }
  //printf("change area1: %d, %d, %d, %d\n", smallX, smallY, largeX, largeY);
  changedPixels += cfg + cbg;
}

void setMCGA640x480pixel(int x, int y, uint8_t color, bool deb = false)
{
  if (deb)
  {
    printf("setMCGA640x480pixel(%d, %d, 0x%02x);\n", x, y, color);
  }
  int cfg = 0;
  int cbg = 0;
  switch(color)
  {
    case 0:
      screenMutex = true;
      display.drawPixel(x, y, fg_color);
      screenMutex = false;
      cfg = 1;
      break;
    case 1:
      screenMutex = true;
      display.drawPixel(x, y, bg_color);
      screenMutex = false;
      cbg = 1;
      break;
    default:
      break;
  }
  if (smallX > x)
  {
    smallX = x;
  }
  if (smallY > y)
  {
    smallY = y;
  }
  if (largeX < x)
  {
    largeX = x;
  }
  if (largeY < y)
  {
    largeY = y;
  }
  //printf("change area1: %d, %d, %d, %d\n", smallX, smallY, largeX, largeY);
//  if (fg_color == GxEPD_BLACK)
//  {
//    changedPixels += cfg;
//  }
//  else if (bg_color == GxEPD_BLACK)
//  {
//    changedPixels += cbg;
//  }
  changedPixels += cfg + cbg;
}


void setMCGA320x200pixel(int x, int y, uint8_t color, bool deb)
{
  // set a 320x200 screen format pixel.  On the physical 648x480 EPD display, this can be done
  // using a 2x2 physical pixel "pixel", and vary the 2x2 pixel pattern to emulate the 16 possible MCGA "colors"

  // convert to physical pixels
  //printf("%d,%d:%d\n", x, y, color);
  int px = x * 2;
  int py = y * 2;
  uint8_t color16 = color & 0x0f; // use only least significant four bits
//  if (deb)
//  {
//    printf("setMCGA320x200pixel(%d, %d, 0x%02x);\n", x, y, color16);
//  }
  int cfg = 0;
  int cbg = 0;
  switch (color16)
  {
    // by default use black for background and white for foreground, the inverse of text modes, otherwise graphics are noticeabley weird
    
    /*   01
     *   23 
     *   
     *   0123
     */
    
  
  //pixel on/off counts
  /*
  0000  0
  0001  1
  0010  1
  0100  1
  1000  1
  0011  2
  0101  2
  0110  2
  1001  2
  1010  2
  1100  2
  0111  3
  1011  3
  1101  3
  1110  3
  1111  4 
  */
  
    case 0:  // all off 0000
      screenMutex = true;
      display.drawPixel(px, py, fg_color);
      display.drawPixel(px + 1, py, fg_color);
      display.drawPixel(px, py + 1, fg_color);
      display.drawPixel(px + 1, py + 1, fg_color);
      screenMutex = false;
      cfg = 4;
      break;
    case 1: // 1000
      screenMutex = true;
      display.drawPixel(px, py, bg_color);
      display.drawPixel(px + 1, py, fg_color);
      display.drawPixel(px, py + 1, fg_color);
      display.drawPixel(px + 1, py + 1, fg_color);
      screenMutex = true;
      cfg = 3;
      cbg = 1;
      break;
    case 2: // 0100
      screenMutex = true;
      display.drawPixel(px, py, fg_color);
      display.drawPixel(px + 1, py, bg_color);
      display.drawPixel(px, py + 1, fg_color);
      display.drawPixel(px + 1, py + 1, fg_color);
      screenMutex = false;
      cfg = 3;
      cbg = 1;
      break;      
    case 3: // 1100
      screenMutex = true;
      display.drawPixel(px, py, bg_color);
      display.drawPixel(px + 1, py, bg_color);
      display.drawPixel(px, py + 1, fg_color);
      display.drawPixel(px + 1, py + 1, fg_color);
      screenMutex = false;
      cfg = 2;
      cbg = 2;
      break;
    case 4: // 0010
      screenMutex = true;
      display.drawPixel(px, py, fg_color);
      display.drawPixel(px + 1, py, fg_color);
      display.drawPixel(px, py + 1, bg_color);
      display.drawPixel(px + 1, py + 1, fg_color);
      screenMutex = false;
      cfg = 3;
      cbg = 1;
      break;
    case 5: // 1010
      screenMutex = true;
      display.drawPixel(px, py, bg_color);
      display.drawPixel(px + 1, py, fg_color);
      display.drawPixel(px, py + 1, bg_color);
      display.drawPixel(px + 1, py + 1, fg_color);
      screenMutex = false;
      cfg = 2;
      cbg = 2;
      break;
    case 6: // 0110
      screenMutex = true;
      display.drawPixel(px, py, fg_color);
      display.drawPixel(px + 1, py, bg_color);
      display.drawPixel(px, py + 1, bg_color);
      display.drawPixel(px + 1, py + 1, fg_color);
      screenMutex = false;
      cfg = 2;
      cbg = 2;
      break;
    case 7: // 1110
      screenMutex = true;
      display.drawPixel(px, py, bg_color);
      display.drawPixel(px + 1, py, bg_color);
      display.drawPixel(px, py + 1, bg_color);
      display.drawPixel(px + 1, py + 1, fg_color);
      screenMutex = false;
      cfg = 1;
      cbg = 3;
      break;
    case 8: // 0001
      screenMutex = true;
      display.drawPixel(px, py, fg_color);
      display.drawPixel(px + 1, py, fg_color);
      display.drawPixel(px, py + 1, fg_color);
      display.drawPixel(px + 1, py + 1, bg_color);
      screenMutex = false;
      cfg = 3;
      cbg = 1;
      break;
    case 9: // 1001
      screenMutex = true;
      display.drawPixel(px, py, bg_color);
      display.drawPixel(px + 1, py, fg_color);
      display.drawPixel(px, py + 1, fg_color);
      display.drawPixel(px + 1, py + 1, bg_color);
      screenMutex = false;
      cfg = 2;
      cbg = 2;
      break;
    case 10: // 0101
      screenMutex = true;
      display.drawPixel(px, py, fg_color);
      display.drawPixel(px + 1, py, bg_color);
      display.drawPixel(px, py + 1, fg_color);
      display.drawPixel(px + 1, py + 1, bg_color);
      screenMutex = false;
      cfg = 2;
      cbg = 2;
      break;
    case 11: // 1101
      screenMutex = true;
      display.drawPixel(px, py, bg_color);
      display.drawPixel(px + 1, py, bg_color);
      display.drawPixel(px, py + 1, fg_color);
      display.drawPixel(px + 1, py + 1, bg_color);
      screenMutex = false;
      cfg = 1;
      cbg = 3;
      break;
    case 12: // 0011
      screenMutex = true;
      display.drawPixel(px, py, fg_color);
      display.drawPixel(px + 1, py, fg_color);
      display.drawPixel(px, py + 1, bg_color);
      display.drawPixel(px + 1, py + 1, bg_color);
      screenMutex = false;
      cfg = 2;
      cbg = 2;
      break;
    case 13: // 1011
      screenMutex = true;
      display.drawPixel(px, py, bg_color);
      display.drawPixel(px + 1, py, fg_color);
      display.drawPixel(px, py + 1, bg_color);
      display.drawPixel(px + 1, py + 1, bg_color);
      screenMutex = false;
      cfg = 1;
      cbg = 3;
      break;
    case 14: // 0111
      screenMutex = true;
      display.drawPixel(px, py, bg_color);
      display.drawPixel(px + 1, py, fg_color);
      display.drawPixel(px, py + 1, fg_color);
      display.drawPixel(px + 1, py + 1, fg_color);
      screenMutex = false;
      cfg = 3;
      cbg = 1;
      break;
    case 15: // 1111
      screenMutex = true;
      display.drawPixel(px, py, bg_color);
      display.drawPixel(px + 1, py, bg_color);
      display.drawPixel(px, py + 1, bg_color);
      display.drawPixel(px + 1, py + 1, bg_color);
      screenMutex = false;
      cbg = 4;
      break;  
    default:
      break;
  }
  if (smallX > px)
  {
    smallX = px;
  }
  if (smallY > py)
  {
    smallY = py;
  }
  if (largeX < px)
  {
    largeX = px;
  }
  if (largeY < py)
  {
    largeY = py;
  }
  //printf("change area1: %d, %d, %d, %d\n", smallX, smallY, largeX, largeY);
//  if (fg_color == GxEPD_BLACK)
//  {
//    changedPixels += cfg;
//  }
//  else if (bg_color == GxEPD_BLACK)
//  {
//    changedPixels += cbg;
//  }
  changedPixels += cfg + cbg;
}


//void Machine::drawCursor()
void drawCursor(int current_cursor_position)
{
  //printf("drawCursor(): cursor_start_scanline = %d, cursor_end_scanline = %d, \n", cursor_start_scanline, cursor_end_scanline);
  if (!cursor_visible)
  {
    //printf("cursor_visible = true, not drawing cursor\n");
    return;
  }
  //uint8_t attribute = m_frameBuffer[(current_cursor_position * 2) + 1];

  // temporary comment out to debug esp32 crashing:
  //printf("attribute = fBuffer[%d]\n", (current_cursor_position * 2) + 1); //attribute = fBuffer[131071] (current_cursor_position = 65535)
  uint8_t attribute = fBuffer[(current_cursor_position * 2) + 1];
  //uint8_t attribute = 0;
  
  uint16_t this_bg_color = bg_color;
  uint16_t this_fg_color = fg_color;
  if (video_mode != MDA_80X25_MONO)
  {
    uint16_t back_color = (attribute & 0x70) >> 4;
    //uint16_t fore_color = attribute & 0x07;
    // foreground bright?
    int fg_bright = (FONT_INTENSITY & attribute) >> 3;
    // background bright?
    int bg_bright = (FONT_BG_INTENSITY & attribute) >> 7;
    bool display_inverse = false;
    if ((back_color != 0) || (bg_bright && !fg_bright))
    {
      // if background color value is larger than foreground color value, display as inverted text, otherwise normal
      // the above method is illogical as these are color bit fields and only the high bit has intensity value
      // better:
      // 1: if foreground is black (n000), invert
      // 2: if background is intense (1nnn) and foreground is not intense (0nnn), then invert
      display_inverse = true;
    }
    
    if (display_inverse)
    {
      //chosen because background color value is larger than foreground color value
      uint16_t this_old_fg_color = this_fg_color;
      this_fg_color = this_bg_color;
      this_bg_color = this_old_fg_color;
    }
  }
  if(video_mode == CGA_40X25_GRAY || video_mode == CGA_40X25_COLOR) //CGA 40x25 modes
  {
    int x = current_cursor_position % 40;
    int y = current_cursor_position / 40;
    //printf("x = %d, y = %d\n", x, y);
    if (x >= 0 && x <= SCREEN40X25_COLS && y >= 0 && y <= SCREEN40X25_ROWS)
    {
      if (cursor_start_scanline <= 7)
      { 
        if (cursor_end_scanline > 7) cursor_end_scanline = 7;
        display.fillRect(x * LORES_FONT_WIDTH, y * LORES_FONT_HEIGHT + (cursor_start_scanline * 2), LORES_FONT_WIDTH, ((cursor_end_scanline - cursor_start_scanline) + 1) * 2, this_fg_color);
        //display.fillRect(x * LORES_FONT_WIDTH, y * LORES_FONT_HEIGHT + (cursor_start_scanline), LORES_FONT_WIDTH, ((cursor_end_scanline - cursor_start_scanline)), this_fg_color);
      }
    }
  }
  else if(video_mode == CGA_80X25_GRAY || video_mode == CGA_80X25_COLOR) 
  {
    int x = current_cursor_position % 80;
    int y = current_cursor_position / 80;
    //printf("x = %d, y = %d\n", x, y);
    if (x >= 0 && x <= SCREEN80X25_COLS && y >= 0 && y <= SCREEN80X25_ROWS)
    {    
      //printf("display.fillRect(%d * %d, %d * %d + (%d * 2), %d, (( %d - %d) + 1) * 2), this_fg_color);\n", x, HIRES_FONT_WIDTH, y, HIRES_FONT_HEIGHT, cursor_start_scanline, HIRES_FONT_WIDTH, cursor_end_scanline, cursor_start_scanline);
      if (cursor_start_scanline <= 7)
      { 
        if (cursor_end_scanline > 7) cursor_end_scanline = 7;
        screenMutex = true;
        display.fillRect(x * HIRES_FONT_WIDTH, y * HIRES_FONT_HEIGHT + (cursor_start_scanline * 2), HIRES_FONT_WIDTH, ((cursor_end_scanline - cursor_start_scanline) + 1) * 2, this_fg_color); // double the scanlines because our actual font vertical resolution is twice that of the original CGA 8x8 narrow font
        screenMutex = false;
        //display.fillRect(x * HIRES_FONT_WIDTH, y * HIRES_FONT_HEIGHT + (cursor_start_scanline), HIRES_FONT_WIDTH, ((cursor_end_scanline - cursor_start_scanline)), this_fg_color); // double the scanlines because our actual font vertical resolution is twice that of the original CGA 8x8 narrow font
      }
    }
  }
  else if(video_mode == MDA_80X25_MONO)
  {
    int x = current_cursor_position % 80;
    int y = current_cursor_position / 80;
    //printf("MDA_80X25_MONO cursor: x = %d, y = %d, startline = %d, endline = %d\n", x, y, cursor_start_scanline, cursor_end_scanline);
    if (x >= 0 && x <= SCREEN80X25_COLS && y >= 0 && y <= SCREEN80X25_ROWS)
    { 
      screenMutex = true;
      display.fillRect(x * HIRES_FONT_WIDTH, y * HIRES_FONT_HEIGHT + cursor_start_scanline, HIRES_FONT_WIDTH, (cursor_end_scanline - cursor_start_scanline) + 1, this_fg_color);
      screenMutex = false;
    }
  }
}

void updateGraphicsByte(uint32_t addr, uint8_t value, bool compare = false, uint8_t oldvalue = 0, bool deb = false)
{
  //static int lastrow = 0xffff;
  if (video_mode == CGA_320X200_COLOR || video_mode == CGA_320X200_GRAY)
  {
    if ( (addr > 7999 && addr < 8192) || addr > 16191 ) // skip addresses that don't represent physical pixels on the screen
    {
      return;
    }

    uint32_t p = addr;
    // 4 color mode, 2 bits per pixel, 1 byte = 4 pixels
    uint8_t pixels4 = 0;
    int n = p + 0;  // even rows start at offset 0000 and end at 7999
    if (p > 7999)
    {
      n = p - (192);  // odd rows start at offset 8192, not 8000
    }
    //printf("%d(%d)",n, p);
    
    pixels4 = value;
    uint8_t pix0 = pixels4 >> 6;  // new left most pixel
    uint8_t pix1 = (pixels4 & B00110000) >> 4; // new second pixel from left
    uint8_t pix2 = (pixels4 & B00001100) >> 2; // new third pixel from left
    uint8_t pix3 = (pixels4 & B00000011); // new right pixel
    
    int x = ((n * 4) % 320);
    int y1 = ((n * 4) / 320);
    int y = 0;
    // lines are interleaved in video RAM with set 1 at 0x0000 with lines 0,2,4,6...198
    // and set two at address 8000 with lines 1,3,5,7...199
    // so when y1 is <= 99, multiply it by two, 
    if (y1 <= 99)
    {
      y = y1 * 2; 
      /*  0 -> 0
       *  1 -> 2
       *  2 -> 4
       *  3 -> 6
       */
    }
    // when y1 is > 99, 
    else if (y1 > 99 && y1 <= 199)
    {
      y = ((y1 - 100) * 2) + 1;
      /*    100 -> 1
       *    101 -> 3
       *    102 -> 5
       *    103 -> 7
       */
    }
//    if((y <= 9 || y >= 190)) //&& y % 2 == 0) //test print top ten bottom and top even numbered rows' pixel values
//    {
//      if (x == 0)
//      {
//        printf("row %03d: ", y);
//      }
//      printf("%d:%02x ", x, value);
//      if(x == 316)
//      {
//        printf("\n");
//      }      
//    }
//    if(y == 199 && x == 316)
//    {
//      printf("\n");
//    }
    

    //printf("%d,%d(%d) 0x%02x->0x%02x\n", x, y, y1, last_frameBuffer[p], m_frameBuffer[p]);
    set320x200pixel(x + 0, y, pix0);
    //printf("pix0 color = %d\n", pix0);
    //printf("%d,%d %d\n", x + 0, y, pix0);
    set320x200pixel(x + 1, y, pix1);
    //printf("pix1 color = %d\n", pix1);
    //printf("%d,%d %d\n", x + 1, y, pix1);
    set320x200pixel(x + 2, y, pix2);
    //printf("pix2 color = %d\n", pix2);
    //printf("%d,%d %d\n", x + 2, y, pix2);
    set320x200pixel(x + 3, y, pix3);
    //printf("pix3 color = %d\n", pix3);
    //printf("%d,%d %d\n", x + 3, y, pix3);
    
//    if (n == 7999)  // if finished all the even rows
//    {
//      n = 8191;  //skip to the odd rows
//    }
  }
  else if (video_mode == MCGA_640X480_MONO)
  {
    if (addr > 0xA9600 ) // skip addresses that don't represent physical pixels on the screen
    {
      return;
    }  
    uint32_t p = addr;
    // two color mode (1 black), 1 bit per pixel, 1 byte = 8 pixels
    // 2 color mode, 1 bit per pixel, 1 byte = 8 pixels
    uint8_t pixels8 = 0;
    uint8_t oldpixels8 = 0;
    int n = p + 0;  // even rows start at offset 0000 and end at 7999

    if (deb)
    {
      printf("value = 0x%02x\n", value);
    }
    pixels8 = value;
    uint8_t pix0 = pixels8 >> 7;  // previous left most pixel
    uint8_t pix1 = (pixels8 & B01000000) >> 6; // previous second pixel from left
    uint8_t pix2 = (pixels8 & B00100000) >> 5; // previous third pixel from left
    uint8_t pix3 = (pixels8 & B00010000) >> 4; // previous forth pixel from left
    uint8_t pix4 = (pixels8 & B00001000) >> 3; 
    uint8_t pix5 = (pixels8 & B00000100) >> 2;
    uint8_t pix6 = (pixels8 & B00000010) >> 1;
    uint8_t pix7 = (pixels8 & B00000001);      // right pixel
    
    uint8_t oldpix0, oldpix1, oldpix2, oldpix3, oldpix4, oldpix5, oldpix6, oldpix7;
    if (compare) 
    {
      //printf("0x%02x:0x%02x\n", value, oldvalue);
      oldpixels8 = oldvalue;
      oldpix0 = oldpixels8 >> 7;  // previous left most pixel
      oldpix1 = (oldpixels8 & B01000000) >> 6; // previous second pixel from left
      oldpix2 = (oldpixels8 & B00100000) >> 5; // previous third pixel from left
      oldpix3 = (oldpixels8 & B00010000) >> 4; // previous forth pixel from left
      oldpix4 = (oldpixels8 & B00001000) >> 3; 
      oldpix5 = (oldpixels8 & B00000100) >> 2;
      oldpix6 = (oldpixels8 & B00000010) >> 1;
      oldpix7 = (oldpixels8 & B00000001);      // right pixel      
    }

    int x = ((n * 8) % 640);
    int y1 = ((n * 8) / 640);
    int y = y1;
    //printf("%d,%d(%d) 0x%02x->0x%02x\n", x, y, y1, last_frameBuffer[p], m_frameBuffer[p]);

    if (compare)
    {
      if (pix0 != oldpix0) setMCGA640x480pixel(x + 0, y, pix0, deb);
      if (pix1 != oldpix1) setMCGA640x480pixel(x + 1, y, pix1, deb);
      if (pix2 != oldpix2) setMCGA640x480pixel(x + 2, y, pix2, deb);
      if (pix3 != oldpix3) setMCGA640x480pixel(x + 3, y, pix3, deb);
      if (pix4 != oldpix4) setMCGA640x480pixel(x + 4, y, pix4, deb);
      if (pix5 != oldpix5) setMCGA640x480pixel(x + 5, y, pix5, deb);
      if (pix6 != oldpix6) setMCGA640x480pixel(x + 6, y, pix6, deb);
      if (pix7 != oldpix7) setMCGA640x480pixel(x + 7, y, pix7, deb);
//      setMCGA640x480pixel(x + 0, y, pix0, deb);
//      //printf("pix0 color = %d\n", pix0);
//      //printf("%d,%d %d\n", x + 0, y, pix0);
//      setMCGA640x480pixel(x + 1, y, pix1, deb);
//      //printf("pix1 color = %d\n", pix1);
//      //printf("%d,%d %d\n", x + 1, y, pix1);
//      setMCGA640x480pixel(x + 2, y, pix2, deb);
//      //printf("pix2 color = %d\n", pix2);
//      //printf("%d,%d %d\n", x + 2, y, pix2);
//      setMCGA640x480pixel(x + 3, y, pix3, deb);
//      //printf("pix3 color = %d\n", pix3);
//      //printf("%d,%d %d\n", x + 3, y, pix3);
//      setMCGA640x480pixel(x + 4, y, pix4, deb);
//      //printf("pix0 color = %d\n", pix0);
//      //printf("%d,%d %d\n", x + 0, y, pix0);
//      setMCGA640x480pixel(x + 5, y, pix5, deb);
//      //printf("pix1 color = %d\n", pix1);
//      //printf("%d,%d %d\n", x + 1, y, pix1);
//      setMCGA640x480pixel(x + 6, y, pix6, deb);
//      //printf("pix2 color = %d\n", pix2);
//      //printf("%d,%d %d\n", x + 2, y, pix2);
//      setMCGA640x480pixel(x + 7, y, pix7, deb);
//      //printf("pix3 color = %d\n", pix3);
//      //printf("%d,%d %d\n", x + 3, y, pix3);
    }
    else
    {
      setMCGA640x480pixel(x + 0, y, pix0, deb);
      //printf("pix0 color = %d\n", pix0);
      //printf("%d,%d %d\n", x + 0, y, pix0);
      setMCGA640x480pixel(x + 1, y, pix1, deb);
      //printf("pix1 color = %d\n", pix1);
      //printf("%d,%d %d\n", x + 1, y, pix1);
      setMCGA640x480pixel(x + 2, y, pix2, deb);
      //printf("pix2 color = %d\n", pix2);
      //printf("%d,%d %d\n", x + 2, y, pix2);
      setMCGA640x480pixel(x + 3, y, pix3, deb);
      //printf("pix3 color = %d\n", pix3);
      //printf("%d,%d %d\n", x + 3, y, pix3);
      setMCGA640x480pixel(x + 4, y, pix4, deb);
      //printf("pix0 color = %d\n", pix0);
      //printf("%d,%d %d\n", x + 0, y, pix0);
      setMCGA640x480pixel(x + 5, y, pix5, deb);
      //printf("pix1 color = %d\n", pix1);
      //printf("%d,%d %d\n", x + 1, y, pix1);
      setMCGA640x480pixel(x + 6, y, pix6, deb);
      //printf("pix2 color = %d\n", pix2);
      //printf("%d,%d %d\n", x + 2, y, pix2);
      setMCGA640x480pixel(x + 7, y, pix7, deb);
      //printf("pix3 color = %d\n", pix3);
      //printf("%d,%d %d\n", x + 3, y, pix3);
    }
//      if (n == 7999)  // if finished all the even rows
//      {
//        n = 8191;  //skip to the odd rows
//      }      
  }
  else if (video_mode == MCGA_320X200_COLOR)
  {
    if (addr > 0xAFA001 ) // skip addresses that don't represent physical pixels on the screen
    {
      return;
    }

    uint32_t p = addr;
    // 256 color mode, 8 bits per pixel, 1 byte = 1 pixels
    uint8_t pixel = 0;
    int n = p;
    
    pixel = value;
   
    int x = (n % 320);
    int y = (n / 320);
//    if (deb)
//    {
//      printf("setMCGA320x200pixel(%d, %d, 0x%02x);\n", x, y, pixel);
//    }
    setMCGA320x200pixel(x , y, pixel, deb);
  }
  else if (video_mode == EGA_320X200_COLOR)
  {
    uint8_t pixels8 = value;
    uint8_t pix0 = pixels8 >> 7;  // previous left most pixel
    uint8_t pix1 = (pixels8 & B01000000) >> 6; // previous second pixel from left
    uint8_t pix2 = (pixels8 & B00100000) >> 5; // previous third pixel from left
    uint8_t pix3 = (pixels8 & B00010000) >> 4; // previous forth pixel from left
    uint8_t pix4 = (pixels8 & B00001000) >> 3; 
    uint8_t pix5 = (pixels8 & B00000100) >> 2;
    uint8_t pix6 = (pixels8 & B00000010) >> 1;
    uint8_t pix7 = (pixels8 & B00000001);      // right pixel

    int n = addr;
    int xoff = 0;
    int yoff = 0;
    //if (addr >= 0 && addr <= 7999) // bit plane 0
    if (addr <= 7999) // bit plane 0
    {
      //upper left set
    }
    if (addr >= 8000 && addr <= 15999) // bit plane 1
    {
      // upper right set, default
      n = addr - 8000;
      xoff = 1;
    }    
    if (addr >= 16000 && addr <= 23999) // bit plane 2
    {
      // lower left set
      n = addr - 16000;
      yoff = 1;
    }  
    if (addr >= 24000 && addr <= 31999) // bit plane 3
    {
      // lower right set
      n = addr - 24000;
      xoff = 1;
      yoff = 1;
    }

    int x = ((n * 8) % 320);
    int y = ((n * 8) / 320);
    
//    int x = ((n % 320)) + xoff;
//    int y = ((n / 320)) + yoff;    
//    setMCGA320x200pixel(x + 0, y, pix0);
//    setMCGA320x200pixel(x + 1, y, pix1);
//    setMCGA320x200pixel(x + 2, y, pix2);
//    setMCGA320x200pixel(x + 3, y, pix3);
//    setMCGA320x200pixel(x + 4, y, pix4);
//    setMCGA320x200pixel(x + 5, y, pix5);
//    setMCGA320x200pixel(x + 6, y, pix6);
//    setMCGA320x200pixel(x + 7, y, pix7);    
    //printf("%d:%d,%d\n", x, y, pix0);     
    set640x480pixel(x * 2 + xoff + 0, y * 2 + yoff, pix0);
    set640x480pixel(x * 2 + xoff + 2, y * 2 + yoff, pix1);
    set640x480pixel(x * 2 + xoff + 4, y * 2 + yoff, pix2);
    set640x480pixel(x * 2 + xoff + 6, y * 2 + yoff, pix3);
    set640x480pixel(x * 2 + xoff + 8, y * 2 + yoff, pix4);
    set640x480pixel(x * 2 + xoff + 10, y * 2 + yoff, pix5);
    set640x480pixel(x * 2 + xoff + 12, y * 2 + yoff, pix6);
    set640x480pixel(x * 2 + xoff + 14, y * 2 + yoff, pix7);









   
//    int x = (addr % 320);
//    int y = (addr / 320);
//    setMCGA320x200pixel(x , y, value, deb);    
  }
  else if (video_mode == CGA_640X200_MONO)
  {
    if ( (addr > 7999 && addr < 8192) || addr > 16191 ) // skip addresses that don't represent physical pixels on the screen
    {
      return;
    }
    uint32_t p = addr;
    // two color mode (1 black), 1 bit per pixel, 1 byte = 8 pixels
    // 2 color mode, 1 bit per pixel, 1 byte = 8 pixels
    uint8_t pixels8 = 0;
    int n = p + 0;  // even rows start at offset 0000 and end at 7999

    if (p > 7999)
    {
      n = p - (192);  // odd rows start at offset 8192, not 8000
    }
    //printf("%d(%d)",n, p);
  
    pixels8 = value;
    uint8_t pix0 = pixels8 >> 7;  // previous left most pixel
    uint8_t pix1 = (pixels8 & B01000000) >> 6; // previous second pixel from left
    uint8_t pix2 = (pixels8 & B00100000) >> 5; // previous third pixel from left
    uint8_t pix3 = (pixels8 & B00010000) >> 4; // previous forth pixel from left
    uint8_t pix4 = (pixels8 & B00001000) >> 3; 
    uint8_t pix5 = (pixels8 & B00000100) >> 2;
    uint8_t pix6 = (pixels8 & B00000010) >> 1;
    uint8_t pix7 = (pixels8 & B00000001);      // right pixel

    int x = ((n * 8) % 640);
    int y1 = ((n * 8) / 640);
    int y = 0;
    // lines are interleaved in video RAM with set 1 at 0x0000 with lines 0,2,4,6...198
    // and set two at addr 8000 with lines 1,3,5,7...199
    // so when y1 is <= 99, multiply it by two, 
    if (y1 <= 99)
    {
      y = y1 * 2; 
      /*  0 -> 0
       *  1 -> 2
       *  2 -> 4
       *  3 -> 6
       */
    }
    // when y1 is > 99, 
    else if (y1 > 99 && y1 <= 199)
    {
      y = ((y1 - 100) * 2) + 1;
      //x -= 128; // x is offset by 128?  try it...
      /*    100 -> 1
       *    101 -> 3
       *    102 -> 5
       *    103 -> 7
       */
    }
    //printf("%d,%d(%d) 0x%02x->0x%02x\n", x, y, y1, last_frameBuffer[p], m_frameBuffer[p]);
    set640x200pixel(x + 0, y, pix0);
    //printf("pix0 color = %d\n", pix0);
    //printf("%d,%d %d\n", x + 0, y, pix0);
    set640x200pixel(x + 1, y, pix1);
    //printf("pix1 color = %d\n", pix1);
    //printf("%d,%d %d\n", x + 1, y, pix1);
    set640x200pixel(x + 2, y, pix2);
    //printf("pix2 color = %d\n", pix2);
    //printf("%d,%d %d\n", x + 2, y, pix2);
    set640x200pixel(x + 3, y, pix3);
    //printf("pix3 color = %d\n", pix3);
    //printf("%d,%d %d\n", x + 3, y, pix3);
    set640x200pixel(x + 4, y, pix4);
    //printf("pix0 color = %d\n", pix0);
    //printf("%d,%d %d\n", x + 0, y, pix0);
    set640x200pixel(x + 5, y, pix5);
    //printf("pix1 color = %d\n", pix1);
    //printf("%d,%d %d\n", x + 1, y, pix1);
    set640x200pixel(x + 6, y, pix6);
    //printf("pix2 color = %d\n", pix2);
    //printf("%d,%d %d\n", x + 2, y, pix2);
    set640x200pixel(x + 7, y, pix7);
    //printf("pix3 color = %d\n", pix3);
    //printf("%d,%d %d\n", x + 3, y, pix3);
//      if (n == 7999)  // if finished all the even rows
//      {
//        n = 8191;  //skip to the odd rows
//      }
  }
  else if(video_mode == HGC_720X348_MONO)
  {
    // two color mode (1 black), 1 bit per pixel, 1 byte = 8 pixels
    // 2 color mode, 1 bit per pixel, 1 byte = 8 pixels

    // skip addresses that don't represent physical pixels on the screen
//    if ( (addr > 7829 && addr < 8192) || (addr > 16021 && addr < 16384) || (addr > 24213 && addr < 24576) || addr > 32405) 
//    {
//      return;
//    }
    uint32_t p = addr;
    uint8_t pixels8 = 0;
    int n = p + 0;  // even rows start at offset 0000 and end at 7999
    // 0 - 7999 need to adjustment, first interleave set starts at 0
    //8000 - 15999 need to adjust downward 192

    // bank 1 (lines 0,4,8...) starts at 0 and ends at 7829, containing 7830(87*90) bytes
    // 8192 - 7830 = 362, so there is a 362 byte empty gap between each bank
    // bank 2 (lines 1,5,9...) starts at 8192 and ends at 16021, containing 7830(87*90) bytes
    // 16384 - 16022 = 362
    // bank 3 (lines 2,6,10...) starts at 16384 and ends at 24213, containing 7830(87*90) bytes
    // 24576 - 24214 = 362
    // bank 4 (lines 3,7,11...) starts at 24576 and ends at 32405, containing 7830(87*90) bytes
    // everthing from 32406 to 32767 is unused, 32768 - 32406 = 362
    if (HGC_width == 720)
    {
      if (p >= 8192 && p <= 16383)
      {
         // second interleave set starts at offset 8192, not 8000
        n = p - (362); //8192 - (362 * 1) = 7830 = 87 * 90 * 1 (1 for second bank)
      }
      else if (p >= 16384 && p <= 24575)
      {
        // third interleave set starts at offset 16384, not 16000
        n = p - (362 * 2); //16384 - (362 * 2) = 15660 = 87 * 90 * 2 (2 for third bank)
      }
      else if (p >= 24576 && p <= 32405)
      {
        // fourth interleave set starts at offset 24576, not 24000
        n = p - (362 * 3); //24576 - (362 * 3) = 23490 = 87 * 90 * 3 (3 for fourth bank)
      }
    }
    else if (HGC_width == 640)
    {
      if (p >= 8192 && p <= 16383)
      {
         // second interleave set starts at offset 8192, not 8000
        n = p - 192; //8192 - (192) = 8000 = 100 * 80 * 1 (1 for second bank)
      }
      else if (p >= 16384 && p <= 24575)
      {
        // third interleave set starts at offset 16384, not 16000
        n = p - 384; //16384 - 384 = 16000 = 100 * 80 * 2 (2 for third bank)
      }
      else if (p >= 24576 && p <= 32767)
      {
        // fourth interleave set starts at offset 24576, not 24000
        n = p - 576; //24576 - 576 = 24000 = 100 * 80 * 3 (3 for fourth bank)
        //printf("n = %d\n", n);
      }
    }
    //printf("%d(%d)",n, p);
    pixels8 = value;
    uint8_t pix0 = (pixels8 & B10000000) >> 7;  // previous left most pixel
    uint8_t pix1 = (pixels8 & B01000000) >> 6; // previous second pixel from left
    uint8_t pix2 = (pixels8 & B00100000) >> 5; // previous third pixel from left
    uint8_t pix3 = (pixels8 & B00010000) >> 4; // previous forth pixel from left
    uint8_t pix4 = (pixels8 & B00001000) >> 3; 
    uint8_t pix5 = (pixels8 & B00000100) >> 2;
    uint8_t pix6 = (pixels8 & B00000010) >> 1;
    uint8_t pix7 = (pixels8 & B00000001);      // right pixel

    //int x = ((n * 8) % 720);  // need to adjust this if using custom Hercules resolution
    int x = ((n * 8) % HGC_width);  // need to adjust this if using custom Hercules resolution
    
    if (HGC_width > 640)
    {
      // since we temporarily only have a 640px wide physical screen, skip every 8th pixel
      x = (x * 8) / 9; //change to x = (x << 3) / 9 ?
    }
    // 0-7    -> 0-6
    // 8-15   -> 7-13
    // 16-23  -> 14-20
    // 24-31  -> 21-27
    // 32 -> 28
    // 40 -> 35
    // 48 -> 42
    // 56 -> 49
    // 64 -> 56
    // 65 -> 57
    // 66 -> 58
    // 67 -> 59
    // 68 -> 60
    // 69 -> 61
    // 70 -> 62
    // 71 -> 63
    // 72 -> 64
    // 73 -> 64
    // 74 -> 65
    // 75 -> 66
    // 76 -> 67
    // 77 -> 68
    // 78 -> 69
    // 79 -> 70
    // 80 -> 71
    // 81 -> 72
    //int y1 = ((n * 8) / 720);  // need to adjust this if using custom Hercules resolution
//    static int lasty1 = -1;
    int y1 = ((n * 8) / HGC_width);  // need to adjust this if using custom Hercules resolution
//    if (y1 != lasty1)
//    {
//      printf("y1=%d\n", y1);
//      lasty1 = y1;
//    }
    int y = 0;
    // lines are interleaved in video RAM with:
    // set 1 at 0x0000 with lines  0,4,8,12...344
    // set 2 at 0x2000 with lines  1,5,9,13...345
    // set 3 at 0x4000 with lines 2,6,10,14...346
    // set 4 at 0x6000 with lines 3,7,11,15...347
    // each set has 87 lines with 720 pixels per line.  720 pixels uses 90 bytes, 87*90 = 7830, so each memory area needs to use 7830 (0x1e96) bytes (0x1e96), but since 8192 (0x2000) bytes are allocated, there should be an unused 8192 - 7830 = 362, so these 362 bytes need to be skipped at the end of each section

    // 348 lines divided into 4 sets of 87 lines:
    // 0 - 86
    // 87 - 173
    // 174 - 260
    // 261 - 347 
    // so when y1 is <= 86, multiply it by 4 to get lines 0,4,8,12...344
    //if (y1 <= 86)
    if (y1 <= ((HGC_height / 4) - 1)) // in 640x400 mode:(y1 <= 99)
    //if (y1 <= ((HGC_height / HGC_scanlines) - 1)) // in 640x400 mode:(y1 <= 99)
    {
      //y = y1 * 4;
      y = y1 * HGC_scanlines;
      /*  0 -> 0
       *  1 -> 4
       *  2 -> 8
       *  3 -> 12 
       */

      /*  0 -> 0
       *  1 -> 3
       *  2 -> 6
       *  3 -> 9 
       */
    }
    // when y1 is >= 87 and <= 173, subtract 87 (to get back to 0), then multiply by 4 and add 1 to get lines 1,5,9,13...345
    //else if (y1 >= 87 && y1 <= 173)
    else if (y1 >= (HGC_height / 4) && y1 <= (HGC_height / 2 - 1)) // in 640x400 mode:(y1 >= 100 && y1 <= 199)
    //else if (y1 >= (HGC_height / HGC_scanlines) && y1 <= (HGC_height / 2 - 1)) // in 640x400 mode:(y1 >= 100 && y1 <= 199)
    {
      //y = ((y1 - (HGC_height / 4)) * 4) + 1;
      y = ((y1 - (HGC_height / 4)) * HGC_scanlines) + 1;
      /*    87 -> 1    100 -> 
       *    88 -> 5
       *    89 -> 9
       *    90 -> 13
       *    
       *    100 -> ((100 - 100) * 4) + 1 = 1
       *    101 -> ((101 - 100) * 4) + 1 = 5
       */

      /*    87 -> 1    100 ->
       *    88 -> 5
       *    89 -> 9
       *    90 -> 13
       *    
       *    100 -> ((100 - 100) * 3) + 1 = 1
       *    101 -> ((101 - 100) * 3) + 1 = 4
       */
    }
    // when y1 is >= 174 and <= 260, subtract 174 (to get back to 0), then multiply by 4 and add 2 to get lines 2,6,10,14...346
    //else if (y1 >= 174 && y1 <= 260)
    else if (y1 >= (HGC_height / 2) && y1 <= (((HGC_height / 2) + (HGC_height / 4)) - 1)) // in 640x400 mode:(y1 >= 200 && y1 <= 299)
    //else if (y1 >= (HGC_height / 2) && y1 <= (((HGC_height / 2) + (HGC_height / HGC_scanlines)) - 1)) // in 640x400 mode:(y1 >= 200 && y1 <= 299)
    {
      //y = ((y1 - (HGC_height / 2)) * 4) + 2;
      y = ((y1 - (HGC_height / 2)) * HGC_scanlines) + 2;
      /*    174 -> 2
       *    175 -> 6
       *    176 -> 10
       *    177 -> 14
       *    
       *    200 -> ((200 - 200) * 4) + 2 = 2
       *    201 -> ((201 - 200) * 4) + 2 = 6
       *    
       *    200 -> ((200 - 200) * 3) + 2 = 2
       *    201 -> ((201 - 200) * 3) + 2 = 5
       */
    }
    // when y1 is >= 261 and <= 347, subtract 261 (to get back to 0), then multiply by 4 and add 3 to get lines 3,7,11,15...347
    //else if (y1 >= 261 && y1 <= 347)
    else if (y1 >= ((HGC_height / 2) + (HGC_height / 4)) && y1 <= (HGC_height - 1)) // in 640x400 mode:(y1 >= 300 && y1 <= 399)
    //else if (y1 >= ((HGC_height / 2) + (HGC_height / HGC_scanlines)) && y1 <= (HGC_height - 1)) // in 640x400 mode:(y1 >= 300 && y1 <= 399)
    // if HGC_scanlines == 3, need to skip this every 4th row (this row), and change y to what it would be if every 4th row didnt' exist.
    {
      if (HGC_scanlines == 3) return;  // no 4th line for 640x300 mode
      //y = ((y1 - (HGC_height / 2 + HGC_height / 4)) * 4) + 3;
      y = ((y1 - (HGC_height / 2 + HGC_height / 4)) * HGC_scanlines) + 3;
      /*    261 -> 3
       *    262 -> 7
       *    263 -> 11
       *    264 -> 15
       *    
       *    300 -> ((300 - 300) * 4) + 3 = 3
       *    301 -> ((301 - 300) * 4) + 3 = 7
       *    
       *    300 -> ((300 - 300) * 3) + 3 = 3
       *    301 -> ((301 - 300) * 3) + 3 = 6
       */
    } 

//    static int lastrow = -1;
//    //if((y <= 147 && y >= 100)) //&& y % 2 == 0) //test print top ten bottom and top even numbered rows' pixel values
//    {
//      if (y != lastrow)
//      {
//        printf("\nrow %03d: ", y);
//        lastrow = y;
//      }
//      //printf("%05d/%05d:%03d,%03d:%02x ", p, n, x, y, value);
//      //printf("%03d,%03d:%02x ", x, y, value);
//      printf("%03d:%02x ", x, value);
//    }

//    if (y == 348 || y == 349)
//    {
//      return;
//    }

    //printf("%d\n",y);
    //printf("%d,%d(%d) 0x%02x->0x%02x\n", x, y, y1, last_frameBuffer[p], m_frameBuffer[p]);
    set720x348pixel(x + 0, y, pix0);
    set720x348pixel(x + 1, y, pix1);
    set720x348pixel(x + 2, y, pix2);
    set720x348pixel(x + 3, y, pix3);
    set720x348pixel(x + 4, y, pix4);
    set720x348pixel(x + 5, y, pix5);
    set720x348pixel(x + 6, y, pix6);
    if (HGC_width <= 640 || (x + 8) % 64 == 0) // only print this pixel if its (x coordinate + 1) is a multiple of 64 or we are in 640x300/400 modes
    {
      set720x348pixel(x + 7, y, pix7);
    }
  } 
}



void Machine::redrawScreen()
{
//  memset(bitTable, 1, 65536);
//  return;
  //s_videoMemory[address - 0xb0000] = value;  // fBuffer = s_videoMemory + n
  //uint32_t addr = address - 0xb0000;
  //uint32_t fbAddr = ((uint32_t)fBuffer - (uint32_t)s_videoMemory);

//  printf("HGC_scanlines = %d\n", HGC_scanlines);
//  printf("HGC_height = %d\n", HGC_height);
//  printf("HGC_width = %d\n", HGC_width);
 
  if (!graphics_mode) // text mode
  {
    uint32_t ascii = 0;
    uint32_t attribute = 0;
    int numbytes = 3999; // 80x25 text modes
    if (video_mode == CGA_40X25_GRAY || video_mode == CGA_40X25_COLOR)
    {
      numbytes = 1999;
    }     
    for (int n = 0; n <= numbytes; n += 2)
    {
      ascii = m_frameBuffer[n];
      attribute = m_frameBuffer[n + 1];
      printCharAttr(ascii, attribute, n / 2);
    }
    drawCursor(cursor_pos);
  }
  else //graphics mode
  {
    int numbytes = 16384;
    if (video_mode == CGA_320X200_COLOR || video_mode == CGA_320X200_GRAY || video_mode == CGA_640X200_MONO)
    {
      numbytes = 16384;
    }
    if (video_mode == HGC_720X348_MONO)
    {
      numbytes = 65536;  
      //numbytes = 32768; 
    }
    if (video_mode == MCGA_320X200_COLOR)
    {
      numbytes = 64000;
    }
    if (video_mode == MCGA_640X480_MONO)
    {
      numbytes = 38400;
    }    
    for (int n = 0; n < numbytes; n++)
    {         
        updateGraphicsByte(n, m_frameBuffer[n]);   
    }
  }
  memset(bitTable, 1, 65536);
  screenDirty = true;
  videoMemoryUpdated = true;
//  screenDirty = true;
//  global_need_refresh = true;
//  last_screen_change_time = millis();
//  
}




void Machine::drawBorders()
{
  //large contiguous black areas seem hard on screen, causing fast image degredation.  Always fill unused area with white
  // also draw black border below and right of active display area
  display.fillRect(640, 0, 8, SCREEN_PIXEL_HEIGHT, GxEPD_WHITE); //narrow strip on right
  if (!graphics_mode)
  {
    display.fillRect(0, HIRES_FONT_HEIGHT * 25, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT - (HIRES_FONT_HEIGHT * 25), GxEPD_WHITE);
    //printf("display.fillRect(%d, %d, %d, %d, %d)\n", 0, HIRES_FONT_HEIGHT * 25, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT - (HIRES_FONT_HEIGHT * 25), GxEPD_WHITE);
    display.drawLine(641, 0, 641, 401, GxEPD_BLACK);
    display.drawLine(0, 401, 641, 401, GxEPD_BLACK);
  }
  else if (video_mode == HGC_720X348_MONO)
  {
    if (HGC_width == 720)
    {
      display.fillRect(0, 348, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT - 348, GxEPD_WHITE);
      display.drawLine(641, 0, 641, 349, GxEPD_BLACK);
      display.drawLine(0, 349, 641, 349, GxEPD_BLACK);
    }
    else if (HGC_width == 640 && HGC_scanlines == 4)
    {
      display.fillRect(0, 400, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT - 400, GxEPD_WHITE);
      display.drawLine(642, 0, 642, 400, GxEPD_BLACK);
      display.drawLine(0, 401, 642, 401, GxEPD_BLACK);
    }
    else if (HGC_width == 640 && HGC_scanlines == 3)
    {
      display.fillRect(0, 300, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT - 300, GxEPD_WHITE);
      display.drawLine(642, 0, 642, 300, GxEPD_BLACK);
      display.drawLine(0, 301, 642, 301, GxEPD_BLACK);
    }
  }
  else if (video_mode == MCGA_640X480_MONO) 
  {
    display.drawLine(641, 0, 641, 401, GxEPD_BLACK);  
  }
  else // graphics mode that fills 640x400 in the physical end
  {
    display.fillRect(0, 400, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT - 400, GxEPD_WHITE);
    display.drawLine(641, 0, 641, 401, GxEPD_BLACK);
    display.drawLine(0, 401, 641, 401, GxEPD_BLACK);
  }
}

//static void adlib_task(void *context)
//{
//  while(true)
//  {
//    if (getAdlibEnabled())
//    {
//      updateAdlibTimer(); 
//      outputAdlibFrame(); 
//    }
//    vTaskDelay(1);
//  }
//}

static void video_task(void *context)
{
  auto m = (Machine*)context;
  printf("video_task running on core %d\n", xPortGetCoreID());

  bitTable = (uint8_t*)(m->memory() + RAM_SIZE);
  //ibox memory should be (uint8_t *)(bitTable + 65536);
  //iBoxMemory = (uint8_t *)(bitTable + 65536);
  
  memset(bitTable, 0, 65536);  //wasting loads (seven-eighths to be precise) of RAM.  Should use a 8192 byte and check each bit.  8192 might also fit in normal RAM without having to access slow PSRAM, making this even faster
  /*
   *  
      s_videoMemory[address - 0xb0000] = value;  // fBuffer = s_videoMemory + n
      uint16_t addr = address - 0xb0000;
      uint32_t fbAddr = ((uint32_t)fBuffer - (uint32_t)s_videoMemory);
   */
  if(start_refresh)
  {
    start_refresh = false;
    m->redrawScreen();
    m->drawBorders();
    //display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT); // full screen partial update
    display.displayFast(smallX, smallY, largeX - smallX, largeY - smallY); // changed portion of screen partial update
    lastDisplayFastTime = millis();
    smallX = 640;
    smallY = 480;
    largeX = 0;
    largeY = 0;
//    //vTaskDelay(100);
//    while(m->epd_busy());
//    display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT); // full screen partial update
//    while(m->epd_busy());
  }
            

   
  uint32_t bt_ascii = 0;
  uint32_t bt_attribute = 0;
//  int numbytes = 3999; // 80x25 text modes
//  if (video_mode == CGA_40X25_GRAY || video_mode == CGA_40X25_COLOR)
//  {
//    numbytes = 1999;
//  } 
  int numbytes = 3999;
  while(true)
  {  
    // lock current cursor position for this iteration into a variable that won't be changed externally during the operation
    
    //check for change in video mode, if video mode changed, clear screen contents
    if (need_change_video_mode)
    {
      need_change_video_mode = false;
      uint32_t fillcolor = bg_color; // text mode
      if (graphics_mode) //graphics mode defaults to black background, so use reverse background
      {
        fillcolor = fg_color;  
      }
      for (int y = 0; y < 480; y++)
      {
        for (int x = 0; x < 648; x++)
        {
          display.drawPixel(x, y, fillcolor);
        }
      }
      m->drawBorders();
    }
      
    //check for change in cursor position

    //check for changes in screen contents
    for (int n = 0; n < 65536; n++)
    {
      while(videoTaskSuspended)
      {
        vTaskDelay(50);  
      }
      if(bitTable[n] == 1)  //This crashes during title screen of Space Quest 3 when selected the non-roland midi device and playing MIDI music, even if ibox menu has MIDI disabled
      { 
        
        bitTable[n] = 0; //revert this back to zero immediately, hoping we don't lose any more immediately coming changes to this "bit"/memory location flag
        
        uint16_t addr = n - (m->m_frameBuffer - m->s_videoMemory);
        
        //printf("addr = %d\n", addr);
        if (!graphics_mode) // text mode
        {
          if (addr % 2 == 0) // only update changes for even bytes to the screen, because odd bytes are attribtes
          {
//            if(m->s_videoMemory[n] == 0)
//            {
//              printf("VT: [0](%02x:%02x)<-%d->%d %d %d %d, %d = %d - (%d - %d)\n", m->s_videoMemory[n], m->s_videoMemory[n + 1], n, addr, m->m_CGAMemoryOffset, m->m_frameBuffer, m->s_videoMemory, addr, n, m->m_frameBuffer, m->s_videoMemory);
//            }
//            else
//            {
//              printf("VT: %c(%02x:%02x)<-%d->%d %d %d %d, %d = %d - (%d - %d)\n", m->s_videoMemory[n], m->s_videoMemory[n], m->s_videoMemory[n + 1], n, addr, m->m_CGAMemoryOffset, m->m_frameBuffer, m->s_videoMemory, addr, n, m->m_frameBuffer, m->s_videoMemory);
//            }
            numbytes = 3999; // 80x25 text modes
            if (video_mode == CGA_40X25_GRAY || video_mode == CGA_40X25_COLOR)
            {
              numbytes = 1999;
            }
            if (addr <= numbytes)
            { 
              bt_ascii = m->s_videoMemory[n]; 
              bt_attribute = m->s_videoMemory[n + 1];
              
              m->printCharAttr(bt_ascii, bt_attribute, addr / 2);
              //printf("PC: %c(%02x:%02x)<-%d\n", bt_ascii, bt_ascii, bt_attribute, addr);
            }
          }
        }
        else //graphics mode
        {
//          int numbytes = 16384;
//          if (video_mode == CGA_320X200_COLOR || video_mode == CGA_320X200_GRAY || video_mode == CGA_640X200_MONO)
//          {
//            numbytes = 16384;
//          }
//          if (video_mode == HGC_720X348_MONO)
//          {
//            numbytes = 32768;  
//          }
//          if (video_mode == MCGA_320X200_COLOR)
//          {
//            numbytes = 64000;
//          }
//          if (video_mode == MCGA_640X480_MONO)
//          {
//            numbytes = 38400;
//          }
          //printf("%d:%02x\n", addr, m->s_videoMemory[n]);     
          updateGraphicsByte(addr, m->s_videoMemory[n]);   
        }
        screenDirty = true;
      }
    }
    
    //if(enableVideo)
    {
      // update display if needed
      //if ((screenDirty && millis() - last_checkscreen_time >= 500) || (millis() - last_checkscreen_time >= 2000)) //only check if screen contents has been updated, or 2s have elapsed since last check
      while(videoTaskSuspended)
      {
        vTaskDelay(50);  
      }
      if(
        screenDirty 
        || cursor_pos != last_cursor_pos
        && enableVideo
  //      && (
  //           (millis() - m->last_checkscreen_time >= 650) ||
  //           (keyspressed > 0 && millis() - last_key_time > 50)
  //         )
        ) //only check if screen contents has been updated, or 2s have elapsed since last check
      {
        //printf("cursor_pos = %d, last_cursor_pos = %d\n", cursor_pos, last_cursor_pos);
        //printf("screen check\n");
        
        while(videoTaskSuspended)
        {
          vTaskDelay(50);  
        }
        //if(!m->epd_busy())
        {
          bool needcheck = false;
          uint32_t sinceLastKey = millis() - last_key_time;
          //printf("sinceLastKey = %d\n", sinceLastKey);

//          if(keyspressed <= 0 && millis() - m->last_checkscreen_time >= 300) // no keys have been pressed and its been over 300ms since last check, just check to see if need to update screen image
//          {
//            needcheck = true;
//            printf("nokeys\n");
//          }
//          else if(keyspressed > 0 && (sinceLastKey > 350))  //at least one keypress, but it has been more than 0.1s since the last key was pressed, probably a pause/stop in typing, good chance to refresh. 
//          {
//            needcheck = true;
//            printf("lastkey > 350\n");
//          }
//          //else if(keyspressed > 0 && (sinceLastKey > 100)) //or 1 or 2 keys have been pressed within 0.1s (user just started typing, show immediate response)
//          else if(keyspressed > 0 && keyspressed <= 6 && sinceLastKey > 50) // 1 to 6 key presses and releases (up to 3 keys pressed/released total) and the last one was at least 50ms ago
//          {
//            needcheck = true;
//            printf("keyspressed 1-6\n");
//          }
//          else if(keyspressed > 10) // or more than 10 key presses and releases (5 keys pressed and released) since last screencheck
//          {
//            needcheck = true;
//            printf("keyspressed > 10 (%d)\n", keyspressed);
//          }
//          else if(keyspressed <= 0 && millis() - m->last_checkscreen_time >= 3000)
//          {
//            needcheck = true;
//            printf("lastcheck > 3000\n");
//          }
          while(videoTaskSuspended)
          {
            vTaskDelay(50);  
          }
          if (millis() - lastDisplayFastTime > 310) // BUSY lasts for at most 351ms, and need to write data for 56ms before BUSY starts, so if 310 seconds have gone by since last BUSY started, and we still have to write data for 56ms, 310+56 = 366, which means 366ms will have elapsed since last BUSY started, which means last BUSY should have ended by then.
          //if (millis() - lastDisplayFastTime > 100) // BUSY lasts for at most 351ms, and need to write data for 56ms before BUSY starts, so if 310 seconds have gone by since last BUSY started, and we still have to write data for 56ms, 310+56 = 366, which means 366ms will have elapsed since last BUSY started, which means last BUSY should have ended by then.
          {
            needcheck = true;
          }
          else
          {
            videoMemoryUpdated = true; //set flag so that process will immediately go another round and finish displaying new content.
          }
          if(needcheck) //if need to check if screen needs updating
          {
            keyspressed = 0;
            last_last_key_time = last_key_time;
            //printf("%d:sd=%d,kp=%d,lkt=%d\n", millis(), screenDirty, keyspressed, last_key_time);
            unsigned char ascii;
      
            unsigned char attribute;
          
            static bool need_refresh = false;
            
            // If in text mode, check if cursor location has changed since last refresh
            // redraw cursor if needed
            while(videoTaskSuspended)
            {
              vTaskDelay(50);  
            }
            if (need_cursor_update && !graphics_mode)
            {
              //redraw cursor
              // first "erase" current cursor by re-printing the character at it's location
              int n = last_cursor_pos * 2;
              //printf("ascii = m_frameBuffer[%d];\n", n);
              ascii = m->m_frameBuffer[n];
              //printf("attribute = m_frameBuffer[%d];\n", n + 1);
              attribute = m->m_frameBuffer[n + 1];
              //printf("erasing old cursor with printCharAttr(%d, %d, %d)\n", ascii, attribute, n / 2);
              //printf("printCharAttr(%d, %d, %d);\n", ascii, attribute, n / 2);
              m->printCharAttr(ascii, attribute, n / 2);
              //last_cursor_pos = cursor_pos;
          
              // then draw cursor at new position
              // cursor_pos may have already been updated since we started this iteration of video_task, resulting in the cursor already being in a new location that is not the same location as the latest character printed to the screen before we started this iteration.  This causes the cursor to display several "spaces" ahead of the character printing when a lot of text is being quickly printed to the screen.
              // I should add a few bytes
              drawCursor(cursor_pos);
              
              need_cursor_update = false;
              need_refresh = true;   
              global_need_refresh = true;     
            }
            while(videoTaskSuspended)
            {
              vTaskDelay(50);  
            }
            last_cursor_pos = cursor_pos;
            //printf("need_refresh = %d, global_need_refresh = %d, waiting_for_initPartialUpdate = %d, epd_busy() = %d\n", need_refresh, global_need_refresh, waiting_for_initPartialUpdate, epd_busy());
//            if (waiting_for_initPartialUpdate) // && !epd_busy())
//            {
//              //didn't send 0x92 command to screen after displayFast, now not busy so send it.
//              //printf("display.epd2.writeCommand(0x92);\n");
//              waiting_for_initPartialUpdate = false;
//              display.epd2.writeCommand(0x92);
//              delay(1);
//              need_fullscreen_refresh = true;
//            }

            while(videoTaskSuspended)
            {
              vTaskDelay(50);  
            }
            //if ((need_refresh || global_need_refresh)) // && !m->epd_busy())  //refresh everything, including cursor area, when !epd_busy()
            {
              uint32_t startdisplaytime = millis();
              int oldChangedPixels = changedPixels;
              cumulativeChangedPixels += changedPixels;
              //CPT = 1000000; // was 40000 before fixed epd's boost settings
              CPT = 40000; // was 40000 before fixed epd's boost settings
              if (!graphics_mode)
              {
                CPT = 1000000;
              }
              need_refresh = false;
              global_need_refresh = false;
              screenDirty = false;
              //printf("%s line %d, screenDirty = false\n", __FILE__, __LINE__);
              //last_screen_change_time = 0xffffffff;
              
              //printf("display.displayFast(), rt=%lu, CP = %u, CCP = %u\n", millis() - startdisplaytime, changedPixels, cumulativeChangedPixels);
              
              //m->redrawScreen();
              static uint8_t borderCount = 0;
              if (borderCount % 50 == 0)
              {
                m->drawBorders();
              }
              borderCount++;
              
              //while(millis() - lastDisplayFastTime < 270);
              //while(m->epd_busy());
              while(videoTaskSuspended)
              {
                vTaskDelay(50);  
              }
              m->last_checkscreen_time = millis();
              uint32_t startTime = millis();
              display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT); // full screen partial update
              lastDisplayFastTime = millis();
              //display.displayFast(smallX, smallY, largeX - smallX, largeY - smallY); // changed portion of screen partial update
              //display.displayWindow(smallX, smallY, largeX - smallX, largeY - smallY);
              smallX = 640;
              smallY = 480;
              largeX = 0;
              largeY = 0;
              static uint32_t lastmillis = millis();
              printf("%dms: used %dms\n", millis() - lastmillis, millis() - startTime); // usually about 60ms without drawBorders(), 108ms with it.  Drawborders takes about 48ms
              lastmillis = millis();
              last_screen_change_time = millis();
              
              if (cumulativeChangedPixels > CPT)
              {
                refreshNow = true;
                printf("> CPT!!! rt=%lu, CP = %u, CCP = %u\n", millis() - startdisplaytime, oldChangedPixels, cumulativeChangedPixels);
              }
              
              lastChangedPixels = changedPixels;
              changedPixels = 0;
              oldChangedPixels = 0;
              waiting_for_initPartialUpdate = true; //didn't send 0x92 command to screen after displayFast, need to wait until not busy to send it.
              delay(1);
              need_fullscreen_refresh = true;   
            }
            //m->last_checkscreen_time = millis();
          }      
        }    
      }
    }
//    else
//    {
//      printf("!enableVideo\n");
//    }
    // if keyboard idle for 10 seconds and screen idle for 20 seconds, and at least 30 seconds since last refresh screen
    if (
      (keyspressed == 0 && millis() - last_key_time > 10000 && 
      millis() - last_screen_change_time > 10000 && 
      need_fullscreen_refresh &&
      millis() - last_fullscreen_refresh > 10000)
    )
    {
      need_fullscreen_refresh = false;
      waiting_for_epd_clean = true;
      last_fullscreen_refresh = millis();    //comment out for debugging, normally use  
      refreshNow = false;
    }
    //printf("(1) wfec = %d, epd_busy() = %d\n", waiting_for_epd_clean, epd_busy());
    if (waiting_for_epd_clean) // && !epd_busy())
    {
      //old slow way
      
      m->drawBorders();
      uint32_t startRefreshTime = millis();
      //printf("PIC_display_Clear()\n");
      PIC_display_Clear();
      display.displayStayOn(false);
      printf("refresh used %lums\n", millis() - startRefreshTime);
      
      helpers = 0;
      changedPixels = 0;
      need_fullscreen_refresh = false;
      last_fullscreen_refresh = millis();    //comment out for debugging, normally use   
      waiting_for_epd_clean = false;
      last_screen_change_time = millis();
      waiting_for_initPartialUpdate = false;    
      refreshNow = false;
      cumulativeChangedPixels = 0;
    }  
    
    if (refreshNow)
    {
      //printf("refreshNow\n");
      if (
          (
            !m->epd_busy() && 
            millis() - lastDisplayFastTime > 100 && 
            //changedPixels < 5000) 
            changedPixels < 1000 &&
            (graphics_mode || (keyspressed <= 0 && (millis() - last_key_time > 500)))
          ) 
          ||
          //(graphics_mode && lastChangedPixels > CPT) || //if last screen update was huge
          (graphics_mode && cumulativeChangedPixels >= 2000000) || 
          (!graphics_mode && cumulativeChangedPixels >= 5000000 && keyspressed <= 0 && (millis() - last_key_time > 2000))
        )   
      {
        //if (graphics_mode)
        {

          uint32_t startRefreshTime = millis();
          while(m->epd_busy());   

          unsigned int i;
          //Write Data
          display.epd2.writeCommand(0x10);     
          for(i = 0;i < 38880; i++)       
          {
            display.epd2.writeData(0xff);  
          }
          delay(1);              //!!!The delay here is necessary, 200uS at least!!!   

          while(m->epd_busy());          //waiting for the electronic paper IC to release the idle signal

          display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT); // partial update
          lastDisplayFastTime = millis();
          lastInstantRefreshTime = millis();
          printf("instant refresh used %lums, CP=%u, CCP = %u\n", millis() - startRefreshTime, changedPixels, cumulativeChangedPixels);
        }
        cumulativeChangedPixels = 0;
        need_fullscreen_refresh = false;
        waiting_for_epd_clean = false; 
        refreshNow = false;  
      }
    }

    //delay for 50ms or until a key is pressed.
    //vTaskDelay(50);
    int startDelayTime = millis();
    //while (millis() - startDelayTime < 100)
    while(1)
    {
      vTaskDelay(10);
//      if (keyspressed > 0) //&& keyspressed < 4)
//      {
//        // this is wrong.  It shouldn't matter how many you pressed.  It should only matter how long it's been since you last pressed.
//        for (int n = 0; n < keyspressed; n++)
//        {
//          vTaskDelay(50);
//          if (n > 5)
//          {
//            break;
//          }
//        }
//        break;
//      }
      if (videoMemoryUpdated)
      {
        videoMemoryUpdated = false;
        while(millis() - lastDisplayFastTime < 300)
        {
          vTaskDelay(10);
        }
        break;
      }
      while(videoTaskSuspended)
      {
        vTaskDelay(50);  
      }
    }
    printf("%d $$\n", millis());
//    vTaskDelay(10);
////    while(millis() - lastDisplayFastTime < 270) // 267 = 341ms for epd_busy time - 65ms it takes to write data to screen 
////    {
////      vTaskDelay(10);
////      if (keyspressed > 0) //&& keyspressed < 4)
////      {
////        break;
////      }
////    }
    while(videoTaskSuspended)
    {
      vTaskDelay(50);  
    }
    //printf(".%d\n", millis());
  }
}


//static void idle_task(void *parm)
//{
//  printf("idle_task running on core ");
//  printf("%d\n", xPortGetCoreID());
//  while(1==1)
//  {
//    int64_t now = esp_timer_get_time();     // time anchor
//    vTaskDelay(0 / portTICK_RATE_MS);
//    int64_t now2 = esp_timer_get_time();
//    idle_cnt += (now2 - now) / 1000;        // diff
//    //idle_cnt++;
//    //printf("idle_cnt = %d\n", idle_cnt);
//  }
//}

//void Machine::testpresskey(char testkey)
//{
//  uint16_t syscode;
//  printf("sending '%c'\n", testkey);
//  if (testkey == 'e')
//  {
//    syscode = 0x1265;
//  }
//  if (testkey == 'w')
//  {
//    syscode = 0x1177;
//  }
//  if (testkey == 0x0A)
//  {
//    syscode = 0x1C0D;
//  }
//  VirtualKey vk = VirtualKey::VK_g;
//  VirtualKeyItem item;
//  item.vk         = vk;
//  item.down       = true;
//  item.CTRL       = false;
//  item.LALT       = false;
//  item.RALT       = false;
//  item.SHIFT      = false;
//  item.GUI        = false;
//  item.CAPSLOCK   = false;
//  item.NUMLOCK    = false;
//  item.SCROLLLOCK = false;
//  PS2Controller::keyboard()->postVirtualKeyItemPublic(item);
//  item.down = false;
//  PS2Controller::keyboard()->postVirtualKeyItemPublic(item);
////  PS2Controller::keyboard()->injectVirtualKey(vk, true, true);
////  PS2Controller::keyboard()->injectVirtualKey(vk, true, false);
//  //m_BIOS.storeKeyInKbdBufferPublic(syscode);
//  //m_BIOS.helpersEntry();
//  //m_BIOS.getKeyFromBufferPublic();
//  //keyboardInterrupt(this);   
//  
//  m_i8042.addKeybIntTrigs();
//  
//  //m_PIC8259A.forceSignalInterrupt(1);
//  //m_PIC8259A.ackPendingInterrupt();
//  /*
//  w = kbd store 1177
//  e = kbd store 1265
//  */
//}

void IRAM_ATTR ethRecvISR()
{
  ethInterrupt = true;
}

void Machine::tick()
{
  //printf("m_ticksCounter=%d\n", m_ticksCounter);
  ++m_ticksCounter;

//  #if ENABLE_SOUNDGEN
//  if ((m_ticksCounter % 20000 == 0))
//  {
//    //m_soundGen.dumpBuffer2((void *)&m_soundGen);
//    m_soundGen.dumpBuffer((void *)&m_soundGen);
//  }
//  #endif //ENABLE_SOUNDGEN

  if ((m_ticksCounter & 0x03) == 0x03) 
  {
    if (m_COM1.assigned())
    {
      m_COM1.tick();
    }
    
    if (networkMode != NETWORKMODE_ETH)
    {  
      if (m_COM2.assigned())
      {
        m_COM2.tick();
      }
    }
      
//    if (m_COM3.assigned())
//      m_COM3.tick();
      
    if ((m_ticksCounter & 0x7f) == 0x7f) 
    {
      //printf("m_PIT8253.tick()\n");
      m_PIT8253.tick();
      // run keyboard controller every PIT tick (just to not overload CPU with continous checks)
      m_i8042.tick();

      if (!ethSleep)
      {
        if (millis() - lastEthSendTime > 60000 && millis() - lastEthRecvTime > 60000)
        {
          ethGoToSleep();
        }
      }
    }
  }

  // 3000000 baud can go as fast as % 1 without int 0x04 problems, but can never get DHCP address
  // 460800 baud has int 0x04 problems at %1.  How fast can it go?  No int 0x04 problems at % 40000.  It an get DHCP address, but cannot connect to ftp server.
  // 460800 baud at %10000:  int 0x04 problems
  // 460800 baud at %20000: no int 0x04 problems.  Can get DHCP address.  
  // 1000000 baud at %20000: no int 0x04 problems
  if (networkMode == NETWORKMODE_ETH)
  {
    if (ethInterrupt)
    {
      ethInterrupt = false;
      //printf("34 LOW!\n");
      // send command to request global interrupt status:
      // 0x30 = CMD_GET_GLOB_INT_STATUS
      Serial2.write(0x57); // command prefix
      Serial2.write(0xAB); // command prefix
      Serial2.write(0x29); // CMD_GET_GLOB_INT_STATUS
      Serial2.write(0x00); // socket number 0x00
      while(!Serial2.available())
      {
        delay(1);
      }
      uint8_t tmpval = Serial2.read();
      //printf("ETHERNET PORT INTERRUPT, CMD_GET_GLOB_INT_STATUS received 0x%02X from ethernet port!\n", tmpval); 
  //          if(tmpval != 0x00)
  //          {
  //            printf("CMD_GET_GLOB_INT_STATUS received 0x%02X from ethernet port!\n", tmpval);
  //            //printf("(0x%02X & 0x10) = 0x%02X\n", tmpval, tmpval & 0x10);
  //          }
      
      if ((tmpval & 0x10) == 0x10) // bit 4 of return value set to 1 indicates there is an interrupt for socket 0
      {
        //printf("CMD_GET_GLOB_INT_STATUS received 0x%02X, BIT 4 SET!\n", tmpval);
  
        // send command to request global interrupt status:
        // 0x30 = CMD_GET_INT_STATUS_SN
        Serial2.write(0x57); // command prefix
        Serial2.write(0xAB); // command prefix
        Serial2.write(0x30); // CMD_GET_INT_STATUS_SN
        Serial2.write(0x00); // socket number 0x00
        while(!Serial2.available())
        {
          delay(1);
        }
        tmpval = Serial2.read();
        //printf("CMD_GET_INT_STATUS_SN received 0x%02X from ethernet port!\n", tmpval);
        if ((tmpval & 0x01) == 0x01)  // bit 2 is SINT_STAT_SENBUF_FREE
        {
          //printf("sendBuf free\n");
          ethSendbufFree = true;
        }
        
        if ((tmpval & 0x4) == 0x4) // bit 3 of return value set to 1 means SINT_STAT_RECV, indicates receive buffer has data waiting to be read.
        { 
          // send command to retrieve number of bytes available
          Serial2.write(0x57); // command prefix
          Serial2.write(0xAB); // command prefix
          Serial2.write(0x3B); // CMD_GET_RECV_LEN_SN
          Serial2.write(0x00); // socket number 0x00
          while(!Serial2.available())
          {
            delay(1);
          }
          uint8_t dataLenLow = Serial2.read();
          uint8_t dataLenHigh = Serial2.read();
          uint16_t numBytes = dataLenHigh;
          numBytes = numBytes << 8;
          numBytes += dataLenLow;
          if(numBytes > 1500)
          {
            numBytes = 1500;
            dataLenHigh = 0x05;
            dataLenLow = 0xDC;
          }
          //printf("%d bytes (%d %d) available to read from ethernet port\n", numBytes, dataLenLow, dataLenHigh);
  
          // send command to receive bytes waiting in ethernet port incoming buffer
          Serial2.write(0x57); // command prefix
          Serial2.write(0xAB); // command prefix
          Serial2.write(0x3C); // CMD_GET_RECV_LEN_SN
          Serial2.write(0x00); // socket number 0x00
          Serial2.write(dataLenLow); // data length low byte
          Serial2.write(dataLenHigh); // data length high byte
          //uint8_t ethBuf[numBytes];
          for(int n = 0; n < numBytes; n++)
          {
            int count = 0;
            if(!Serial2.available())
            {
              count++;
              if (count > 1000)
              {
                break;  //timeout after 1 second without receive data that ethernet port said it would be sending.
              }
              delay(1);
            }
            ethBuf[n] = Serial2.read();
          }
          //printf("\n");
          if (NE2000Adapter)
          {
            ne2000_receive(NE2000Adapter, ethBuf, numBytes);
          }
          //printf("Read %d bytes (%d %d) from ethernet port\n", numBytes, dataLenLow, dataLenHigh);
          printf("ETH read %dB", numBytes, dataLenLow, dataLenHigh);
          
          if (memcmp(ethBuf, mac_addr, 6) == 0) // get destination MAC address and check if it's ours
          {
            printf(" for this MAC addr");
            lastEthRecvTime = millis();
          }
          printf("\n");
          
  //        for (int n = 0; n < numBytes; n++)
  //        {
  //          if (n != 0)
  //          {
  //            if (n % 8 == 0)
  //            {
  //             printf("  ");
  //            }
  //            if (n % 16 == 0)
  //            {
  //              
  //              for (int i = n - 16; i < n; i++)
  //              {
  //                if (ethBuf[i] > 31 && ethBuf[i] < 127)
  //                {
  //                  printf("%c", ethBuf[i]);
  //                }
  //                else
  //                {
  //                  printf("%c", 255);
  //                }
  //              }
  //              printf("\n%02X: ", (n / 16));
  //            }
  //          }
  //          else
  //          {
  //            printf("\n%02X: ", 0);
  //          }
  //          printf("%02X ", ethBuf[n]);
  //        }
  //        printf("\n");
        }
      }
    }
  }




//  if ((m_ticksCounter % 20000 == 0))
//  {
//    printf("m_ticksCounter = %d, stepsCount = %d\n", m_ticksCounter, stepsCount);
//  }

#if ENABLE_SOUNDGEN
#if ENABLE_ADLIB
//  m_adlibSoundGenerator.updateAdlibTimer(); 
  if (m_ticksCounter % 20 == 0)
  {
    m_adlibSoundGenerator.updateAdlibTimer(); 
  }
#endif // ENABLE_ADLIB
#endif // ENABLE_SOUNDGEN  
  


  /* the following test shows a very stable difference of about 52ms (+-2) between every 1 million ticks when idling at the DOS prompt or
   constantly looping calculating square roots in qbasic.  This might be able to be used to detect idle for light sleep/hibernation purposes.
   */
  //if (powersaveEnabled && (m_ticksCounter % 10000 == 0))
  if (powersaveEnabled)
  if ((m_ticksCounter % 20000 == 0))
  {
    static bool runningLightOn = true;
    if ((m_ticksCounter % 200000 == 0 && !runningLightOn) ||  (m_ticksCounter % 20000 == 0 && runningLightOn))
    {
      runningLightOn = !runningLightOn;
    }
   
    //temporary for testing
//    if (hlts > 0)
//    {
//      printf("hlts = %d\n", hlts);
//    }
    if (helpers == 0)
    {
      zeroHelpers++;
      //printf("zeroHelpers = %d\n", zeroHelpers);
    }
    else
    {
      zeroHelpers = 0;
    }
    //if (helpers > 0)
//    {
//      printf("helpers = %d\n", helpers);
//    }
    //if (readkeys > 0)
//    {
//      printf("readkeys = %d\n", readkeys);
//    }
    //if (read8042s > 0)
//    {
//      printf("read8042s = %d\n", read8042s);
//    }  
    memsTotal += mems; // add most recent "mems" value to total
    memsTotal -= memsarr[memsIdx]; // subtract the "about to be overwritten", oldest "mems" value from total.  This way we keep an accurate total of the past 64 "mems" values, without having to re-add them together every cycle.
    memsarr[memsIdx] = mems;  // overwrite the oldest "mems" value with the newest.
    memsIdx++;
    if (memsIdx >= 64)
    {
      memsIdx = 0;
    }    
    uint32_t memsAvg = memsTotal >> 6;  //divide by 64 to get average
    //printf("memsAvg = %d\n", memsAvg);
    memsAvgTotal += memsAvg;
    memsAvgTotal -= memsAvgArr[memsAvgIdx];
    memsAvgArr[memsAvgIdx] = memsAvg;
    memsAvgIdx++;
    if(memsAvgIdx >= 16)
    {
      memsAvgIdx = 0;
    }
    uint32_t memsAvgAvg = memsAvgTotal >> 4;
    uint32_t memsAvgAvgMaxdiff = abs((int)memsAvg - (int)memsAvgAvg);
    static uint32_t memsAvgAvgMaxdiffConsec = 0;
    if (memsAvgAvgMaxdiff < 50)
    {
      memsAvgAvgMaxdiffConsec++;
    }
    else
    {
      memsAvgAvgMaxdiffConsec = 0;
    }
    //printf("helpers = %d, memsAvg = %d, memsAvgAvg = %d, <> %d, md = %d\n", helpers, memsAvg, memsAvgAvg, memsAvgAvgMaxdiff, memsAvgAvgMaxdiffConsec);
    
    //printf("mems = %d\n", mems);
//    uint32_t memsHigh = 0;
//    uint32_t memsLow = 9999999999;
//    for (int n = 0; n < 64; n++)
//    {
//      if (memsarr[n] > memsHigh) memsHigh = memsarr[n];
//      if (memsarr[n] < memsLow) memsLow = memsarr[n];
//      //printf("%d ", memsarr[n]);
//    }
//    printf("\n%d %d %d %d\n", memsAvg, memsLow, memsHigh, abs((int)memsHigh - (int)memsLow));

    // try commenting out below to stop auto-shutoff circuit from shutting off at startup:
    //ledcWrite(1, runningLightOn); //flash running light
    
    //printf("checking power saving conditions\n");
    static uint32_t tickspeed = 0;
    static uint32_t last_measure_tickspeed = 0;
    static bool sleeping = false;
    static uint32_t lastBusy = 0;
//    char pcWriteBuffer[400];
//    vTaskGetRunTimeStats(pcWriteBuffer);
//    printf("pcWriteBuffer = %s\n", pcWriteBuffer);
    //if (m_ticksCounter % 10000 == 0)
    {
      //printf("t %d\n", millis());
      tickspeed = millis() - last_measure_tickspeed;
      if (tickspeed > 40)
      {
        lastBusy = millis();
      }
      //if (tickspeed > 0)
//      {
//        printf("tickspeed = %d\n", tickspeed);
//      } 
      // I should calculate the recent frequency of querying keyboard input or reading keyboard buffer if possible to estimate if the program is just waiting for key input
      
//      //printf("idle_cnt: %d\n", idle_cnt);
//      int32_t idle_diff = 0;
//      static int32_t latest_idle_diff_time = 0;
//      static int32_t latest_idle_diff = 0;
//      delay(1);
//      if (last_idle_cnt > 0)
//      {
//        int32_t idle_diff = idle_cnt - last_idle_cnt;
//        if (idle_diff > 0)
//        {
//          printf("%d:%d\n", idle_diff, millis() - latest_idle_diff_time);
//          latest_idle_diff = idle_diff;
//          //printf("%dms since last idle_diff\n", millis() - latest_idle_diff_time);
//          latest_idle_diff_time = millis();
//        }
//      }
//      last_idle_cnt = idle_cnt;
      //printf("ts=%d\n", tickspeed);

      //printf("%d:%d\n", lastVideoMemoryWrite, lastVideoMemoryRead);
      // if screen is up to date and there has been no keyboard or screen activity for at least one second, and cpu seems idle, enter light sleep mode
      //printf("lkt:%d lvw:%d lvh:%d ldh:%d maamdc:%d\n", millis() - last_key_time, millis() - lastVideoWrite, millis() - lastVideoHandler, millis() - lastDiskHandler, memsAvgAvgMaxdiffConsec);
      //printf("1. %d:%d\n", int28s, int2fs);
      bool halfSatisfied = (
        !screenDirty 
        && 
          (
            (
              (millis() - last_key_time > 5000) 
              && (millis() - lastVideoWrite > 5000) 
              //&& (millis() - lastBusy > 2000)
              && (millis() - lastVideoHandler > 5000)
              && (millis() - lastDiskHandler > 5000)
              //&& (millis() - lastVideoMemoryWrite > 5000)
              //&& (millis() - lastVideoMemoryRead > 5000)  
              //&& abs((int)memsAvg - (int)memsAvgAvg) <= 10 
              && memsAvgAvgMaxdiffConsec >= 50         
              //&& (helpers > 0)
              
              /*
              //&& speakerEnabled == false
              //&& (millis() - lastVideoMemoryWrite > 1000)
              //&& (millis() - lastVideoMemoryRead > 1000)
              //&& need_fullscreen_refresh == false
              //&& refreshNow == false
              //&& global_need_refresh == false
              */              
            ) 
            || 
            hlts > 8000 || (helpers > /* 55 */ 96 && helpers < 250 && millis() - last_fullscreen_refresh > 1000) // || abs((int)memsAvg - (int)memsAvgAvg) <= 1 //|| zeroHelpers > 200
            || (int28s > 27 && (millis() - last_key_time > 1500))
          )
      );
      //printf("%d, %d, %d, %d, %d, %d, %d, %d\n", tickspeed, screenDirty, millis() - last_key_time, millis() - lastVideoWrite, millis() - lastBusy, millis() - lastVideoHandler, millis() - lastDiskHandler);

      bool fullSatisfied = (
        !screenDirty 
        && 
          (
            (
              (millis() - last_key_time > 500) 
              && (millis() - lastVideoWrite > 500)
              && (millis() - lastDiskHandler > 1500)
              //&& (millis() - lastVideoMemoryWrite > 1500)
              //&& (millis() - lastVideoMemoryRead > 1500)
              
              /*
              // && speakerEnabled == false // in PWR_SAVE_FULL mode, sleep even when sound is playing 
              */
            )
            || 
            (hlts > 8000) || (helpers > 60 && helpers < 250)  //|| abs((int)memsAvg - (int)memsAvgAvg) <= 1 //|| zeroHelpers > 200
            
          )
      );

      //printf("2. %d:%d\n", int28s, int2fs);
      if(
        (powersaveEnabled == PWR_SAVE_HALF && halfSatisfied)
        || (powersaveEnabled == PWR_SAVE_FULL && fullSatisfied)
        )
      {
        if (!sleeping)
        {
          //delay(10);
          //printf("3. %d:%d\n", int28s, int2fs);
          printf("SLEEP: hlt:%d hlp:%d mdf:%d lkt:%d lvw:%d lvh:%d int28s:%d int2fs:%d", hlts, helpers, zeroHelpers, memsAvgAvgMaxdiffConsec, millis() - last_key_time, millis() - lastVideoWrite, millis() - lastVideoHandler, millis() - lastDiskHandler, int28s, int2fs);
          //printf("4. %d:%d\n", int28s, int2fs);
          //printf("last_key_time:%d lastVideoWrite:%d lastVideoHandler:%d\n", millis() - last_key_time, millis() - lastVideoWrite, millis() - lastVideoHandler, millis() - lastDiskHandler);
      
//          if (i8086::halted() || hlts > 8000)
//          {
//            printf(" due to HLT, hlts = %d", hlts);
//          }
//          if (helpers > 40 && helpers < 250)
//          {
//            printf( " due to helpers = %d\n", helpers);
//          }
          //if (zeroHelpers > 200)
          //{
          //  printf( ", zeroHelpers = %d\n", zeroHelpers);
          //}
          zeroHelpers = 0;
          helpers = 0;
          memsAvgAvgMaxdiffConsec = 0;
          printf("\n");
          sleeping = true;
          //setCpuFrequencyMhz(10);  // valid values are 240, 160, 80, 40, 20, 10
          printf("5. %d:%d\n", int28s, int2fs);
        }
        lightSleep();
        sleeping = false; 
      }
      //printf("6. %d:%d\n", int28s, int2fs);  
       
      last_measure_tickspeed = millis();
     }
     hlts = 0;
     mems = 0;
     helpers = 0;
     readkeys = 0;
     read8042s = 0;
     printf("28:%d 2f:%d\n", int28s, int2fs);
     int28s = 0;
     int2fs = 0;
     //printf("hlts = 0\n");
  }

  
  if (m_PIC8259A.pendingInterrupt() && i8086::IRQ(m_PIC8259A.pendingInterruptNum()))
  {
    m_PIC8259A.ackPendingInterrupt();
  }
  if (m_PIC8259B.pendingInterrupt() && i8086::IRQ(m_PIC8259B.pendingInterruptNum()))
  {
    m_PIC8259B.ackPendingInterrupt();
  }
}

void Machine::lightSleep()
{
  delay(1);
  ulp_run(0);
  //if(powersaveEnabled == PWR_SAVE_HALF)
  {
    {
      esp_sleep_enable_timer_wakeup(60000000);   // 60 seconds
      //esp_sleep_enable_timer_wakeup(100000);   // 0.1 seconds
    }
  }
  //uint32_t sleep_time = millis();
  bool gpio_wake = false;

  //digitalWrite(21, LOW);
  
  // try commenting out below to stop auto-shutoff circuit from shutting off at startup:
  //ledcWrite(1, 0); //turn off running light
  
  //m_i8042.disableKeyboard(); 
  uint32_t lightSleepStart = millis();
  while(gpio_wake == false)  //wake up every 60 seconds to "keep everything fresh"
  {
    esp_light_sleep_start();
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER)
    {
      //printf("timer wakeup\n");
    }
    else
    {
      //m_i8042.enableKeyboard();
      gpio_wake = true;
      //digitalWrite(21, HIGH);
      
      // try commenting out below to stop auto-shutoff circuit from shutting off at startup:
      //ledcWrite(1, 1); // turn on running light
      
    }
    if (millis() - lightSleepStart > 300000) //if been light sleeping for over 5 minutes
    {
      hibernate();  //temporarily commented out to test light sleep power consumption
    }
  }
//        if(millis() - sleep_time > 3000) // if slept for more than 5 minutes, delay for a second to allow keystrokes to catch up
//        {
//          delay(1000);
//        }
  // the problem is that after sleeping for about 10 seconds, 
  // it takes about 2 seconds or longer (depending on how long sleeping?) to wake up.  
  // It needs to be instant.
  // When typing, keystrokes are missed if it takes longer than instantly to wake up.
  //setCpuFrequencyMhz(240); 
  printf("wake\n");
/*
      (keyspressed == 0 && millis() - last_key_time > 10000 && 
      millis() - last_screen_change_time > 10000 && 
      need_fullscreen_refresh &&
      millis() - last_fullscreen_refresh > 10000)
*/        
  last_key_time = millis();
  last_screen_change_time = millis();
  need_fullscreen_refresh = false;
  last_fullscreen_refresh = millis();
  waiting_for_epd_clean = false;  
}



void Machine::setMCGARegister(uint8_t value)
{
  //printf("setting m_MCGA[%d] = 0x%02x\n", m_MCGASelectRegister, value);
  m_MCGA[m_MCGASelectRegister] = value;
  switch (m_MCGASelectRegister) 
  {
    case 0x05:
      {
        egaWriteMode = value;
        //printf("egaWriteMode = 0x%02x\n", egaWriteMode);
        uint16_t old_MCGAMemoryOffset = m_MCGAMemoryOffset;
        m_MCGAMemoryOffset = (uint16_t)0xa0000;
        if ((SQRegisters[4] & 0x04) == 0)
        {
          setMCGAMode();
        }
        if(old_MCGAMemoryOffset != m_MCGAMemoryOffset)
        {
          videoTaskSuspended = true;
          redrawScreen();
          videoTaskSuspended = false;
        }
      }
      break;
    default:
      break;
  }
}

void Machine::setCGA6845Register(uint8_t value)
{
  m_CGA6845[m_CGA6845SelectRegister] = value;
  switch (m_CGA6845SelectRegister) 
  {
    // cursor start: (bits 5,6 = cursor blink and visibility control), bits 0..4 cursor start scanline
    case 0x0a:  // set cursor top line
      //m_graphicsAdapter.setCursorVisible((m_CGA6845[0xa] >> 5) >= 2);
       // 0x0a is cursor start register:
       /* 
        *  Cursor Start Register (R10) - This 7 bit write-only register controls the cursor format.
            Bit 5 is the blink timing control. When bit 5 is low, the blink frequency is 1/16 of the
            vertical field rate, and when bit 5 is high, the blink frequency is 1/32 of the vertical
            field rate. Bit 6 is used to enable a blink. The cursor start scan line is set by the
            lower 5 bits. (from Motorola 6845 CRT Controller datasheet)
        */
      //printf("cursor blinkbits = %d\n", m_CGA6845[0xa] >> 5);
      if ((m_CGA6845[0xa] >> 5) >= 2) // check value of highest 2 bits of cursor register.  
      {
        //printf("cursor on\n");
        if (!cursor_visible)
        {
          need_cursor_update = true;
          screenDirty = true;
          last_screen_change_time = millis();
        }
        cursor_visible = true;
      }
      else
      {
        //printf("cursor off\n");
        if (cursor_visible)
        {
          need_cursor_update = true;
          screenDirty = true;
          last_screen_change_time = millis();
        }        
        cursor_visible = false;
      }
      cursor_start_scanline = m_CGA6845[0xa] & 0x1f;
      //printf("CGA ssl = %d\n", cursor_start_scanline);
      // no break!
    // cursor end: bits 0..4 cursor end scanline
    case 0x0b:  // set cursor bottom line
      //m_graphicsAdapter.setCursorShape(2 * (m_CGA6845[0xa] & 0x1f), 2 * (m_CGA6845[0xb] & 0x1f));
      if ((cursor_start_scanline != (m_CGA6845[0xa] & 0x1f)) || (cursor_end_scanline != (m_CGA6845[0xb] & 0x1f)))
      {
        need_cursor_update = true;
        screenDirty = true;
        last_screen_change_time = millis();
      }
      //cursor_start_scanline = m_CGA6845[0xa] & 0x1f;
      cursor_end_scanline =   m_CGA6845[0xb] & 0x1f;
      //printf("CGA ssl = %d, esl = %d\n", cursor_start_scanline, cursor_end_scanline);
      //printf("CGA esl = %d\n", cursor_end_scanline);
      videoMemoryUpdated = true;
      break;

    // video memory start offset (0x0c = H, 0x0d = L)
    case 0x0c:
      {
        //m_CGAMemoryOffset = ((m_CGA6845[0xc] << 8) | m_CGA6845[0xd]) << 1; //td.exe (turbo debugger) breaks
        uint16_t old_CGAMemoryOffset = m_CGAMemoryOffset;
        m_CGAMemoryOffset = ((m_CGA6845[0xc] << 8) | m_CGA6845[0xd]); //dracula.exe (drakula.exe what?) castle.exe breaks
        printf("Setting m_CGAMemoryOffset(H) = 0x%02x\n", m_CGAMemoryOffset);
        setCGAMode();
        if(old_CGAMemoryOffset != m_CGAMemoryOffset)
        {
          videoTaskSuspended = true;
          // might need a delay here to avoid jumbled display
          redrawScreen();
          videoTaskSuspended = false;
        }
        videoMemoryUpdated = true;
      }
      break;
          
    case 0x0d:
      {
        //m_CGAMemoryOffset = ((m_CGA6845[0xc] << 8) | m_CGA6845[0xd]) << 1; //td.exe (turbo debugger) breaks
        uint16_t old_CGAMemoryOffset = m_CGAMemoryOffset;
        m_CGAMemoryOffset = ((m_CGA6845[0xc] << 8) | m_CGA6845[0xd]); //dracula.exe (drakula.exe what?) castle.exe breaks
        printf("Setting m_CGAMemoryOffset(L) = 0x%02x\n", m_CGAMemoryOffset);
        setCGAMode();
        if(old_CGAMemoryOffset != m_CGAMemoryOffset)
        {
          videoTaskSuspended = true;
          // might need a delay here to avoid jumbled display
          redrawScreen();
          videoTaskSuspended = false;
        }
        videoMemoryUpdated = true;
      }
      break;

    // cursor position (0x0e = H and 0x0f = L)
    case 0x0e:
    case 0x0f:
    {
      int pos = (m_CGA6845[0xe] << 8) | m_CGA6845[0xf];
      if (pos > 4801) pos = 4801; // large enough to fit VGA 80 col 60 row text mode (640x480, 8x8pixel chars)
      if (pos != cursor_pos)
      {
        need_cursor_update = true;
        screenDirty = true;
        last_screen_change_time = millis();
        //printf("CGA cursor_pos changed from %d to %d\n", cursor_pos, pos);
        cursor_pos = pos;
        lastVideoWrite = millis();
      }
      
      
      
      //printf("pos = %d, last = %d\n", pos, last_cursor_pos);
      //m_graphicsAdapter.setCursorPos(pos / m_graphicsAdapter.getTextColumns(), pos % m_graphicsAdapter.getTextColumns());
      videoMemoryUpdated = true;
      break;
    }
  }
}

void Machine::setMCGAMode()
{
  if ((SQRegisters[4] & 0x04) == 0)  // if
  {
    //printf("setMCGAMode() (unimplemented)\n");
    //return;
    printf("setMCGAMode()\n");
    printf("m_MCGA[5] = 0x%02x\n", m_MCGA[5]);
    if ((m_MCGA[5] & 0x40) != 0)
    {
      
      lastVideoWrite = millis();
      video_mode = MCGA_320X200_COLOR;
      graphics_mode = true;
      if (video_mode != last_video_mode)
      {
        printf("MCGA, 320x200 256 color mode\n");
        need_change_video_mode = true;
        // display.fillscreen(fg_color) needs to move to video_task
        //display.fillScreen(fg_color); // fill with fg_color because we swap fg/bg in graphics modes
        //screenDirty = true;
      }
      last_video_mode = video_mode;
      //m_frameBuffer = s_videoMemory;
      m_frameBuffer = MCGA_videoMemory;
      fBuffer = m_frameBuffer;
      cursor_visible = false;
      last_screen_change_time = millis();       
      //screenDirty; //redrawScreen();
      //while(epd_busy());
      //display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT);
      //changedPixels = 0;
      //waiting_for_initPartialUpdate = true;
      //need_fullscreen_refresh = true;
      enableVideo = true;
      videoMemoryUpdated = true;
    }
    else //if(m_MCGA[5] & 0x10 != 0)
    {
      
      lastVideoWrite = millis();
      video_mode = MCGA_640X480_MONO;
      graphics_mode = true;
      if (video_mode != last_video_mode)
      {
        printf("MCGA, 640x480 2 color mode\n");
        need_change_video_mode = true;
        // display.fillscreen(fg_color) needs to move to video_task
        //display.fillScreen(fg_color); // fill with fg_color because we swap fg/bg in graphics modes
        //screenDirty = true;
      }
      last_video_mode = video_mode;
      //m_frameBuffer = s_videoMemory + 0x8000 + m_CGAMemoryOffset;
      //m_frameBuffer = s_videoMemory;
      m_frameBuffer = MCGA_videoMemory;
      fBuffer = m_frameBuffer;
      cursor_visible = false;
      last_screen_change_time = millis();
      //screenDirty = true; //redrawScreen();
      //while(epd_busy());
      //display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT);
      //changedPixels = 0;
      waiting_for_initPartialUpdate = true;
      need_fullscreen_refresh = true;
      enableVideo = true;  
      videoMemoryUpdated = true;     
    }
  }
}

void Machine::setCGAMode()
{
  videoMemoryUpdated = true;
  if ((m_CGAModeReg & CGA_MODECONTROLREG_ENABLED) == 0) 
  {

    // video disabled
    printf("CGA, video disabled\n");
    m_frameBuffer = s_videoMemory + 0x8000 + m_CGAMemoryOffset;
    lastVideoWrite = millis();
    //display.fillScreen(bg_color);
    
    //graphics_mode = false; 
//    display.fillScreen(bg_color);
//    int b = 3999;
//    if (graphics_mode)
//    {
//      b = 16191;
//    }
//    for (int n = 0; n < b; n++) // clear the screen buffers
//    {
//      //m_frameBuffer[n] = 0;
//      last_frameBuffer[n] = 0; //force redraw of text locations
//    }     
    //m_graphicsAdapter.enableVideo(false);
    enableVideo = false;

  }
  else if ((m_CGAModeReg & CGA_MODECONTROLREG_TEXT80) == 0 && (m_CGAModeReg & CGA_MODECONTROLREG_GRAPHICS) == 0) 
  {

    // 40 column text mode
    printf("CGA, 40 columns text mode\n");
    lastVideoWrite = millis();
    m_frameBuffer = s_videoMemory + 0x8000 + m_CGAMemoryOffset;
    fBuffer = m_frameBuffer;
    last_screen_change_time = millis();
    video_mode = CGA_40X25_GRAY;
    graphics_mode = false;
    if (video_mode != last_video_mode)
    {
      need_change_video_mode = true;
    }
    last_video_mode = video_mode;
    display.setFont(&BIOS8x8);
    display.setTextSize(2);
    font_width = LORES_FONT_WIDTH;
    font_height = LORES_FONT_HEIGHT;
    font_baseline = LORES_FONT_BASELINE;      
    //screenDirty = true; //redrawScreen();
    enableVideo = true;
  } 
  else if ((m_CGAModeReg & CGA_MODECONTROLREG_TEXT80) && (m_CGAModeReg & CGA_MODECONTROLREG_GRAPHICS) == 0) 
  {
    // 80 column text mode
    printf("CGA, 80 column text mode\n");
    lastVideoWrite = millis();
    m_frameBuffer = s_videoMemory + 0x8000 + m_CGAMemoryOffset;    // use with mode 3 BIOS
    fBuffer = m_frameBuffer;
    last_screen_change_time = millis();
    //printf("m_frameBuffer = s_videoMemory + 0x8000 + m_CGAMemoryOffset;\n");
    //printf("m_frameBuffer = %p + 0x8000 + 0x%02x;\n", s_videoMemory, m_CGAMemoryOffset);
    //printf("m_frameBuffer = %p\n", m_frameBuffer);
  
    video_mode = CGA_80X25_GRAY;
    graphics_mode = false;
    if (video_mode != last_video_mode)
    {   
      need_change_video_mode = true;   
    }
    display.setFont(&VGA8x16);
    display.setTextSize(1);
    font_width = HIRES_FONT_WIDTH;
    font_height = HIRES_FONT_HEIGHT;
    font_baseline = HIRES_FONT_BASELINE; 
    last_video_mode = video_mode;
    screenDirty = true; //redrawScreen();
    enableVideo = true;

//    if (start_refresh)  // just do this once when first starting "machine"
//    {
//      //clearLastFrameBuffer();
//      //printf("redrawScreen() on line %d\n", __LINE__ + 1);
//      redrawScreen();
//      last_screen_change_time = millis();      
//      //start_refresh = false;
//      //while(epd_busy());
//      // display.anything() needs to move to video_task
//      //need_change_video_mode = true;
//      //screenDirty = true; 
//      display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT); // partial update
//      //vTaskDelay(5000);
//      changedPixels = 0;
//      //waiting_for_initPartialUpdate = true;
//    }
    //m_frameBuffer = s_videoMemory + 0x0000 + m_CGAMemoryOffset;  // use with mode 7 BIOS
  }
  else if ((m_CGAModeReg & CGA_MODECONTROLREG_GRAPHICS) && (m_CGAModeReg & CGA_MODECONTROLREG_GRAPH640) == 0) 
  {
    //AL = 0x05, INT 10
    // 320x200 graphics
    printf("CGA, 320x200 graphics mode\n");
    lastVideoWrite = millis();
    video_mode = CGA_320X200_GRAY;
    graphics_mode = true;
    if (video_mode != last_video_mode)
    {
      // display.anything() needs to move to video_task
      need_change_video_mode = true;
//      screenMutex = true;
//      screenDirty = true; //display.fillScreen(fg_color); // fill with fg_color because we swap fg/bg in graphics modes
//      screenMutex = false;
//      screenDirty = true;
    }
    last_video_mode = video_mode;
    m_frameBuffer = s_videoMemory + 0x8000 + m_CGAMemoryOffset;
    fBuffer = m_frameBuffer;
    cursor_visible = false;
    last_screen_change_time = millis();  
    enableVideo = true;     
    //screenDirty = true; //redrawScreen();
    //while(epd_busy());
    //display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT);
    //changedPixels = 0;
    //waiting_for_initPartialUpdate = true;
    //need_fullscreen_refresh = true;         
    
    //int paletteIndex = (bool)(m_CGAColorReg & CGA_COLORCONTROLREG_PALETTESEL) * 2 + (bool)(m_CGAColorReg & CGA_COLORCONTROLREG_HIGHINTENSITY);
  }
  else if ((m_CGAModeReg & CGA_MODECONTROLREG_GRAPHICS) && (m_CGAModeReg & CGA_MODECONTROLREG_GRAPH640)) 
  {
    //AL = 0x06, INT 10
    // 640x200 graphics
    printf("CGA, 640x200 graphics mode\n");
    lastVideoWrite = millis();
    video_mode = CGA_640X200_MONO;
    graphics_mode = true;
    if (video_mode != last_video_mode)
    {
      need_change_video_mode = true;
      // display.anything() needs to move to video_task
//      screenMutex = false;
//      screenDirty = true; //display.fillScreen(fg_color); // fill with fg_color because we swap fg/bg in graphics modes
//      screenMutex = false;
//      screenDirty = true;
    }
    last_video_mode = video_mode;
    m_frameBuffer = s_videoMemory + 0x8000 + m_CGAMemoryOffset;
    fBuffer = m_frameBuffer;
    cursor_visible = false;
    enableVideo = true;
    //last_screen_change_time = millis();
    //waiting_for_initPartialUpdate = true;
    //need_fullscreen_refresh = true;         
    //screenDirty = true; //redrawScreen();
    //while(epd_busy());
    //display.displayFast(0, 0, SCREEN_PIXEL_WIDTH, SCREEN_PIXEL_HEIGHT);
    //changedPixels = 0;      
  }
}


void Machine::setHGC6845Register(uint8_t value)
{
  videoMemoryUpdated = true;
  m_HGC6845[m_HGC6845SelectRegister] = value;
  printf("Setting HGC register %d = %d\n", m_HGC6845SelectRegister, value);
  switch (m_HGC6845SelectRegister) 
  {

    // cursor start: (bits 5,6 = cursor blink and visibility control), bits 0..4 cursor start scanline
    case 0x0a:
      //m_graphicsAdapter.setCursorVisible((m_HGC6845[0xa] >> 5) >= 2);
       // 0x0a is cursor start register:
       /* 
        *  Cursor Start Register (R10) - This 7 bit write-only register controls the cursor format.
            Bit 5 is the blink timing control. When bit 5 is low, the blink frequency is 1/16 of the
            vertical field rate, and when bit 5 is high, the blink frequency is 1/32 of the vertical
            field rate. Bit 6 is used to enable a blink. The cursor start scan line is set by the
            lower 5 bits. (from Motorola 6845 CRT Controller datasheet)
        */
      //printf("cursor blinkbits = %d\n", m_CGA6845[0xa] >> 5);
      if ((m_HGC6845[0xa] >> 5) >= 2) // check value of highest 2 bits of cursor register.  
      {
        //printf("cursor on\n");
        if (!cursor_visible)
        {
          need_cursor_update = true;
          screenDirty = true;
          last_screen_change_time = millis();
          lastVideoWrite = millis();
        }
        cursor_visible = true;
      }
      else
      {
        //printf("cursor off\n");
        if (cursor_visible)
        {
          need_cursor_update = true;
          screenDirty = true;
          last_screen_change_time = millis();
          lastVideoWrite = millis();
        }        
        cursor_visible = false;
      }
      cursor_start_scanline = m_HGC6845[0xa] & 0x1f;
      //printf("ssl = %d\n", cursor_start_scanline);
      // no break!

      
    // cursor end: bits 0..4 cursor end scanline
    case 0x0b:
      //m_graphicsAdapter.setCursorShape((m_HGC6845[0xa] & 0x1f), (m_HGC6845[0xb] & 0x1f));
      if ((cursor_start_scanline != (m_HGC6845[0xa] & 0x1f)) || (cursor_end_scanline != (m_HGC6845[0xb] & 0x1f)))
      {
        need_cursor_update = true;
        screenDirty = true;
        last_screen_change_time = millis();
        lastVideoWrite = millis();
      }
      //cursor_start_scanline = m_HGC6845[0xa] & 0x1f;
      cursor_end_scanline =   m_HGC6845[0xb] & 0x1f;
      //printf("HGC ssl = %d, esl = %d\n", cursor_start_scanline, cursor_end_scanline);
      //printf("esl = %d\n", cursor_end_scanline);
      
      break;

    // video memory start offset (0x0c = H, 0x0d = L)
    case 0x0c:
    case 0x0d:
      {
        uint16_t old_HGCMemoryOffset = m_HGCMemoryOffset;
        m_HGCMemoryOffset = ((m_HGC6845[0xc] << 8) | m_HGC6845[0xd]) << 1;
        printf("Setting m_HGCMemoryOffset = 0x%02x\n", m_HGCMemoryOffset);
        lastVideoWrite = millis();
        if(old_HGCMemoryOffset != m_HGCMemoryOffset)
        {
          videoTaskSuspended = true;
          redrawScreen();
          videoTaskSuspended = false;
        }
  //      screenDirty = true;
  //      last_screen_change_time = millis();
  //      global_need_refresh = true;
  //      last_screen_change_time = millis();
        setHGCMode();
      }
      break;
    // cursor position (0x0e = H and 0x0f = L)
    case 0x0e:
    case 0x0f:
    {
      int pos = (m_HGC6845[0xe] << 8) | m_HGC6845[0xf];
      if (pos != cursor_pos)
      {
        need_cursor_update = true;
        screenDirty = true;
        last_screen_change_time = millis();
        //printf("HGC cursor_pos changed from %d to %d\n", cursor_pos, pos);
        cursor_pos = pos;
        lastVideoWrite = millis();
      }      
      //m_graphicsAdapter.setCursorPos(pos / m_graphicsAdapter.getTextColumns(), pos % m_graphicsAdapter.getTextColumns());
      break;
    }
    case 0x01:
      HGC_width = value * 16;
      printf("HGC_width = %d\n", HGC_width);
      break;
    case 0x06:
      HGC_height = value * 4;
      //HGC_height = value * HGC_scanlines;
      //HGC_height = value * m_HGC6845[0x09];
      printf("HGC_height = %d\n", HGC_height);
      break;
    case 0x09: // set number of scan lines per HGC "row", minus 1.  For 720x348 mode, this is 3 (since number of scan lines is 4). 
      //int HGC_height_fraction = HGC_height / HGC_scanlines;
      HGC_scanlines = value + 1; // scanlines per row
      //HGC_height = HGC_height_fraction * HGC_scanlines;
      //HGC_height = m_HGC6845[0x06] * HGC_scanlines;
      printf("HGC_scanlines = %d\n", HGC_scanlines);
      break;
  }
}


void Machine::setHGCMode()
{
  videoMemoryUpdated = true;
  constexpr int HGC_OFFSET_PAGE0 = 0x0000;
  constexpr int HGC_OFFSET_PAGE1 = 0x8000;
  
  if ((m_HGCModeReg & HGC_MODECONTROLREG_ENABLED) == 0) 
  {
    // video disabled
    printf("Hercules, video disabled\n");
    lastVideoWrite = millis();
    //m_graphicsAdapter.enableVideo(false);
    enableVideo = false;
  }
  else if ((m_HGCModeReg & HGC_MODECONTROLREG_GRAPHICS) == 0 || (m_HGCSwitchReg & HGC_CONFSWITCH_ALLOWGRAPHICSMODE) == 0) 
  {
//    printf("m_HGCModeReg = %d\n", m_HGCModeReg);
//    printf("HGC_MODECONTROLREG_GRAPHICS = %d\n", HGC_MODECONTROLREG_GRAPHICS);
//    printf("m_HGCSwitchReg = %d\n", m_HGCSwitchReg);
//    printf("HGC_CONFSWITCH_ALLOWGRAPHICSMODE = %d\n", HGC_CONFSWITCH_ALLOWGRAPHICSMODE);
//    // text mode
    printf("Hercules, text mode\n");
    lastVideoWrite = millis();
    m_frameBuffer = s_videoMemory + HGC_OFFSET_PAGE1;  //this should probably be dynamically chosen between page 1 and 0, not fixed at page 1
    fBuffer = m_frameBuffer;
    //printf("m_frameBuffer = s_videoMemory + HGC_OFFSET_PAGE1;\n");
    //printf("m_frameBuffer = %p + 0x%02x;\n", s_videoMemory, HGC_OFFSET_PAGE1);
    printf("m_frameBuffer = %p\n", m_frameBuffer); 
       
    video_mode = MDA_80X25_MONO;
    graphics_mode = false;
    if (video_mode != last_video_mode)
    {
      // display.anything() needs to move to video_task
      need_change_video_mode = true;
//      screenMutex = true;
//      screenDirty = true; //display.fillScreen(bg_color);
//      screenMutex = false;
//      screenDirty = true;
    }
    last_video_mode = video_mode;
    font_width = HIRES_FONT_WIDTH;
    font_height = HIRES_FONT_HEIGHT;
    font_baseline = HIRES_FONT_BASELINE;  
    enableVideo = true;    
    //screenDirty = true; //redrawScreen();
  }
  else if ((m_HGCModeReg & HGC_MODECONTROLREG_GRAPHICS)) 
  {
    // HGC graphics mode
    //HGC_720X348_MONO
    //    printf("offset = 0x%02x\n", offset);
//    printf("m_HGCModeReg = 0x%02x\n", m_HGCModeReg);
//    printf("HGC_MODECONTROLREG_GRAPHICSPAGE = 0x%02x\n", HGC_MODECONTROLREG_GRAPHICSPAGE);
//    printf("m_HGCSwitchReg = 0x%02x\n", m_HGCSwitchReg);
//    printf("HGC_CONFSWITCH_ALLOWPAGE1 = 0x%02x\n", HGC_CONFSWITCH_ALLOWPAGE1);
    printf("Hercules, graphics mode\n"); 
    lastVideoWrite = millis();   
    int offset = (m_HGCModeReg & HGC_MODECONTROLREG_GRAPHICSPAGE) && (m_HGCSwitchReg & HGC_CONFSWITCH_ALLOWPAGE1) ? HGC_OFFSET_PAGE1 : HGC_OFFSET_PAGE0;
    video_mode = HGC_720X348_MONO;
    graphics_mode = true;
    if (video_mode != last_video_mode)
    {
      // display.anything() needs to move to video_task
      need_change_video_mode = true;
//      screenMutex = true;
//      screenDirty = true; //display.fillScreen(fg_color); // fill with fg_color because we swap fg/bg in graphics modes
//      screenMutex = false;
//      screenDirty = true;
    }
    last_video_mode = video_mode;
    m_frameBuffer = s_videoMemory + offset;
    fBuffer = m_frameBuffer;
    cursor_visible = false;     

    //printf("int offset = (m_HGCModeReg & HGC_MODECONTROLREG_GRAPHICSPAGE) && (m_HGCSwitchReg & HGC_CONFSWITCH_ALLOWPAGE1) ? HGC_OFFSET_PAGE1 : HGC_OFFSET_PAGE0;\n");
    //printf("int offset = (0x%02x & 0x%02x) && (0x%02x & 0x%02x) ? 0x%02x : 0x%02x\n", m_HGCModeReg, HGC_MODECONTROLREG_GRAPHICSPAGE, m_HGCSwitchReg, HGC_CONFSWITCH_ALLOWPAGE1, HGC_OFFSET_PAGE1, HGC_OFFSET_PAGE0);
    printf("offset = 0x%02x\n", offset);
    //printf("m_frameBuffer = s_videoMemory + offset;\n");
    //printf("m_frameBuffer = %p + 0x%02x;\n", s_videoMemory, offset);
    printf("m_frameBuffer = %p\n", m_frameBuffer); 
    enableVideo = true;

  }
}

void Machine::sound(int freq, uint32_t duration)
{
#if ENABLE_SOUNDGEN
#if ENABLE_PC_SPEAKER
  m_squareWaveGen.setFrequency(freq);
  m_squareWaveGen.enable(true);
  delay(duration);
  m_squareWaveGen.setFrequency(0);
  m_squareWaveGen.enable(false);
#endif //ENABLE_PC_SPEAKER
#endif //ENABLE_SOUNDGEN
}

// stuff for MIDI
#define mpuCommandReset 0xFF
#define mpuCommandAck  0xFE
#define mpuCommandUART  0x3F
unsigned char lastMpuCommand = 0;
bool shouldSentAck = false;

#define MIDI_BUF_LEN 512
unsigned char midiBuffer[MIDI_BUF_LEN];
int bytesRecv = 0;
bool isInSysEx = false;
unsigned char lastMidiParamCount;
bool isPendingMidiCmd = false;

//MIDIHDR     midiHdr;
union { unsigned long word; unsigned char data[4]; } midiMessage;
//end stuff for MIDI

// stuff for Adlib
bool hasReadPortVal = false;
unsigned char lastAdlibAddrPortVal = 0;
// end stuff for Adlib


void Machine::writePort(void * context, int address, uint8_t value, bool isWord, bool is2ndByte)
{

  auto m = (Machine*)context;
  //if (m->need_unhibernate)
  //{
    // testing after unhibernating, for development only
    //printf("Out %02x, %02x     OUT %d, %d\n", address, value, address, value);
  //}
//  if (address > 0xa1)
//  if (address >= 0x3b4 && address < 0x3f8) //|| (address >= 0x03b4 && address <= 0x03bf))
//  //3b4 - 3bf
//  {
    //printf("Out %02x, %02x     OUT %d, %d\n", address, value, address, value);
    //printf("Out %02x,%02x\n", address, value);
//  }
//  }

  switch (address) {

    // PIC8259A   //8259A interrupt controller
    case 0x20:
    case 0x21:
      m->m_PIC8259A.write(address & 1, value);
      break;

    // PIC8259B   //PIC8259B?
    case 0xa0:
    case 0xa1:
      m->m_PIC8259B.write(address & 1, value);
      break;

    // PIT8253    //Programmable Interval Timer
    case 0x0040:
    case 0x0041:
    case 0x0042:
    case 0x0043:
      m->m_PIT8253.write(address & 3, value);
      if ((address == 0x43 && (value >> 6) == 2) || address == 0x42)
      {
#if ENABLE_SOUNDGEN
#if ENABLE_PC_SPEAKER
        m->speakerSetFreq();
#endif //ENABLE_PC_SPEAKER       
#endif //ENABLE_SOUNDGEN
      }
      break;

    // 8042 keyboard controller input
    case 0x0060:
      m->m_i8042.write(0, value);
      break;

    // PortB
    //   bit 1 : speaker data enable
    //   bit 0 : timer 2 gate
    case 0x0061:
#if ENABLE_SOUNDGEN
#if ENABLE_PC_SPEAKER    
//      m->m_speakerDataEnable = value & 0x02;
//      m->m_PIT8253.setGate(2, value & 0x01);
      m->m_speakerDataEnable = ((value & 0x02) == 0x02);
      m->m_PIT8253.setGate(2, ((value & 0x01) == 0x01));
      m->speakerEnableDisable();
#endif // ENABLE_PC_SPEAKER   
#endif // ENABLE_SOUNDGEN   
      break;

    // 8042 keyboard controller input
    case 0x0064:
      m->m_i8042.write(1, value);
      break;

    // MC146818 RTC & RAM  REAL-TIME CLOCK PLUS RAM (RTC)
    case 0x0070:
    case 0x0071:
      m->m_MC146818.write(address & 1, value);
      break;

#if ENABLE_CH376

    case 0x260:
    {
      if (serial1Mode = 0 && usbSpeed == 1000000)
      {
        serial1Mode = 0;
        Serial1.begin(1000000, SERIAL_8N1, 39, 0);
      }
      // write data byte to USB flash drive adapter
      
      if (ch375Command != 0x2b) // for debugging
      {
        // should find a reliable way to reduce this 100us delay, it's slowing down data writes
        delayMicroseconds(100);  // if this time is too short, protocol fails
        //printf("%d D ch375Command = 0x%02x data = 0x%02x\n", millis(), ch375Command, value);  //not running this line results in wrong chip version reported!
      }
      ch375Data = value;
//      if (ch375Command != 0x0b && ch375Command != 0x0a)
//      {
        
        //delayMicroseconds(2);
        Serial1.write(value);
       Serial1.flush();
        //delayMicroseconds(2);    
//      }
//      else
//      {
//        //printf("IGNORED\n");
//      }
      break;
    }

    case 0x261:
    {
      if (serial1Mode != 0 && usbSpeed == 1000000)
      {
        serial1Mode = 0;
        Serial1.begin(1000000, SERIAL_8N1, 39, 0);
      }
      // write command byte to USB flash drive adapter
      ch375PreviousCommand = ch375Command;
      ch375Command = value;
      //delayMicroseconds(2);;
//      if (ch375Command == 0x0b || ch375Command == 0x0a)
//      {
//        // do not send 0x0b and 0x0a commands from driver to ch375
//        break;
//      }
      //printf("%d C 0x%02x\n", millis(), value);
      Serial1.write(value);
      Serial1.flush();
      ch375IntCleared = true;
      if(ch375Command == 0x15) // SET_USB_MODE
      {
        delay(1);
      }
      //delayMicroseconds(2);
      break;
    }
#endif // ENABLE_CH376
    
    // COM2
    case 0x2f8 ... 0x2ff:
      if (networkMode != NETWORKMODE_ETH)
      { 
        if (m->m_COM2.assigned())
        {
          m->m_COM2.write(address, value);
        }
      }
      break;



#if ENABLE_ETHERNET
    //if (NE2000Enabled && address >= NE2000_ADDR && address <= NE2000_ADDR + 0x1F)
    case 0x0300 ... 0x031f:  
    {
      int baseAddress = address - NE2000_ADDR;
  
      /*
      if (isWord && is2ndByte)
        baseAddress = baseAddress - 1;
      */
  
      if (baseAddress >= NE_NOVELL_ASIC_OFFSET)
      {
        if (isWord)
        {
          if (is2ndByte)
          {
            // second (high) byte of 16-bit operation, merge high byte and low byte into word before writing
            uint16_t wordToWrite = lastNE2000ByteWritten | (value << 8);
            ne2000_asic_ioport_write(NE2000Adapter, baseAddress, wordToWrite);
  
            //printf("16-bit ne2000_asic_ioport_write second byte. baseAddress=0x%04x. value=0x%02x. wordToWrite=0x%04x\n", baseAddress, value, wordToWrite);
          }
          else 
          {
            // first (low) byte of 16-bit operation, store first
            lastNE2000ByteWritten = value;
  
            //printf("16-bit ne2000_asic_ioport_write first  byte. baseAddress=0x%04x. value=0x%02x\n", baseAddress, value);
          }
        }
        else 
        {
          ne2000_asic_ioport_write(NE2000Adapter, baseAddress, value);
  
          //printf("8-bit ne2000_asic_ioport_write. baseAddress=0x%04x. value=0x%02x\n", baseAddress, value);
        }
      }
      else 
      {
        ne2000_ioport_write(NE2000Adapter, baseAddress, value);
        //printf("ne2000_ioport_write. baseAddress=0x%04x. value=0x%02x\n", baseAddress, value);
      }
    } 
    break;
#endif //ENABLE_ETHERNET







#if ENABLE_SOUNDGEN
#if ENABLE_MIDI  
    // MIDI output device
    case 0x330:
    {
      if (m->m_midiSoundGenerator.getMidiPort() >= 0)
      {
        midiBuffer[bytesRecv] = value;
        bytesRecv++;

        //printf("value=%02x bytesRecv=%d\r\n", value, bytesRecv);

        // http://www.indiana.edu/~emusic/etext/MIDI/chapter3_MIDI9.shtml
        if (value == 0xF0)
        {
          isInSysEx = true;
        }

        if (isInSysEx)
        {
          if (value == 0xF7 || value >= 0xF8 || bytesRecv >= MIDI_BUF_LEN)
          {
            // end of sys ex or MIDI real-time msgs detected
            isInSysEx = false;
            //printf("Finished SysEx. Length = %d\r\n", bytesRecv);

            // if terminated by real-time msg, count this byte as beginning of next buffer
            // else reset buffer
            int sendUntil = 0;
            if (value == 0xF7)
            {
              sendUntil = bytesRecv;
              bytesRecv = 0;
            }
            else 
            {
              sendUntil = bytesRecv - 1;
              bytesRecv = 1;
            }

            m->m_midiSoundGenerator.sendMidiLongMsg(midiBuffer, sendUntil);
          }
        }
        else 
        {
          bool shouldSend = false;
          if (value >= 0b10000000)
          {
            // MIDI command byte, check to see if we should send now (command with no param)
            // or should wait until the correct parameter arrives
            lastMidiParamCount = m->m_midiSoundGenerator.getExpectedParamCount(value);
            //printf("lastMidiParamCount = %d\n", lastMidiParamCount);
            if (lastMidiParamCount == 0)
            {
              //printf("Sending cmd with no param\r\n");
              midiMessage.data[0] = midiBuffer[0]; // midi Command with no parameters
              midiMessage.data[1] = 0;
              midiMessage.data[2] = 0;
              midiMessage.data[3] = 0;

              shouldSend = true;
            }
            else
            {
              //printf("Not sent yet. Expected param count=%d\r\n", lastMidiParamCount);

              isPendingMidiCmd = true;
              shouldSend = false;
            }
          }
          else 
          {
            if (isPendingMidiCmd)
            {
              if (lastMidiParamCount == 2 && bytesRecv == 3) // 2 parameters
              {
                //printf("Sending 1 cmd byte + 2 param\r\n");
                midiMessage.data[0] = midiBuffer[0];
                midiMessage.data[1] = midiBuffer[1];
                midiMessage.data[2] = midiBuffer[2];
                midiMessage.data[3] = 0;

                shouldSend = true;
                isPendingMidiCmd = false;
              }
              else if (lastMidiParamCount == 1 && bytesRecv == 2) // 1 parameter
              {
                //printf("Sending 1 cmd byte + 1 param\r\n");
                midiMessage.data[0] = midiBuffer[0];
                midiMessage.data[1] = midiBuffer[1];
                midiMessage.data[2] = 0;
                midiMessage.data[3] = 0;

                shouldSend = true;
                isPendingMidiCmd = false;
              }
              else 
              {
                //printf("Not sent yet. Wait for correct param combination\r\n");
              }
            }
            else 
            {
              if (bytesRecv >= 2) // 2 parameters
              {
                // printf("Sending 2 param, no cmd byte\r\n");
                midiMessage.data[0] = midiBuffer[0];
                midiMessage.data[1] = midiBuffer[1];
                midiMessage.data[2] = 0;
                midiMessage.data[3] = 0;

                shouldSend = true;
              }
              else 
              {
                //printf("Not sent yet. Waiting for next param\r\n");
                shouldSend = false;
              }
            }
          }

          if (shouldSend)
          {
            //printf("midiMessage: %02x %02x %02x %02x\n", midiMessage.data[0], midiMessage.data[1], midiMessage.data[2], midiMessage.data[3]);
            //m->m_midiSoundGenerator.sendMidiMsg(midiMessage.word);
            m->m_midiSoundGenerator.sendMidiMsg(midiMessage.data);

            for (int i = 0; i < MIDI_BUF_LEN; i++)
            {
              midiBuffer[i] = 0;
            }
            bytesRecv = 0; // reset MIDI buffer
          }
        }
      }
      break;
    }
    
    case 0x331:
    {
      if (m->m_midiSoundGenerator.getMidiPort() >= 0)
      {
        //printf("Write MPU401 Command value: %d\r\n", value);
        lastMpuCommand = value;
      }

      break;
    }
#endif // ENABLE_MIDI 
#endif // ENABLE_SOUNDGEN

#if ENABLE_SOUNDGEN
#if ENABLE_DISNEY
    case 0x378: // Data Register (Write only for standard parallel port)
    {
      //Serial.println("write 0x378");
      if (m->m_disneySoundGenerator.checkLptSndType() > 0)
      {
        m->m_disneySoundGenerator.writeByteToDisneyBuf(value);
        //Serial.printf("Write Data Port, value=%d, time=%d, bufSize=%d\n", value, millis(), m->m_disneySoundGenerator.getDisneyBufSize());
        m->m_disneySoundGenerator.setUnreadByteCount(m->m_disneySoundGenerator.getUnreadByteCount() + 1); //++;
        //printf("(write)unreadStatusByteCount = %d\n", m->m_disneySoundGenerator.getUnreadByteCount());
      }

      break;
    }
    case 0x379: // Status Register (Read only)
    {
      printf("Incorrect write to parallel port status register, value=%d\n", value);
      break;
    }
    case 0x37a: // Control Register (Write only)
    {
      //printf("Write Control Port value=%d\n", value);
      if (value == 0x04 && m->m_disneySoundGenerator.getLastDisneyOut() == 0)
      {
        printf("Disney sound source turned on\n");
        m->m_disneySoundGenerator.setLastDisneyOut(millis());
      }
      break;
    }
#endif //ENABLE_DISNEY
#endif // ENABLE_SOUNDGEN 
   
    case 0x03c4:
      //printf("port 0x03c4<--0x%02x\n", value);
      SQIndex = value;
      break;

    case 0x03c5:
      //printf("port 0x03c5<--0x%02x\n", value);
      if (SQIndex < SQ_REG_COUNT)
      {
        SQRegisters[SQIndex] = value;
      }
      break;

    // MCGA controller register index
    case 0x3ce:
      //printf("(port 0x3ce) selected MCGA register 0x%02x\n", value);
      m->m_MCGASelectRegister = value;
      break;

    // MCGA selected register write
    case 0x3cf:
      //printf("(port 0x3cf) writing to selected MCGA register (0x%02x) value 0x%02x\n", m->m_MCGASelectRegister, value);
      m->setMCGARegister(value);
      break;

    // CGA - CRT 6845 - register selection register
    case 0x3d4:
      //printf("(port 0x3d4) selected CGA6845 register 0x%02x\n", value);
      m->m_CGA6845SelectRegister = value;
      break;

    // CGA - CRT 6845 - selected register write
    case 0x3d5:
      //printf("(port 0x3d5) writing to selected CGA6845 register value 0x%02x\n", value);
//      if(m->m_CGA6845SelectRegister != 0x0e && m->m_CGA6845SelectRegister != 0x0f) //do not display cursor position updates debugging info
//      {
//        printf("%02x<-%02x\n", m->m_CGA6845SelectRegister, value);
//      }
      videoMemoryUpdated = true;
      m->setCGA6845Register(value);
      break;

    // CGA - Mode Control Register
    case 0x3d8:
      {
        m->m_CGAModeReg = value;
        uint16_t old_CGAMemoryOffset = m->m_CGAMemoryOffset;
        m->m_CGAMemoryOffset = 0;   // when changing modes, reset page offset to zero?
        printf("m->m_CGAModeReg = 0x%02x\n", value);
        m->setCGAMode();
        if(old_CGAMemoryOffset != m->m_CGAMemoryOffset)
        {
          videoTaskSuspended = true;
          m->redrawScreen();
          videoTaskSuspended = false;
        } 
      }     
      break;

    // CGA - Color Select register
    case 0x3d9:
      {
        m->m_CGAColorReg = value;
        uint16_t old_CGAMemoryOffset = m->m_CGAMemoryOffset;
        m->m_CGAMemoryOffset = 0;   // when changing modes, reset page offset to zero?
        printf("m->m_CGAColorReg = 0x%02x\n", value);
        m->setCGAMode();
        if(old_CGAMemoryOffset != m->m_CGAMemoryOffset)
        {
          videoTaskSuspended = true;
          m->redrawScreen();
          videoTaskSuspended = false;
        }
      }
      break;

    // Hercules (HGC) - CRT 6845 - register selection register
    case 0x3b4:
      //printf("(port 0x3b4) selected Hercules (HGC) - CRT 6845 register: 0x%02x\n", value);
      m->m_HGC6845SelectRegister = value;
      break;

    // Hercules (HGC) - CRT 6845 - selected register write
    case 0x3b5:
      //printf("(port 0x3b5) set selected Hercules (HGC) - CRT 6845 register write: to 0x%02x\n", value);
      m->setHGC6845Register(value);
      break;

    // Hercules (HGC) - Display Mode Control Port
    case 0x3b8:
      m->m_HGCModeReg = value;
      //printf("(port 0x3b8) Hercules (HGC) - Display Mode Control Port: setHGCMode(), value = 0x%02x\n", value);
      m->setHGCMode();
      break;
#if ENABLE_SOUNDGEN      
#if ENABLE_ADLIB
    //Adlib register select
    case 0x388:
    {
      //printf("Preparing to write to Adlib register %02xh\r\n", value);
      if (m->m_adlibSoundGenerator.isAdlibEnabled())
      {
        //printf("Preparing to write to Adlib register %02xh\r\n", value);
//        Serial.print("Preparing to write to Adlib register ");
//        Serial.println(value, HEX);
        lastAdlibAddrPortVal = value;
        hasReadPortVal = false;
      }
    }
    break;

    //Adlib register write
    case 0x389:
    {
      //printf("Write Adlib Register (unknown) value %02xh\r\n", value);
      if (m->m_adlibSoundGenerator.isAdlibEnabled())
      {
        //printf("Write Adlib Register %02xh value %02xh\r\n", lastAdlibAddrPortVal, value);
//        Serial.print("Write Adlib Register ");
//        Serial.print(lastAdlibAddrPortVal, HEX);
//        Serial.print(" value ");
//        Serial.println(value, HEX);
        m->m_adlibSoundGenerator.writeAdlibRegister(lastAdlibAddrPortVal, value);
        hasReadPortVal = false;
      }
    }
    break;
#endif // ENABLE_ADLIB
#endif // ENABLE_SOUNDGEN

    // Hercules (HGC) - Configuration Switch
    case 0x3bf:
      m->m_HGCSwitchReg = value;
      //printf("(port 0x3bf) Hercules (HGC) - Configuration Switch: setHGCMode() value = 0x%02x\n", value);
      m->setHGCMode();
      break;

    // COM3
//    case 0x3e8 ... 0x3ef:
//      if (m->m_COM3.assigned())
//        m->m_COM3.write(address, value);
//      break;  
      
    // COM1
    case 0x3f8 ... 0x3ff:
      if (m->m_COM1.assigned())
        m->m_COM1.write(address, value);
      break;

    //send bytes to LoRa transmitter
    case 0x22b8: //port 8888
      if (serial1Mode != 1)
      {
        serial1Mode = 1;
        Serial1.begin(9600, SERIAL_8N1, 39, 0);
      }
      Serial1.write(value);
      Serial1.flush();
      //Serial.print("W-"); Serial.println(value, HEX);
      break;

    case 0x22ba: // port 8890 // set status of LoRa M0 pin high = >0, low = 0
      //not yet implemented
      if(value == 0)
      {
        //digitalWrite(whateverpin, LOW); 
      }
      else
      {
        //digitalWrite(whateverpin, HIGH);
      }
      break;

    case 0x22bb: // port 8891 // set status of LoRa M1 pin high = >0, low = 0
      //not yet implemented
      if(value == 0)
      {
        //digitalWrite(whateverpin, LOW); 
      }
      else
      {
        //digitalWrite(whateverpin, HIGH);
      }
      break;

    // auto power off control. Write 0xff to cut power circuit
    case 0x22bc: //port 8892
      m->powerOff();
      break;

    // Adjust CPU speed
    case 0x22bd: //port 8893
      //setCpuFrequencyMhz(10);  // valid values are 240, 160, 80, 40, 20, 10, 13?, 26?
      /*
       * 1 = 10Mhz
       * 2 = 20
       * 3 = 40
       * 4 = 80
       * 5 = 160
       * 6 = 240
       */
       switch(value)
       {
          case 1:
            Serial.end();
            setCpuFrequencyMhz(10);
            Serial.begin(115200); 
            break;
          case 2:
            Serial.end();
            setCpuFrequencyMhz(20);
            Serial.begin(115200); 
            break;
          case 3:
            Serial.end();
            setCpuFrequencyMhz(40);
            Serial.begin(115200); 
            break;
          case 4:
            Serial.end();
            setCpuFrequencyMhz(80);
            Serial.begin(115200); 
            break;
          case 5:
            Serial.end();
            setCpuFrequencyMhz(160);
            Serial.begin(115200); 
            break;
          case 6:
            Serial.end();
            setCpuFrequencyMhz(240);
            Serial.begin(115200); 
            break;
          default:
            break;       
       }
      break;

    // I/O expander - Configuration
    case EXTIO_CONFIG:
      //m->m_MCP23S17.setINTActiveHigh(value & EXTIO_CONFIG_INT_POLARITY);
      break;

    // I/O expander - Port A/B Direction
    case EXTIO_DIRA ... EXTIO_DIRB:
      //m->m_MCP23S17.setPortDir(address - EXTIO_DIRA + MCP_PORTA, ~value);
//      printf("dir %d = %02X\n", address - EXTIO_DIRA + MCP_PORTA, ~value);
      break;

    // I/O expander - Port A/B pullup
    case EXTIO_PULLUPA ... EXTIO_PULLUPB:
      //m->m_MCP23S17.enablePortPullUp(address - EXTIO_PULLUPA + MCP_PORTA, value);
      break;

    // I/O expander - Port A/B write
    case EXTIO_PORTA ... EXTIO_PORTB:
      //m->m_MCP23S17.writePort(address - EXTIO_PORTA + MCP_PORTA, value);
//      printf("set %d = %02X\n", address - EXTIO_PORTA + MCP_PORTA, value);
      break;

    // I/O expander - GPIO selection
    case EXTIO_GPIOSEL:
      //m->m_MCP23S17Sel = value & 0xf;
      break;

    // I/O expander - GPIO direction and pullup
    case EXTIO_GPIOCONF:
      //m->m_MCP23S17.configureGPIO(m->m_MCP23S17Sel, value & 1 ? fabgl::MCPDir::Output : fabgl::MCPDir::Input, value & 2);
      break;

    

    // I/O expander - GPIO write
    case EXTIO_GPIO:
      //m->m_MCP23S17.writeGPIO(m->m_MCP23S17Sel, value);
      break;

    default:
      //printf("OUT %04x=%02x\n", address, value);
      break;
  }
}

#define MPU401_INT_NO 0x0A     // for IRQ 2, cascaded to IRQ 9
static int MPU401IRQPending = 0; // for MT32

uint8_t Machine::readPort(void * context, int address, bool isWord, bool is2ndByte)
{
  auto m = (Machine*)context;
  uint8_t read61 = 0;
  
  //unsigned char retval = Port[address]; // good idea, but would require 65536 bytes of RAM.  Or I could make a map to just the ports being used, if needed.
  unsigned char retval = 0;
  
  //if (m->need_unhibernate)
  //{
    //printf("In %02x\n", address);
  //}  
//  if (address != 0x03da && address != 0x0060 && address != 0x0020)
//  {
    //printf("In %02x\n", address);
//  }

  // CH376
  switch (address) {
    uint8_t dbbout;
    
    // PIC8259A
    case 0x0020:
    case 0x0021:
      return m->m_PIC8259A.read(address & 1);

    // PIC8259B
    case 0x00a0:
    case 0x00a1:
      return m->m_PIC8259B.read(address & 1);

    // PIT8253
    case 0x0040:
    case 0x0041:
    case 0x0042:
    case 0x0043:
      return m->m_PIT8253.read(address & 3);

    // 8042 keyboard controller output
    case 0x0060:
      //printf("%s:%d\n", __FILE__, __LINE__);
      //uint8_t dbbout;
      dbbout= m->m_i8042.read(0);
      readkeys++;
      //printf("dbbout = %d\n", (uint8_t)dbbout);
      return dbbout;
      //return m->m_i8042.read(0);

    // Port B
    //   bit 5 : timer 2 out
    //   bit 4 : toggles every 15.085us (DMA refresh)
    //   bit 1 : speaker data enable
    //   bit 0 : timer 2 gate
    case 0x0061:
#if ENABLE_SOUNDGEN
#if ENABLE_PC_SPEAKER    
      read61 = ((int)m->m_PIT8253.getOut(2) << 5) |   // bit 5
             (esp_timer_get_time() & 0x10)                |   // bit 4 (toggles every 16us)
             ((int)m->m_speakerDataEnable << 1)           |   // bit 1
             ((int)m->m_PIT8253.getGate(2));                  // bit 0
      //printf("read61 = 0x%02x\n", read61);
#endif // ENABLE_PC_SPEAKER  
#endif // ENABLE_SOUNDGEN   
      return read61;
    // I/O port
    case 0x0062:
      return 0x20 * m->m_PIT8253.getOut(2);  // bit 5 = timer 2 output

    // 8042 keyboard controller status register
    case 0x0064:
      read8042s++;
      return m->m_i8042.read(1);

    // MC146818 RTC & RAM
    case 0x0070:
    case 0x0071:
      return m->m_MC146818.read(address & 1);

#if ENABLE_CH376

    case 0x260:
    {
      if (serial1Mode != 0 && usbSpeed == 1000000)
      {
        serial1Mode = 0;
        Serial1.begin(1000000, SERIAL_8N1, 39, 0);
      }
      int8_t tmpval = -1;
      bool readSerial = false;
      //delayMicroseconds(2);
//      if (ch375Command == 0x0a)
//      {
//        delayMicroseconds(5);;
//        return 0x05;
//      }
      for (int n = 0; n < 200; n++)
      {
        if (Serial1.available() > 0) 
        {
          tmpval = Serial1.read();
          //printf("read 0x%02x from Serial1\n", tmpval);
          readSerial = true;
          if (n > 0)
          {
            printf("read Serial1 %d times before getting response\n", n + 1);
          }
          break;
        }
        delay(1);
      }
      uint8_t retval = (uint8_t)tmpval;

      int tries = 0;
      while(ch375Command == 0x22 && !readSerial)
      {
        delay(1);
        tries++;
        if(tries > 0)
        {
          printf("Try %d: resending GET_STATUS\n", tries);
        }
        Serial1.write(0x22);  //resend GET_STATUS command
        for (int n = 0; n < 200; n++)
        {
          if (Serial1.available() > 0) 
          {
            tmpval = Serial1.read();
            //printf("read 0x%02x from Serial1\n", tmpval);
            readSerial = true;
            break;
          }
          delay(1);
        }

        if (tries > 60 && !readSerial)
        {
          printf("FAILED READING SERIAL1 > 60 times!  Giving up and returning 0x2E fail:stall message!\n");
          retval = 0x2e;  //emulate device failure:stall response
          break;
        }

        retval = (uint8_t)tmpval;
      }
      
      //delayMicroseconds(2);
      
//      if (ch375Command != 0x28 && ch375Command != 0x2b)
//      {
//        printf("%d R c:0x%02X r:0x%02X\n", millis(), ch375Command, retval);
//      }      
      
      if (ch375Command == 0x15 && ch375Data == 0x06 && retval == 0x51)
      {
        // USB mode is set to 0x06, now initialize disk
        //first switch to faster baud rate
        delay(10);


       
        printf("Switching ch375 to %dbps\n", 1000000); // Don't use 2000000baud.  It messes up ethernet port being used simultaneously.
        Serial1.write(0x02);  //set baud rate command
        delay(10);
        //Serial1.write(0x03);  //frequency coefficient for 115200
        //Serial1.write(0x03);  //frequency coefficient for 250000
        Serial1.write(0x03);  //frequency coefficient for 1000000
        //Serial1.write(0x03);  //frequency coefficient for 2000000
        //Serial1.write(0x03);  //frequency coefficient for 500000
        delay(10);
        //Serial1.write(0xcc);  //frequency constant for 115200
        //Serial1.write(0xE8);  //frequency constant for 250000
        //Serial1.write(0xf4);  //frequency constant for 500000
        Serial1.write(0xfa);  //frequency constant for 1000000
        //Serial1.write(0xfd);  //frequency constant for 2000000
        delay(10);
        while(Serial1.available() != 0)
        {
          Serial1.read(); // read in any garbage data from the 0x51 operation status return value, clearing buffer
        }
        Serial1.begin(1000000, SERIAL_8N1, 39, 0);
        usbSpeed = 1000000;

/*     
//        //try setting to USB1.0 speed mode (1.5Mbps)
        delay(10);
        printf("Sending 0x04 SET_USB_SPEED command\n");
        Serial1.write(0x04);
        delay(10);
        printf("Sending 0x02 LOW SPEED parameter\n");
        Serial1.write(0x02);
        delay(100);

*/
        // now initialize disk
        delay(10);
        printf("Sending 0x22 GET_STATUS command\n");
        delay(10);
        Serial1.write(0x22);
        delay(10);
        while(Serial1.available() == 0)
        {
          delay(1);
        }
        uint8_t status22 = Serial1.read();
        printf("0x22 GET_STATUS command returned 0x%02x\n", status22);  //should be 0x15

        delay(10);
        printf("Sending 0x51 DISK_INIT command\n");
        Serial1.write(0x51);
        delay(250);
        printf("Sending 0x22 GET_STATUS command\n");
        Serial1.write(0x22);
        delay(10);
        while(Serial1.available() == 0)
        {
          delay(1);
        }
        status22 = Serial1.read();
        printf("0x22 GET_STATUS command returned 0x%02x\n", status22);  //should be 0x14
        
        delay(10);
        printf("Sending 0x53 DISK_SIZE command\n");
        Serial1.write(0x53);
        delay(100);
        printf("Sending 0x22 GET_STATUS command\n");
        Serial1.write(0x22);
        delay(10);
        while(Serial1.available() == 0)
        {
          delay(1);
        }
        status22 = Serial1.read();
        printf("0x22 GET_STATUS command returned 0x%02x\n", status22);  //should be 0x14

        delay(10);
        printf("Sending 0x59 DISK_READY command\n");
        Serial1.write(0x59);
        delay(100);
        printf("Sending 0x22 GET_STATUS command\n");
        Serial1.write(0x22);
        delay(10);
        while(Serial1.available() == 0)
        {
          delay(1);
        }
        status22 = Serial1.read();
        printf("0x22 GET_STATUS command returned 0x%02x\n", status22);  //should be 0x14



//        // set retries to 64 times
//        delay(1000);
//        printf("Sending 0x0B SET_RETRY command\n");
//        Serial1.write(0x0B);
//        delay(10);
//        printf("Sending 0x25 SET_RETRY DATA\n");
//        Serial1.write(0x25);
//        delay(10);
//        printf("Sending 0xFF SET_RETRY to 64 times\n");
//        Serial1.write(0xFF);
//        delay(1000);

        // SET_RETRY RETURNS NO data and generates no interrupt, so don't try to read the serial port here.

        

        /*
        // 切换使375B进入低速模式
        CH375_WR_CMD_PORT(0x0b);    
        CH375_WR_DAT_PORT(0x17);
        CH375_WR_DAT_PORT(0xd8);
        */
        // set slow speed mode (from https://bbs.21ic.com/blog-77381-52400.html)

//        delay(1000);
//        printf("Sending 0x0b command and parameters to enter CH375B low speed mode\n");
//        Serial1.write(0x0b);
//        delay(10);
//        printf("Sending 0x17 DATA\n");
//        Serial1.write(0x17);
//        delay(10);
//        printf("Sending 0xd8 DATA\n");
//        Serial1.write(0xd8);
//        delay(1000);
//        printf("Sending 0x22 GET_STATUS command\n");
//        Serial1.write(0x22);
//        delay(10);
//        while(Serial1.available() == 0)
//        {
//          delay(1);
//        }
//        status22 = Serial1.read();
//        printf("0x22 GET_STATUS command returned 0x%02x\n", status22);  //should be 0x14
//        delay(1000);

      }
      return retval;
    }
    case 0x261:
    {
      if (serial1Mode != 0 && usbSpeed == 1000000)
      {
        serial1Mode = 0;
        Serial1.begin(1000000, SERIAL_8N1, 39, 0);
      }
      // Here we should just check if the INT pin on CH375 is high, and return 0x80 if it is, 0x00 if not.
      retval = 0x00;
      static uint32_t num261NoInt = 0;
      if (digitalRead(4) == HIGH)  // datasheet says active with low-level voltage.  So LOW means there is an active interrupt?
      {
        retval = 0x80;
//        if(millis() - num261NoInt < 2 && !ch375IntCleared)
//        {
//          printf("%d I c:0x%02X r:0x%02X\n", millis(), ch375Command, retval);
//        }
        num261NoInt++; 
        ch375IntCleared = false;
        if (num261NoInt > 5)
        {
          //printf("%d I c:0x%02X r:0x%02X, num261NoInt=%d\n", millis(), ch375Command, retval, num261NoInt);
          delayMicroseconds(100);
        }
        if (num261NoInt > 100)
        {
          printf("%d I c:0x%02X r:0x%02X, num261NoInt=%d\n", millis(), ch375Command, retval, num261NoInt);
        }
      }
      else
      {
        num261NoInt = 0;
      }
      //printf("%d I c:0x%02X r:0x%02X\n", millis(), ch375Command, retval);
      return retval;
    }
    
#endif // ENABLE_CH376

    // COM2
    case 0x2f8 ... 0x2ff:
      if (networkMode != NETWORKMODE_ETH)
      { 
        //printf("COM2 read\n");
        static uint32_t COM2reads = 0;
        COM2reads++;
        if (COM2reads >> 14 == 1)
        {
          //reads 16384 times about every 1628ms, or about 10 times per ms, or 10,000 times per second.  This theoretically can keep up with a baud rate of at least 57600, maybe close to 115200
          //printf("COM2read %d\n", COM2reads);
          COM2reads = 0;
        }
        if (m->m_COM2.assigned())
        {
          uint8_t com2byte = m->m_COM2.read(address);
          if (com2byte != 0xb0)
          {
            //printf("com2(0x%04x)<-%c:0x%02x\n", address, (char)com2byte,(char)com2byte);
          }
          return com2byte;
        }
        else
        {
          return 0;
        }
        //return m->m_COM2.assigned() ? m->m_COM2.read(address) : 0;
      }

#if ENABLE_ETHERNET
    //if (NE2000Enabled && Address >= NE2000_ADDR && Address <= NE2000_ADDR + 0x1F)
    case 0x0300 ... 0x031f:
    {
      int baseAddress = address - NE2000_ADDR;
  
      /*
      if (isWord && is2ndByte)
        baseAddress = baseAddress - 1;
      */
  
      if (baseAddress >= NE_NOVELL_ASIC_OFFSET)
      {
        if (isWord)
        {
          if (is2ndByte)
          {         
            // second byte of 16-bit operation, transfer high byte
            retval = (lastNE2000WordRead >> 8) & 0xFF;
  
            //printf("16-bit ne2000_asic_ioport_read. baseAddress=0x%04x. lastNE2000WordRead=0x%04x. 2ndByte = 0x%02x\n", baseAddress, lastNE2000WordRead, retval);
          }
          else 
          {
            // first byte of 16-bit operation, transfer low byte 
            lastNE2000WordRead = ne2000_asic_ioport_read(NE2000Adapter, baseAddress);
            retval = lastNE2000WordRead & 0xFF;
  
            //printf("16-bit ne2000_asic_ioport_read. baseAddress=0x%04x. lastNE2000WordRead=0x%04x. 1stByte = 0x%02x\n", baseAddress, lastNE2000WordRead, retval);
          }
        }
        else 
        {
          retval = ne2000_asic_ioport_read(NE2000Adapter, baseAddress);
  
          //printf("8-bit ne2000_asic_ioport_read. baseAddress=0x%04x. retval=0x%02x\n", baseAddress, retval);
        }
      }
      else 
      {
        retval = ne2000_ioport_read(NE2000Adapter, baseAddress);
  
        //printf("ne2000_ioport_read. baseAddress=0x%04x. retval=0x%02x\n", baseAddress, retval);
      }
      return retval;
    }
    break;
#endif //ENABLE_ETHERNET





#if ENABLE_SOUNDGEN  
#if ENABLE_MIDI
    // MIDI ports
    case 0x330:
    {
      if (m->m_midiSoundGenerator.getMidiPort() >= 0)
      {
        /*
        In Uart mode, the MPU recognizes only 1 command; the command to put it back into Intelligent mode, the Reset command.
        After you write a command byte to the COMMAND port, the MPU acknowledges this command.
        It sends an FE (hex) byte to its DATA port
        */
        if (shouldSentAck)
        {
          retval = mpuCommandAck;
          shouldSentAck = false;
          MPU401IRQPending++;
          //m->m_PIC8259B.signalInterrupt(9); // IRQ 9 on PIC8259B(slave PIC) translates to int 0x0A 
        }
        else 
        {
          retval = 0;
        }

        //printf("Read MPU401 Data Port. lastMpuCommand = %d. retval = %d\r\n", lastMpuCommand, retval);
        lastMpuCommand = 0;
        return retval;
      }
      else
      {
        return 0xff;
      }
    }
    break;

    case 0x331:
    {
      if (m->m_midiSoundGenerator.getMidiPort() >= 0)
      {
        /*
        The highest bit (7) of this byte is the state of the DATA SET READY (DSR) line.
        This bit will be clear (0) if the MPU has some incoming MIDI byte (or Ack byte or Operation byte) waiting for you to read.
        The DSR line will remain low until you've read all of the bytes that the MPU has waiting in its hardware input buffer.
        You should continue reading bytes from the DATA port until the DSR bit of the STATUS port toggles to a 1 (ie, sets).
        When that happens, the MPU has no more bytes waiting to be read. The next highest bit (6) is the state of the DATA READ READY (DRR) line.
        This bit will be clear whenever it's OK for you to write a byte to the MPU's DATA or COMMAND ports.
        The MPU sets bit 6 of the STATUS port whenever it is NOT ready for you to write a byte to the DATA or COMMAND ports.
        */
        if (lastMpuCommand != 0 && !shouldSentAck)
        {
          retval = 0b10000000;
          shouldSentAck = true;
          MPU401IRQPending++;
          //m->m_PIC8259B.signalInterrupt(9); // IRQ 9 on PIC8259B(slave PIC) translates to int 0x0A 
        }
        else if (lastMpuCommand != 0 && shouldSentAck)
        {
          retval = 0b00000000;
        }
        else 
        {
          retval = 0b10000000;
        }
        //printf("Read MPU401 Status Port. lastMpuCommand = %d. RetVal = %d\r\n", lastMpuCommand, retval);
        return retval;
      }
      else
      {
        return 0xff;
      }
    }
    break;    
#endif // ENABLE_MIDI 
#endif // ENABLE_SOUNDGEN

#if ENABLE_SOUNDGEN
#if ENABLE_DISNEY
    case 0x378: // Data Register (Write-only)
    {
      printf("Incorect read from Data Port\n");
      return 0xff;
      break;
    }
    case 0x379: // Status Register (Read-only)
    {
      /*
      Some games (Keen Dreams) attempt to detect the Disney Sound Source by doing the following (BASE_PORT = 0x378)

      1. Send the value 0x04 to BASE_PORT+2 to turn the device on
      2. Read parallel port ACK status, make sure it's not on
      3. Send a lot of bytes (~32) to the device (raw value to BASE_PORT followed by control bytes 0x0C to BASE_PORT+2 and 0x04 to BASE_PORT+2)
      4. Read parallel port ACK status, make sure it's on to indicate buffer overflow
      5. Send the value 0x0C to BASE_PORT+2 to turn the device off

      The device is considered detected if ACK is off in step (2) and on in step (4).
      Our dummy implementation attempts to pass the above detection logic. Control bytes are ignored.
      */
      //printf("readPort 0x379, checking DSS/LPT1 status\n");
      if (m->m_disneySoundGenerator.checkLptSndType() > 0)
      {
        //printf("%d checkLptSndType > 0\n", millis());
        //printf("(read)unreadStatusByteCount = %d\n", m->m_disneySoundGenerator.getUnreadByteCount());
        if (m->m_disneySoundGenerator.getUnreadByteCount() > 16)
        {
          retval = 0b01000000;  // ACK ON
          //printf("%d ACK ON\n", millis());
          m->m_disneySoundGenerator.setUnreadByteCount(0);
        }
        else
        {
          retval = 0b00000000;  // ACK OFF
          //printf("%d ACK OFF\n", millis());
        }

        // printf("Read Status Port, retVal=%d\n", retval);
        //m->m_disneySoundGenerator.setUnreadByteCount(0);
      }
      return retval;
      break;
    }
    case 0x37a: // Control Register (Write-only)
    {
      //printf("Incorrect read from write-only control port!\n");
      return 0xff;
      break;
    }
#endif //ENABLE_DISNEY
#endif // ENABLE_SOUNDGEN 

#if ENABLE_SOUNDGEN
#if ENABLE_ADLIB  
    // Read Adlib status port
    case 0x388:
    {
      //Serial.println("Reading Adlib Status Port 0x388\n");
      if (m->m_adlibSoundGenerator.isAdlibEnabled())
      {
        /*
        Bit 7 - set if either timer has expired.
        6 - set if timer 1 has expired.
        5 - set if timer 2 has expired
        */
        retval = m->m_adlibSoundGenerator.getAdlibStatus();
        if (retval!= 0)
        {
          //printf("rv=%d\n", retval);
        }
        if (!hasReadPortVal)
        {
          //printf("Read Adlib Status Port. RetVal = %02xh\r\n", retval);
          //Serial.print("Read Adlib Status Port. RetVal = ");
          //Serial.println(retval);
          hasReadPortVal = true;
        }
        return retval;
      }
      return 0xff;
    }
    break;
  
    case 0x389:
    {
      static bool printedError = false;
      if (m->m_adlibSoundGenerator.isAdlibEnabled())
      {
        //printf("Invalid read from write-only Adlib Data Port!\n");
        if (printedError == true)
        {
          Serial.println("Invalid read from write-only Adlib Data Port! (future identical messages will be surpressed)\n");
        }
      }
      return 0xff;
    }
    break;
#endif // ENABLE_ADLIB 
#endif // ENABLE_SOUNDGEN

    // CGA - CRT 6845 - register selection register
    case 0x3d4:
      //printf("Trying to read ISA port 0x3d4 Hercules (CRT 6845) - register selection register, returning 0x00 (not readable)\n");
      return 0x00;  // not readable

    // CGA - CRT 6845 - selected register read
    case 0x3d5:
      return m->m_CGA6845SelectRegister >= 14 && m->m_CGA6845SelectRegister < 16 ? m->m_CGA6845[m->m_CGA6845SelectRegister] : 0x00;

    // CGA - Color Select register
    // note: this register should be write-only, but some games do not work if it isn't readable
    case 0x3d9:
      return m->m_CGAColorReg;

    // CGA - Status Register
    // real vertical sync is too fast for our slowly emulated 8086, so
    // here it is just a fake, just to allow programs that check it to keep going anyway.
    case 0x3da:
      m->m_CGAVSyncQuery += 1;
      //printf("Read ISA port 0x3da CGA - Display Status Port, returning %02x\n", (m->m_CGAVSyncQuery & 0x7) != 0 ? 0x09 : 0x00);
      return (m->m_CGAVSyncQuery & 0x7) != 0 ? 0x09 : 0x00; // "not VSync" (0x00) every 7 queries

    // Hercules (HGC) - register selection register
    case 0x3b4:
      printf("Trying to read ISA port 0x3b4 Hercules (HGC) - register selection register, returning 0x00 (not readable)\n");
      return 0x00;  // not readable

    // Hercules (HGC) - selected register read
    case 0x3b5:
      //printf("Read ISA port 0x3b5 Hercules (HGC) - selected register read, returning 0x%02x\n", m->m_HGC6845SelectRegister >= 14 && m->m_HGC6845SelectRegister < 16 ? m->m_HGC6845[m->m_HGC6845SelectRegister] : 0x00);
      return m->m_HGC6845SelectRegister >= 14 && m->m_HGC6845SelectRegister < 16 ? m->m_HGC6845[m->m_HGC6845SelectRegister] : 0x00;

    // Hercules (HGC) - Display Status Port
    // real vertical sync is too fast for our slowly emulated 8086, so
    // here it is just a fake, just to allow programs that check it to keep going anyway.
    case 0x3ba:
      m->m_HGCVSyncQuery += 1;
      //printf("Read ISA port 0x3ba Hercules (HGC) - Display Status Port, returning %02x\n", (m->m_HGCVSyncQuery & 0x7) != 0 ? 0x00 : 0x80);
      return (m->m_HGCVSyncQuery & 0x7) != 0 ? 0x00 : 0x80; // "not VSync" (0x80) every 7 queries

    // COM3
//    case 0x3e8 ... 0x3ef:
//      //printf("COM3 read\n");
//      static uint32_t COM3reads = 0;
//      COM3reads++;
//      if (COM3reads >> 14 == 1)
//      {
//        //reads 16384 times about every 1628ms, or about 10 times per ms, or 10,000 times per second.  This theoretically can keep up with a baud rate of at least 57600, maybe close to 115200
//        printf("COM3read %d\n", COM3reads);
//        COM3reads = 0;
//      }
//      if (m->m_COM3.assigned())
//      {
//        uint8_t com3byte = m->m_COM3.read(address);
//        if (com3byte != 0xb0)
//        {
//          printf("com3(0x%04x)<-%c:0x%02x\n", address, (char)com3byte,(char)com3byte);
//        }
//        return com3byte;
//      }
//      else
//      {
//        return 0;
//      }

    // COM1
    case 0x3f8 ... 0x3ff:
      return m->m_COM1.assigned() ? m->m_COM1.read(address) : 0;

    //receive byte from LoRa transmitter
    case 0x22b8: //port 8888
      if (serial1Mode != 1)
      {
        serial1Mode = 1;
        Serial1.begin(9600, SERIAL_8N1, 39, 0);
      }
      retval = Serial1.read();
      //Serial.print("R-"); Serial.println(retval, HEX);
      return retval;

    // check if LoRa has data
    case 0x22b9: // port 8889
      retval = 0;
      if (Serial1.available())
      {
        retval = 1;
      }
      return retval;

    case 0x22ba: // port 8890 // read status of LoRa M0 pin high = 0x01, low = 0x00
      //not yet implemented
      retval = 0x00;
      return retval;

    case 0x22bb: // port 8891 // read status of LoRa M1 pin high = 0x01, low = 0x00
      //not yet implemented
      retval = 0x00;
      return retval;

    // I/O expander - Configuration
    case EXTIO_CONFIG:
      //return (m->m_MCP23S17.available()        ? EXTIO_CONFIG_AVAILABLE    : 0) |
      //       (m->m_MCP23S17.getINTActiveHigh() ? EXTIO_CONFIG_INT_POLARITY : 0);

    // I/O expander - Port A/B Direction
    case EXTIO_DIRA ... EXTIO_DIRB:
      //return m->m_MCP23S17.getPortDir(address - EXTIO_DIRA + MCP_PORTA);

    // I/O expander - Port A/B pullup
    case EXTIO_PULLUPA ... EXTIO_PULLUPB:
      //return m->m_MCP23S17.getPortPullUp(address - EXTIO_PULLUPA + MCP_PORTA);

    // I/O expander - Port A/B read
    case EXTIO_PORTA ... EXTIO_PORTB:
      //return m->m_MCP23S17.readPort(address - EXTIO_PORTA + MCP_PORTA);

    // I/O expander - GPIO selection
    case EXTIO_GPIOSEL:
      //return m->m_MCP23S17Sel;

    // I/O expander - GPIO read
    case EXTIO_GPIO:
      //return m->m_MCP23S17.readGPIO(m->m_MCP23S17Sel);
      break;

  }
  //static uint8_t unassigned_port = 0;
  //if (unassigned_port != address)
  //{
    //printf("!!! READPORT on unassigned port %04X returning default 0xff (shown only once for this port)!!!\n", address);
  //}
  //unassigned_port = address;
  
  return 0xff;
}


void Machine::PITChangeOut(void * context, int timerIndex)
{
  auto m = (Machine*)context;

  // timer 0 trigged?
  if (timerIndex == 0 &&  m->m_PIT8253.getOut(0) == true) 
  {
    // yes, report 8259A-IRQ0 (IRQ0, INT 08h)
    m->m_PIC8259A.signalInterrupt(0);
  }
}


// reset from 8042
bool Machine::resetMachine(void * context)
{
  auto m = (Machine*)context;
  m->trigReset();
  return true;
}

// windows or menu key pressed, quick screen refresh
bool Machine::sysReq(void * context)
{
  auto m = (Machine*)context;
  if (m->m_sysReqCallback)
    m->m_sysReqCallback();
  return true;
}

// SYSREQ2 (ALT + PRINTSCREEN), full, slow screen refresh
bool Machine::sysReq2(void * context)
{
  auto m = (Machine*)context;
  m->m_BIOS.emptyKbdBufferPublic();
  if (m->m_sysReqCallback2)
    m->m_sysReqCallback2();
  return true;
}

// SYSREQ3 (MENU KEY), invert black/white pixels, sometimes improves graphics, black background not recommended for text modes due to degredation of black pixels
bool Machine::sysReq3(void * context)
{
  auto m = (Machine*)context;
  if (m->m_sysReqCallback3)
    m->m_sysReqCallback3();
  return true;
}

// SYSREQ4 (CTRL + ALT + PRINTSCREEN, RELEASE CTRL FIRST), CHANGE POWER SAVING MODE
bool Machine::sysReq4(void * context)
{
  auto m = (Machine*)context;
  m->m_BIOS.emptyKbdBufferPublic();
  if (m->m_sysReqCallback4)
    m->m_sysReqCallback4();
  return true;
}

// SYSREQ5 (LSHIFT + ALT + PRINTSCREEN, RELEASE LSHIFT FIRST), HIBERNATE
bool Machine::sysReq5(void * context) 
{
  printf("entering sysReq5...\n");
  auto m = (Machine*)context;
  m->m_BIOS.emptyKbdBufferPublic();
  m->hibernate();
  return true;
}

// SYSREQ5 (RSHIFT + ALT + PRINTSCREEN, RELEASE RSHIFT FIRST), UNHIBERNATE
bool Machine::sysReq6(void * context)
{
  printf("entering sysReq6...\n");
  auto m = (Machine*)context;
  m->m_BIOS.emptyKbdBufferPublic();
  m->unhibernate();
  return true;
}

// 8259A-IR1 (IRQ1, INT 09h)
bool Machine::keyboardInterrupt(void * context)
{
  //printf("key\n");
  last_key_time = millis();
  keyspressed++;
  auto m = (Machine*)context;
  return m->m_PIC8259A.signalInterrupt(1);
}


// 8259B-IR4 (IRQ12, INT 074h)
bool Machine::mouseInterrupt(void * context)
{
  auto m = (Machine*)context;
  return m->m_PIC8259B.signalInterrupt(4);

}


// interrupt from MC146818, trig 8259B-IR0 (IRQ8, INT 70h)
bool Machine::MC146818Interrupt(void * context)
{
  auto m = (Machine*)context;
  return m->m_PIC8259B.signalInterrupt(0);
}


// interrupt from COM1, trig 8259A-IR4 (IRQ4, INT 0Ch)
bool Machine::COM1Interrupt(PC8250 * source, void * context)
{
  auto m = (Machine*)context;
  return source->getOut2() && m->m_PIC8259A.signalInterrupt(4);
}


// interrupt from COM2, trig 8259A-IR3 (IRQ3, INT 0Bh)
bool Machine::COM2Interrupt(PC8250 * source, void * context)
{
  if (networkMode != NETWORKMODE_ETH)
  { 
    //this gets called from PC8250.cpp's ::tick() routine
    auto m = (Machine*)context;
    //printf("c2int\n");
    return source->getOut2() && m->m_PIC8259A.signalInterrupt(3);
  }
  else
  {
    return false;
  }
}

// interrupt from COM3, trig 8259A-IR4 (IRQ4, INT 0Ch)
//bool Machine::COM3Interrupt(fake8250 * source, void * context)
//{
//  auto m = (Machine*)context;
//  printf("c3int\n");
//  return source->getOut2() && m->m_PIC8259A.signalInterrupt(4);
//}

void Machine::writeVideoMemory8(void * context, int address, uint8_t value)
{
  mems++;
  vidmems++;
  lastVideoMemoryWrite = millis();
  auto m = (Machine*)context;
  //printf("WVMEM8 %05X <= %02X, video_mode=0x%02x, egaWriteMode=%d\n", address, value, video_mode, egaWriteMode);
  // write all changes directly to EPD screen right here rather than wait and scan for changes periodically
  //if (address >= 0xa0000 && address < 0xb0000)
//  {
//    if(value && (value != 0xff) && (value != 0x8f) && (value != 0x07))
//    //if (value)
//    {
//      printf("WVMEM8 %05X <= %02X %c\n", address, value, value);
//    }
//  }
  
  //if (address >= 0xb0000)
  if (address >= 0xa0000 && address < 0xb0000 )
  {
    //printf("WVMEM8 %05X <= %02X %c\n", address, value, value);
    //if (s_videoMemory[address - 0xa0000] != value)
    if (video_mode == MCGA_320X200_COLOR || video_mode == MCGA_640X480_MONO)
    {
      if (MCGA_videoMemory[address - 0xa0000] != value)
      {    
        lastVideoWrite = millis();
        uint32_t oldvalue = MCGA_videoMemory[address - 0xa0000];
        MCGA_videoMemory[address - 0xa0000] = value;  
        uint32_t addr = address - 0xa0000;
        uint32_t fbAddr = ((uint32_t)fBuffer - (uint32_t)MCGA_videoMemory);
        //screenDirty = true; //updateGraphicsByte((addr - fbAddr), value, true, oldvalue);  
        //screenDirty = true;
        //last_screen_change_time = millis();
        //global_need_refresh = true;      
        bitTable[(addr - fbAddr)] = 1;  
        videoMemoryUpdated = true; 
        screenDirty = true; 
      }
    }
    if (video_mode == EGA_320X200_COLOR)
    {
      
      uint32_t addr = address - 0xa0000;
      //SQRegisters[2] is the Map Mask register
      //printf("WVMEM8 %d <= %02X, WM=0x%02x, MM=0x%02x, BM=0x%02x\n", addr, value, m->m_MCGA[5], SQRegisters[2], m->m_MCGA[8]);
      uint32_t fbAddr = ((uint32_t)fBuffer - (uint32_t)MCGA_videoMemory);
      if (egaWriteMode == 0)  // if using EGA Write Mode 0 (instead of 2 or 1)
      {
        //printf("WVMEM8 %d <= %02X, WM=0x%02x, MM=0x%02x, BM=0x%02x\n", addr, value, m->m_MCGA[5], SQRegisters[2], m->m_MCGA[8]);
        uint8_t mapmask = SQRegisters[2];
        mapmask &= 0x0f;  // clear high 4 bits
        for (int planenum = 0; planenum < 4; planenum++) //loop through all four lower bits and write to corresponding bit plane if bit is set
        {
          uint32_t plane_addr = addr;
          uint8_t thisbit = mapmask >> planenum & 0x01;  //isolate this bitplane's mask bit
          if (thisbit == 1)  // if this bit is set, write to that plane
          {
            plane_addr = addr + (8000 * planenum);
            if(MCGA_videoMemory[plane_addr] != value)
            {
              lastVideoWrite = millis();
              uint32_t oldvalue = MCGA_videoMemory[plane_addr];
              MCGA_videoMemory[plane_addr] = value;  
              //screenDirty = true; //updateGraphicsByte((plane_addr - fbAddr), value, true, oldvalue);  
              //screenDirty = true;
              //last_screen_change_time = millis();
              //global_need_refresh = true;
              bitTable[(plane_addr - fbAddr)] = 1;
              videoMemoryUpdated = true;
              screenDirty = true; 
            }
          }
        }
      }
      else if (egaWriteMode == 2) // if using EGA Write Mode 2 (instead of 0 or 1)
      {
        //printf("***** ATTEMPTING TO USE UNSUPPORTED EGA WRITE MODE %d ********\n", egaWriteMode);
        /* Mode 2 writes a color to all pixels in the addressed byte of
              video memory. Bit 0 of the CPU data is written to plane 0, 
              bit 1 to plane 1, bit 2 to plane 2, bit 3 to plane 3.
              Individual pixels can be enabled or disabled through the
              Bit Mask register (3CEh index 8).
         *  
         *  example bit masks:
         *  0xc0 = b11000000 // 1st and 2nd pixels
         *  0x30 = b00110000 // 3rd and 4th pixels
         *  0x0c = b00001100 // 5th and 6th pixels
         *  0x03 = b00000011 // 7th and 8th pixels
         */
        uint8_t bitMask = m->m_MCGA[8];
        uint8_t mapmask = SQRegisters[2];
        mapmask &= 0x0f;  // clear high 4 bits
        for (int pixnum = 0; pixnum < 8; pixnum++)
        {
          uint8_t thispix = (bitMask >> pixnum) & 0x01; //isolate this pixel's mask bit
          if (thispix == 1) // if this pixel is set to be updated with this color
          {
            for (int planenum = 0; planenum < 4; planenum++) //loop through all four lower bits and write to corresponding bit plane if bit is set
            {
              uint32_t plane_addr = addr;
              //uint8_t thisbit = (mapmask >> planenum) & 0x01;  //isolate this bitplane's mask bit
              //if (thisbit == 1)  // if this bit is set, write to that plane
              {
                plane_addr = addr + (8000 * planenum);
                //if(MCGA_videoMemory[plane_addr] != value)
                {
                  lastVideoWrite = millis();
                  uint8_t oldvalue = MCGA_videoMemory[plane_addr];
                  uint8_t newmask = 0x01 << pixnum; 
                  newmask = ~newmask;                 
                  uint8_t newoldvalue = oldvalue & newmask; // zero out the bit that needs to be updated 
                  uint8_t newvalue = value >> planenum; 
                  newvalue = newvalue & 0x01; //isolate the bit to be set 
                  newvalue = newvalue << pixnum; // set up a bit value to be OR'd with newoldvalue 
                  newvalue = newoldvalue | newvalue; //update the bit 
                  MCGA_videoMemory[plane_addr] = newvalue;  
                  //printf("planenum=%d pixnum=%d plane_addr=%d, newmask=%d oldvalue=%d newoldvalue=%d newvalue=%d\n", planenum, pixnum, plane_addr, newmask, oldvalue, newoldvalue, newvalue);
                  //screenDirty = true; //updateGraphicsByte((plane_addr - fbAddr), newvalue, true, oldvalue);  
                  //screenDirty = true;
                  //last_screen_change_time = millis();
                  //global_need_refresh = true;
                  bitTable[(plane_addr - fbAddr)] = 1;
                  videoMemoryUpdated = true;
                  screenDirty = true; 
                }
              }
            }
          }
        }        
      }
      else if (egaWriteMode == 1 || egaWriteMode == 3) // if using EGA Write Mode 1 or 3 (instead of 2 or 0)
      {
        static bool EGA_WM_warning_printed = false;
        if (!EGA_WM_warning_printed)
        {
          printf("***** ATTEMPTING TO USE UNSUPPORTED EGA WRITE MODE %d ********\n", egaWriteMode);
          EGA_WM_warning_printed = true;
        }
      }
    }    
  }


  if (address >= 0xb0000)
  {
    //printf("WVMEM8 %05X <= 0x%02X %c\n", address, value, value);
    if (s_videoMemory[address - 0xb0000] != value)
    {
      lastVideoWrite = millis();
      s_videoMemory[address - 0xb0000] = value;  // fBuffer = s_videoMemory + n
      uint16_t addr = address - 0xb0000;
      uint32_t fbAddr = ((uint32_t)fBuffer - (uint32_t)s_videoMemory);
      if (!graphics_mode) // text mode
      {
        uint32_t ascii = 0;
        uint32_t attribute = 0;
        if (addr % 2 != 0) // in text modes, odd bytes are attributes, so need to drop back to previous byte if this is odd 
        {
          addr--;
        }
        //ascii = s_videoMemory[addr];
        //attribute = s_videoMemory[addr + 1];
        //screenDirty = true; //printCharAttr(ascii, attribute, (addr - fbAddr) / 2);

        //turn on a bit in the bit table at addr location
        //printf("WVM8: %c(%02x:%02x)<-%d\n", s_videoMemory[addr], s_videoMemory[addr], s_videoMemory[addr + 1], addr);
        bitTable[addr] = 1;
        videoMemoryUpdated = true;
        screenDirty = true; 
      }
      else //graphics mode
      {
        if (video_mode != MCGA_320X200_COLOR && video_mode != MCGA_640X480_MONO)
        {
          //screenDirty = true; //updateGraphicsByte((addr - fbAddr), value);   
          bitTable[addr] = 1;  
          videoMemoryUpdated = true;  
          screenDirty = true;   
        }
      }
      //screenDirty = true;
      last_screen_change_time = millis();
      global_need_refresh = true;   
    }
  }
}


void Machine::writeVideoMemory16(void * context, int address, uint16_t value)
{
  mems++;
  vidmems++;
  lastVideoMemoryWrite = millis();
  auto m = (Machine*)context;
  //printf("WVMEM16 %05X <= %04X\n", address, value);
  //printf("wvmem16 %d\n", millis());
//  if (address >= 0xa0000 && address < 0xb0000)
//  {
//    if(value)
//    {
//      printf("WVMEM16 %05X <= %04X %c %c\n", address, value, (value >> 8) & 0xff, value & 0xff);
//    }
//  }

  if (address >= 0xa0000 && address < 0xb0000 )
  {
    //printf("WVMEM16 %05X <= %04X %c %c\n", address, value, (value >> 8) & 0xff, value & 0xff);
    //if (s_videoMemory[address - 0xa0000] != value)
    //if (MCGA_videoMemory[address - 0xa0000] != value)
    if (video_mode == MCGA_320X200_COLOR || video_mode == MCGA_640X480_MONO)
    {
      if(*(uint16_t*)(MCGA_videoMemory + address - 0xa0000) != value)
      {    
        lastVideoWrite = millis();
        uint16_t old2bytes = *(uint16_t*)(MCGA_videoMemory + address - 0xa0000);
        *(uint16_t*)(MCGA_videoMemory + address - 0xa0000) = value;
        uint32_t addr = address - 0xa0000;
        uint32_t fbAddr = ((uint32_t)fBuffer - (uint32_t)s_videoMemory);
        //updateGraphicsByte((addr - fbAddr), value, false);   
        uint32_t value8 = uint8_t(value & 0xff); // low byte
        uint32_t oldvalue8 = uint8_t(old2bytes & 0xff); // low byte
        //screenDirty = true; //updateGraphicsByte((addr - fbAddr), value8, true, oldvalue8);
        bitTable[(addr - fbAddr)] = 1; 
        value8 = uint8_t(value >> 8); // high byte
        oldvalue8 = uint8_t(old2bytes >> 8); // high byte
        //screenDirty = true; //updateGraphicsByte((addr - fbAddr) + 1, value8, true, oldvalue8);
        bitTable[(addr - fbAddr) + 1] = 1; 
        videoMemoryUpdated = true;
        screenDirty = true; 
        //screenDirty = true;
        //last_screen_change_time = millis();
        //global_need_refresh = true;   
            
      }
    }
    else if (video_mode == EGA_320X200_COLOR)
    {
      uint32_t addr = address - 0xa0000;
      //SQRegisters[2] is the Map Mask register
      //printf("WVMEM16 %d <= %04X, WM=0x%02x, MMR=0x%02x\n", addr, value, SQRegisters[2]);

      uint32_t fbAddr = ((uint32_t)fBuffer - (uint32_t)MCGA_videoMemory);
      if (egaWriteMode == 0)  // if using EGA Write Mode 0 (instead of 2 or 1)
      {

/*
        //printf("WVMEM8 %d <= %02X, WM=0x%02x, MM=0x%02x, BM=0x%02x\n", addr, value, m->m_MCGA[5], SQRegisters[2], m->m_MCGA[8]);
        uint8_t mapmask = SQRegisters[2];
        mapmask &= 0x0f;  // clear high 4 bits
        for (int planenum = 0; planenum < 4; planenum++) //loop through all four lower bits and write to corresponding bit plane if bit is set
        {
          uint32_t plane_addr = addr;
          uint8_t thisbit = mapmask >> planenum & 0x01;  //isolate this bitplane's mask bit
          if (thisbit == 1)  // if this bit is set, write to that plane
          {
            plane_addr = addr + (8000 * planenum);
            if(MCGA_videoMemory[plane_addr] != value)
            {
              lastVideoWrite = millis();
              uint32_t oldvalue = MCGA_videoMemory[plane_addr];
              MCGA_videoMemory[plane_addr] = value;  
              updateGraphicsByte((plane_addr - fbAddr), value, true, oldvalue);  
              screenDirty = true;
              last_screen_change_time = millis();
              global_need_refresh = true;
            }
          }
        }
 */
        
        //printf("WVMEM16 0x%02x <= %04X, WM=0, MMR=0x%02x\n", addr, value, SQRegisters[2]);
        uint8_t mapmask = SQRegisters[2];
        mapmask &= 0x0f;  // clear high 4 bits
        for (int planenum = 0; planenum < 4; planenum++) //loop through all four lower bits and write to corresponding bit plane if bit is set
        {
          uint32_t plane_addr = addr;
          uint8_t thisbit = mapmask >> planenum & 0x01;  //isolate this bitplane's mask bit
          if (thisbit == 1)  // if this bit is set, write to that plane
          {
            plane_addr = addr + (8000 * planenum);
            //if(*(uint16_t*)(MCGA_videoMemory + addr) != value)
            {
              lastVideoWrite = millis();
              //uint16_t old2bytes = *(uint16_t*)(MCGA_videoMemory + plane_addr - 0xa0000);
              uint16_t old2bytes = *(uint16_t*)(MCGA_videoMemory + plane_addr);
              *(uint16_t*)(MCGA_videoMemory + plane_addr) = value;
              uint8_t value8 = uint8_t(value & 0xff); // low byte
              uint8_t oldvalue8 = uint8_t(old2bytes & 0xff); // low byte
              //screenDirty = true; //updateGraphicsByte((plane_addr - fbAddr), value8, true, oldvalue8);
              bitTable[(plane_addr - fbAddr)] = 1; 
              value8 = uint8_t(value >> 8); // high byte
              oldvalue8 = uint8_t(old2bytes >> 8); // high byte
              //screenDirty = true; //updateGraphicsByte((plane_addr - fbAddr) + 1, value8, true, oldvalue8); 
              bitTable[(plane_addr - fbAddr) + 1] = 1;  
              videoMemoryUpdated = true;
              screenDirty = true; 
              //screenDirty = true;
              //last_screen_change_time = millis();
              //global_need_refresh = true;
            }
          }
        }
      }
      else if (egaWriteMode == 2) // if using EGA Write Mode 2 (instead of 0 or 1)
      {
        //printf("***** EGA 16BIT WRITE MODE %d ********\n", egaWriteMode);
        //printf("WVMEM16 0x%02x <= %04X, WM=2, MMR=0x%02x\n", addr, value, SQRegisters[2]);
        /* Mode 2 writes a color to all pixels in the addressed byte of
              video memory. Bit 0 of the CPU data is written to plane 0, 
              bit 1 to plane 1, bit 2 to plane 2, bit 3 to plane 3.
              Individual pixels can be enabled or disabled through the
              Bit Mask register (3CEh index 8).
         *  
         *  example bit masks:
         *  0xc0 = b11000000 // 1st and 2nd pixels
         *  0x30 = b00110000 // 3rd and 4th pixels
         *  0x0c = b00001100 // 5th and 6th pixels
         *  0x03 = b00000011 // 7th and 8th pixels
         */
        uint8_t bitMask = m->m_MCGA[8];
        uint8_t mapmask = SQRegisters[2];
        mapmask &= 0x0f;  // clear high 4 bits
        for (int pixnum = 0; pixnum < 8; pixnum++)
        {
          uint8_t thispix = (bitMask >> pixnum) & 0x01; //isolate this pixel's mask bit
          if (thispix == 1) // if this pixel is set to be updated with this color
          {
            for (int planenum = 0; planenum < 4; planenum++) //loop through all four lower bits and write to corresponding bit plane if bit is set
            {
              uint32_t plane_addr = addr;
              //uint8_t thisbit = (mapmask >> planenum) & 0x01;  //isolate this bitplane's mask bit
              //if (thisbit == 1)  // if this bit is set, write to that plane
              {
                plane_addr = addr + (8000 * planenum);
                //if(MCGA_videoMemory[plane_addr] != value)
                {
                  lastVideoWrite = millis();
                  uint8_t oldvalue = MCGA_videoMemory[plane_addr + 1];
                  uint8_t newmask = 0x01 << pixnum; 
                  newmask = ~newmask;                 
                  uint8_t newoldvalue = oldvalue & newmask; // zero out the bit that needs to be updated 
                  uint8_t newvalue = value >> planenum; 
                  newvalue = newvalue & 0x01; //isolate the bit to be set 
                  newvalue = newvalue << pixnum; // set up a bit value to be OR'd with newoldvalue 
                  newvalue = newoldvalue | newvalue; //update the bit 
                  MCGA_videoMemory[plane_addr] = newvalue;  
                  //printf("planenum=%d pixnum=%d plane_addr=%d, newmask=%d oldvalue=%d newoldvalue=%d newvalue=%d\n", planenum, pixnum, plane_addr, newmask, oldvalue, newoldvalue, newvalue);
                  //screenDirty = true; //updateGraphicsByte((plane_addr - fbAddr), newvalue, true, oldvalue);
                  bitTable[(plane_addr - fbAddr)] = 1;  
                  videoMemoryUpdated = true; 
                  screenDirty = true; 
                  //screenDirty = true;
                  last_screen_change_time = millis();
                  
                  global_need_refresh = true;
                }
              }
            }
          }
        }         
      }
      else if (egaWriteMode == 1 || egaWriteMode == 3) // if using EGA Write Mode 1 (instead of 2 or 1)
      {
        printf("***** ATTEMPTING TO USE UNSUPPORTED EGA 16BIT WRITE MODE %d ********\n", egaWriteMode);
      }
    }    
  }
  
  if (address >= 0xb0000)
  {
    //printf("WVMEM16 %05X <= 0x%04X %c %c\n", address, value, (value >> 8) & 0xff, value & 0xff);
    if(*(uint16_t*)(s_videoMemory + address - 0xb0000) != value)
    {
      lastVideoWrite = millis();
      *(uint16_t*)(s_videoMemory + address - 0xb0000) = value;
      //printf("w8:%c\n", value);
      uint16_t addr = address - 0xb0000;      
      uint32_t fbAddr = ((uint32_t)fBuffer - (uint32_t)s_videoMemory);
      if(!graphics_mode) //text mode
      {
        //printf(".\n");
        uint32_t ascii = 0;
        uint32_t attribute = 0;
        
        /*
         * If addr is even, then first byte is ascii char and second byte is attribute.
         * If addr is odd, it is attribute of previous byte in s_videoMemory, and second byte is ascii char
         * In odd case, need to call printCharAttr for both previous byte in s_videoMemory and its attriute, as well as next byte in s_videoMemory and its attribute.
         */
        bool printnext = false;
        if (addr % 2 != 0) // in text modes, odd bytes are attributes to the character in the previous byte, so need to drop back to previous byte if this is odd 
        {
          addr--;
          printnext = true; // very unlikely, I think
        }
        //ascii = s_videoMemory[addr]; 
        //attribute = s_videoMemory[addr + 1];
        //screenDirty = true; //printCharAttr(ascii, attribute, (addr - fbAddr) / 2);
        //turn on a bit in the bit table at addr location
        //printf("WVM16: %c(%02x:%02x)<-%d\n", s_videoMemory[addr], s_videoMemory[addr], s_videoMemory[addr + 1], addr);
        bitTable[addr] = 1;
        videoMemoryUpdated = true;
        screenDirty = true; 
        if (printnext) // very unlikely, I think
        {
          //printf("PN\n");
          //ascii = s_videoMemory[addr + 2]; // second byte just written to s_videoMemory, in this case an ascii char
          //attribute = s_videoMemory[addr + 3]; // the attribute of this ascii char, already present in s_videoMemory, not written just now
          //screenDirty = true; //printCharAttr(ascii, attribute, ((addr - fbAddr) / 2) + 1);      
          addr += 2;
          //turn on a bit in the bit table at addr location
          //printf("WVM16: printnext: %c(%02x:%02x)<-%d\n", s_videoMemory[addr], s_videoMemory[addr], s_videoMemory[addr + 1], addr);
          bitTable[addr] = 1;
          videoMemoryUpdated = true;
          screenDirty = true; 
        }
      }
      else
      {
        if (video_mode != MCGA_320X200_COLOR && video_mode != MCGA_640X480_MONO)  
        {      
          //printf(":\n");
          uint32_t value8 = uint8_t(value & 0xff); // low byte
          //screenDirty = true; //updateGraphicsByte((addr - fbAddr), value8);
          bitTable[addr] = 1;
          value8 = uint8_t(value >> 8); // high byte
          //screenDirty = true; //updateGraphicsByte((addr - fbAddr) + 1, value8);
          bitTable[addr + 1] = 1;
          videoMemoryUpdated = true;
          screenDirty = true; 
        }
      }
      screenDirty = true;
      //last_screen_change_time = millis();
      //global_need_refresh = true;
    }
  }
}


uint8_t Machine::readVideoMemory8(void * context, int address)
{
  mems++;
  lastVideoMemoryRead = millis();
  if (address >= 0xa0000 && address < 0xb0000)
  {
    //printf("RVMEM8 %05X < 0xb0000, %02X %c\n", address, s_videoMemory[address - 0xb0000], MCGA_videoMemory[address - 0xa0000]);
    return MCGA_videoMemory[address - 0xa0000];
  }
  else if (address >= 0xb0000)
  {
    //printf("RVMEM8 %05X >= 0xb0000, %02X %c\n", address, s_videoMemory[address - 0xb0000], s_videoMemory[address - 0xb0000]);
    return s_videoMemory[address - 0xb0000]; 
  }   
  else
  {
    //printf("RVMEM8 return 0xff\n");
    return 0xff;
  }
  
//  if (address >= 0xb0000)
//    return s_videoMemory[address - 0xb0000];
//  else
//    return 0xff;
}


uint16_t Machine::readVideoMemory16(void * context, int address)
{
  mems++;
  lastVideoMemoryRead = millis();
  if (address >= 0xa0000  && address < 0xb0000)
  {
    //printf("RVMEM16 %05X < 0xb0000, %02X %c\n", address, s_videoMemory[address - 0xb0000], MCGA_videoMemory[address - 0xa0000]);
    return *(uint16_t*)(MCGA_videoMemory + address - 0xa0000);
  }
  else if (address >= 0xb0000)
  {
    //printf("RVMEM16 %05X >= 0xb0000, %02X %c\n", address, s_videoMemory[address - 0xb0000], s_videoMemory[address - 0xb0000]);
    return *(uint16_t*)(s_videoMemory + address - 0xb0000);
  }
  else
  {
    //printf("RVMEM16 return 0xff\n");
    return 0xffff;
  }
}

void Machine::incHltCount()
{
  hlts++;
}

void Machine::incMemsCount()
{
  mems++;
}


bool Machine::interrupt(void * context, int num)
{
  auto m = (Machine*)context;
  if (num == 0x28)
  {
    //printf("INT 28h\n");
    //m->lightSleep();
    int28s++;
  }
  if (num == 0x2f && i8086::AX() == 0x1680)
  {
    int2fs++;
  }
//  if (num != 0xf7 && num != 0xf8 && num != 0x2f && num != 0x21 && num != 0x16 && num != 0xf5 && num != 0x08 && num != 0x1c && num != 0x2a)
//  {
//    printf("int %02x\n", num);
//  }
  
  // emu interrupts callable only inside the BIOS segment
  if (i8086::CS() == BIOS_SEG)
  {
    //printf("num = 0x%02x\n", num);
    switch (num) 
    {
      // Put Char for debug (AL)  //this seems to be where we get the output like "VID01", "VID04", etc.
      case 0xf4:
        printf("%c", i8086::AX() & 0xff);
        return true;

      // BIOS helpers (AH = select helper function)
      case 0xf5:
        //printf("helpersEntry(), AH = 0x%02x\n", i8086::AH());
        //printf("H: 0x%02x\n", i8086::AH());
        m->m_BIOS.helpersEntry();
        if (i8086::AH() < 3)
        {
          // reading keyboard
          helpers++;
          lastHelpers = millis();
        }
        return true;

      // set or reset flag CF before IRET, replacing value in stack with current value
      case 0xf6:
      {
        auto savedFlags = (uint16_t*) (s_memory + i8086::SS() * 16 + (uint16_t)(i8086::SP() + 4));
        *savedFlags = (*savedFlags & 0xfffe) | i8086::flagCF();
        return true;
      }

      // set or reset flag ZF before IRET, replacing value in stack with current value
      case 0xf7:
      {
        auto savedFlags = (uint16_t*) (s_memory + i8086::SS() * 16 + (uint16_t)(i8086::SP() + 4));
        *savedFlags = (*savedFlags & 0xffbf) | (i8086::flagZF() << 6);
        return true;
      }

      // set or reset flag IF before IRET, replacing value in stack with current value
      case 0xf8:
      {
        auto savedFlags = (uint16_t*) (s_memory + i8086::SS() * 16 + (uint16_t)(i8086::SP() + 4));
        *savedFlags = (*savedFlags & 0xfdff) | (i8086::flagIF() << 9);
        return true;
      }

      // test point P0
      case 0xf9:
        printf("P0 AX=%04X BX=%04X CX=%04X DX=%04X DS=%04X\n", i8086::AX(), i8086::BX(), i8086::CX(), i8086::DX(), i8086::DS());
        return true;

      // test point P1
      case 0xfa:
        printf("P1 AX=%04X BX=%04X CX=%04X DX=%04X DS=%04X\n", i8086::AX(), i8086::BX(), i8086::CX(), i8086::DX(), i8086::DS());
        return true;

      // BIOS disk handler (INT 13h)
      case 0xfb:
        m->m_BIOS.diskHandlerEntry();
        lastDiskHandler = millis();
        return true;

      // BIOS video handler (INT 10h)
      case 0xfc:
        //printf("INT 10h\n");
        m->m_BIOS.videoHandlerEntry();
        lastVideoHandler = millis();
        return true;
    }
    //printf("i8086::CS() == BIOS_SEG, unknown interrupt %d\n", num);
  }
  //printf("unknown interrupt %d\n", num);
  // not hanlded
  return false;
}

bool Machine::intPending(int &IntNumber)
{
//  if (Int8Pending > 0)
//  {
//    IntNumber = 8;
//    Int8Pending--;
//    return true;
//  }
  //printf("inside intPending, NE2000IRQPending = %d\n", NE2000IRQPending);
//  if (NE2000IRQPending > 0)
//  {
//    //printf("Raising Hardware Interrupt %d\n", NE2000_INT_NO);
//    IntNumber = NE2000_INT_NO;
//    NE2000IRQPending--;
//    return true;
//  }
  if (MPU401IRQPending > 0)
  {
    IntNumber = MPU401_INT_NO;
    MPU401IRQPending--;
    return true;
  }

//  if (IsKeyEventAvailable() && !KeyInputFull)
//  {
//    KeyInputBuffer = NextKeyEvent();
//    KeyInputFull = true;
//    IntNumber = 9;
//    return true;
//  }

//  return SERIAL_IntPending(IntNumber);
  return false;
}

void Machine::speakerSetFreq()
{
#if ENABLE_SOUNDGEN
#if ENABLE_PC_SPEAKER
  //if (speakerEnabled)
  //{
    int timerCount = m_PIT8253.timerInfo(2).resetCount;
    if (timerCount == 0)
      timerCount = 65536;
    int freq = PIT_TICK_FREQ / timerCount;
    m_squareWaveGen.setFrequency(freq);
  
    speakerFreq = freq;
  //}
#endif // ENABLE_PC_SPEAKER
#endif // ENABLE_SOUNDGEN
}

void Machine::speakerEnableDisable()
{
#if ENABLE_SOUNDGEN
#if ENABLE_PC_SPEAKER
  bool genEnabled = m_PIT8253.getGate(2);
  if (genEnabled && m_speakerDataEnable) 
  {
    // speaker enabled
    m_squareWaveGen.enable(true);
    //speakerEnabled = true;
  }
  else 
  {
    // speaker disabled
    m_squareWaveGen.enable(false);
    //speakerEnabled = false;
  }
#endif //ENABLE_PC_SPEAKER
#endif // ENABLE_SOUNDGEN
}


void Machine::dumpMemory(char const * filename)
{
  constexpr int BLOCKLEN = 1024;
  auto file = FileBrowser(m_baseDir).openFile(filename, "wb");
  if (file) {
    for (int i = 0; i < RAM_SIZE; i += BLOCKLEN)
      fwrite(s_memory + i, 1, BLOCKLEN, file);
    fclose(file);
  }
}

void Machine::printDumpInfo()
{
    // CPU state
    printf(" CS   DS   ES   SS\n");
    printf("%04X %04X %04X %04X\n\n", i8086::CS(), i8086::DS(), i8086::ES(), i8086::SS());
    printf(" IP   AX   BX   CX   DX   SI   DI   BP   SP\n");
    printf("%04X %04X %04X %04X %04X %04X %04X %04X %04X\n\n", i8086::IP(), i8086::AX(), i8086::BX(), i8086::CX(), i8086::DX(), i8086::SI(), i8086::DI(), i8086::BP(), i8086::SP());
    printf("O D I T S Z A P C\n");
    printf("%d %d %d %d %d %d %d %d %d\n\n", i8086::flagOF(), i8086::flagDF(), i8086::flagIF(), i8086::flagTF(), i8086::flagSF(), i8086::flagZF(), i8086::flagAF(), i8086::flagPF(), i8086::flagCF());
    printf("CS+IP: %05X\n", i8086::CS() * 16 + i8086::IP());
    printf("SS+SP: %05X\n\n", i8086::SS() * 16 + i8086::SP());
    printf("seg_override_en = %d\n", *i8086::get_seg_override_en());
    printf("rep_override_en = %d\n", *i8086::get_rep_override_en());

    // video state
    printf("video_mode = %p\n", video_mode); 
    printf("graphics_mode = %p\n", graphics_mode); 
    printf("fBuffer = %p\n", fBuffer);  
    printf("m_frameBuffer = %p\n", m_frameBuffer);  
    printf("egaWriteMode = %02X\n", egaWriteMode);  
    printf("m_CGAMemoryOffset  = %04X\n", m_CGAMemoryOffset); 
    printf("m_MCGAMemoryOffset = %04X\n", m_MCGAMemoryOffset); 
    printf("m_HGCMemoryOffset  = %04X\n", m_HGCMemoryOffset);
    printf("SQRegisters = %02X %02X %02X %02X %02X\n", SQRegisters[0], SQRegisters[1], SQRegisters[2], SQRegisters[3], SQRegisters[4]); 
    // PIT8253 state
    printf("m_PIT8253.m_timer = ");
    for (int n = 0; n < 90; n++)
    {
      printf("%02X ", m_PIT8253.get_m_timer()[n]);
    }
    printf("\n");
    printf("m_PIT8253.get_m_lastTickTime = %04X\n", *m_PIT8253.get_m_lastTickTime()); 
    printf("m_PIT8253.get_m_acc = %04X\n", *m_PIT8253.get_m_acc()); 


    // RAM state
    printf("First 50 bytes of RAM: ");
    for (int n = 0; n < 50; n++)
    {
      printf("%02X ", (uint8_t)s_memory[n]);
    }
    printf("\n");
    printf("Last  50 bytes of RAM: ");
    for (int n = RAM_SIZE - 51; n < RAM_SIZE - 1; n++)
    {
      printf("%02X ", (uint8_t)s_memory[n]);
    }
    printf("\n");
    printf("First 50 bytes CS+IP:  ");
    for (int n = i8086::CS() * 16 + i8086::IP(); n < i8086::CS() * 16 + i8086::IP() + 50; n++)
    {
      printf("%02X ", (uint8_t)s_memory[n]);
    }   
    printf("\n");
    printf("First 50 bytes SS+SP:  ");
    for (int n = i8086::SS() * 16 + i8086::SP(); n < i8086::SS() * 16 + i8086::SP() + 50; n++)
    {
      printf("%02X ", (uint8_t)s_memory[n]);
    }     
    printf("\n");
}

void Machine::dumpInfo(char const * filename)
{
  auto file = FileBrowser(m_baseDir).openFile(filename, "wb");
  if (file) {
    // CPU state
    fprintf(file, " CS   DS   ES   SS\n");
    fprintf(file, "%04X %04X %04X %04X\n\n", i8086::CS(), i8086::DS(), i8086::ES(), i8086::SS());
    fprintf(file, " IP   AX   BX   CX   DX   SI   DI   BP   SP\n");
    fprintf(file, "%04X %04X %04X %04X %04X %04X %04X %04X %04X\n\n", i8086::IP(), i8086::AX(), i8086::BX(), i8086::CX(), i8086::DX(), i8086::SI(), i8086::DI(), i8086::BP(), i8086::SP());
    fprintf(file, "O D I T S Z A P C\n");
    fprintf(file, "%d %d %d %d %d %d %d %d %d\n\n", i8086::flagOF(), i8086::flagDF(), i8086::flagIF(), i8086::flagTF(), i8086::flagSF(), i8086::flagZF(), i8086::flagAF(), i8086::flagPF(), i8086::flagCF());
    fprintf(file, "CS+IP: %05X\n", i8086::CS() * 16 + i8086::IP());
    fprintf(file, "SS+SP: %05X\n\n", i8086::SS() * 16 + i8086::SP());
    fclose(file);
  }
}
