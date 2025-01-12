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



#include <alloca.h>
#include <stdarg.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "../fabutils.h"
#include "vgabasecontroller.h"


//epd stuff
#define ENABLE_GxEPD2_GFX 0
#include "../../../GxEPD2/src/GxEPD2_BW.h"
#include "../../../../board_def.h"
extern GxEPD2_BW<GxEPD2_583_T8, GxEPD2_583_T8::HEIGHT> display;
extern uint8_t * iBoxMemory;

#pragma GCC optimize ("O2")


namespace fabgl {


#if FABGLIB_VGAXCONTROLLER_PERFORMANCE_CHECK
  volatile uint64_t s_vgapalctrlcycles = 0;
#endif



VGABaseController::VGABaseController()
{
}


void VGABaseController::init()
{
  m_viewPort                      = nullptr;
  m_viewPortMemoryPool            = nullptr;
}




// initializer for default configuration
void VGABaseController::begin()
{
  init();
  //begin(GPIO_NUM_22, GPIO_NUM_21, GPIO_NUM_13, GPIO_NUM_25, GPIO_NUM_14, GPIO_NUM_16, GPIO_NUM_23, GPIO_NUM_15);
}


void VGABaseController::end()
{
    // Serial.println(__LINE__);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    // Serial.println(__LINE__);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    // Serial.println(__LINE__);
}

void VGABaseController::freeViewPort()
{
  if (m_viewPortMemoryPool) {
    for (auto poolPtr = m_viewPortMemoryPool; *poolPtr; ++poolPtr)
      heap_caps_free((void*) *poolPtr);
    heap_caps_free(m_viewPortMemoryPool);
    m_viewPortMemoryPool = nullptr;
  }
  m_viewPort = nullptr;
}

void VGABaseController::setResolution(int viewPortWidth, int viewPortHeight, bool clearScreen)
{
  // just in case setResolution() was called before
  end();
  
  // inform base class about screen size
  setScreenSize(640, 480);
  //Serial.println("calling setDoubleBuffered(false)");
  m_viewPortWidth = 640;
  m_viewPortHeight = 480;
  

  // allocate the viewport
  allocateViewPort();
  resetPaintState();
}


// this method may adjust m_viewPortHeight to the actual number of allocated rows.
// to reduce memory allocation overhead try to allocate the minimum number of blocks.
void VGABaseController::allocateViewPort(uint32_t allocCaps, int rowlen)
{
  // I think this tries to find mallocable places in memory where individual lines (rows) can be stored, rather
  // than insisting that m_viewPort is all one contiguous block of RAM.  Good idea!
  
  int linesCount[FABGLIB_VIEWPORT_MEMORY_POOL_COUNT]; // where store number of lines for each pool
  int poolsCount = 0; // number of allocated pools
  int remainingLines = m_viewPortHeight;
  m_viewPortHeight = 0; // m_viewPortHeight needs to be recalculated

  // allocate pools
  //m_viewPortMemoryPool = (uint8_t * *) heap_caps_malloc(sizeof(uint8_t*) * (FABGLIB_VIEWPORT_MEMORY_POOL_COUNT + 1), MALLOC_CAP_32BIT);
  m_viewPortMemoryPool = (uint8_t * * )(iBoxMemory + (81 * 480) + 4800); //iBoxMemory + (number of physical pixels on screen / 8, this for m_eventsQueue) + size of ucQueueStorageArea
  while (remainingLines > 0 && poolsCount < FABGLIB_VIEWPORT_MEMORY_POOL_COUNT) 
  {
    // int largestBlock = heap_caps_get_largest_free_block(allocCaps);
    // if (largestBlock < FABGLIB_MINFREELARGESTBLOCK)
    // {
      // break;
    // }
    linesCount[poolsCount] = remainingLines;
    printf("linesCount[%d] = %d\n", poolsCount, linesCount[poolsCount]);
    printf("iBoxMemory = %p\n", iBoxMemory);
    m_viewPortMemoryPool[poolsCount] = iBoxMemory;  // use range after emulator RAM and video_task bit table in PSRAM
    printf("m_viewPortMemoryPool[poolsCount] = %p\n", m_viewPortMemoryPool[poolsCount]);
    if (m_viewPortMemoryPool[poolsCount] == nullptr)
    {
      printf("breaking in VGABaseController::allocateViewPort()(1)\n");
      break;
    }
    remainingLines -= linesCount[poolsCount];
    m_viewPortHeight += linesCount[poolsCount];
    ++poolsCount;
  }
  m_viewPortMemoryPool[poolsCount] = nullptr;

  //m_viewPort = (volatile uint8_t * *) heap_caps_malloc(sizeof(uint8_t*) * m_viewPortHeight, MALLOC_CAP_32BIT | MALLOC_CAP_INTERNAL);  //m_viewPortHeight = 480
  //uint8_t  * ucQueueStorageArea = iBoxMemory + (81 * 480);  // iBoxMemory + number of physical pixels on screen / eight bits per pixel
  m_viewPort = (volatile uint8_t**)(m_viewPortMemoryPool + (FABGLIB_VIEWPORT_MEMORY_POOL_COUNT + 1)); //m_viewPortMemoryPool + (m_viewPortMemoryPool's size in bytes).  This is a bug.  Should be sizeof(uint8_t*) * (FABGLIB_VIEWPORT_MEMORY_POOL_COUNT + 1).  But actually FABGLIB_VIEWPORT_MEMORY_POOL_COUNT should be set to 1 since we're not using fragmented memory for this, instead using one contiguous block of PSRAM.  So leave this to fix later.
  if (!m_viewPort)
  {
    printf("FAILED ALLOCATING m_viewPort!!!!\n");
  }
  for (int p = 0, l = 0; p < poolsCount; ++p) 
  {
    uint8_t * pool = m_viewPortMemoryPool[p];
    for (int i = 0; i < linesCount[p]; ++i) 
    {
      if (l + i < m_viewPortHeight)
      {
        m_viewPort[l + i] = pool;
      }    
      pool += rowlen;
    }
    l += linesCount[p];
  }
  printf("m_viewPortHeight = %d\n", m_viewPortHeight);
}



} // end of namespace


