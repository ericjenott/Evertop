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

#include "esp_heap_caps.h"

#include "../fabutils.h"
#include "vgapalettedcontroller.h"

//epd stuff
#define ENABLE_GxEPD2_GFX 0
#include "../../../GxEPD2/src/GxEPD2_BW.h"
#include "../../../../board_def.h"
extern GxEPD2_BW<GxEPD2_583_T8, GxEPD2_583_T8::HEIGHT> display;


#pragma GCC optimize ("O2")



namespace fabgl {





/*************************************************************************************/
/* VGAPalettedController definitions */


volatile uint8_t * * VGAPalettedController::s_viewPort;




VGAPalettedController::VGAPalettedController(int linesCount, int viewPortRatioDiv, int viewPortRatioMul)
  : m_linesCount(linesCount),
    m_viewPortRatioDiv(viewPortRatioDiv),
    m_viewPortRatioMul(viewPortRatioMul)
{
  m_lines   = (volatile uint8_t**) heap_caps_malloc(sizeof(uint8_t*) * m_linesCount, MALLOC_CAP_32BIT | MALLOC_CAP_INTERNAL);
  m_palette = (RGB222*) heap_caps_malloc(sizeof(RGB222) * 2, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
}


VGAPalettedController::~VGAPalettedController()
{
  heap_caps_free(m_palette);
  heap_caps_free(m_lines);
}


void VGAPalettedController::init()
{
  VGABaseController::init();
}


void VGAPalettedController::end()
{
  // Serial.println(__LINE__);
  VGABaseController::end();
  // Serial.println(__LINE__);
}

void VGAPalettedController::allocateViewPort()
{
  VGABaseController::allocateViewPort(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL, m_viewPortWidth / m_viewPortRatioDiv * m_viewPortRatioMul);

  for (int i = 0; i < m_linesCount; ++i)
    m_lines[i] = (uint8_t*) heap_caps_malloc(m_viewPortWidth, MALLOC_CAP_DEFAULT);
}


void VGAPalettedController::freeViewPort()
{
  VGABaseController::freeViewPort();

  for (int i = 0; i < m_linesCount; ++i) {
    heap_caps_free((void*)m_lines[i]);
    m_lines[i] = nullptr;
  }
}


void VGAPalettedController::setResolution(int viewPortWidth, int viewPortHeight, bool clearScreen)
{
  VGABaseController::setResolution(viewPortWidth, viewPortHeight);

  s_viewPort        = m_viewPort;


  printf("m_viewPortWidth / m_viewPortRatioDiv * m_viewPortRatioMul = %d / %d * %d, result = %d\n", m_viewPortWidth, m_viewPortRatioDiv, m_viewPortRatioMul, m_viewPortWidth / m_viewPortRatioDiv * m_viewPortRatioMul);
  // fill view port with 0x00
  for (int i = 0; i < m_viewPortHeight; ++i)
  {
    // fill "i" row of view port?
    // memset((void*)(m_viewPort[i]), 0, m_viewPortWidth / m_viewPortRatioDiv * m_viewPortRatioMul);
    memset((void*)(m_viewPort[i]), 255, m_viewPortWidth / m_viewPortRatioDiv * m_viewPortRatioMul);  //initially fill with background white (255) rather than black (0)
    int y = i;
    if (clearScreen)
    {
      for (int x = 0; x < m_viewPortWidth; x++)
      {
        // experimentally commented out
        display.drawPixel(x, y, 255);
      }
    }
  }
  printf("getCycleCount() returned %d\n", getCycleCount());
  setupDefaultPalette();
  updateRGB2PaletteLUT();
 
}


// rebuild m_packedRGB222_to_PaletteIndex
void VGAPalettedController::updateRGB2PaletteLUT()
{
  auto paletteSize = 2;
  for (int r = 0; r < 4; ++r)
    for (int g = 0; g < 4; ++g)
      for (int b = 0; b < 4; ++b) 
      {
        double H1, S1, V1;
        rgb222_to_hsv(r, g, b, &H1, &S1, &V1);
        int bestIdx = 0;
        int bestDst = 1000000000;
        for (int i = 0; i < paletteSize; ++i) 
        {
          double H2, S2, V2;
          rgb222_to_hsv(m_palette[i].R, m_palette[i].G, m_palette[i].B, &H2, &S2, &V2);
          double AH = H1 - H2;
          double AS = S1 - S2;
          double AV = V1 - V2;
          int dst = AH * AH + AS * AS + AV * AV;
          if (dst <= bestDst)  // "<=" to prioritize higher indexes
          {  
            bestIdx = i;
            bestDst = dst;
            if (bestDst == 0)
              break;
          }
        }
        m_packedRGB222_to_PaletteIndex[r | (g << 2) | (b << 4)] = bestIdx;
      }
}


} // end of namespace

