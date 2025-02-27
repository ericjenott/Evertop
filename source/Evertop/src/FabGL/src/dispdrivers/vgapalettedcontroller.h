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


#pragma once



/**
 * @file
 *
 * @brief This file contains fabgl::VGAPalettedController definition.
 */


#include <stdint.h>
#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "../fabglconf.h"
#include "../fabutils.h"
#include "../displaycontroller.h"
#include "vgabasecontroller.h"




namespace fabgl {






/**
* @brief Represents the base class for paletted bitmapped controllers like VGA16Controller, VGA8Controller, etc..
*/
class VGAPalettedController : public VGABaseController {

public:

  VGAPalettedController(int linesCount, int viewPortRatioDiv, int viewPortRatioMul);
  ~VGAPalettedController();

  // unwanted methods
  VGAPalettedController(VGAPalettedController const&) = delete;
  void operator=(VGAPalettedController const&)        = delete;

  void end();

  // abstract method of BitmappedDisplayController
  //void suspendBackgroundPrimitiveExecution();

  // import "modeline" version of setResolution
  using VGABaseController::setResolution;

  void setResolution(int viewPortWidth = -1, int viewPortHeight = -1, bool clearScreen = true);

  // returns "static" version of m_viewPort
  static uint8_t * sgetScanline(int y)                  { return (uint8_t*) s_viewPort[y]; }
  
  
  
  // abstract method of BitmappedDisplayController
  //NativePixelFormat nativePixelFormat()                 { return m_nativePixelFormat; }
  


protected:

  void init();

  virtual void setupDefaultPalette() = 0;

  void updateRGB2PaletteLUT();
  //static void primitiveExecTask(void * arg);

  uint8_t RGB888toPaletteIndex(RGB888 const & rgb) {
    return m_packedRGB222_to_PaletteIndex[RGB888toPackedRGB222(rgb)];
  }

  uint8_t RGB2222toPaletteIndex(uint8_t value) {
    return m_packedRGB222_to_PaletteIndex[value & 0b00111111];
  }



  //TaskHandle_t                m_primitiveExecTask;

  volatile uint8_t * *        m_lines;

  // optimization: clone of m_viewPort
  static volatile uint8_t * * s_viewPort;

  RGB222 *                    m_palette;


private:

  void allocateViewPort();
  void freeViewPort();

  // Maximum time (in CPU cycles) available for primitives drawing
  volatile uint32_t           m_primitiveExecTimeoutCycles;
  
  //volatile bool               m_taskProcessingPrimitives;
  
  uint8_t                     m_packedRGB222_to_PaletteIndex[64];

  // configuration
  int                         m_linesCount;     // viewport height must be divisible by m_linesCount
  //NativePixelFormat           m_nativePixelFormat;
  int                         m_viewPortRatioDiv;
  int                         m_viewPortRatioMul;


};



} // end of namespace








