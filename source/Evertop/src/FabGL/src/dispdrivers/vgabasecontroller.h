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
 * @brief This file contains fabgl::VGABaseController definition.
 */


#include <stdint.h>
#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "../fabglconf.h"
#include "../fabutils.h"
#include "../displaycontroller.h"




namespace fabgl {


class VGABaseController : public GenericBitmappedDisplayController {

public:

  VGABaseController();

  // unwanted methods
  VGABaseController(VGABaseController const&) = delete;
  void operator=(VGABaseController const&)    = delete;

  /**
   * @brief This is the 64 colors (8 GPIOs) initializer using default pinout.
   *
   * Two GPIOs per channel, plus horizontal and vertical sync signals.
   * Use GPIO 22-21 for red, GPIO 19-18 for green, GPIO 5-4 for blue, GPIO 23 for HSync and GPIO 15 for VSync
   *
   * Example:
   *
   *     VGAController.begin();
   */
  void begin();


  virtual void end();
  
  // abstract method of BitmappedDisplayController
  //virtual void suspendBackgroundPrimitiveExecution();

  // abstract method of BitmappedDisplayController
  //virtual void resumeBackgroundPrimitiveExecution();
  
  /**
   * @brief Sets current resolution using linux-like modeline.
   *
   * Modeline must have following syntax (non case sensitive):
   *
   *
   * In fabglconf.h there are macros with some predefined modelines for common resolutions.
   * When MultiScanBlank and DoubleScan is specified then additional rows are not repeated, but just filled with blank lines.
   *
   * @param modeline Linux-like modeline as specified above.
   * @param viewPortWidth Horizontal viewport size in pixels. If less than zero (-1) it is sized to modeline visible area width.
   * @param viewPortHeight Vertical viewport size in pixels. If less then zero (-1) it is sized to maximum allocable.

   * Example:
   *
   *     // Use predefined modeline for 640x480@60Hz
   *     VGAController.setResolution(VGA_640x480_60Hz);
   *
   *     // The same of above using modeline string
   *     VGAController.setResolution("\"640x480@60Hz\" 25.175 640 656 752 800 480 490 492 525 -HSync -VSync");
   *
   *     // Set 640x382@60Hz but limit the viewport to 640x350
   *     VGAController.setResolution(VGA_640x382_60Hz, 640, 350);
   *
   */
  //void setResolution(int viewPortWidth = -1, int viewPortHeight = -1);

  virtual void setResolution(int viewPortWidth = -1, int viewPortHeight = -1, bool clearScreen = true);


protected:

  virtual void freeViewPort();

  virtual void init();

  void allocateViewPort(uint32_t allocCaps, int rowlen);
  virtual void allocateViewPort() = 0;


  volatile uint8_t * *   m_viewPort;

private:
  uint8_t * *            m_viewPortMemoryPool;  // array ends with nullptr

};








} // end of namespace

