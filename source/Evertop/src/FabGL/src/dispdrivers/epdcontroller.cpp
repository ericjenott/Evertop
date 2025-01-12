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
#include "epdcontroller.h"


//epd stuff
#define ENABLE_GxEPD2_GFX 0
#include "../../../GxEPD2/src/GxEPD2_BW.h"
#include "../../../../board_def.h"
extern GxEPD2_BW<GxEPD2_583_T8, GxEPD2_583_T8::HEIGHT> display;


#pragma GCC optimize ("O2")




namespace fabgl {
















static inline __attribute__((always_inline)) void EPD_SETPIXELINROW(uint8_t * row, int x, int value) 
{
  int brow = x >> 3;  
  row[brow] ^= (-value ^ row[brow]) & (0x80 >> (x & 7));   //to set only one pixel in the byte at this offset
  
  uint16_t y = (row - EPDController::sgetScanline(0)) / 80;
  uint8_t color = (value > 0) ? GxEPD_WHITE : GxEPD_BLACK;
  //printf("1: %d, %d, %d\n", x, y, color);
  display.drawPixel(x, y, color);
}

static inline __attribute__((always_inline)) int EPD_GETPIXELINROW(uint8_t * row, int x) 
{
  int brow = x >> 3;
  return (row[brow] & (0x80 >> (x & 7))) != 0;
}

//#define EPD_INVERTPIXELINROW(row, x)       (row)[(x) >> 3] ^= (0x80 >> ((x) & 7))

static inline __attribute__((always_inline)) void EPD_INVERTPIXELINROW(volatile unsigned char * row, int x)
{
  // row[x >> 3] ^= (0x80 >> (x & 7));
  // uint16_t y = (row - EPDController::sgetScanline(0)) / 80;
  // //int color = EPD_GETPIXELINROW((uint8_t *)row, x);
  // int brow = x >> 3;
  // int oldcolor = ((row[brow] & (0x80 >> (x & 7))) != 0);
  // int newcolor = (oldcolor > 0) ? 255 : 0;
  // printf("oldcolor = %d, newcolor = %d\n", oldcolor, newcolor);
  // display.drawPixel(x, y, newcolor);
}

static inline __attribute__((always_inline)) void EPD_SETPIXEL(int x, int y, int value) 
{
  auto row = (uint8_t*) EPDController::sgetScanline(y);
  int brow = x >> 3;
  row[brow] ^= (-value ^ row[brow]) & (0x80 >> (x & 7));
  uint8_t color = (value > 0) ? GxEPD_WHITE : GxEPD_BLACK;
  //color = !color;
  //if(color == 1) color = 0;   else color = 1;

  //printf("x%dr%dy%dc%dv%d\n", x, row, y, color, value);
  //printf("1: %d, %d, %d\n", x, y, color);
  display.drawPixel(x, y, color);
}

#define EPD_GETPIXEL(x, y)                 EPD_GETPIXELINROW((uint8_t*)EPDController::s_viewPort[(y)], (x))

#define EPD_INVERT_PIXEL(x, y)             EPD_INVERTPIXELINROW((uint8_t*)EPDController::s_viewPort[(y)], (x))


//#define EPD_COLUMNSQUANTUM 16



/*************************************************************************************/
/* EPDController definitions */


EPDController::EPDController()
  : VGAPalettedController(EPD_LinesCount, 8, 1)
{
  m_packedPaletteIndexOctet_to_signals = (uint64_t *) heap_caps_malloc(256 * sizeof(uint64_t), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
}


EPDController::~EPDController()
{
  heap_caps_free((void *)m_packedPaletteIndexOctet_to_signals);
}


void EPDController::setupDefaultPalette()
{
  setPaletteItem(0, RGB888(0, 0, 0));       // 0: black
  setPaletteItem(1, RGB888(255, 255, 255)); // 1: white
}


void EPDController::setPaletteItem(int index, RGB888 const & color)
{
  index %= 2;
  m_palette[index] = color;
  auto packed222 = RGB888toPackedRGB222(color);
  for (int i = 0; i < 256; ++i) 
  {
    auto b = (uint8_t *) (m_packedPaletteIndexOctet_to_signals + i);
    for (int j = 0; j < 8; ++j) 
    {
      auto aj = 7 - j;
      if ((index == 0 && ((1 << aj) & i) == 0) || (index == 1 && ((1 << aj) & i) != 0)) 
      {
        b[j ^ 2] = packed222;
      }
    }
  }
}


void EPDController::setPixelAt(PixelDesc const & pixelDesc, Rect & updateRect)
{
  genericSetPixelAt(pixelDesc, updateRect,
                    [&] (RGB888 const & color) { return RGB888toPaletteIndex(color); },
                    EPD_SETPIXEL
                   );
}


// coordinates are absolute values (not relative to origin)
// line clipped on current absolute clipping rectangle
void EPDController::absDrawLine(int X1, int Y1, int X2, int Y2, RGB888 color)
{
  genericAbsDrawLine(X1, Y1, X2, Y2, color,
                     [&] (RGB888 const & color)                      { return RGB888toPaletteIndex(color); },
                     [&] (int Y, int X1, int X2, uint8_t colorIndex) { rawFillRow(Y, X1, X2, colorIndex); },
                     [&] (int Y, int X1, int X2)                     { rawInvertRow(Y, X1, X2); },
                     EPD_SETPIXEL,
                     [&] (int X, int Y)                              { EPD_INVERT_PIXEL(X, Y); }
                     );
}


// parameters not checked
void EPDController::rawFillRow(int y, int x1, int x2, RGB888 color)
{
  rawFillRow(y, x1, x2, RGB888toPaletteIndex(color));
}


// parameters not checked
void EPDController::rawFillRow(int y, int x1, int x2, uint8_t colorIndex)
{
  uint8_t * row = (uint8_t*) m_viewPort[y];
  // fill first pixels before full 8 bits word
  int x = x1;
  uint8_t testColor = 0;
  for (; x <= x2 && (x & 7) != 0; ++x) 
  {
    EPD_SETPIXELINROW(row, x, colorIndex);
	//EPD_SETPIXELINROW(row, x, testColor);
  }
  
  if (x <= x2) 
  {
    int sz = (x2 & ~7) - x;
    memset((void*)(row + x / 8), colorIndex ? 0xFF : 0x00, sz / 8);
    uint16_t y = (row - EPDController::sgetScanline(0)) / 80;
    for(int n = x; n < x2; n++)
    { 
      display.drawPixel(n, y, colorIndex ? GxEPD_WHITE : GxEPD_BLACK);
      //printf("1: %d, %d, %d\n", n, y, colorIndex ? GxEPD_WHITE : GxEPD_BLACK);
    }
    x += sz;
  }  
   
  // fill last unaligned pixels
  for (; x <= x2; ++x) 
  {
    EPD_SETPIXELINROW(row, x, colorIndex);
	//EPD_SETPIXELINROW(row, x, testColor);
  }
}


// parameters not checked
void EPDController::rawInvertRow(int y, int x1, int x2)
{
  auto row = m_viewPort[y];
  for (int x = x1; x <= x2; ++x)
    EPD_INVERTPIXELINROW(row, x);
}


void EPDController::rawCopyRow(int x1, int x2, int srcY, int dstY)
{
  auto srcRow = (uint8_t*) m_viewPort[srcY];
  auto dstRow = (uint8_t*) m_viewPort[dstY];
  // copy first pixels before full 8 bits word
  int x = x1;
  for (; x <= x2 && (x & 7) != 0; ++x) {
    EPD_SETPIXELINROW(dstRow, x, EPD_GETPIXELINROW(srcRow, x));
  }
  // copy whole 8 bits words (8 pixels)
  auto src = (uint8_t*)(srcRow + x / 8);
  auto dst = (uint8_t*)(dstRow + x / 8);
  for (int right = (x2 & ~7); x < right; x += 8)
    *dst++ = *src++;
  // copy last unaligned pixels
  for (x = (x2 & ~7); x <= x2; ++x) {
    EPD_SETPIXELINROW(dstRow, x, EPD_GETPIXELINROW(srcRow, x));
  }
}


void EPDController::swapRows(int yA, int yB, int x1, int x2)
{
  auto rowA = (uint8_t*) m_viewPort[yA];
  auto rowB = (uint8_t*) m_viewPort[yB];
  // swap first pixels before full 8 bits word
  int x = x1;
  for (; x <= x2 && (x & 7) != 0; ++x) {
    uint8_t a = EPD_GETPIXELINROW(rowA, x);
    uint8_t b = EPD_GETPIXELINROW(rowB, x);
    EPD_SETPIXELINROW(rowA, x, b);
    EPD_SETPIXELINROW(rowB, x, a);
  }
  // swap whole 8 bits words (8 pixels)
  auto a = (uint8_t*)(rowA + x / 8);
  auto b = (uint8_t*)(rowB + x / 8);
  for (int right = (x2 & ~7); x < right; x += 8)
    tswap(*a++, *b++);
  // swap last unaligned pixels
  for (x = (x2 & ~7); x <= x2; ++x) {
    uint8_t a = EPD_GETPIXELINROW(rowA, x);
    uint8_t b = EPD_GETPIXELINROW(rowB, x);
    EPD_SETPIXELINROW(rowA, x, b);
    EPD_SETPIXELINROW(rowB, x, a);
  }
}


void EPDController::drawEllipse(Size const & size, Rect & updateRect)
{
  genericDrawEllipse(size, updateRect,
                     [&] (RGB888 const & color)  { return RGB888toPaletteIndex(color); },
                     EPD_SETPIXEL
                    );
}


void EPDController::clear(Rect & updateRect)
{
  //printf("EPDController::clear()\n");
  hideSprites(updateRect);
  uint8_t paletteIndex = RGB888toPaletteIndex(getActualBrushColor());
  uint8_t pattern8 = paletteIndex ? 0xFF : 0x00;
  uint8_t color = (pattern8) ? GxEPD_WHITE : GxEPD_BLACK;
  //printf("RUNNING EPDController::clear()\n");
  for (int y = 0; y < m_viewPortHeight; ++y)
  {
    memset((uint8_t*) m_viewPort[y], pattern8, m_viewPortWidth / 8);
    for (int x = 0; x < m_viewPortWidth; x++)
    {
      //display.drawPixel(x, y, pattern8 & 0x01);   // clears screen to all white if bgcolor is (255, 255, 255)
      
      // for some reason, if I comment out next line (display.drawPixel(x, y, color)), the iBox menu frequently freezes on exit and no more display updates are made, although keyboard input is still recognized.
      // I suspect this might have something to do with timing, the delay caused by all these "drawPixel()s", since this shouldn't be necessary to keep running.
      display.drawPixel(x, y, color);   // clears screen to all white if bgcolor is (255, 255, 255), to black if bgcolor is (0, 0, 0)
      
      //printf("1: %d, %d, %d\n", x, y, color);
      //display.drawPixel(x, y, 1);   // clears screen to all white
      //display.drawPixel(x, y, 0);   // clears screen to all black
    }
  }
}


// scroll < 0 -> scroll UP
// scroll > 0 -> scroll DOWN
void EPDController::VScroll(int scroll, Rect & updateRect)
{
  genericVScroll(scroll, updateRect,
                 [&] (int yA, int yB, int x1, int x2)        { swapRows(yA, yB, x1, x2); },              // swapRowsCopying
                 [&] (int yA, int yB)                        { tswap(m_viewPort[yA], m_viewPort[yB]); }, // swapRowsPointers
                 [&] (int y, int x1, int x2, RGB888 color)   { rawFillRow(y, x1, x2, color); }           // rawFillRow
                );
}


void EPDController::HScroll(int scroll, Rect & updateRect)
{
  hideSprites(updateRect);
  uint8_t back  = RGB888toPaletteIndex(getActualBrushColor());
  uint8_t back8 = back ? 0xFF : 0x00;

  int Y1 = paintState().scrollingRegion.Y1;
  int Y2 = paintState().scrollingRegion.Y2;
  int X1 = paintState().scrollingRegion.X1;
  int X2 = paintState().scrollingRegion.X2;

  int width = X2 - X1 + 1;
  bool HScrolllingRegionAligned = ((X1 & 7) == 0 && (width & 7) == 0);  // 8 pixels aligned

  if (scroll < 0) {
    // scroll left
    for (int y = Y1; y <= Y2; ++y) {
      if (HScrolllingRegionAligned) {
        // aligned horizontal scrolling region, fast version
        uint8_t * row = (uint8_t*) (m_viewPort[y]) + X1 / 8;
        for (int s = -scroll; s > 0;) {
          if (s < 8) {
            // scroll left by 1..7
            int sz = width / 8;
            uint8_t prev = back8;
            for (int i = sz - 1; i >= 0; --i) {
              uint8_t lowbits = prev >> (8 - s);
              prev = row[i];
              row[i] = (row[i] << s) | lowbits;
            }
            s = 0;
          } else {
            // scroll left by multiplies of 8
            auto sc = s & ~7;
            auto sz = width & ~7;
            memmove(row, row + sc / 8, (sz - sc) / 8);
            rawFillRow(y, X2 - sc + 1, X2, back);
            s -= sc;
          }
        }
      } else {
        // unaligned horizontal scrolling region, fallback to slow version
        auto row = (uint8_t*) m_viewPort[y];
        for (int x = X1; x <= X2 + scroll; ++x)
          EPD_SETPIXELINROW(row, x, EPD_GETPIXELINROW(row, x - scroll));
        // fill right area with brush color
        rawFillRow(y, X2 + 1 + scroll, X2, back);
      }
    }
  } else if (scroll > 0) {
    // scroll right
    for (int y = Y1; y <= Y2; ++y) {
      if (HScrolllingRegionAligned) {
        // aligned horizontal scrolling region, fast version
        uint8_t * row = (uint8_t*) (m_viewPort[y]) + X1 / 8;
        for (int s = scroll; s > 0;) {
          if (s < 8) {
            // scroll right by 1..7
            int sz = width / 8;
            uint8_t prev = back8;
            for (int i = 0; i < sz; ++i) {
              uint8_t highbits = prev << (8 - s);
              prev = row[i];
              row[i] = (row[i] >> s) | highbits;
            }
            s = 0;
          } else {
            // scroll right by multiplies of 8
            auto sc = s & ~7;
            auto sz = width & ~7;
            memmove(row + sc / 8, row, (sz - sc) / 8);
            rawFillRow(y, X1, X1 + sc - 1, back);
            s -= sc;
          }
        }
      } else {
        // unaligned horizontal scrolling region, fallback to slow version
        auto row = (uint8_t*) m_viewPort[y];
        for (int x = X2 - scroll; x >= X1; --x)
          EPD_SETPIXELINROW(row, x + scroll, EPD_GETPIXELINROW(row, x));
        // fill left area with brush color
        rawFillRow(y, X1, X1 + scroll - 1, back);
      }
    }

  }
}


void EPDController::drawGlyph(Glyph const & glyph, GlyphOptions glyphOptions, RGB888 penColor, RGB888 brushColor, Rect & updateRect)
{
  genericDrawGlyph(glyph, glyphOptions, penColor, brushColor, updateRect,
                   [&] (RGB888 const & color)                     { return RGB888toPaletteIndex(color); },
                   [&] (int y)                                    { return (uint8_t*) m_viewPort[y]; },
                   EPD_SETPIXELINROW
                  );
}


void EPDController::invertRect(Rect const & rect, Rect & updateRect)
{
  genericInvertRect(rect, updateRect,
                    [&] (int Y, int X1, int X2) { rawInvertRow(Y, X1, X2); }
                   );
}


void EPDController::swapFGBG(Rect const & rect, Rect & updateRect)
{
  genericSwapFGBG(rect, updateRect,
                  [&] (RGB888 const & color)                     { return RGB888toPaletteIndex(color); },
                  [&] (int y)                                    { return (uint8_t*) m_viewPort[y]; },
                  EPD_GETPIXELINROW,
                  EPD_SETPIXELINROW
                 );
}


// Slow operation!
// supports overlapping of source and dest rectangles
void EPDController::copyRect(Rect const & source, Rect & updateRect)
{
  genericCopyRect(source, updateRect,
                  [&] (int y)                                    { return (uint8_t*) m_viewPort[y]; },
                  EPD_GETPIXELINROW,
                  EPD_SETPIXELINROW
                 );
}


// no bounds check is done!
void EPDController::readScreen(Rect const & rect, RGB888 * destBuf)
{
  for (int y = rect.Y1; y <= rect.Y2; ++y) {
    auto row = (uint8_t*) m_viewPort[y];
    for (int x = rect.X1; x <= rect.X2; ++x, ++destBuf) {
      const RGB222 v = m_palette[EPD_GETPIXELINROW(row, x)];
      *destBuf = RGB888(v.R * 85, v.G * 85, v.B * 85);  // 85 x 3 = 255
    }
  }
}


void EPDController::rawDrawBitmap_Native(int destX, int destY, Bitmap const * bitmap, int X1, int Y1, int XCount, int YCount)
{
  genericRawDrawBitmap_Native(destX, destY, (uint8_t*) bitmap->data, bitmap->width, X1, Y1, XCount, YCount,
                              [&] (int y)                             { return (uint8_t*) m_viewPort[y]; },  // rawGetRow
                              EPD_SETPIXELINROW
                             );
}


void EPDController::rawDrawBitmap_Mask(int destX, int destY, Bitmap const * bitmap, void * saveBackground, int X1, int Y1, int XCount, int YCount)
{
  auto foregroundColorIndex = RGB888toPaletteIndex(bitmap->foregroundColor);
  genericRawDrawBitmap_Mask(destX, destY, bitmap, (uint8_t*)saveBackground, X1, Y1, XCount, YCount,
                            [&] (int y)                  { return (uint8_t*) m_viewPort[y]; },                   // rawGetRow
                            EPD_GETPIXELINROW,
                            [&] (uint8_t * row, int x)   { EPD_SETPIXELINROW(row, x, foregroundColorIndex); }  // rawSetPixelInRow
                           );
}


void EPDController::rawDrawBitmap_RGBA2222(int destX, int destY, Bitmap const * bitmap, void * saveBackground, int X1, int Y1, int XCount, int YCount)
{
  genericRawDrawBitmap_RGBA2222(destX, destY, bitmap, (uint8_t*)saveBackground, X1, Y1, XCount, YCount,
                                [&] (int y)                             { return (uint8_t*) m_viewPort[y]; },       // rawGetRow
                                EPD_GETPIXELINROW,
                                [&] (uint8_t * row, int x, uint8_t src) { EPD_SETPIXELINROW(row, x, RGB2222toPaletteIndex(src)); }       // rawSetPixelInRow
                               );
}

} // end of namespace
