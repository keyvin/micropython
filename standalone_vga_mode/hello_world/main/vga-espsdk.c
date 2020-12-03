
#include <stdint.h>
#include "driver/gpio.h"
#include "rom/lldesc.h"


typedef uint8_t Color;


typedef struct RGBT {
  uint8_t R;
  uint8_t G;
  uint8_t B;
} RGB222;
    
//typedef int bool;


//defines for vga text mode
#define VGATextController_CHARWIDTH      8    // max 8
#define VGATextController_CHARWIDTHBYTES ((VGATextController_CHARWIDTH + 7) / 8)
#define VGATextController_CHARHEIGHT     14
#define VGATextController_COLUMNS        80
#define VGATextController_ROWS           34
#define VGATextController_WIDTH          640
#define VGATextController_HEIGHT         480
#define VGATextController_MODELINE       VGA_640x480_60Hz



#define VGA_RED_BIT   0
#define VGA_GREEN_BIT 2
#define VGA_BLUE_BIT  4
#define VGA_HSYNC_BIT 6
#define VGA_VSYNC_BIT 7

#define VGA_SYNC_MASK ((1 << VGA_HSYNC_BIT) | (1 << VGA_VSYNC_BIT))



typedef enum VGASCanStart {
  FrontPorch,   /**< Horizontal line sequence is: FRONTPORCH -> SYNC -> BACKPORCH -> VISIBLEAREA */
  Sync,         /**< Horizontal line sequence is: SYNC -> BACKPORCH -> VISIBLEAREA -> FRONTPORCH */
  BackPorch,    /**< Horizontal line sequence is: BACKPORCH -> VISIBLEAREA -> FRONTPORCH -> SYNC */
  VisibleArea   /**< Horizontal line sequence is: VISIBLEAREA -> FRONTPORCH -> SYNC -> BACKPORCH */
} VGAScanStart;


typedef struct VGATImings {
  char          label[22];       /**< Resolution text description */
  int           frequency;       /**< Pixel frequency (in Hz) */
  int16_t       HVisibleArea;    /**< Horizontal visible area length in pixels */
  int16_t       HFrontPorch;     /**< Horizontal Front Porch duration in pixels */
  int16_t       HSyncPulse;      /**< Horizontal Sync Pulse duration in pixels */
  int16_t       HBackPorch;      /**< Horizontal Back Porch duration in pixels */
  int16_t       VVisibleArea;    /**< Vertical number of visible lines */
  int16_t       VFrontPorch;     /**< Vertical Front Porch duration in lines */
  int16_t       VSyncPulse;      /**< Vertical Sync Pulse duration in lines */
  int16_t       VBackPorch;      /**< Vertical Back Porch duration in lines */
  char          HSyncLogic;      /**< Horizontal Sync polarity '+' or '-' */
  char          VSyncLogic;      /**< Vertical Sync polarity '+' or '-' */
  uint8_t       scanCount;       /**< Scan count. 1 = single scan, 2 = double scan (allowing low resolutions like 320x240...) */
  uint8_t       multiScanBlack;  /**< 0 = Additional rows are the repetition of the first. 1 = Additional rows are blank. */
  VGAScanStart  HStartingBlock;  /**< Horizontal starting block. DetermineshHorizontal order of signals */
} VGATimings;

  static volatile int        s_scanLine;
  static uint32_t            s_blankPatternDWord;
  static uint32_t *          s_fgbgPattern;
  static int                 s_textRow;
  static bool                s_upperRow;
  static volatile lldesc_t * s_frameResetDesc;

  VGATimings             m_timings;

//GPIOStream             m_GPIOStream;
  int                    m_bitsPerChannel;  // 1 = 8 colors, 2 = 64 colors, set by begin()
  lldesc_t volatile *    m_DMABuffers;
  int                    m_DMABuffersCount;

  uint32_t *             m_lines;

  int                    m_rows;

  volatile uint8_t *     m_blankLine; // for vertical porch lines
  volatile uint8_t *     m_syncLine;  // for vertical sync lines

  intr_handle_t          m_isr_handle;

  // contains H and V signals for visible line
  volatile uint8_t       m_HVSync;

  uint8_t *              m_charData;
  uint32_t const *       m_map;

  // cursor props
  bool                   m_cursorEnabled;
  int                    m_cursorCounter; // trip from -m_cursorSpeed to +m_cursorSpeed (>= cursor is visible)
  int                    m_cursorSpeed;
  int                    m_cursorRow;
  int                    m_cursorCol;
  uint8_t                m_cursorForeground;
  uint8_t                m_cursorBackground;
  



// pixel 0 = byte 2, pixel 1 = byte 3, pixel 2 = byte 0, pixel 3 = byte 1 :
// pixel : 0  1  2  3  4  5  6  7  8  9 10 11 ...etc...
// byte  : 2  3  0  1  6  7  4  5 10 11  8  9 ...etc...
// dword : 0           1           2          ...etc...
// Thanks to https://github.com/paulscottrobson for the new macro. Before was: (row[((X) & 0xFFFC) + ((2 + (X)) & 3)])
#define VGA_PIXELINROW(row, X) (row[(X) ^ 2])

// requires variables: m_viewPort
#define VGA_PIXEL(X, Y)    VGA_PIXELINROW(m_viewPort[(Y)], X)
#define VGA_INVERT_PIXEL(X, Y) { auto px = &VGA_PIXEL((X), (Y)); *px = ~(*px ^ VGA_SYNC_MASK); }





  void setResolution(VGATimings const timings);
  void init(gpio_num_t VSyncGPIO);
  void setupGPIO(gpio_num_t gpio, int bit, gpio_mode_t mode);
  void freeBuffers();

  void fillDMABuffers();
  uint8_t packHVSync(bool HSync, bool VSync);
  uint8_t preparePixelWithSync(RGB222 rgb, bool HSync, bool VSync);

  uint8_t IRAM_ATTR preparePixel(RGB222 rgb) { return m_HVSync | (rgb.B << VGA_BLUE_BIT) | (rgb.G << VGA_GREEN_BIT) | (rgb.R << VGA_RED_BIT); }

  static void I2SInterrupt(void * arg);

  
  
void begin(gpio_num_t redGPIO, gpio_num_t greenGPIO, gpio_num_t blueGPIO, gpio_num_t HSyncGPIO, gpio_num_t VSyncGPIO);
 
void setResolution(char const * modeline, int viewPortWidth, int viewPortHeight, bool doubleBuffered);

void setTextMap(uint32_t const * map, int rows);

  void adjustMapSize(int * columns, int * rows);

  int getScreenWidth()    { return m_timings.HVisibleArea; }
  int getScreenHeight()   { return m_timings.VVisibleArea; }

  int getColumns()        { return VGATextController_COLUMNS; }
  int getRows()           { return VGATextController_ROWS; }

  void enableCursor(bool value)            { m_cursorEnabled = value; }
  void setCursorPos(int row, int col)      { m_cursorRow = row; m_cursorCol = col; m_cursorCounter = 0; }
  void setCursorSpeed(int value)           { m_cursorSpeed = value; }
  void setCursorForeground(Color value);
  void setCursorBackground(Color value);
  
 void begin(gpio_num_t redGPIO, gpio_num_t greenGPIO, gpio_num_t blueGPIO, gpio_num_t HSyncGPIO, gpio_num_t VSyncGPIO)
{
  init(VSyncGPIO);

  // GPIO configuration for bit 0
  setupGPIO(redGPIO,   VGA_RED_BIT,   GPIO_MODE_OUTPUT);
  setupGPIO(greenGPIO, VGA_GREEN_BIT, GPIO_MODE_OUTPUT);
  setupGPIO(blueGPIO,  VGA_BLUE_BIT,  GPIO_MODE_OUTPUT);

  // GPIO configuration for VSync and HSync
  setupGPIO(HSyncGPIO, VGA_HSYNC_BIT, GPIO_MODE_OUTPUT);
  setupGPIO(VSyncGPIO, VGA_VSYNC_BIT, GPIO_MODE_INPUT_OUTPUT);  // input/output so can be generated interrupt on falling/rising edge

  //  RGB222::lowBitOnly = true;
  m_bitsPerChannel = 1;
}
  
  void setResolution(char const * modeline, int viewPortWidth, int viewPortHeight, bool doubleBuffered)
{
  VGATimings timings;
  if (convertModelineToTimings(VGATextController_MODELINE, &timings))
    setResolution(timings);
}

void setResolution(VGATimings const timings)
{
  if (m_DMABuffers) {
    m_GPIOStream.stop();
    freeBuffers();
  }

  m_timings = timings;

  m_HVSync = packHVSync(false, false);

  m_DMABuffersCount = 2 * m_timings.VVisibleArea + m_timings.VFrontPorch + m_timings.VSyncPulse + m_timings.VBackPorch;

  m_DMABuffers = (lldesc_t*) heap_caps_malloc(m_DMABuffersCount * sizeof(lldesc_t), MALLOC_CAP_DMA);

  m_lines = (uint32_t*) heap_caps_malloc(VGATextController_CHARHEIGHT * VGATextController_WIDTH, MALLOC_CAP_DMA);

  int rawLineWidth = m_timings.HFrontPorch + m_timings.HSyncPulse + m_timings.HBackPorch + m_timings.HVisibleArea;
  m_blankLine = (uint8_t*) heap_caps_malloc(rawLineWidth, MALLOC_CAP_DMA);
  m_syncLine  = (uint8_t*) heap_caps_malloc(rawLineWidth, MALLOC_CAP_DMA);

  // horiz: FRONTPORCH -> SYNC -> BACKPORCH -> VISIBLEAREA
  //
  // vert:  VISIBLEAREA
  //        FRONTPORCH
  //        SYNC
  //        BACKPORCH

  for (int i = 0, visLine = 0, invLine = 0; i < m_DMABuffersCount; ++i) {

    if (i < m_timings.VVisibleArea * 2) {

      // first part is the same of a blank line
      m_DMABuffers[i].eof          = (visLine == 0 || visLine == VGATextController_CHARHEIGHT / 2 ? 1 : 0);
      m_DMABuffers[i].sosf         = 0;
      m_DMABuffers[i].offset       = 0;
      m_DMABuffers[i].owner        = 1;
      m_DMABuffers[i].qe.stqe_next = (lldesc_t*) &m_DMABuffers[i + 1];
      m_DMABuffers[i].length       = m_timings.HFrontPorch + m_timings.HSyncPulse + m_timings.HBackPorch;
      m_DMABuffers[i].size         = (m_DMABuffers[i].length + 3) & (~3);
      m_DMABuffers[i].buf          = (uint8_t*) m_blankLine;

      ++i;

      // second part is the visible line
      m_DMABuffers[i].eof          = 0;
      m_DMABuffers[i].sosf         = 0;
      m_DMABuffers[i].offset       = 0;
      m_DMABuffers[i].owner        = 1;
      m_DMABuffers[i].qe.stqe_next = (lldesc_t*) &m_DMABuffers[i + 1];
      m_DMABuffers[i].length       = m_timings.HVisibleArea;
      m_DMABuffers[i].size         = (m_DMABuffers[i].length + 3) & (~3);
      m_DMABuffers[i].buf          = (uint8_t*)(m_lines) + visLine * VGATextController_WIDTH;

      ++visLine;
      if (visLine == VGATextController_CHARHEIGHT)
        visLine = 0;

    } else {

      // vertical porchs and sync

      bool frameResetDesc = (invLine == 0);

      if (frameResetDesc)
        s_frameResetDesc = &m_DMABuffers[i];

      m_DMABuffers[i].eof          = (frameResetDesc ? 1 : 0); // prepare for next frame
      m_DMABuffers[i].sosf         = 0;
      m_DMABuffers[i].offset       = 0;
      m_DMABuffers[i].owner        = 1;
      m_DMABuffers[i].qe.stqe_next = (lldesc_t*) (i ==  m_DMABuffersCount - 1 ? &m_DMABuffers[0] : &m_DMABuffers[i + 1]);
      m_DMABuffers[i].length       = rawLineWidth;
      m_DMABuffers[i].size         = (m_DMABuffers[i].length + 3) & (~3);

      if (invLine < m_timings.VFrontPorch || invLine >= m_timings.VFrontPorch + m_timings.VSyncPulse)
        m_DMABuffers[i].buf = (uint8_t*) m_blankLine;
      else
        m_DMABuffers[i].buf = (uint8_t*) m_syncLine;

      ++invLine;

    }
  }

  fillDMABuffers();

  s_scanLine = 0;

  s_blankPatternDWord = m_HVSync | (m_HVSync << 8) | (m_HVSync << 16) | (m_HVSync << 24);

  if (s_fgbgPattern == nullptr) {
    s_fgbgPattern = (uint32_t*) heap_caps_malloc(16384, MALLOC_CAP_8BIT);
    for (int i = 0; i < 16; ++i)
      for (int fg = 0; fg < 16; ++fg)
        for (int bg = 0; bg < 16; ++bg) {
          uint8_t fg_pat = preparePixel(RGB222((Color)fg));
          uint8_t bg_pat = preparePixel(RGB222((Color)bg));
          s_fgbgPattern[i | (bg << 4) | (fg << 8)] = (i & 0b1000 ? (fg_pat << 16) : (bg_pat << 16)) |
                                                     (i & 0b0100 ? (fg_pat << 24) : (bg_pat << 24)) |
                                                     (i & 0b0010 ? (fg_pat << 0) : (bg_pat << 0)) |
                                                     (i & 0b0001 ? (fg_pat << 8) : (bg_pat << 8));
        }
  }


  // ESP_INTR_FLAG_LEVEL1: should be less than PS2Controller interrupt level, necessary when running on the same core
  CoreUsage::setBusiestCore(FABGLIB_VIDEO_CPUINTENSIVE_TASKS_CORE);
  esp_intr_alloc_pinnedToCore(ETS_I2S1_INTR_SOURCE, ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM, I2SInterrupt, this, &m_isr_handle, FABGLIB_VIDEO_CPUINTENSIVE_TASKS_CORE);

  m_GPIOStream.play(m_timings.frequency, m_DMABuffers);

  I2S1.int_clr.val     = 0xFFFFFFFF;
  I2S1.int_ena.out_eof = 1;
}


void freeBuffers()
{
  heap_caps_free( (void*) m_DMABuffers );
  heap_caps_free((void*) m_lines);
  m_DMABuffers = nullptr;
  heap_caps_free((void*) m_blankLine);
  heap_caps_free((void*) m_syncLine);
}

uint8_t IRAM_ATTR packHVSync(bool HSync, bool VSync)
{
  uint8_t hsync_value = (m_timings.HSyncLogic == '+' ? (HSync ? 1 : 0) : (HSync ? 0 : 1));
  uint8_t vsync_value = (m_timings.VSyncLogic == '+' ? (VSync ? 1 : 0) : (VSync ? 0 : 1));
  return (vsync_value << VGA_VSYNC_BIT) | (hsync_value << VGA_HSYNC_BIT);
}


uint8_t IRAM_ATTR preparePixelWithSync(RGB222 rgb, bool HSync, bool VSync)
{
  return packHVSync(HSync, VSync) | (rgb.B << VGA_BLUE_BIT) | (rgb.G << VGA_GREEN_BIT) | (rgb.R << VGA_RED_BIT);
}


void fillDMABuffers()
{
  int x = 0;
  for (; x < m_timings.HFrontPorch; ++x) {
    VGA_PIXELINROW(m_blankLine, x) = preparePixelWithSync((RGB222){0, 0, 0}, false, false);
    VGA_PIXELINROW(m_syncLine, x)  = preparePixelWithSync((RGB222){0, 0, 0}, false, true);
  }
  for (; x < m_timings.HFrontPorch + m_timings.HSyncPulse; ++x) {
    VGA_PIXELINROW(m_blankLine, x) = preparePixelWithSync((RGB222){0, 0, 0}, true, false);
    VGA_PIXELINROW(m_syncLine, x)  = preparePixelWithSync((RGB222){0, 0, 0}, true, true);
  }
  for (; x < m_timings.HFrontPorch + m_timings.HSyncPulse + m_timings.HBackPorch; ++x) {
    VGA_PIXELINROW(m_blankLine, x) = preparePixelWithSync((RGB222){0, 0, 0}, false, false);
    VGA_PIXELINROW(m_syncLine, x)  = preparePixelWithSync((RGB222){0, 0, 0}, false, true);
  }
  int rawLineWidth = m_timings.HFrontPorch + m_timings.HSyncPulse + m_timings.HBackPorch + m_timings.HVisibleArea;
  for (int rx = 0; x < rawLineWidth; ++x, ++rx) {
    VGA_PIXELINROW(m_blankLine, x) = preparePixelWithSync((RGB222){0, 0, 0}, false, false);
    VGA_PIXELINROW(m_syncLine, x)  = preparePixelWithSync((RGB222){0, 0, 0}, false, true);
    for (int i = 0; i < VGATextController_CHARHEIGHT; ++i)
      VGA_PIXELINROW( ((uint8_t*)(m_lines) + i * VGATextController_WIDTH), rx)  = preparePixelWithSync((RGB222){0, 0, 0}, false, false);
  }
}


void IRAM_ATTR I2SInterrupt(void * arg)
{
  #ifdef VGATextController_PERFORMANCE_CHECK
  auto s1 = getCycleCount();
  #endif

  VGATextController * ctrl = (VGATextController *) arg;

  if (I2S1.int_st.out_eof && ctrl->m_charData != nullptr) {

    auto desc = (volatile lldesc_t*) I2S1.out_eof_des_addr;

    if (desc == s_frameResetDesc) {

      s_scanLine = 0;
      s_textRow  = 0;
      s_upperRow = true;

      if (ctrl->m_cursorEnabled) {
        ++ctrl->m_cursorCounter;
        if (ctrl->m_cursorCounter >= ctrl->m_cursorSpeed)
          ctrl->m_cursorCounter = -ctrl->m_cursorSpeed;
      }

      if (ctrl->m_map == nullptr) {
        I2S1.int_clr.val = I2S1.int_st.val;
        #ifdef VGATextController_PERFORMANCE_CHECK
        s_cycles += getCycleCount() - s1;
        #endif
        return;
      }

    } else if (s_scanLine == 0) {
      // out of sync, wait for next frame
      I2S1.int_clr.val = I2S1.int_st.val;
      #ifdef VGATextController_PERFORMANCE_CHECK
      s_cycles += getCycleCount() - s1;
      #endif
      return;
    }

    int scanLine = s_scanLine;

    const int lineIndex = scanLine % VGATextController_CHARHEIGHT;

    auto lines = ctrl->m_lines;

    if (s_textRow < ctrl->m_rows) {

      int cursorCol = 0;
      int cursorFGBG = 0;
      const auto cursorVisible = (ctrl->m_cursorEnabled && ctrl->m_cursorCounter >= 0 && s_textRow == ctrl->m_cursorRow);
      if (cursorVisible) {
        cursorCol  = ctrl->m_cursorCol;
        cursorFGBG = (ctrl->m_cursorForeground << 4) | (ctrl->m_cursorBackground << 8);
      }

      const auto charData = ctrl->m_charData + (s_upperRow ? 0 : VGATextController_CHARHEIGHT / 2);
      auto mapItemPtr = ctrl->m_map + s_textRow * VGATextController_COLUMNS;

      for (int col = 0; col < VGATextController_COLUMNS; ++col, ++mapItemPtr) {

        const auto mapItem = *mapItemPtr;

        int fgbg = (mapItem >> 4) & 0b111111110000;

        const auto options = glyphMapItem_getOptions(mapItem);

        // invert?
        if (options.invert)
          fgbg = ((fgbg >> 4) & 0b11110000) | ((fgbg << 4) & 0b111100000000);

        // cursor?
        if (cursorVisible && col == cursorCol)
          fgbg = cursorFGBG;

        uint32_t * dest = lines + lineIndex * VGATextController_WIDTH / sizeof(uint32_t) + col * VGATextController_CHARWIDTHBYTES * 2;

        // blank?
        if (options.blank) {

          for (int rowInChar = 0; rowInChar < VGATextController_CHARHEIGHT / 2; ++rowInChar) {
            int v = s_fgbgPattern[fgbg];
            *dest       = v;
            *(dest + 1) = v;
            dest += VGATextController_WIDTH / sizeof(uint32_t);
          }

        } else {

          const bool underline = (s_upperRow == false && options.underline);
          const bool bold      = options.bold;

          auto charRowPtr = charData + glyphMapItem_getIndex(mapItem) * VGATextController_CHARHEIGHT * VGATextController_CHARWIDTHBYTES;

          for (int rowInChar = 0; rowInChar < VGATextController_CHARHEIGHT / 2; ++rowInChar) {
            auto charRowData = *charRowPtr;

            // bold?
            if (bold)
              charRowData |= charRowData >> 1;

            *dest       = s_fgbgPattern[(charRowData >> 4)  | fgbg];
            *(dest + 1) = s_fgbgPattern[(charRowData & 0xF) | fgbg];

            dest += VGATextController_WIDTH / sizeof(uint32_t);
            charRowPtr += VGATextController_CHARWIDTHBYTES;
          }

          // underline?
          if (underline) {
            dest -= VGATextController_WIDTH / sizeof(uint32_t);
            uint32_t v = s_fgbgPattern[0xF | fgbg];
            *dest       = v;
            *(dest + 1) = v;
          }

        }

      }

      if (s_upperRow) {
        s_upperRow = false;
      } else {
        s_upperRow = true;
        ++s_textRow;
      }

    } else {
      for (int i = 0; i < VGATextController_CHARHEIGHT / 2; ++i) {
        auto dest = lines + ((scanLine + i) % VGATextController_CHARHEIGHT) * VGATextController_WIDTH / sizeof(uint32_t);
        for (int i = 0; i < VGATextController_COLUMNS; ++i) {
          *dest++ = s_blankPatternDWord;
          *dest++ = s_blankPatternDWord;
        }
      }
    }

    scanLine += VGATextController_CHARHEIGHT / 2;

    s_scanLine = scanLine;

  }

  #ifdef VGATextController_PERFORMANCE_CHECK
  s_cycles += getCycleCount() - s1;
  #endif

  I2S1.int_clr.val = I2S1.int_st.val;
}



/*****************************************************************************************
******************************************************************************************************
* 
* 
* 
* */


#include <alloca.h>
#include <stdarg.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "soc/i2s_struct.h"
#include "soc/i2s_reg.h"
#include "driver/periph_ctrl.h"
#include "rom/lldesc.h"
#include "soc/rtc.h"

#include "fabutils.h"
#include "devdrivers/swgenerator.h"
#include "dispdrivers/vgabasecontroller.h"



namespace fabgl {




void VGABaseInit()
{
  m_DMABuffers                   = nullptr;
  m_DMABuffersCount              = 0;
  m_DMABuffersHead               = nullptr;
  m_DMABuffersVisible            = nullptr;
  m_primitiveProcessingSuspended = 1; // >0 suspended
  m_isr_handle                   = nullptr;
  m_doubleBufferOverDMA          = false;

  m_GPIOStream.begin();
}


// initializer for 8 colors configuration
void VGABaseBegin(gpio_num_t redGPIO, gpio_num_t greenGPIO, gpio_num_t blueGPIO, gpio_num_t HSyncGPIO, gpio_num_t VSyncGPIO)
{
  VGABaseInit();

  // GPIO configuration for bit 0
  setupGPIO(redGPIO,   VGA_RED_BIT,   GPIO_MODE_OUTPUT);
  setupGPIO(greenGPIO, VGA_GREEN_BIT, GPIO_MODE_OUTPUT);
  setupGPIO(blueGPIO,  VGA_BLUE_BIT,  GPIO_MODE_OUTPUT);

  // GPIO configuration for VSync and HSync
  setupGPIO(HSyncGPIO, VGA_HSYNC_BIT, GPIO_MODE_OUTPUT);
  setupGPIO(VSyncGPIO, VGA_VSYNC_BIT, GPIO_MODE_OUTPUT);

  //  RGB222::lowBitOnly = true;
  m_bitsPerChannel = 1;
}


// initializer for 64 colors configuration
void VGABaseBegin(gpio_num_t red1GPIO, gpio_num_t red0GPIO, gpio_num_t green1GPIO, gpio_num_t green0GPIO, gpio_num_t blue1GPIO, gpio_num_t blue0GPIO, gpio_num_t HSyncGPIO, gpio_num_t VSyncGPIO)
{
  begin(red0GPIO, green0GPIO, blue0GPIO, HSyncGPIO, VSyncGPIO);

  // GPIO configuration for bit 1
  setupGPIO(red1GPIO,   VGA_RED_BIT + 1,   GPIO_MODE_OUTPUT);
  setupGPIO(green1GPIO, VGA_GREEN_BIT + 1, GPIO_MODE_OUTPUT);
  setupGPIO(blue1GPIO,  VGA_BLUE_BIT + 1,  GPIO_MODE_OUTPUT);

  //  RGB222::lowBitOnly = false;
  m_bitsPerChannel = 2;
}


// initializer for default configuration
/*void VGABaseController::begin()
{
  begin(GPIO_NUM_22, GPIO_NUM_21, GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_5, GPIO_NUM_4, GPIO_NUM_23, GPIO_NUM_15);
}


void VGABaseController::end()
{
  if (m_DMABuffers) {
    if (m_isr_handle) {
      esp_intr_free(m_isr_handle);
      m_isr_handle = nullptr;
    }
    suspendBackgroundPrimitiveExecution();
    m_GPIOStream.stop();
    freeBuffers();
  }
}

*/
void setupGPIO(gpio_num_t gpio, int bit, gpio_mode_t mode)
{
  configureGPIO(gpio, mode);
  gpio_matrix_out(gpio, I2S1O_DATA_OUT0_IDX + bit, false, false);
}


/*
void VGABaseController::freeBuffers()
{
  if (m_DMABuffersCount > 0) {
    heap_caps_free((void*)m_HBlankLine_withVSync);
    heap_caps_free((void*)m_HBlankLine);

    freeViewPort();

    setDMABuffersCount(0);
  }
}


void VGABaseController::freeViewPort()
{
  for (uint8_t * * poolPtr = m_viewPortMemoryPool; *poolPtr; ++poolPtr) {
    heap_caps_free((void*) *poolPtr);
    *poolPtr = nullptr;
  }
  heap_caps_free(m_viewPort);
  m_viewPort = nullptr;
  if (isDoubleBuffered())
    heap_caps_free(m_viewPortVisible);
  m_viewPortVisible = nullptr;
}
*/

/*
// Can be used to change buffers count, maintainig already set pointers.
// If m_doubleBufferOverDMA = true, uses m_DMABuffersHead and m_DMABuffersVisible to implement
// double buffer on DMA level.
bool VGABaseController::setDMABuffersCount(int buffersCount)
{
  if (buffersCount == 0) {
    if (m_DMABuffersVisible && m_DMABuffersVisible != m_DMABuffers)
      heap_caps_free( (void*) m_DMABuffersVisible );
    heap_caps_free( (void*) m_DMABuffers );
    m_DMABuffers = nullptr;
    m_DMABuffersVisible = nullptr;
    m_DMABuffersCount = 0;
    return true;
  }

  if (buffersCount != m_DMABuffersCount) {

    // buffers head
    if (m_doubleBufferOverDMA && m_DMABuffersHead == nullptr) {
      m_DMABuffersHead = (lldesc_t*) heap_caps_malloc(sizeof(lldesc_t), MALLOC_CAP_DMA);
      m_DMABuffersHead->eof    = m_DMABuffersHead->sosf = m_DMABuffersHead->offset = 0;
      m_DMABuffersHead->owner  = 1;
      m_DMABuffersHead->size   = 0;
      m_DMABuffersHead->length = 0;
      m_DMABuffersHead->buf    = m_HBlankLine;   // dummy valid address. Setting nullptr crashes DMA!
      m_DMABuffersHead->qe.stqe_next = nullptr;  // this will be set before the first frame
    }

    // (re)allocate and initialize DMA descs
    m_DMABuffers = (lldesc_t*) heap_caps_realloc((void*)m_DMABuffers, buffersCount * sizeof(lldesc_t), MALLOC_CAP_DMA);
    if (m_doubleBufferOverDMA && isDoubleBuffered())
      m_DMABuffersVisible = (lldesc_t*) heap_caps_realloc((void*)m_DMABuffersVisible, buffersCount * sizeof(lldesc_t), MALLOC_CAP_DMA);
    else
      m_DMABuffersVisible = m_DMABuffers;
    if (!m_DMABuffers || !m_DMABuffersVisible)
      return false;

    auto buffersHead = m_DMABuffersHead ? m_DMABuffersHead : &m_DMABuffers[0];

    for (int i = 0; i < buffersCount; ++i) {
      m_DMABuffers[i].eof          = 0;
      m_DMABuffers[i].sosf         = 0;
      m_DMABuffers[i].offset       = 0;
      m_DMABuffers[i].owner        = 1;
      m_DMABuffers[i].qe.stqe_next = (lldesc_t*) (i == buffersCount - 1 ? buffersHead : &m_DMABuffers[i + 1]);
      if (m_doubleBufferOverDMA && isDoubleBuffered()) {
        m_DMABuffersVisible[i].eof          = 0;
        m_DMABuffersVisible[i].sosf         = 0;
        m_DMABuffersVisible[i].offset       = 0;
        m_DMABuffersVisible[i].owner        = 1;
        m_DMABuffersVisible[i].qe.stqe_next = (lldesc_t*) (i == buffersCount - 1 ? buffersHead : &m_DMABuffersVisible[i + 1]);
      }
    }

    m_DMABuffersCount = buffersCount;
  }

  return true;
}
*/

// modeline syntax:
//   "label" clock_mhz hdisp hsyncstart hsyncend htotal vdisp vsyncstart vsyncend vtotal (+HSync | -HSync) (+VSync | -VSync) [DoubleScan | QuadScan] [FrontPorchBegins | SyncBegins | BackPorchBegins | VisibleBegins] [MultiScanBlank]
bool convertModelineToTimings(char const * modeline, VGATimings * timings)
{
  float freq;
  int hdisp, hsyncstart, hsyncend, htotal, vdisp, vsyncstart, vsyncend, vtotal;
  char HSyncPol = 0, VSyncPol = 0;
  int pos = 0;

  int count = sscanf(modeline, "\"%[^\"]\" %g %d %d %d %d %d %d %d %d %n", timings->label, &freq, &hdisp, &hsyncstart, &hsyncend, &htotal, &vdisp, &vsyncstart, &vsyncend, &vtotal, &pos);

  if (count == 10 && pos > 0) {

    timings->frequency      = freq * 1000000;
    timings->HVisibleArea   = hdisp;
    timings->HFrontPorch    = hsyncstart - hdisp;
    timings->HSyncPulse     = hsyncend - hsyncstart;
    timings->HBackPorch     = htotal - hsyncend;
    timings->VVisibleArea   = vdisp;
    timings->VFrontPorch    = vsyncstart - vdisp;
    timings->VSyncPulse     = vsyncend - vsyncstart;
    timings->VBackPorch     = vtotal - vsyncend;
    timings->HSyncLogic     = '-';
    timings->VSyncLogic     = '-';
    timings->scanCount      = 1;
    timings->multiScanBlack = 0;
    timings->HStartingBlock = VGAScanStart::FrontPorch;

    // get (+HSync | -HSync) (+VSync | -VSync)
    char const * pc = modeline + pos;
    for (; *pc; ++pc) {
      if (*pc == '+' || *pc == '-') {
        if (!HSyncPol)
          timings->HSyncLogic = HSyncPol = *pc;
        else if (!VSyncPol) {
          timings->VSyncLogic = VSyncPol = *pc;
          while (*pc && *pc != ' ')
            ++pc;
          break;
        }
      }
    }

    // get [DoubleScan | QuadScan] [FrontPorchBegins | SyncBegins | BackPorchBegins | VisibleBegins] [MultiScanBlank]
    // actually this gets only the first character
    while (*pc) {
      switch (*pc) {
        case 'D':
        case 'd':
          timings->scanCount = 2;
          break;
        case 'Q':
        case 'q':
          timings->scanCount = 4;
          break;
        case 'F':
        case 'f':
          timings->HStartingBlock = VGAScanStart::FrontPorch;
          break;
        case 'S':
        case 's':
          timings->HStartingBlock = VGAScanStart::Sync;
          break;
        case 'B':
        case 'b':
          timings->HStartingBlock = VGAScanStart::BackPorch;
          break;
        case 'V':
        case 'v':
          timings->HStartingBlock = VGAScanStart::VisibleArea;
          break;
        case 'M':
        case 'm':
          timings->multiScanBlack = 1;
          break;
        case ' ':
          ++pc;
          continue;
      }
      ++pc;
      while (*pc && *pc != ' ')
        ++pc;
    }

    return true;

  }
  return false;
}


// Suspend vertical sync interrupt
// Warning: After call to suspendBackgroundPrimitiveExecution() adding primitives may cause a deadlock.
// To avoid this a call to "processPrimitives()" should be performed very often.
// Can be nested
/*


void VGABaseController::startGPIOStream()
{
  m_GPIOStream.play(m_timings.frequency, m_DMABuffers);
}


void VGABaseController::setResolution(char const * modeline, int viewPortWidth, int viewPortHeight, bool doubleBuffered)
{
  VGATimings timings;
  if (convertModelineToTimings(modeline, &timings))
    setResolution(timings, viewPortWidth, viewPortHeight, doubleBuffered);
}


void VGABaseController::setResolution(VGATimings const& timings, int viewPortWidth, int viewPortHeight, bool doubleBuffered)
{
  end();

  m_timings = timings;
  setDoubleBuffered(doubleBuffered);

  m_HVSync = packHVSync(false, false);

  m_HLineSize = m_timings.HFrontPorch + m_timings.HSyncPulse + m_timings.HBackPorch + m_timings.HVisibleArea;

  m_HBlankLine_withVSync = (uint8_t*) heap_caps_malloc(m_HLineSize, MALLOC_CAP_DMA);
  m_HBlankLine           = (uint8_t*) heap_caps_malloc(m_HLineSize, MALLOC_CAP_DMA);

  m_viewPortWidth  = ~3 & (viewPortWidth <= 0 || viewPortWidth >= m_timings.HVisibleArea ? m_timings.HVisibleArea : viewPortWidth); // view port width must be 32 bit aligned
  m_viewPortHeight = viewPortHeight <= 0 || viewPortHeight >= m_timings.VVisibleArea ? m_timings.VVisibleArea : viewPortHeight;

  // need to center viewport?
  m_viewPortCol = (m_timings.HVisibleArea - m_viewPortWidth) / 2;
  m_viewPortRow = (m_timings.VVisibleArea - m_viewPortHeight) / 2;

  // view port col and row must be 32 bit aligned
  m_viewPortCol = m_viewPortCol & ~3;
  m_viewPortRow = m_viewPortRow & ~3;

  m_rawFrameHeight = m_timings.VVisibleArea + m_timings.VFrontPorch + m_timings.VSyncPulse + m_timings.VBackPorch;

  // allocate DMA descriptors
  setDMABuffersCount(calcRequiredDMABuffersCount(m_viewPortHeight));

  // allocate the viewport
  allocateViewPort();

  // this may free space if m_viewPortHeight has been reduced
  setDMABuffersCount(calcRequiredDMABuffersCount(m_viewPortHeight));

  // fill buffers
  fillVertBuffers(0);
  fillHorizBuffers(0);

  resetPaintState();

  if (m_doubleBufferOverDMA)
    m_DMABuffersHead->qe.stqe_next = (lldesc_t*) &m_DMABuffersVisible[0];
}


// this method may adjust m_viewPortHeight to the actual number of allocated rows.
// to reduce memory allocation overhead try to allocate the minimum number of blocks.
void VGABaseController::allocateViewPort(uint32_t allocCaps, int rowlen)
{
  int linesCount[FABGLIB_VIEWPORT_MEMORY_POOL_COUNT]; // where store number of lines for each pool
  int poolsCount = 0; // number of allocated pools
  int remainingLines = m_viewPortHeight;
  m_viewPortHeight = 0; // m_viewPortHeight needs to be recalculated

  if (isDoubleBuffered())
    remainingLines *= 2;

  // allocate pools
  while (remainingLines > 0 && poolsCount < FABGLIB_VIEWPORT_MEMORY_POOL_COUNT) {
    int largestBlock = heap_caps_get_largest_free_block(allocCaps);
    linesCount[poolsCount] = tmin(remainingLines, largestBlock / rowlen);
    if (linesCount[poolsCount] == 0)  // no more memory available for lines
      break;
    m_viewPortMemoryPool[poolsCount] = (uint8_t*) heap_caps_malloc(linesCount[poolsCount] * rowlen, allocCaps);
    remainingLines -= linesCount[poolsCount];
    m_viewPortHeight += linesCount[poolsCount];
    ++poolsCount;
  }
  m_viewPortMemoryPool[poolsCount] = nullptr;

  // fill m_viewPort[] with line pointers
  if (isDoubleBuffered()) {
    m_viewPortHeight /= 2;
    m_viewPortVisible = (volatile uint8_t * *) heap_caps_malloc(sizeof(uint8_t*) * m_viewPortHeight, MALLOC_CAP_32BIT);
  }
  m_viewPort = (volatile uint8_t * *) heap_caps_malloc(sizeof(uint8_t*) * m_viewPortHeight, MALLOC_CAP_32BIT);
  if (!isDoubleBuffered())
    m_viewPortVisible = m_viewPort;
  for (int p = 0, l = 0; p < poolsCount; ++p) {
    uint8_t * pool = m_viewPortMemoryPool[p];
    for (int i = 0; i < linesCount[p]; ++i) {
      if (l + i < m_viewPortHeight)
        m_viewPort[l + i] = pool;
      else
        m_viewPortVisible[l + i - m_viewPortHeight] = pool; // set only when double buffered is enabled
      pool += rowlen;
    }
    l += linesCount[p];
  }
}
*/

uint8_t IRAM_ATTR VGABaseController::packHVSync(bool HSync, bool VSync)
{
  uint8_t hsync_value = (m_timings.HSyncLogic == '+' ? (HSync ? 1 : 0) : (HSync ? 0 : 1));
  uint8_t vsync_value = (m_timings.VSyncLogic == '+' ? (VSync ? 1 : 0) : (VSync ? 0 : 1));
  return (vsync_value << VGA_VSYNC_BIT) | (hsync_value << VGA_HSYNC_BIT);
}


/*uint8_t IRAM_ATTR VGABaseController::preparePixelWithSync(RGB222 rgb, bool HSync, bool VSync)
{
  return packHVSync(HSync, VSync) | (rgb.B << VGA_BLUE_BIT) | (rgb.G << VGA_GREEN_BIT) | (rgb.R << VGA_RED_BIT);
}
*/

int calcRequiredDMABuffersCount(int viewPortHeight)
{
  int rightPadSize = m_timings.HVisibleArea - m_viewPortWidth - m_viewPortCol;
  int buffersCount = m_timings.scanCount * (m_rawFrameHeight + viewPortHeight);

  switch (m_timings.HStartingBlock) {
    case VGAScanStart::FrontPorch:
      // FRONTPORCH -> SYNC -> BACKPORCH -> VISIBLEAREA
      buffersCount += m_timings.scanCount * (rightPadSize > 0 ? viewPortHeight : 0);
      break;
    case VGAScanStart::Sync:
      // SYNC -> BACKPORCH -> VISIBLEAREA -> FRONTPORCH
      buffersCount += m_timings.scanCount * viewPortHeight;
      break;
    case VGAScanStart::BackPorch:
      // BACKPORCH -> VISIBLEAREA -> FRONTPORCH -> SYNC
      buffersCount += m_timings.scanCount * viewPortHeight;
      break;
    case VGAScanStart::VisibleArea:
      // VISIBLEAREA -> FRONTPORCH -> SYNC -> BACKPORCH
      buffersCount += m_timings.scanCount * (m_viewPortCol > 0 ? viewPortHeight : 0);
      break;
  }

  return buffersCount;
}


// refill buffers changing Front Porch and Back Porch
// offsetX : (< 0 : left  > 0 right)
void fillHorizBuffers(int offsetX)
{
  // fill all with no hsync

  fill(m_HBlankLine,           0, m_HLineSize, 0, 0, 0, false, false);
  fill(m_HBlankLine_withVSync, 0, m_HLineSize, 0, 0, 0, false,  true);

  // calculate hsync pos and fill it

  int16_t porchSum = m_timings.HFrontPorch + m_timings.HBackPorch;
  m_timings.HFrontPorch = tmax(8, (int16_t)m_timings.HFrontPorch - offsetX);
  m_timings.HBackPorch  = tmax(8, porchSum - m_timings.HFrontPorch);
  m_timings.HFrontPorch = porchSum - m_timings.HBackPorch;

  int syncPos = 0;

  switch (m_timings.HStartingBlock) {
    case VGAScanStart::FrontPorch:
      // FRONTPORCH -> SYNC -> BACKPORCH -> VISIBLEAREA
      syncPos = m_timings.HFrontPorch;
      break;
    case VGAScanStart::Sync:
      // SYNC -> BACKPORCH -> VISIBLEAREA -> FRONTPORCH
      syncPos = 0;
      break;
    case VGAScanStart::BackPorch:
      // BACKPORCH -> VISIBLEAREA -> FRONTPORCH -> SYNC
      syncPos = m_timings.HBackPorch + m_timings.HVisibleArea + m_timings.HFrontPorch;
      break;
    case VGAScanStart::VisibleArea:
      // VISIBLEAREA -> FRONTPORCH -> SYNC -> BACKPORCH
      syncPos = m_timings.HVisibleArea + m_timings.HFrontPorch;
      break;
  }

  fill(m_HBlankLine,           syncPos, m_timings.HSyncPulse, 0, 0, 0, true,  false);
  fill(m_HBlankLine_withVSync, syncPos, m_timings.HSyncPulse, 0, 0, 0, true,   true);
}


void fillVertBuffers(int offsetY)
{
  int16_t porchSum = m_timings.VFrontPorch + m_timings.VBackPorch;
  m_timings.VFrontPorch = tmax(1, (int16_t)m_timings.VFrontPorch - offsetY);
  m_timings.VBackPorch  = tmax(1, porchSum - m_timings.VFrontPorch);
  m_timings.VFrontPorch = porchSum - m_timings.VBackPorch;

  // associate buffers pointer to DMA info buffers
  //
  //  Vertical order:
  //    VisibleArea
  //    Front Porch
  //    Sync
  //    Back Porch

  int VVisibleArea_pos = 0;
  int VFrontPorch_pos  = VVisibleArea_pos + m_timings.VVisibleArea;
  int VSync_pos        = VFrontPorch_pos  + m_timings.VFrontPorch;
  int VBackPorch_pos   = VSync_pos        + m_timings.VSyncPulse;

  int rightPadSize = m_timings.HVisibleArea - m_viewPortWidth - m_viewPortCol;

  for (int line = 0, DMABufIdx = 0; line < m_rawFrameHeight; ++line) {

    bool isVVisibleArea = (line < VFrontPorch_pos);
    bool isVFrontPorch  = (line >= VFrontPorch_pos && line < VSync_pos);
    bool isVSync        = (line >= VSync_pos && line < VBackPorch_pos);
    bool isVBackPorch   = (line >= VBackPorch_pos);

    for (int scan = 0; scan < m_timings.scanCount; ++scan) {

      bool isStartOfVertFrontPorch = (line == VFrontPorch_pos && scan == 0);

      if (isVSync) {

        setDMABufferBlank(DMABufIdx++, m_HBlankLine_withVSync, m_HLineSize, scan, isStartOfVertFrontPorch);

      } else if (isVFrontPorch || isVBackPorch) {

        setDMABufferBlank(DMABufIdx++, m_HBlankLine, m_HLineSize, scan, isStartOfVertFrontPorch);

      } else if (isVVisibleArea) {

        int visibleAreaLine = line - VVisibleArea_pos;
        bool isViewport = visibleAreaLine >= m_viewPortRow && visibleAreaLine < m_viewPortRow + m_viewPortHeight;
        int HInvisibleAreaSize = m_HLineSize - m_timings.HVisibleArea;

        if (isViewport) {
          // visible, this is the viewport
          switch (m_timings.HStartingBlock) {
            case VGAScanStart::FrontPorch:
              // FRONTPORCH -> SYNC -> BACKPORCH -> VISIBLEAREA
              setDMABufferBlank(DMABufIdx++, m_HBlankLine, HInvisibleAreaSize + m_viewPortCol, scan, isStartOfVertFrontPorch);
              setDMABufferView(DMABufIdx++, visibleAreaLine - m_viewPortRow, scan, isStartOfVertFrontPorch);
              if (rightPadSize > 0)
                setDMABufferBlank(DMABufIdx++, m_HBlankLine + HInvisibleAreaSize, rightPadSize, scan, isStartOfVertFrontPorch);
              break;
            case VGAScanStart::Sync:
              // SYNC -> BACKPORCH -> VISIBLEAREA -> FRONTPORCH
              setDMABufferBlank(DMABufIdx++, m_HBlankLine, m_timings.HSyncPulse + m_timings.HBackPorch + m_viewPortCol, scan, isStartOfVertFrontPorch);
              setDMABufferView(DMABufIdx++, visibleAreaLine - m_viewPortRow, scan, isStartOfVertFrontPorch);
              setDMABufferBlank(DMABufIdx++, m_HBlankLine + m_HLineSize - m_timings.HFrontPorch - rightPadSize, m_timings.HFrontPorch + rightPadSize, scan, isStartOfVertFrontPorch);
              break;
            case VGAScanStart::BackPorch:
              // BACKPORCH -> VISIBLEAREA -> FRONTPORCH -> SYNC
              setDMABufferBlank(DMABufIdx++, m_HBlankLine, m_timings.HBackPorch + m_viewPortCol, scan, isStartOfVertFrontPorch);
              setDMABufferView(DMABufIdx++, visibleAreaLine - m_viewPortRow, scan, isStartOfVertFrontPorch);
              setDMABufferBlank(DMABufIdx++, m_HBlankLine + m_HLineSize - m_timings.HFrontPorch - m_timings.HSyncPulse - rightPadSize, m_timings.HFrontPorch + m_timings.HSyncPulse + rightPadSize, scan, isStartOfVertFrontPorch);
              break;
            case VGAScanStart::VisibleArea:
              // VISIBLEAREA -> FRONTPORCH -> SYNC -> BACKPORCH
              if (m_viewPortCol > 0)
                setDMABufferBlank(DMABufIdx++, m_HBlankLine, m_viewPortCol, scan, isStartOfVertFrontPorch);
              setDMABufferView(DMABufIdx++, visibleAreaLine - m_viewPortRow, scan, isStartOfVertFrontPorch);
              setDMABufferBlank(DMABufIdx++, m_HBlankLine + m_timings.HVisibleArea - rightPadSize, HInvisibleAreaSize + rightPadSize, scan, isStartOfVertFrontPorch);
              break;
          }

        } else {
          // not visible
          setDMABufferBlank(DMABufIdx++, m_HBlankLine, m_HLineSize, scan, isStartOfVertFrontPorch);
        }

      }

    }

  }
}


// address must be allocated with MALLOC_CAP_DMA or even an address of another already allocated buffer
// allocated buffer length (in bytes) must be 32 bit aligned
// Max length is 4092 bytes
void VGABaseController::setDMABufferBlank(int index, void volatile * address, int length, int scan, bool isStartOfVertFrontPorch)
{
  int size = (length + 3) & (~3);
  m_DMABuffers[index].eof    = 0;
  m_DMABuffers[index].size   = size;
  m_DMABuffers[index].length = length;
  m_DMABuffers[index].buf    = (uint8_t*) address;
  onSetupDMABuffer(&m_DMABuffers[index], isStartOfVertFrontPorch, scan, false, 0);
  if (m_doubleBufferOverDMA && isDoubleBuffered()) {
    m_DMABuffersVisible[index].eof    = 0;
    m_DMABuffersVisible[index].size   = size;
    m_DMABuffersVisible[index].length = length;
    m_DMABuffersVisible[index].buf    = (uint8_t*) address;
    onSetupDMABuffer(&m_DMABuffersVisible[index], isStartOfVertFrontPorch, scan, false, 0);
  }
}


bool VGABaseController::isMultiScanBlackLine(int scan)
{
  return (scan > 0 && m_timings.multiScanBlack == 1 && m_timings.HStartingBlock == FrontPorch);
}


// address must be allocated with MALLOC_CAP_DMA or even an address of another already allocated buffer
// allocated buffer length (in bytes) must be 32 bit aligned
// Max length is 4092 bytes
void VGABaseController::setDMABufferView(int index, int row, int scan, volatile uint8_t * * viewPort, bool onVisibleDMA)
{
  uint8_t * bufferPtr = nullptr;
  if (isMultiScanBlackLine(scan))
    bufferPtr = (uint8_t *) (m_HBlankLine + m_HLineSize - m_timings.HVisibleArea);  // this works only when HSYNC, FrontPorch and BackPorch are at the beginning of m_HBlankLine
  else if (viewPort)
    bufferPtr = (uint8_t *) viewPort[row];
  lldesc_t volatile * DMABuffers = onVisibleDMA ? m_DMABuffersVisible : m_DMABuffers;
  DMABuffers[index].size   = (m_viewPortWidth + 3) & (~3);
  DMABuffers[index].length = m_viewPortWidth;
  DMABuffers[index].buf    = bufferPtr;
}


// address must be allocated with MALLOC_CAP_DMA or even an address of another already allocated buffer
// allocated buffer length (in bytes) must be 32 bit aligned
// Max length is 4092 bytes
void VGABaseController::setDMABufferView(int index, int row, int scan, bool isStartOfVertFrontPorch)
{
  setDMABufferView(index, row, scan, m_viewPort, false);
  if (!isMultiScanBlackLine(scan))
    onSetupDMABuffer(&m_DMABuffers[index], isStartOfVertFrontPorch, scan, true, row);
  if (isDoubleBuffered()) {
    setDMABufferView(index, row, scan, m_viewPortVisible, true);
    if (!isMultiScanBlackLine(scan))
      onSetupDMABuffer(&m_DMABuffersVisible[index], isStartOfVertFrontPorch, scan, true, row);
  }
}


void volatile * VGABaseController::getDMABuffer(int index, int * length)
{
  *length = m_DMABuffers[index].length;
  return m_DMABuffers[index].buf;
}


// buffer: buffer to fill (buffer size must be 32 bit aligned)
// startPos: initial position (in pixels)
// length: number of pixels to fill
//
// Returns next pos to fill (startPos + length)
int VGABaseController::fill(uint8_t volatile * buffer, int startPos, int length, uint8_t red, uint8_t green, uint8_t blue, bool HSync, bool VSync)
{
  uint8_t pattern = preparePixelWithSync((RGB222){red, green, blue}, HSync, VSync);
  for (int i = 0; i < length; ++i, ++startPos)
    VGA_PIXELINROW(buffer, startPos) = pattern;
  return startPos;
}


void VGABaseController::moveScreen(int offsetX, int offsetY)
{
  suspendBackgroundPrimitiveExecution();
  fillVertBuffers(offsetY);
  fillHorizBuffers(offsetX);
  resumeBackgroundPrimitiveExecution();
}


void VGABaseController::shrinkScreen(int shrinkX, int shrinkY)
{
  VGATimings * currTimings = getResolutionTimings();

  currTimings->HBackPorch  = tmax(currTimings->HBackPorch + 4 * shrinkX, 4);
  currTimings->HFrontPorch = tmax(currTimings->HFrontPorch + 4 * shrinkX, 4);

  currTimings->VBackPorch  = tmax(currTimings->VBackPorch + shrinkY, 1);
  currTimings->VFrontPorch = tmax(currTimings->VFrontPorch + shrinkY, 1);

  setResolution(*currTimings, m_viewPortWidth, m_viewPortHeight, isDoubleBuffered());
}


void IRAM_ATTR VGABaseController::swapBuffers()
{
  tswap(m_viewPort, m_viewPortVisible);
  if (m_doubleBufferOverDMA) {
    tswap(m_DMABuffers, m_DMABuffersVisible);
    m_DMABuffersHead->qe.stqe_next = (lldesc_t*) &m_DMABuffersVisible[0];
  }
}




} // end of namespace




} // end of namespace

  bool                m_DMAStarted;
  volatile lldesc_t * m_DMABuffer;
  volatile uint8_t *  m_DMAData;

  
  
  
  
