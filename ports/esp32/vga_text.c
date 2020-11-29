#include <stdint.h>
#include <stddef.h>

#include "rom/lldesc.h"
#include "driver/gpio.h"

#include "fabglconf.h"
#include "fabutils.h"
#include "devdrivers/swgenerator.h"
#include "dispdrivers/vgacontroller.h"
#include "fonts/font_8x14.h"

#include "soc/i2s_struct.h"
#include "soc/i2s_reg.h"
#include "driver/periph_ctrl.h"
#include "rom/lldesc.h"
#include "soc/rtc.h"

#include "fabutils.h"
#include "vgatextcontroller.h"
#include "devdrivers/swgenerator.h"



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


// pixel 0 = byte 2, pixel 1 = byte 3, pixel 2 = byte 0, pixel 3 = byte 1 :
// pixel : 0  1  2  3  4  5  6  7  8  9 10 11 ...etc...
// byte  : 2  3  0  1  6  7  4  5 10 11  8  9 ...etc...
// dword : 0           1           2          ...etc...
// Thanks to https://github.com/paulscottrobson for the new macro. Before was: (row[((X) & 0xFFFC) + ((2 + (X)) & 3)])
#define VGA_PIXELINROW(row, X) (row[(X) ^ 2])

// requires variables: m_viewPort
#define VGA_PIXEL(X, Y)    VGA_PIXELINROW(m_viewPort[(Y)], X)
#define VGA_INVERT_PIXEL(X, Y) { auto px = &VGA_PIXEL((X), (Y)); *px = ~(*px ^ VGA_SYNC_MASK); }

enum VGAScanStart {
  FrontPorch,   /**< Horizontal line sequence is: FRONTPORCH -> SYNC -> BACKPORCH -> VISIBLEAREA */
  Sync,         /**< Horizontal line sequence is: SYNC -> BACKPORCH -> VISIBLEAREA -> FRONTPORCH */
  BackPorch,    /**< Horizontal line sequence is: BACKPORCH -> VISIBLEAREA -> FRONTPORCH -> SYNC */
  VisibleArea   /**< Horizontal line sequence is: VISIBLEAREA -> FRONTPORCH -> SYNC -> BACKPORCH */
};


/** @brief Specifies the VGA timings. This is a modeline decoded. */
struct VGATimings {
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
};
   


//#define VGATextController_PERFORMANCE_CHECK




/**
* @brief Represents the VGA text-only controller
*
* The text only VGA controller allows only text, but requires less than 50K of RAM.
* Resolution is fixed at 640x480, with 80 columns by 34 rows, 16 colors.
*
* Text only output is very CPU intensive process and consumes up to 30% of one CPU core. Anyway this allows to have
* more than 290K free for your application.
*
* Graphics (Canvas) aren't possible. Also, some character styles aren't also possible (double size, 132 columns, italic).
*
*
* This example initializes VGA Text Controller with 64 colors (16 usable):
*
*     fabgl::VGATextController VGAController;
*     // the default assigns GPIO22 and GPIO21 to Red, GPIO19 and GPIO18 to Green, GPIO5 and GPIO4 to Blue, GPIO23 to HSync and GPIO15 to VSync
*     VGAController.begin();
*     VGAController.setResolution();
*/




  VGATextController();


  /**
   * @brief This is the 8 colors (5 GPIOs) initializer.
   *
   * One GPIO per channel, plus horizontal and vertical sync signals.
   *
   * @param redGPIO GPIO to use for red channel.
   * @param greenGPIO GPIO to use for green channel.
   * @param blueGPIO GPIO to use for blue channel.
   * @param HSyncGPIO GPIO to use for horizontal sync signal.
   * @param VSyncGPIO GPIO to use for vertical sync signal.
   *
   * Example:
   *
   *     // Use GPIO 22 for red, GPIO 21 for green, GPIO 19 for blue, GPIO 18 for HSync and GPIO 5 for VSync
   *     VGAController.begin(GPIO_NUM_22, GPIO_NUM_21, GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_5);
   */
  void begin(gpio_num_t redGPIO, gpio_num_t greenGPIO, gpio_num_t blueGPIO, gpio_num_t HSyncGPIO, gpio_num_t VSyncGPIO);

  /**
   * @brief This is the 64 colors (8 GPIOs) initializer.
   *
   * Two GPIOs per channel, plus horizontal and vertical sync signals.
   *
   * @param red1GPIO GPIO to use for red channel, MSB bit.
   * @param red0GPIO GPIO to use for red channel, LSB bit.
   * @param green1GPIO GPIO to use for green channel, MSB bit.
   * @param green0GPIO GPIO to use for green channel, LSB bit.
   * @param blue1GPIO GPIO to use for blue channel, MSB bit.
   * @param blue0GPIO GPIO to use for blue channel, LSB bit.
   * @param HSyncGPIO GPIO to use for horizontal sync signal.
   * @param VSyncGPIO GPIO to use for vertical sync signal.
   *
   * Example:
   *
   *     // Use GPIO 22-21 for red, GPIO 19-18 for green, GPIO 5-4 for blue, GPIO 23 for HSync and GPIO 15 for VSync
   *     VGAController.begin(GPIO_NUM_22, GPIO_NUM_21, GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_5, GPIO_NUM_4, GPIO_NUM_23, GPIO_NUM_15);
   */
  void begin(gpio_num_t red1GPIO, gpio_num_t red0GPIO, gpio_num_t green1GPIO, gpio_num_t green0GPIO, gpio_num_t blue1GPIO, gpio_num_t blue0GPIO, gpio_num_t HSyncGPIO, gpio_num_t VSyncGPIO);

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

  /**
   * @brief Sets fixed resolution
   *
   * This call is required, even you cannot set or change resolution.
   */
  void setResolution(char const * modeline = nullptr, int viewPortWidth = -1, int viewPortHeight = -1, bool doubleBuffered = false);

  /**
   * @brief Sets text map to display
   *
   * This is set automatically by the terminal.
   */
  void setTextMap(uint32_t const * map, int rows);

  /**
   * @brief Adjust columns and rows to the controller limits
   *
   * @param columns If > 0 then it is set to 80.
   * @param rows If > 0 then it is limited to 1..34 range.
   */
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




  void setResolution(VGATimings const& timings);
  void init(gpio_num_t VSyncGPIO);
  void setupGPIO(gpio_num_t gpio, int bit, gpio_mode_t mode);
  void freeBuffers();

  void fillDMABuffers();
  uint8_t packHVSync(bool HSync, bool VSync);
  uint8_t preparePixelWithSync(RGB222 rgb, bool HSync, bool VSync);

  uint8_t IRAM_ATTR preparePixel(RGB222 rgb) { return m_HVSync | (rgb.B << VGA_BLUE_BIT) | (rgb.G << VGA_GREEN_BIT) | (rgb.R << VGA_RED_BIT); }

  static void I2SInterrupt(void * arg);


  static volatile int        s_scanLine;
  static uint32_t            s_blankPatternDWord;
  static uint32_t *          s_fgbgPattern;
  static int                 s_textRow;
  static bool                s_upperRow;
  static lldesc_t volatile * s_frameResetDesc;

  VGATimings             m_timings;

  GPIOStream             m_GPIOStream;
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

  // cursor propsvolatile int        VGATextController::s_scanLine;
uint32_t            VGATextController::s_blankPatternDWord;
uint32_t *          VGATextController::s_fgbgPattern = nullptr;
int                 VGATextController::s_textRow;
bool                VGATextController::s_upperRow;
lldesc_t volatile * VGATextController::s_frameResetDesc;


//start of C file.

//default init
VGATextController() {
   m_charData = NULL;
   m_map = NULL;
   m_cursorEnabled = 0;
   m_cursorCounter = 0;
   m_cursorSpeed = 20;
   m_cursorRow = 0;
   m_cursorCol = 0;
   m_cursorForeground = 0;
   m_cursorBackground = 15;

}


end_VGATextController()
{
  free((void*)m_charData);
}


void setTextMap(uint32_t const * map, int rows)
{
  // wait for the end of frame
  while (m_map != nullptr && s_scanLine < m_timings.VVisibleArea)
    ;
  m_rows = rows;
  m_map  = map;
}


void adjustMapSize(int * columns, int * rows)
{
  if (*columns > 0)
    *columns = VGATextController_COLUMNS;
  if (*rows > VGATextController_ROWS)
    *rows = VGATextController_ROWS;

}


void VGATextController::init(gpio_num_t VSyncGPIO)
{
  m_DMABuffers = nullptr;

  m_GPIOStream.begin();

  // load font into RAM
  int charDataSize = 256 * FONT_8x14.height * ((FONT_8x14.width + 7) / 8);
  m_charData = (uint8_t*) heap_caps_malloc(charDataSize, MALLOC_CAP_8BIT);
  memcpy(m_charData, FONT_8x14.data, charDataSize);
}


// initializer for 8 colors configuration
void VGATextController::begin(gpio_num_t redGPIO, gpio_num_t greenGPIO, gpio_num_t blueGPIO, gpio_num_t HSyncGPIO, gpio_num_t VSyncGPIO)
{
  init(VSyncGPIO);

  // GPIO configuration for bit 0
  setupGPIO(redGPIO,   VGA_RED_BIT,   GPIO_MODE_OUTPUT);
  setupGPIO(greenGPIO, VGA_GREEN_BIT, GPIO_MODE_OUTPUT);
  setupGPIO(blueGPIO,  VGA_BLUE_BIT,  GPIO_MODE_OUTPUT);

  // GPIO configuration for VSync and HSync
  setupGPIO(HSyncGPIO, VGA_HSYNC_BIT, GPIO_MODE_OUTPUT);
  setupGPIO(VSyncGPIO, VGA_VSYNC_BIT, GPIO_MODE_INPUT_OUTPUT);  // input/output so can be generated interrupt on falling/rising edge

  RGB222::lowBitOnly = true;
  m_bitsPerChannel = 1;
}


// initializer for 64 colors configuration
void VGATextController::begin(gpio_num_t red1GPIO, gpio_num_t red0GPIO, gpio_num_t green1GPIO, gpio_num_t green0GPIO, gpio_num_t blue1GPIO, gpio_num_t blue0GPIO, gpio_num_t HSyncGPIO, gpio_num_t VSyncGPIO)
{
  begin(red0GPIO, green0GPIO, blue0GPIO, HSyncGPIO, VSyncGPIO);

  // GPIO configuration for bit 1
  setupGPIO(red1GPIO,   VGA_RED_BIT + 1,   GPIO_MODE_OUTPUT);
  setupGPIO(green1GPIO, VGA_GREEN_BIT + 1, GPIO_MODE_OUTPUT);
  setupGPIO(blue1GPIO,  VGA_BLUE_BIT + 1,  GPIO_MODE_OUTPUT);

  RGB222::lowBitOnly = false;
  m_bitsPerChannel = 2;
}


// initializer for default configuration
void VGATextController::begin()
{
  begin(GPIO_NUM_22, GPIO_NUM_21, GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_5, GPIO_NUM_4, GPIO_NUM_23, GPIO_NUM_15);
}


void VGATextController::setCursorForeground(Color value)
{
  m_cursorForeground = (int) value;
}


void VGATextController::setCursorBackground(Color value)
{
  m_cursorBackground = (int) value;
}


void VGATextController::setupGPIO(gpio_num_t gpio, int bit, gpio_mode_t mode)
{
  configureGPIO(gpio, mode);
  gpio_matrix_out(gpio, I2S1O_DATA_OUT0_IDX + bit, false, false);
}


void VGATextController::setResolution(char const * modeline, int viewPortWidth, int viewPortHeight, bool doubleBuffered)
{
  VGATimings timings;
  if (VGABaseController::convertModelineToTimings(VGATextController_MODELINE, &timings))
    setResolution(timings);
}


void VGATextController::setResolution(VGATimings const& timings)
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


void VGATextController::freeBuffers()
{
  heap_caps_free( (void*) m_DMABuffers );
  heap_caps_free((void*) m_lines);
  m_DMABuffers = nullptr;
  heap_caps_free((void*) m_blankLine);
  heap_caps_free((void*) m_syncLine);
}


uint8_t IRAM_ATTR VGATextController::packHVSync(bool HSync, bool VSync)
{
  uint8_t hsync_value = (m_timings.HSyncLogic == '+' ? (HSync ? 1 : 0) : (HSync ? 0 : 1));
  uint8_t vsync_value = (m_timings.VSyncLogic == '+' ? (VSync ? 1 : 0) : (VSync ? 0 : 1));
  return (vsync_value << VGA_VSYNC_BIT) | (hsync_value << VGA_HSYNC_BIT);
}


uint8_t IRAM_ATTR VGATextController::preparePixelWithSync(RGB222 rgb, bool HSync, bool VSync)
{
  return packHVSync(HSync, VSync) | (rgb.B << VGA_BLUE_BIT) | (rgb.G << VGA_GREEN_BIT) | (rgb.R << VGA_RED_BIT);
}


void VGATextController::fillDMABuffers()
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


void IRAM_ATTR VGATextController::I2SInterrupt(void * arg)
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

  bool                   m_cursorEnabled;
  int                    m_cursorCounter; // trip from -m_cursorSpeed to +m_cursorSpeed (>= cursor is visible)
  int                    m_cursorSpeed;
  int                    m_cursorRow;
  int                    m_cursorCol;
  uint8_t                m_cursorForeground;
  uint8_t                m_cursorBackground;






