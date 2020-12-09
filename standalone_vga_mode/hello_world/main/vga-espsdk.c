//This code is entirely derived from FABGL
//http://www.fabglib.org/
//Full Credit to Fabrizio Di Vittorio
//I never would have attempted to understand something like
//this without
//such high quality and re-usable code.
//Assume that all code in this module is licensed GPLv3


//requires latest SDK for proper operation. 
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "rom/lldesc.h"
#include "esp_heap_caps.h"
#include "driver/i2s.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "soc/i2s_struct.h"
#include "soc/i2s_reg.h"
#include "driver/periph_ctrl.h"
#include "font_8x14.h"
#include <string.h>
#define BACKCORE 0

#include "configure_i2s.h"

//#include <stdint.h>

#include "freertos/queue.h"
#define nullptr  NULL


uint32_t counter;

typedef uint8_t Color;

typedef struct  ESP_intr_alloc_args {
  int             source;
  int             flags;
  intr_handler_t  handler;
  void *          arg;
  intr_handle_t * ret_handle;
  TaskHandle_t    waitingTask;
} esp_intr_alloc_args;

typedef struct RGBT {
  uint8_t R: 2;
  uint8_t G: 2;
  uint8_t B: 2;
} RGB222;
    

//necessary for the underline, bold, blank attributes
/*
#define GLYPHMAP_INDEX_BIT    0
#define GLYPHMAP_BGCOLOR_BIT  8
#define GLYPHMAP_FGCOLOR_BIT 12
#define GLYPHMAP_OPTIONS_BIT 16
#define GLYPHMAP_ITEM_MAKE(index, bgColor, fgColor, options) (((uint32_t)(index) << GLYPHMAP_INDEX_BIT) | ((uint32_t)(bgColor) << GLYPHMAP_BGCOLOR_BIT) | ((uint32_t)(fgColor) << GLYPHMAP_FGCOLOR_BIT) | ((uint32_t)((options).value) << GLYPHMAP_OPTIONS_BIT))

inline uint8_t glyphMapItem_getIndex(uint32_t const volatile * mapItem) { return *mapItem >> GLYPHMAP_INDEX_BIT & 0xFF; }
inline uint8_t glyphMapItem_getIndex(uint32_t const & mapItem)          { return mapItem >> GLYPHMAP_INDEX_BIT & 0xFF; }

inline Color glyphMapItem_getBGColor(uint32_t const volatile * mapItem) { return (Color)(*mapItem >> GLYPHMAP_BGCOLOR_BIT & 0x0F); }
inline Color glyphMapItem_getBGColor(uint32_t const & mapItem)          { return (Color)(mapItem >> GLYPHMAP_BGCOLOR_BIT & 0x0F); }

inline Color glyphMapItem_getFGColor(uint32_t const volatile * mapItem) { return (Color)(*mapItem >> GLYPHMAP_FGCOLOR_BIT & 0x0F); }
inline Color glyphMapItem_getFGColor(uint32_t const & mapItem)          { return (Color)(mapItem >> GLYPHMAP_FGCOLOR_BIT & 0x0F); }

inline GlyphOptions glyphMapItem_getOptions(uint32_t const volatile * mapItem) { return (GlyphOptions){.value = (uint16_t)(*mapItem >> GLYPHMAP_OPTIONS_BIT & 0xFFFF)}; }
inline GlyphOptions glyphMapItem_getOptions(uint32_t const & mapItem)          { return (GlyphOptions){.value = (uint16_t)(mapItem >> GLYPHMAP_OPTIONS_BIT & 0xFFFF)}; }

*/

void configureGPIO(gpio_num_t gpio, gpio_mode_t mode)
{
  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[gpio], PIN_FUNC_GPIO);
  gpio_set_direction(gpio, mode);
}


void esp_intr_alloc_pinnedToCore_task(void * arg)
{
  esp_intr_alloc_args *args = (esp_intr_alloc_args*) arg;
  esp_intr_alloc(args->source, args->flags, args->handler, args->arg, args->ret_handle);
  //  xTaskNotifyGive(args->waitingTask);
  vTaskDelete(NULL);
}



//defines for vga text mode
#define VGATextController_CHARWIDTH      8    // max 8
#define VGATextController_CHARWIDTHBYTES ((VGATextController_CHARWIDTH + 7) / 8)
#define VGATextController_CHARHEIGHT     14
#define VGATextController_COLUMNS        80
#define VGATextController_ROWS           34
#define VGATextController_WIDTH          640
#define VGATextController_HEIGHT         480
#define VGATextController_MODELINE "\"640x480@60Hz\" 25.175 640 656 752 800 480 490 492 525 -HSync -VSync"



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
  volatile lldesc_t * s_frameResetDesc;

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
  uint8_t       m_HVSync;

  uint8_t *              m_charData;
uint32_t *       m_map;
static uint8_t m_screenContents[80*34];
  // cursor props
  bool                   m_cursorEnabled;
  int                    m_cursorCounter; // trip from -m_cursorSpeed to +m_cursorSpeed (>= cursor is visible)
  int                    m_cursorSpeed;
  int                    m_cursorRow;
  int                    m_cursorCol;
  uint8_t                m_cursorForeground;
  uint8_t                m_cursorBackground;
  



void setupGPIO(gpio_num_t gpio, int bit, gpio_mode_t mode)
{
  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[gpio], PIN_FUNC_GPIO);
  configureGPIO(gpio, mode);
  gpio_matrix_out(gpio, I2S1O_DATA_OUT0_IDX + bit, false, false);
}


void init(gpio_num_t VSyncGPIO)
{
  m_DMABuffers = nullptr;
  init_i2s_module();

  // load font into RAM
  int charDataSize = 256 * FONT_8x14.height * ((FONT_8x14.width + 7) / 8);
  m_charData = (uint8_t*) heap_caps_malloc(charDataSize, MALLOC_CAP_8BIT);
  memcpy(m_charData, FONT_8x14.data, charDataSize);

  //usually this is 32 bits for fabgl.. 8 bits of attributes. 8 bits of character data
  //processed 2 glyphs/characters on the screen at a time. 
  m_map = (uint32_t *) heap_caps_malloc(sizeof(uint32_t)*40*32, MALLOC_CAP_8BIT);
					//  #define VGATextController_COLUMNS        80
					//  #define VGATextController_ROWS           34
  s_fgbgPattern = NULL;
}

//rows fixed at 34.
void update_text(uint8_t const *char_data){
  // wait for the end of frame
  // Wait for vblank.
  while (s_scanLine < m_timings.VVisibleArea)
    ;
  
  memcpy(m_screenContents, char_data, sizeof(uint8_t)*80*34);
}


void bbegin(gpio_num_t redGPIO, gpio_num_t greenGPIO, gpio_num_t blueGPIO, gpio_num_t HSyncGPIO, gpio_num_t VSyncGPIO)
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

void rbegin(gpio_num_t red1GPIO, gpio_num_t red0GPIO, gpio_num_t green1GPIO, gpio_num_t green0GPIO, gpio_num_t blue1GPIO, gpio_num_t blue0GPIO, gpio_num_t HSyncGPIO, gpio_num_t VSyncGPIO)
{
  bbegin(red0GPIO, green0GPIO, blue0GPIO, HSyncGPIO, VSyncGPIO);

  // GPIO configuration for bit 1
  setupGPIO(red1GPIO,   VGA_RED_BIT + 1,   GPIO_MODE_OUTPUT);
  setupGPIO(green1GPIO, VGA_GREEN_BIT + 1, GPIO_MODE_OUTPUT);
  setupGPIO(blue1GPIO,  VGA_BLUE_BIT + 1,  GPIO_MODE_OUTPUT);

  //RGB222::lowBitOnly = false;
  m_bitsPerChannel = 2;
}


void begin()
{
  rbegin(GPIO_NUM_22, GPIO_NUM_21, GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_5, GPIO_NUM_4, GPIO_NUM_23, GPIO_NUM_15);
}





// pixel 0 = byte 2, pixel 1 = byte 3, pixel 2 = byte 0, pixel 3 = byte 1 :
// pixel : 0  1  2  3  4  5  6  7  8  9 10 11 ...etc...
// byte  : 2  3  0  1  6  7  4  5 10 11  8  9 ...etc...
// dword : 0           1           2          ...etc...
// Thanks to https://github.com/paulscottrobson for the new macro. Before was: (row[((X) & 0xFFFC) + ((2 + (X)) & 3)])
#define VGA_PIXELINROW(row, X) (row[(X) ^ 2])





  void setResolution();
  void init(gpio_num_t VSyncGPIO);
//  void setupGPIO(gpio_num_t gpio, int bit, gpio_mode_t mode);
  void freeBuffers();

  void fillDMABuffers();
  uint8_t packHVSync(bool HSync, bool VSync);
  uint8_t preparePixelWithSync(RGB222 rgb, bool HSync, bool VSync);
bool convertModelineToTimings(char const *, VGATimings *);

  uint8_t IRAM_ATTR preparePixel(RGB222 rgb) { return m_HVSync | (rgb.B << VGA_BLUE_BIT) | (rgb.G << VGA_GREEN_BIT) | (rgb.R << VGA_RED_BIT); }

  static void I2SInterrupt(void * arg);

  
  

 

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
//uint8_t preparePixelWithSync(RGB222 rgb) { return m_HVSync | (rgb.B << VGA_BLUE_BIT) | (rgb.G << VGA_GREEN_BIT) | (rgb.R << VGA_RED_BIT); }

//sets 640x480
void setResolution()
{
  
  
  VGATimings timings;
  if (!convertModelineToTimings(VGATextController_MODELINE, &timings)){
    printf("Failure converting timings\n");
     return;
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
  //s_white_on_back
  //16kb we probably don't need.
  RGB222 white_f= {2, 2,2};
  RGB222 black_bg = {0,0,0};
  if (s_fgbgPattern == NULL) {
    s_fgbgPattern = (uint32_t*) heap_caps_malloc(16384, MALLOC_CAP_8BIT);
    for (int i = 0; i < 16; ++i)
      for (int fg = 0; fg < 16; ++fg)
        for (int bg = 0; bg < 16; ++bg) {
	  uint8_t fg_pat = preparePixel(white_f);
	  uint8_t bg_pat = preparePixel(black_bg);
	  //this is some kind of time saving thing in the interrupt handler
	  //this stores the different colors so they don't have to be calced
	  //in the time sensitive interrupt. I believe this is copied to the
	  //i2s DMA buffer if the bit in the bitmapped font character
	  //is enabled. Or maybe this is all possible combinations of characters

          s_fgbgPattern[i | (bg << 4) | (fg << 8)] = (i & 0b1000 ? (fg_pat << 16) : (bg_pat << 16)) |
                                                     (i & 0b0100 ? (fg_pat << 24) : (bg_pat << 24)) |
                                                     (i & 0b0010 ? (fg_pat << 0) : (bg_pat << 0)) |
                                                     (i & 0b0001 ? (fg_pat << 8) : (bg_pat << 8));
	}
    
    //s_fgbgPattern[i | (bg <<4)| (fg << 8)] = s_blankPatternDWord;
  }
  

    // ESP_INTR_FLAG_LEVEL1: should be less than PS2Controller interrupt level, necessary when running on the same core
  //CoreUsage::setBusiestCore(FABGLIB_VIDEO_CPUINTENSIVE_TASKS_CORE);


    esp_intr_alloc_args args = {ETS_I2S1_INTR_SOURCE, ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM, I2SInterrupt, NULL, NULL, NULL};
  xTaskCreatePinnedToCore(esp_intr_alloc_pinnedToCore_task, "" , 1024, &args, 3, NULL, BACKCORE);
  // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  
  play_i2s(m_timings.frequency, m_DMABuffers);
  
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
      VGA_PIXELINROW( ((uint8_t*)(m_lines) + i * VGATextController_WIDTH), rx)  = preparePixelWithSync((RGB222){0, 2, 0}, false, false);
  }


}


static void IRAM_ATTR I2SInterrupt(void * arg)
{
  //VGATextController * ctrl = (VGATextController *) arg;
  //m_charData is the font
  if (I2S1.int_st.out_eof &&  m_charData != nullptr) {

    volatile lldesc_t *desc = (volatile lldesc_t*) I2S1.out_eof_des_addr;

    if (desc == s_frameResetDesc) {
      //reset scanline count
      s_scanLine = 0;
      s_textRow  = 0;
      s_upperRow = true;
      //no cursor for now.
      /*      if (ctrl->m_cursorEnabled) {
        ++ctrl->m_cursorCounter;
        if (ctrl->m_cursorCounter >= ctrl->m_cursorSpeed)
          ctrl->m_cursorCounter = -ctrl->m_cursorSpeed;
	  }*/
      /* we know this is set
      // if (ctrl->m_map == nullptr) {
      //  I2S1.int_clr.val = I2S1.int_st.val;
      //  #ifdef VGATextController_PERFORMANCE_CHECK
      //  s_cycles += getCycleCount() - s1;
      //  #endif
      //  return;
      //}*/

     } else if (s_scanLine == 0) {
      //out of sync, wait for next frame
      //this means we are in the DMA buffer somewhere in the middle of the screen
      //but the scanline is at zero. We should keep the DMA playback going
      //eventually we will hit the end of the buffers and pass to the rest of the code. 
      I2S1.int_clr.val = I2S1.int_st.val;
      //#ifdef VGATextController_PERFORMANCE_CHECK
      //s_cycles += getCycleCount() - s1;
      //#endif
      return;
    }
    //process data to put in buffers.
    int scanLine = s_scanLine;

    const int lineIndex = scanLine % VGATextController_CHARHEIGHT;
    //This buffer is 8 bit accessible, and looks to be big enough to hold 16 lines of data
    //which is the height of one line
    uint32_t *lines = m_lines;
    //This is the number of rows in the character data we are rendering. 
    if (s_textRow< m_rows) {
      /* This code sets up the buffers to show the cursor. 
      int cursorCol = 0;
      int cursorFGBG = 0;
      const auto cursorVisible = (ctrl->m_cursorEnabled && ctrl->m_cursorCounter >= 0 && s_textRow == ctrl->m_cursorRow);
      if (cursorVisible) {
        cursorCol  = ctrl->m_cursorCol;
        cursorFGBG = (ctrl->m_cursorForeground << 4) | (ctrl->m_cursorBackground << 8);
      }
      */
      //this is the font data
      uint8_t *charData = m_charData + (s_upperRow ? 0 : VGATextController_CHARHEIGHT / 2);
      //this is the contents of the screen. 
      uint32_t *mapItemPtr = m_map + (s_textRow * VGATextController_COLUMNS);

      for (int col = 0; col < VGATextController_COLUMNS; ++col, ++mapItemPtr) {

        uint32_t mapItem = *mapItemPtr;

        int fgbg = (mapItem >> 4) & 0b111111110000;

	//const auto options = glyphMapItem_getOptions(mapItem);

        // invert?
	//        if (options.invert)
	//          fgbg = ((fgbg >> 4) & 0b11110000) | ((fgbg << 4) & 0b111100000000);

        // cursor?
	//        if (cursorVisible && col == cursorCol)
	//          fgbg = cursorFGBG;

        uint32_t * dest = lines + (lineIndex * VGATextController_WIDTH / sizeof(uint32_t)) + col * VGATextController_CHARWIDTHBYTES * 2;

        //the data has either a space or no data, so there is nothing to copy here. essentially
	//non printing characters.
	/*        if (options.blank) {

          for (int rowInChar = 0; rowInChar < VGATextController_CHARHEIGHT / 2; ++rowInChar) {
            int v = s_fgbgPattern[fgbg];
            *dest       = v;
            *(dest + 1) = v;
            dest += VGATextController_WIDTH / sizeof(uint32_t);
          }

	  } */
	
	//this sets the buffers to their proper values.
	//	const bool underline = (s_upperRow == false && options.underline);
	//	const bool bold      = options.bold;
	//40ths character
	uint8_t *charRowPtr = charData + (m_screenContents[(s_textRow*VGATextController_ROWS)+col]*  VGATextController_CHARHEIGHT * VGATextController_CHARWIDTHBYTES);
	
	for (int rowInChar = 0; rowInChar < VGATextController_CHARHEIGHT / 2; ++rowInChar) {
	  //this is the bitmap for this row of the  character in the font.
	  uint8_t charRowData = *charRowPtr;
	    
	    //            // bold?
	    //            if (bold)
	  //            charRowData |= charRowData >> 1;
	    
	  *dest       = s_fgbgPattern[(charRowData >> 4)  | fgbg];
	  *(dest + 1) = s_fgbgPattern[(charRowData & 0xF) | fgbg];
	  
	  dest += VGATextController_WIDTH / sizeof(uint32_t);
	  charRowPtr += VGATextController_CHARWIDTHBYTES;
	}
	  
	// underline - "rewind" dest and add the underline?
	
	/*if (underline) {
	  dest -= VGATextController_WIDTH / sizeof(uint32_t);
            uint32_t v = s_fgbgPattern[0xF | fgbg];
            *dest       = v;
            *(dest + 1) = v;
	    }*/
	
	  
	
      }
      
      if (s_upperRow) {
	s_upperRow = false;
      } else {
	s_upperRow = true;
	++s_textRow;
      }
      
    } else {
      //we've hit the end of the text buffer we are rendering. Just fill with blank lines. 
      for (int i = 0; i < VGATextController_CHARHEIGHT / 2; ++i) {
	uint32_t *dest = lines + ((scanLine + i) % VGATextController_CHARHEIGHT) * VGATextController_WIDTH / sizeof(uint32_t);
	for (int i = 0; i < VGATextController_COLUMNS; ++i) {
	  *dest++ = s_blankPatternDWord;
	  *dest++ = s_blankPatternDWord;
	}
      }
    }
     
    scanLine += VGATextController_CHARHEIGHT / 2;
    s_scanLine = scanLine;
    
  }
  
  // #ifdef VGATextController_PERFORMANCE_CHECK
  //s_cycles += getCycleCount() - s1;
  //#endif
  
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
    timings->HStartingBlock = FrontPorch;

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
          timings->HStartingBlock = FrontPorch;
          break;
        case 'S':
        case 's':
          timings->HStartingBlock = Sync;
          break;
        case 'B':
        case 'b':
          timings->HStartingBlock = BackPorch;
          break;
        case 'V':
        case 'v':
          timings->HStartingBlock = VisibleArea;
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



  
void app_main(){
  m_rows = 34;
  m_map  = NULL;
  printf("Error starting\n");
  begin();
  setResolution();
  for (int i = 0; i < 80*34; i++)
    m_screenContents[i] = 0x40;
  while (1) {
    vTaskDelay(100);
    printf("this is the counter: %d\n", counter);
    printf("scanline: %d\n", s_scanLine);
  }
  
}

  
