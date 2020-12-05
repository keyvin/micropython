//This code is entirely derived from FABGL
//http://www.fabglib.org/
//Full Credit to Fabrizio Di Vittorio
//I never would have attempted to understand something like
//this without
//such high quality and re-usable code.
//Assume that all code in this module is licensed GPLv3


//just for compilation
typedef int bool;
#define nullptr NULL
//typedef int gpio_mode_t; 
//typedef int gpio_num_t;


#define FABGLIB_USE_APLL_AB_COEF 0

#define FABGLIB_XTAL 40000000
#define GPIO_UNUSED GPIO_NUM_MAX



#include "driver/gpio.h"
#include <stdint.h>
#include <stddef.h>

#include "rom/lldesc.h"



bool                m_DMAStarted;
volatile lldesc_t * m_DMABuffer;
volatile uint8_t *  m_DMAData;

int tmax(int a, int b){
  return (a < b) ? b : a;
}

int tmin(int a, int b) {
  return !(b < a) ? a : b;
}


void init_i2s_module();

  /*
   * Initializes GPIOStream and associate GPIOs to the outputs.
   *
   * div1_onGPIO0 If true the undivided frequency is delivered on GPIO0.
   * div2 Specifies the GPIO where to send frequency / 2 (set GPIO_UNUSED to disable output).
   * div4 Specifies the GPIO where to send frequency / 4 (set GPIO_UNUSED to disable output).
   * div8 Specifies the GPIO where to send frequency / 8 (set GPIO_UNUSED to disable output).
   * div16 Specifies the GPIO where to send frequency / 16 (set GPIO_UNUSED to disable output).
   * div32 Specifies the GPIO where to send frequency / 32 (set GPIO_UNUSED to disable output).
   * div64 Specifies the GPIO where to send frequency / 64 (set GPIO_UNUSED to disable output).
   * div128 Specifies the GPIO where to send frequency / 128 (set GPIO_UNUSED to disable output).
   * div256 Specifies the GPIO where to send frequency / 256 (set GPIO_UNUSED to disable output).
   *
   * Example:
   *
   *     // Outputs 25Mhz on GPIO0 and 6.25Mhz on GPIO5, for 5 seconds
   *     GPIOStream.begin(true, GPIO_UNUSED, GPIO_NUM_5);
   *     GPIOStream.play(25000000);
   *     delay(5000);
   *     // Outputs 20Mhz on GPIO and 5Mhz on GPIO5, for 10 seconds
   *     GPIOStream.play(20000000);
   *     delay(10000);
   *     GPIOStream.stop();
   */

  /*
   * Disables all outputs.
   */
  void end_i2s();

  /*
   * Sets the main frequency.
   *
   * freq Frequency in Hertz.
   * dmaBuffers Use only to provide custom DMA buffers.
   *
   * Example:
   *
   *     // Set 25MHz as main frequency
   *     GPIOStream.play(25000000);
   */


  /*
   * Disables all outputs.
   */
void stop();


void setupClock(int freq);


  
  
#include <alloca.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "soc/i2s_struct.h"
#include "soc/i2s_reg.h"
#include "driver/periph_ctrl.h"
#include "rom/lldesc.h"
#include "soc/rtc.h"
typedef struct APLLPARams {
  uint8_t sdm0;
  uint8_t sdm1;
  uint8_t sdm2;
  uint8_t o_div;
} APLLParams;





void init_i2s_module();
void end_i2s();
void setupGPIO_i2s(gpio_num_t, int, gpio_mode_t);
void play_i2s(int, lldesc_t volatile *);


void play_i2s(int freq, lldesc_t volatile * dmaBuffers);





/*************************************************************************************/
/* GPIOStream definitions */

void init_i2s_module()
{
  m_DMAStarted = false;
  m_DMABuffer  = nullptr;
  m_DMAData    = nullptr;
}



void end_i2s()
{
  stop();
}





void play_i2s(int freq, lldesc_t volatile * dmaBuffers)
{
  if (!m_DMAStarted) {

    // Power on device
    periph_module_enable(PERIPH_I2S1_MODULE);

    // Initialize I2S device
    I2S1.conf.tx_reset = 1;
    I2S1.conf.tx_reset = 0;

    // Reset DMA
    I2S1.lc_conf.out_rst = 1;
    I2S1.lc_conf.out_rst = 0;

    // Reset FIFO
    I2S1.conf.tx_fifo_reset = 1;
    I2S1.conf.tx_fifo_reset = 0;

    // LCD mode
    I2S1.conf2.val            = 0;
    I2S1.conf2.lcd_en         = 1;
    I2S1.conf2.lcd_tx_wrx2_en = 1;
    I2S1.conf2.lcd_tx_sdx2_en = 0;

    I2S1.sample_rate_conf.val         = 0;
    I2S1.sample_rate_conf.tx_bits_mod = 8;

    setupClock(freq);

    I2S1.fifo_conf.val                  = 0;
    I2S1.fifo_conf.tx_fifo_mod_force_en = 1;
    I2S1.fifo_conf.tx_fifo_mod          = 1;
    I2S1.fifo_conf.tx_fifo_mod          = 1;
    I2S1.fifo_conf.tx_data_num          = 32;
    I2S1.fifo_conf.dscr_en              = 1;

    I2S1.conf1.val           = 0;
    I2S1.conf1.tx_stop_en    = 0;
    I2S1.conf1.tx_pcm_bypass = 1;

    I2S1.conf_chan.val         = 0;
    I2S1.conf_chan.tx_chan_mod = 1;

    I2S1.conf.tx_right_first = 1;

    I2S1.timing.val = 0;

    // Reset AHB interface of DMA
    I2S1.lc_conf.ahbm_rst      = 1;
    I2S1.lc_conf.ahbm_fifo_rst = 1;
    I2S1.lc_conf.ahbm_rst      = 0;
    I2S1.lc_conf.ahbm_fifo_rst = 0;

    // Start DMA
    I2S1.lc_conf.val    = I2S_OUT_DATA_BURST_EN | I2S_OUTDSCR_BURST_EN;
    I2S1.out_link.addr  = (uint32_t) (dmaBuffers ? &dmaBuffers[0] : m_DMABuffer);
    I2S1.out_link.start = 1;
    I2S1.conf.tx_start  = 1;

    m_DMAStarted = true;

  }
}


void stop_i2s()
{
  if (m_DMAStarted) {
    rtc_clk_apll_enable(false, 0, 0, 0, 0);
    periph_module_disable(PERIPH_I2S1_MODULE);

    m_DMAStarted = false;
  }
}


// Must be:
//   maxDen > 1
//   value >= 0
#if FABGLIB_USE_APLL_AB_COEF
void floatToFraction(double value, int maxDen, int * num, int * den)
{
  int64_t a, h[3] = { 0, 1, 0 }, k[3] = { 1, 0, 0 };
  int64_t x, d, n = 1;
  while (value != floor(value)) {
    n <<= 1;
    value *= 2;
  }
  d = value;
  for (int i = 0; i < 64; ++i) {
    a = n ? d / n : 0;
    if (i && !a)
      break;
    x = d;
    d = n;
    n = x % n;
    x = a;
    if (k[1] * a + k[0] >= maxDen) {
      x = (maxDen - k[0]) / k[1];
      if (x * 2 >= a || k[1] >= maxDen)
        i = 65;
      else
        break;
    }
    h[2] = x * h[1] + h[0];
    h[0] = h[1];
    h[1] = h[2];
    k[2] = x * k[1] + k[0];
    k[0] = k[1];
    k[1] = k[2];
  }
  *den = k[1];
  *num = h[1];
}
#endif



// definitions:
//   apll_clk = XTAL * (4 + sdm2 + sdm1 / 256 + sdm0 / 65536) / (2 * o_div + 4)
//     dividend = XTAL * (4 + sdm2 + sdm1 / 256 + sdm0 / 65536)
//     divisor  = (2 * o_div + 4)
//   freq = apll_clk / (2 + b / a)        => assumes  tx_bck_div_num = 1 and clkm_div_num = 2
// Other values range:
//   sdm0  0..255
//   sdm1  0..255
//   sdm2  0..63
//   o_div 0..31
// Assume xtal = FABGLIB_XTAL (40MHz)
// The dividend should be in the range of 350 - 500 MHz (350000000-500000000), so these are the
// actual parameters ranges (so the minimum apll_clk is 5303030 Hz and maximum is 125000000Hz):
//  MIN 87500000Hz - sdm0 = 0 sdm1 = 192 sdm2 = 4 o_div = 0
//  MAX 125000000Hz - sdm0 = 0 sdm1 = 128 sdm2 = 8 o_div = 0
//
//  MIN 58333333Hz - sdm0 = 0 sdm1 = 192 sdm2 = 4 o_div = 1
//  MAX 83333333Hz - sdm0 = 0 sdm1 = 128 sdm2 = 8 o_div = 1
//
//  MIN 43750000Hz - sdm0 = 0 sdm1 = 192 sdm2 = 4 o_div = 2
//  MAX 62500000Hz - sdm0 = 0 sdm1 = 128 sdm2 = 8 o_div = 2
//
//  MIN 35000000Hz - sdm0 = 0 sdm1 = 192 sdm2 = 4 o_div = 3
//  MAX 50000000Hz - sdm0 = 0 sdm1 = 128 sdm2 = 8 o_div = 3
//
//  MIN 29166666Hz - sdm0 = 0 sdm1 = 192 sdm2 = 4 o_div = 4
//  MAX 41666666Hz - sdm0 = 0 sdm1 = 128 sdm2 = 8 o_div = 4
//
//  MIN 25000000Hz - sdm0 = 0 sdm1 = 192 sdm2 = 4 o_div = 5
//  MAX 35714285Hz - sdm0 = 0 sdm1 = 128 sdm2 = 8 o_div = 5
//
//  MIN 21875000Hz - sdm0 = 0 sdm1 = 192 sdm2 = 4 o_div = 6
//  MAX 31250000Hz - sdm0 = 0 sdm1 = 128 sdm2 = 8 o_div = 6
//
//  MIN 19444444Hz - sdm0 = 0 sdm1 = 192 sdm2 = 4 o_div = 7
//  MAX 27777777Hz - sdm0 = 0 sdm1 = 128 sdm2 = 8 o_div = 7
//
//  MIN 17500000Hz - sdm0 = 0 sdm1 = 192 sdm2 = 4 o_div = 8
//  MAX 25000000Hz - sdm0 = 0 sdm1 = 128 sdm2 = 8 o_div = 8
//
//  MIN 15909090Hz - sdm0 = 0 sdm1 = 192 sdm2 = 4 o_div = 9
//  MAX 22727272Hz - sdm0 = 0 sdm1 = 128 sdm2 = 8 o_div = 9
//
//  MIN 14583333Hz - sdm0 = 0 sdm1 = 192 sdm2 = 4 o_div = 10
//  MAX 20833333Hz - sdm0 = 0 sdm1 = 128 sdm2 = 8 o_div = 10
//
//  MIN 13461538Hz - sdm0 = 0 sdm1 = 192 sdm2 = 4 o_div = 11
//  MAX 19230769Hz - sdm0 = 0 sdm1 = 128 sdm2 = 8 o_div = 11
//
//  MIN 12500000Hz - sdm0 = 0 sdm1 = 192 sdm2 = 4 o_div = 12
//  MAX 17857142Hz - sdm0 = 0 sdm1 = 128 sdm2 = 8 o_div = 12
//
//  MIN 11666666Hz - sdm0 = 0 sdm1 = 192 sdm2 = 4 o_div = 13
//  MAX 16666666Hz - sdm0 = 0 sdm1 = 128 sdm2 = 8 o_div = 13
//
//  MIN 10937500Hz - sdm0 = 0 sdm1 = 192 sdm2 = 4 o_div = 14
//  MAX 15625000Hz - sdm0 = 0 sdm1 = 128 sdm2 = 8 o_div = 14
//
//  MIN 10294117Hz - sdm0 = 0 sdm1 = 192 sdm2 = 4 o_div = 15
//  MAX 14705882Hz - sdm0 = 0 sdm1 = 128 sdm2 = 8 o_div = 15
//
//  MIN 9722222Hz - sdm0 = 0 sdm1 = 192 sdm2 = 4 o_div = 16
//  MAX 13888888Hz - sdm0 = 0 sdm1 = 128 sdm2 = 8 o_div = 16
//
//  MIN 9210526Hz - sdm0 = 0 sdm1 = 192 sdm2 = 4 o_div = 17
//  MAX 13157894Hz - sdm0 = 0 sdm1 = 128 sdm2 = 8 o_div = 17
//
//  MIN 8750000Hz - sdm0 = 0 sdm1 = 192 sdm2 = 4 o_div = 18
//  MAX 12500000Hz - sdm0 = 0 sdm1 = 128 sdm2 = 8 o_div = 18
//
//  MIN 8333333Hz - sdm0 = 0 sdm1 = 192 sdm2 = 4 o_div = 19
//  MAX 11904761Hz - sdm0 = 0 sdm1 = 128 sdm2 = 8 o_div = 19
//
//  MIN 7954545Hz - sdm0 = 0 sdm1 = 192 sdm2 = 4 o_div = 20
//  MAX 11363636Hz - sdm0 = 0 sdm1 = 128 sdm2 = 8 o_div = 20
//
//  MIN 7608695Hz - sdm0 = 0 sdm1 = 192 sdm2 = 4 o_div = 21
//  MAX 10869565Hz - sdm0 = 0 sdm1 = 128 sdm2 = 8 o_div = 21
//
//  MIN 7291666Hz - sdm0 = 0 sdm1 = 192 sdm2 = 4 o_div = 22
//  MAX 10416666Hz - sdm0 = 0 sdm1 = 128 sdm2 = 8 o_div = 22
//
//  MIN 7000000Hz - sdm0 = 0 sdm1 = 192 sdm2 = 4 o_div = 23
//  MAX 10000000Hz - sdm0 = 0 sdm1 = 128 sdm2 = 8 o_div = 23
//
//  MIN 6730769Hz - sdm0 = 0 sdm1 = 192 sdm2 = 4 o_div = 24
//  MAX 9615384Hz - sdm0 = 0 sdm1 = 128 sdm2 = 8 o_div = 24
//
//  MIN 6481481Hz - sdm0 = 0 sdm1 = 192 sdm2 = 4 o_div = 25
//  MAX 9259259Hz - sdm0 = 0 sdm1 = 128 sdm2 = 8 o_div = 25
//
//  MIN 6250000Hz - sdm0 = 0 sdm1 = 192 sdm2 = 4 o_div = 26
//  MAX 8928571Hz - sdm0 = 0 sdm1 = 128 sdm2 = 8 o_div = 26
//
//  MIN 6034482Hz - sdm0 = 0 sdm1 = 192 sdm2 = 4 o_div = 27
//  MAX 8620689Hz - sdm0 = 0 sdm1 = 128 sdm2 = 8 o_div = 27
//
//  MIN 5833333Hz - sdm0 = 0 sdm1 = 192 sdm2 = 4 o_div = 28
//  MAX 8333333Hz - sdm0 = 0 sdm1 = 128 sdm2 = 8 o_div = 28
//
//  MIN 5645161Hz - sdm0 = 0 sdm1 = 192 sdm2 = 4 o_div = 29
//  MAX 8064516Hz - sdm0 = 0 sdm1 = 128 sdm2 = 8 o_div = 29
//
//  MIN 5468750Hz - sdm0 = 0 sdm1 = 192 sdm2 = 4 o_div = 30
//  MAX 7812500Hz - sdm0 = 0 sdm1 = 128 sdm2 = 8 o_div = 30
//
//  MIN 5303030Hz - sdm0 = 0 sdm1 = 192 sdm2 = 4 o_div = 31
//  MAX 7575757Hz - sdm0 = 0 sdm1 = 128 sdm2 = 8 o_div = 31

static void APLLCalcParams(double freq, APLLParams * params, uint8_t * a, uint8_t * b, double * out_freq, double * error)
{
  double FXTAL = FABGLIB_XTAL;

  *error = 999999999;

  double apll_freq = freq * 2;

  for (int o_div = 0; o_div <= 31; ++o_div) {

    int idivisor = (2 * o_div + 4);

    for (int sdm2 = 4; sdm2 <= 8; ++sdm2) {

      // from tables above
      int minSDM1 = (sdm2 == 4 ? 192 : 0);
      int maxSDM1 = (sdm2 == 8 ? 128 : 255);
      // apll_freq = XTAL * (4 + sdm2 + sdm1 / 256) / divisor   ->   sdm1 = (apll_freq * divisor - XTAL * 4 - XTAL * sdm2) * 256 / XTAL
      int startSDM1 = ((apll_freq * idivisor - FXTAL * 4.0 - FXTAL * sdm2) * 256.0 / FXTAL);
#if FABGLIB_USE_APLL_AB_COEF
      for (int isdm1 = tmax(minSDM1, startSDM1); isdm1 <= maxSDM1; ++isdm1) {
#else
      int isdm1 = startSDM1; {
#endif

        int sdm1 = isdm1;
        sdm1 = tmax(minSDM1, sdm1);
        sdm1 = tmin(maxSDM1, sdm1);

        // apll_freq = XTAL * (4 + sdm2 + sdm1 / 256 + sdm0 / 65536) / divisor   ->   sdm0 = (apll_freq * divisor - XTAL * 4 - XTAL * sdm2 - XTAL * sdm1 / 256) * 65536 / XTAL
        int sdm0 = ((apll_freq * idivisor - FXTAL * 4.0 - FXTAL * sdm2 - FXTAL * sdm1 / 256.0) * 65536.0 / FXTAL);
        // from tables above
        sdm0 = (sdm2 == 8 && sdm1 == 128 ? 0 : tmin(255, sdm0));
        sdm0 = tmax(0, sdm0);

        // dividend inside 350-500Mhz?
        double dividend = FXTAL * (4.0 + sdm2 + sdm1 / 256.0 + sdm0 / 65536.0);
        if (dividend >= 350000000 && dividend <= 500000000) {
          // adjust output frequency using "b/a"
          double oapll_freq = dividend / idivisor;

          // Calculates "b/a", assuming tx_bck_div_num = 1 and clkm_div_num = 2:
          //   freq = apll_clk / (2 + clkm_div_b / clkm_div_a)
          //     abr = clkm_div_b / clkm_div_a
          //     freq = apll_clk / (2 + abr)    =>    abr = apll_clk / freq - 2
          uint8_t oa = 1, ob = 0;
#if FABGLIB_USE_APLL_AB_COEF
          double abr = oapll_freq / freq - 2.0;
          if (abr > 0 && abr < 1) {
            int num, den;
            floatToFraction(abr, 63, &num, &den);
            ob = tclamp(num, 0, 63);
            oa = tclamp(den, 0, 63);
          }
#endif

          // is this the best?
          double ofreq = oapll_freq / (2.0 + (double)ob / oa);
          double err = freq - ofreq;
          if (abs(err) < abs(*error)) {
            *params = (APLLParams){(uint8_t)sdm0, (uint8_t)sdm1, (uint8_t)sdm2, (uint8_t)o_div};
            *a = oa;
            *b = ob;
            *out_freq = ofreq;
            *error = err;
            if (err == 0.0)
              return;
          }
        }
      }

    }
  }
}


void setupClock(int freq)
{
  APLLParams p = {0, 0, 0, 0};
  double error, out_freq;
  uint8_t a = 1, b = 0;
  APLLCalcParams(freq, &p, &a, &b, &out_freq, &error);
  printf("clock freq: %d, %d, %d, %d, %f, %f\n",p.sdm0, p.sdm1, p.sdm2, p.o_div, out_freq, error);
  I2S1.clkm_conf.val          = 0;
  I2S1.clkm_conf.clkm_div_b   = b;
  I2S1.clkm_conf.clkm_div_a   = a;
  I2S1.clkm_conf.clkm_div_num = 2;  // not less than 2
  printf("I2s clocks: %d, %d\n", a,b);
  I2S1.sample_rate_conf.tx_bck_div_num = 1; // this makes I2S1O_BCK = I2S1_CLK

  rtc_clk_apll_enable(true, p.sdm0, p.sdm1, p.sdm2, p.o_div);

  I2S1.clkm_conf.clka_en = 1;
}


