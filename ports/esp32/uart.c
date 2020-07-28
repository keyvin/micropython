/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * Development of the code in this file was sponsored by Microbric Pty Ltd
 *
 * The MIT License (MIT) 
 *
 * Copyright (c) 2016 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdio.h>

#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "py/runtime.h"
#include "py/mphal.h"
#include "sdkconfig.h"
#include "uart.h"
/*Edited by KNP - add interrupt handler on GPIO 15. On interrupt, read one byte from I2C bus.
  Put byte read into ring buffer as if it came from the uart. */

#include "driver/gpio.h"




STATIC void uart_irq_handler(void *arg);
STATIC void key_irq_handler(void *arg);

void i2c_init(void);
void uart_init(void) {
    uart_isr_handle_t handle;
    uart_isr_register(UART_NUM_0, uart_irq_handler, NULL, ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM, &handle);
    uart_enable_rx_intr(UART_NUM_0);
    i2c_init();
}

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

void i2c_init(void) {
  //
#define I2C_MASTER_SCL_IO 5               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 4               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(0) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 50000        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
  
  int i2c_master_port = I2C_NUM_0;
  i2c_config_t conf;
  	i2c_driver_delete(I2C_NUM_0);
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_MASTER_SDA_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = I2C_MASTER_SCL_IO;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

  i2c_param_config(I2C_NUM_0, &conf);

	i2c_driver_install(i2c_master_port, conf.mode,
			    I2C_MASTER_RX_BUF_DISABLE,
			    I2C_MASTER_TX_BUF_DISABLE, 0);

#define GPIO_INPUT_IO_0     15
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_IO_0)
#define ESP_INTR_FLAG_DEFAULT 0
 	 
    gpio_config_t io_conf;


    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;

    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    
    io_conf.pull_down_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    gpio_isr_handler_add(GPIO_INPUT_IO_0, key_irq_handler, (void*) GPIO_INPUT_IO_0);

    
}

#define PCONTROL_ADDRESS 4
#define KB_I2C 0
//
STATIC void IRAM_ATTR key_irq_handler(void *arg) {
  uint32_t num = 0;
  xQueueSendFromISR(kbd_queue, &num, NULL);
    
}


// all code executed in ISR must be in IRAM, and any const data must be in DRAM
STATIC void IRAM_ATTR uart_irq_handler(void *arg) {
    volatile uart_dev_t *uart = &UART0;
    uart->int_clr.rxfifo_full = 1;
    uart->int_clr.frm_err = 1;
    uart->int_clr.rxfifo_tout = 1;
    while (uart->status.rxfifo_cnt) {
        uint8_t c = uart->fifo.rw_byte;
        if (c == mp_interrupt_char) {
            mp_keyboard_interrupt();
        } else {
            // this is an inline function so will be in IRAM
            ringbuf_put(&stdin_ringbuf, c);
        }
    }
}
