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
#include <string.h>
#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task.h"
#include "soc/cpu.h"
#include "esp_log.h"
#if MICROPY_ESP_IDF_4
#include "esp32/spiram.h"
#else
#include "esp_spiram.h"
#endif

#include "py/stackctrl.h"
#include "py/nlr.h"
#include "py/compile.h"
#include "py/runtime.h"
#include "py/persistentcode.h"
#include "py/repl.h"
#include "py/gc.h"
#include "py/mphal.h"
#include "lib/mp-readline/readline.h"
#include "lib/utils/pyexec.h"
#include "uart.h"
#include "modmachine.h"
#include "modnetwork.h"
#include "mpthreadport.h"

#if MICROPY_BLUETOOTH_NIMBLE
#include "extmod/modbluetooth.h"
#endif
#include "driver/i2c.h"





// MicroPython runs as a task under FreeRTOS
#define MP_TASK_PRIORITY        (ESP_TASK_PRIO_MIN + 1)
#define MP_TASK_STACK_SIZE      (16 * 1024)

int vprintf_null(const char *format, va_list ap) {
    // do nothing: this is used as a log target during raw repl mode
    return 0;
}


void kb_event_poll (void *pvParameter) {
  
  uint32_t io_num;
    for(;;) {
        if(xQueueReceive(kbd_queue, &io_num, portMAX_DELAY)) {
	 
	  uint8_t input = 0;
	  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	  cmd = i2c_cmd_link_create();
	  i2c_master_start(cmd);
	  i2c_master_write_byte(cmd, 4 << 1  | I2C_MASTER_READ, ACK_CHECK_DIS);
	  i2c_master_read_byte(cmd, &input, NACK_VAL);
	  i2c_master_stop(cmd);
	  i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
	  i2c_cmd_link_delete(cmd);
	  if (input == mp_interrupt_char) {
	    mp_keyboard_interrupt();
	  } else {
	    // this is an inline function so will be in IRAM                                                                  
	    ringbuf_put(&stdin_ringbuf, input);

	  }
	  

        }
    }
}



void mp_task(void *pvParameter) {
    volatile uint32_t sp = (uint32_t)get_sp();
    #if MICROPY_PY_THREAD
    mp_thread_init(pxTaskGetStackStart(NULL), MP_TASK_STACK_SIZE / sizeof(uintptr_t));
    #endif
    uart_init();

    // TODO: CONFIG_SPIRAM_SUPPORT is for 3.3 compatibility, remove after move to 4.0.
    #if CONFIG_ESP32_SPIRAM_SUPPORT || CONFIG_SPIRAM_SUPPORT
    // Try to use the entire external SPIRAM directly for the heap
    size_t mp_task_heap_size;
    void *mp_task_heap = (void *)0x3f800000;
    switch (esp_spiram_get_chip_size()) {
        case ESP_SPIRAM_SIZE_16MBITS:
            mp_task_heap_size = 2 * 1024 * 1024;
            break;
        case ESP_SPIRAM_SIZE_32MBITS:
        case ESP_SPIRAM_SIZE_64MBITS:
            mp_task_heap_size = 4 * 1024 * 1024;
            break;
        default:
            // No SPIRAM, fallback to normal allocation
            mp_task_heap_size = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
            mp_task_heap = malloc(mp_task_heap_size);
            break;
    }
    #else
    // Allocate the uPy heap using malloc and get the largest available region
    size_t mp_task_heap_size = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    void *mp_task_heap = malloc(mp_task_heap_size);
    #endif

soft_reset:
    // initialise the stack pointer for the main thread
    mp_stack_set_top((void *)sp);
    mp_stack_set_limit(MP_TASK_STACK_SIZE - 1024);
    gc_init(mp_task_heap, mp_task_heap + mp_task_heap_size);
    mp_init();
    mp_obj_list_init(mp_sys_path, 0);
    mp_obj_list_append(mp_sys_path, MP_OBJ_NEW_QSTR(MP_QSTR_));
    mp_obj_list_append(mp_sys_path, MP_OBJ_NEW_QSTR(MP_QSTR__slash_lib));
    mp_obj_list_init(mp_sys_argv, 0);
    readline_init0();

    // initialise peripherals
    machine_pins_init();
    // init uart again here (hook interrupt?)
    uart_init();
    
    // run boot-up scripts
    pyexec_frozen_module("_boot.py");
    pyexec_file_if_exists("boot.py");
    if (pyexec_mode_kind == PYEXEC_MODE_FRIENDLY_REPL) {
        pyexec_file_if_exists("main.py");
    }

    for (;;) {
        if (pyexec_mode_kind == PYEXEC_MODE_RAW_REPL) {
            vprintf_like_t vprintf_log = esp_log_set_vprintf(vprintf_null);
            if (pyexec_raw_repl() != 0) {
                break;
            }
            esp_log_set_vprintf(vprintf_log);
        } else {
            if (pyexec_friendly_repl() != 0) {
                break;
            }
        }
    }

    #if MICROPY_BLUETOOTH_NIMBLE
    mp_bluetooth_deinit();
    #endif

    machine_timer_deinit_all();

    #if MICROPY_PY_THREAD
    mp_thread_deinit();
    #endif

    gc_sweep_all();

    mp_hal_stdout_tx_str("MPY: soft reboot\r\n");

    // deinitialise peripherals
    machine_pins_deinit();
    usocket_events_deinit();

    mp_deinit();
    fflush(stdout);
    goto soft_reset;
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    kbd_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(kb_event_poll, "gpio_task_example", 2048, NULL,  ESP_TASK_PRIO_MIN + 1, NULL);
    xTaskCreatePinnedToCore(mp_task, "mp_task", MP_TASK_STACK_SIZE / sizeof(StackType_t), NULL, MP_TASK_PRIORITY, &mp_main_task_handle, MP_TASK_COREID);
    
}

void nlr_jump_fail(void *val) {
    printf("NLR jump failed, val=%p\n", val);
    esp_restart();
}

// modussl_mbedtls uses this function but it's not enabled in ESP IDF
void mbedtls_debug_set_threshold(int threshold) {
    (void)threshold;
}

void *esp_native_code_commit(void *buf, size_t len, void *reloc) {
    len = (len + 3) & ~3;
    uint32_t *p = heap_caps_malloc(len, MALLOC_CAP_EXEC);
    if (p == NULL) {
        m_malloc_fail(len);
    }
    if (reloc) {
        mp_native_relocate(reloc, buf, (uintptr_t)p);
    }
    memcpy(p, buf, len);
    return p;
}
