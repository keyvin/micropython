#include "py/runtime.h"
#include "py/ringbuf.h"
#include "py/mphal.h"
#include "py/obj.h"
#include "extmod/virtpin.h"
#include "input.h"
#include "mphalport.h"
#include "modmachine.h"
#include "modesp32.h"
#include <stdio.h>

/*KNP - This module allows for putting keystrokes into the STD_IN 
 * This will allow us to write the interrupt handler
 * Or polling routine in Micropython. It should also let us disable UART */

extern ringbuf_t stdin_ringbuf;



STATIC mp_obj_t put_input(mp_obj_t input)
{
  const char *in;
  if (mp_obj_is_str(input)){
    
    if (mp_obj_len(input) !=0){
     in =  mp_obj_str_get_str(input);
     while (*in !='\0'){
       ringbuf_put(&stdin_ringbuf, *in);
       in++;
     }
     
    }     
  }
  return mp_const_none;
}

void put_input_callback(const char *string, uint32_t len)
{
}

// Define a Python reference to the function above
STATIC MP_DEFINE_CONST_FUN_OBJ_1(put_input_obj, put_input);

// Define all properties of the example module.
// Table entries are key/value pairs of the attribute name (a string)
// and the MicroPython object reference.
// All identifiers and strings are written as MP_QSTR_xxx and will be
// optimized to word-sized integers by the build system (interned strings).
STATIC const mp_rom_map_elem_t input_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_input) },
    { MP_ROM_QSTR(MP_QSTR_put_input), MP_ROM_PTR(&put_input_obj) }

};

STATIC MP_DEFINE_CONST_DICT(input_module_globals, input_globals_table);

// Define module object.
const mp_obj_module_t input_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&input_module_globals,
};

// Register the module to make it available in Python
MP_REGISTER_MODULE(MP_QSTR_input, input_user_cmodule, MODULE_INPUT_ENABLED);
