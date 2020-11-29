// Include required definitions first.


#include "py/runtime.h"
#include "py/mphal.h"
#include "extmod/virtpin.h"
#include "output.h"
#include "mphalport.h"
#include "modmachine.h"
#include "modesp32.h"
#include "mphalport.h"
#include <stdio.h>




int was_set = 0;
STATIC mp_obj_t set_print(mp_obj_t fun)
{

  if (fun == mp_const_none)
    was_set = 0;
  MP_STATE_PORT(output_func) = fun;
  was_set = 1;
  return mp_const_none;
  //output_func = NULL;
}


void output_call_callback(const char *string, uint32_t len)
{
  //  #mp_obj_t of = mp_load_global(qstr_from_str("output_fun"));
  if ( was_set == 1){
    if (strlen(string) == 0)
      return;
    
    //    #printf("str - %s\n", string);
    mp_obj_t obj = mp_obj_new_str(string,len);
    mp_call_function_1(MP_STATE_PORT(output_func), obj);
  }
}

// Define a Python reference to the function above
STATIC MP_DEFINE_CONST_FUN_OBJ_1(set_print_obj, set_print);

//This function disables serial output. 
STATIC mp_obj_t disable_uart_output(mp_obj_t disable_uart_output) {


  mp_obj_t ret = mp_const_none;
  return ret;

}

STATIC MP_DEFINE_CONST_FUN_OBJ_1(disable_uart_obj, disable_uart_output);


// Define all properties of the example module.
// Table entries are key/value pairs of the attribute name (a string)
// and the MicroPython object reference.
// All identifiers and strings are written as MP_QSTR_xxx and will be
// optimized to word-sized integers by the build system (interned strings).
STATIC const mp_rom_map_elem_t output_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_output) },
    { MP_ROM_QSTR(MP_QSTR_set_print), MP_ROM_PTR(&set_print_obj) },
    { MP_ROM_QSTR(MP_QSTR_disable_uart_output), MP_ROM_PTR(&disable_uart_obj)}

};

STATIC MP_DEFINE_CONST_DICT(output_module_globals, output_globals_table);

// Define module object.
const mp_obj_module_t output_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&output_module_globals,
};

// Register the module to make it available in Python
MP_REGISTER_MODULE(MP_QSTR_output, output_user_cmodule, MODULE_OUTPUT_ENABLED);
