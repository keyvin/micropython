// Include required definitions first.


#include "py/runtime.h"
#include "py/mphal.h"
#include "py/obj.h"
#include "py/objarray.h"
#include "mphalport.h"
#include "modmachine.h"
#include "modesp32.h"
#include "ili9341.h"
#include <stdio.h>





//positional (byte array, len, height, sx, sy)
STATIC mp_obj_t screen_blit(size_t nargs,const mp_obj_t *args)
{
  //should raise type error if not the right number of parameters.
  mp_buffer_info_t buff;
  mp_get_buffer(args[0], &buff,0);
  
  uint16_t *buffer = buff.buf; //will blit this.
  //could should check if buff.len.ould also check buff.code
  int length=mp_obj_get_int(args[1]);
  int height=mp_obj_get_int(args[2]);
  int sx=mp_obj_get_int(args[3]);
  int sy=mp_obj_get_int(args[4]);
  blit_rect(buffer, sx, sy, sx+length, sy+height);
  return mp_const_none;
  //output_func = NULL;
}



#define arg_min (5)
#define arg_max (5)
// Define a Python reference to the function above
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(screen_blit_obj,arg_min,arg_max, screen_blit);


// Define all properties of the example module.
// Table entries are key/value pairs of the attribute name (a string)
// and the MicroPython object reference.
// All identifiers and strings are written as MP_QSTR_xxx and will be
// optimized to word-sized integers by the build system (interned strings).
STATIC const mp_rom_map_elem_t ili9341gfx_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_ili9341gfx) },
    { MP_ROM_QSTR(MP_QSTR_screen_blit), MP_ROM_PTR(&screen_blit_obj) }

};

STATIC MP_DEFINE_CONST_DICT(ili9341gfx_module_globals, ili9341gfx_globals_table);

// Define module object.
const mp_obj_module_t ili9341gfx_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&ili9341gfx_module_globals,
};

// Register the module to make it available in Python
MP_REGISTER_MODULE(MP_QSTR_ili9341gfx, ili9341gfx_user_cmodule, MODULE_ILI9341GFX_ENABLED);
