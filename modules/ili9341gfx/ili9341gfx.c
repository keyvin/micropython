// Include required definitions first.


#include "py/runtime.h"
#include "py/mphal.h"
#include "py/obj.h"
#include "py/objarray.h"
#include "mphalport.h"
#include "modmachine.h"
#include "modesp32.h"
#include "ili9341.h"
#include <string.h>
#include <stdio.h>



STATIC mp_obj_t py_put_text_at(mp_obj_t row, mp_obj_t col, mp_obj_t txt){
  int r = mp_obj_get_int(row);
  int c = mp_obj_get_int(col);
  
  const char *s_ptr = mp_obj_str_get_str(txt);
  int length = strlen(s_ptr);
  put_text_at(r, c, s_ptr, length);
  return mp_const_none;
}

STATIC mp_obj_t py_init_text(){
  return mp_const_none;
}

STATIC mp_obj_t py_fill_screen(mp_obj_t color) {
  return mp_const_none;
}

STATIC MP_DEFINE_CONST_FUN_OBJ_0(py_init_text_obj, py_init_text);
STATIC MP_DEFINE_CONST_FUN_OBJ_3(py_put_text_at_obj, py_put_text_at);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(py_fill_screen_obj, py_fill_screen);


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
// optimized to word-sized integers by the build system (interned stwrings).
STATIC const mp_rom_map_elem_t ili9341gfx_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_ili9341gfx) },
    { MP_ROM_QSTR(MP_QSTR_screen_blit), MP_ROM_PTR(&screen_blit_obj) },
    { MP_ROM_QSTR(MP_QSTR_put_text_at), MP_ROM_PTR(&py_put_text_at_obj)},
    { MP_ROM_QSTR(MP_QSTR_init_text_mode), MP_ROM_PTR(&py_init_text_obj)},
    { MP_ROM_QSTR(MP_QSTR_fill_screen), MP_ROM_PTR(&py_fill_screen_obj)}
};

STATIC MP_DEFINE_CONST_DICT(ili9341gfx_module_globals, ili9341gfx_globals_table);

// Define module object.
const mp_obj_module_t ili9341gfx_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&ili9341gfx_module_globals,
};

// Register the module to make it available in Python
MP_REGISTER_MODULE(MP_QSTR_ili9341gfx, ili9341gfx_user_cmodule, MODULE_ILI9341GFX_ENABLED);
