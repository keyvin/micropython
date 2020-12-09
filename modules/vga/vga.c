// Include required definitions first.


#include "py/runtime.h"
#include "py/mphal.h"
#include "py/obj.h"
#include "py/objarray.h"
#include "mphalport.h"
#include "modmachine.h"
#include "modesp32.h"
//#include "ili9341.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

extern void update_text(uint8_t *);
STATIC mp_obj_t py_fill_screen(mp_obj_t txt){
  const char *s_ptr = mp_obj_str_get_str(txt);
  update_text((uint8_t *)s_ptr);  
  return mp_const_none;
}

//STATIC mp_obj_t py_init_text(){
//  return mp_const_none;
//}

//STATIC mp_obj_t py_fill_screen(mp_obj_t color) {
//  return mp_const_none;

STATIC MP_DEFINE_CONST_FUN_OBJ_1(py_fill_screen_obj, py_fill_screen);


// Define all properties of the example module.
// Table entries are key/value pairs of the attribute name (a string)
// and the MicroPython object reference.
// All identifiers and strings are written as MP_QSTR_xxx and will be
// optimized to word-sized integers by the build system (interned stwrings).
STATIC const mp_rom_map_elem_t vga_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_vga) },
    { MP_ROM_QSTR(MP_QSTR_fill_screen), MP_ROM_PTR(&py_fill_screen_obj)}
};

STATIC MP_DEFINE_CONST_DICT(vga_module_globals, vga_globals_table);

// Define module object.
const mp_obj_module_t vga_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&vga_module_globals,
};

// Register the module to make it available in Python
MP_REGISTER_MODULE(MP_QSTR_vga, vga_user_cmodule, MODULE_VGA_ENABLED);
