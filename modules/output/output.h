#ifndef OUTPUT_H
#define OUTPUT_H 1

//mp_obj_t output_func;

uint8_t uart_output_enabled;

STATIC mp_obj_t output_set();
void output_call_callback(const char *, uint32_t);
#endif



