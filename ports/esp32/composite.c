//look up ulong
#define Screen_WIDTH  384
#define Screen_HEIGHT 240
extern uint32_t *composite_screen;


void* MALLOC32(int x, const char* label)
{
    printf("MALLOC32 %d free, %d biggest, allocating %s:%d\n",
      heap_caps_get_free_size(MALLOC_CAP_32BIT),heap_caps_get_largest_free_block(MALLOC_CAP_32BIT),label,x);
    void * r = heap_caps_malloc(x,MALLOC_CAP_32BIT);
    if (!r) {
        printf("MALLOC32 FAILED allocation of %s:%d!!!!####################\n",label,x);
        esp_restart();
    }
    else
        printf("MALLOC32 allocation of %s:%d %08X\n",label,x,r);
    return r;
}

static int PIN(int x)
{
    if (x < 0) return 0;
    if (x > 255) return 255;
    return x;
}

static uint32_t rgb(int r, int g, int b)
{
    return (PIN(r) << 16) | (PIN(g) << 8) | PIN(b);
}



void init_screen()
{
  composite_screen = (ULONG*)MALLOC32(Screen_WIDTH*Screen_HEIGHT,"Screen_atari");    // 32 bit access plz                                                      
  _lines = (uint8_t**)MALLOC32(height*sizeof(uint8_t*),"_lines");
  const uint8_t* s = (uint8_t*)Screen_atari;
  for (int y = 0; y < height; y++) {
    _lines[y] = (uint8_t*)s;
    s += width;
  }
 
  // clear_screen();
}
