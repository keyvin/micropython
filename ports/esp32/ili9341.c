
/* SPI Master example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "ili9341.h"

//#include "pretty_effect.h"

/*
 This code displays some fancy graphics on the 320x240 LCD on an ESP-WROVER_KIT board.
 This example demonstrates the use of both spi_device_transmit as well as
 spi_device_queue_trans/spi_device_get_trans_result and pre-transmit callbacks.

 Some info about the ILI9341/ST7789V: It has an C/D line, which is connected to a GPIO here. It expects this
 line to be low for a command and high for data. We use a pre-transmit callback here to control that
 line: every transaction has as the user-definable argument the needed state of the D/C line and just
 before the transaction is sent, the callback will set this line to the correct state.
*/


//To speed up transfers, every SPI transfer sends a bunch of lines. This define specifies how many. More means more memory use,
//but less overhead for setting up / finishing transfers. Make sure 240 is dividable by this.


/* Send a command to the LCD. Uses spi_device_polling_transmit, which waits
 * until the transfer is complete.
 *
 * Since command transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */


void start_text_mode() {
  gfx_mode = FULL_TEXT;
  if (lines[0])
    free(lines[0]);
  if (lines[1])
    free(lines[1]);
  for (int i=0; i<2; i++) {
    lines[i]=heap_caps_malloc( MAX_LINES * (320 *sizeof(uint16_t)), MALLOC_CAP_DMA);
    assert(lines[i]!=NULL);
  }
  //generate fonts at gfx_high_mem
  for (uint8_t glyph = 0; glyph < 0x7F; glyph++)
    get_char( ((uint16_t *)gfx_high) + (FONT_SIZE*FONT_SIZE)*glyph, glyph);
  text_console_row = 0;
  text_console_column = 0;
  text_console_line_length = 20;
  gfx_mode = FULL_TEXT;
}

//This function unpacks a bit mapped char to a byte mapped char 
void get_char(uint16_t *buffer, int ord) {
  int x,y;
  int set;
  int mask;
    
  char *bitmap = font8x8_basic[ord];    

  for (x=0; x < 8; x++) {
    for (y=0; y < 8; y++) {
      set = bitmap[x] & (1 << y);


      buffer[((x*2)*16)+(y*2)] = set ? 0xFFFF: 0x0000;
      buffer[((x*2)*16)+((y*2)+1)] = set ? 0xFFFF: 0x0000;
      buffer[(((x*2)+1)*16)+(y*2)] = set ? 0xFFFF: 0x0000;
      buffer[(((x*2)+1)*16)+(y*2)+1] = set ? 0xFFFF: 0x0000;
      
      
      
      //printf("%c", set ? 'X' : ' ');
    }
    //    printf("\n");
  }

}

void fill_line(uint16_t *buffer, char *text, uint8_t length){
  //at most line_length characters
  uint16_t position = 0;
  int ili_window_size; //Number of pixels that constitute a row (ili9341 window pointers)

  //partial_window
  if (length < text_console_line_length){
    ili_window_size = length*FONT_SIZE;
    printf("Shorter Size\n");
  }
  //full window size (320px)
  else {
    ili_window_size=320; //320/16 =20 (0..19)
    printf("Greatest Size\n");
  }
  printf("ili_window_size:%d\nili_font_size:%d", ili_window_size, FONT_SIZE);
  while (position < length){     
    uint16_t *char_loc = ((uint16_t *) gfx_high) + (text[position] * FONT_SIZE*FONT_SIZE)-FONT_SIZE;
    uint16_t col = position*FONT_SIZE;    
    for (int j = 0; j < FONT_SIZE; j++){ //0..15
      uint16_t row = j*ili_window_size;
      memcpy(buffer + row + col, char_loc+(j*FONT_SIZE),sizeof(uint16_t)*FONT_SIZE);
    }    
    position++;
  }
  
}


void send_text_line(int x, int y, uint16_t *buffer) {
  
  
}

//this function puts text with wrap.
  

void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd)
{
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t));       //Zero out the transaction
  t.length=8;                     //Command is 8 bits
  t.tx_buffer=&cmd;               //The data is the cmd itself
  t.user=(void*)0;                //D/C needs to be set to 0
  ret=spi_device_polling_transmit(spi, &t);  //Transmit!
  assert(ret==ESP_OK);            //Should have had no issues.
}

/* Send data to the LCD. Uses spi_device_polling_transmit, which waits until the
 * transfer is complete.
 *
 * Since data transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
  esp_err_t ret;
  spi_transaction_t t;
  if (len==0) return;             //no need to send anything
  memset(&t, 0, sizeof(t));       //Zero out the transaction
  t.length=len*8;                 //Len is in bytes, transaction length is in bits.
  t.tx_buffer=data;               //Data
  t.user=(void*)1;                //D/C needs to be set to 1
  ret=spi_device_polling_transmit(spi, &t);  //Transmit!
  assert(ret==ESP_OK);            //Should have had no issues.
}

//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
  int dc=(int)t->user;
  gpio_set_level(PIN_NUM_DC, dc);
}

uint32_t lcd_get_id(spi_device_handle_t spi)
{
  //get_id cmd
  lcd_cmd(spi, 0x04);
  
  spi_transaction_t t;
  memset(&t, 0, sizeof(t));
  t.length=8*3;
  t.flags = SPI_TRANS_USE_RXDATA;
  t.user = (void*)1;
  
  esp_err_t ret = spi_device_polling_transmit(spi, &t);
  assert( ret == ESP_OK );
  
  return *(uint32_t*)t.rx_data;
}

//Initialize the display
void lcd_init(spi_device_handle_t spi)
{
  int cmd=0;
  const lcd_init_cmd_t* lcd_init_cmds;
  
  //Initialize non-SPI GPIOs
  gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
  gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
  gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);
  
  //Reset the display
  gpio_set_level(PIN_NUM_RST, 0);
  vTaskDelay(100 / portTICK_RATE_MS);
  gpio_set_level(PIN_NUM_RST, 1);
  vTaskDelay(100 / portTICK_RATE_MS);
  lcd_init_cmds = ili_init_cmds;
  
  
  //Send all the commands
  while (lcd_init_cmds[cmd].databytes!=0xff) {
    lcd_cmd(spi, lcd_init_cmds[cmd].cmd);
    lcd_data(spi, lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].databytes&0x1F);
        if (lcd_init_cmds[cmd].databytes&0x80) {
	  vTaskDelay(100 / portTICK_RATE_MS);
        }
        cmd++;
  }
  
  ///Enable backlight - 0 for dev board, 1 for regular
  gpio_set_level(PIN_NUM_BCKL, 0);
}


/* To send a set of lines we have to send a command, 2 data bytes, another command, 2 more data bytes and another command
 * before sending the line data itself; a total of 6 transactions. (We can't put all of this in just one transaction
 * because the D/C line needs to be toggled in the middle.)
 * This routine queues these commands up as interrupt transactions so they get
 * sent faster (compared to calling spi_device_transmit several times), and at
 * the mean while the lines for next transactions can get calculated.
 */
void send_lines(spi_device_handle_t spi, uint16_t sx,uint16_t sy, uint16_t ex, uint16_t ey, uint16_t *linedata)
{
  esp_err_t ret;
  int x;
  uint16_t rex = ex -1;
  //Transaction descriptors. Declared static so they're not allocated on the stack; we need this memory even when this
  //function is finished because the SPI driver needs access to it even while we're already calculating the next line.
  static spi_transaction_t trans[6];
  
  //In theory, it's better to initialize trans and data only once and hang on to the initialized
  //variables. We allocate them on the stack, so we need to re-init them each call.
  
  for (x=0; x<6; x++) {
    memset(&trans[x], 0, sizeof(spi_transaction_t));
    if ((x&1)==0) {
      //Even transfers are commands
      trans[x].length=8;
      trans[x].user=(void*)0;
    } else {
      //Odd transfers are data
      trans[x].length=8*4;
      trans[x].user=(void*)1;
    }
    trans[x].flags=SPI_TRANS_USE_TXDATA;
  }
  trans[0].tx_data[0]=0x2A;           //Column Address Set
  trans[1].tx_data[0]=(sx>>8);         //Start Col High
  trans[1].tx_data[1]=(sx&0x00ff);              //Start Col Low
  trans[1].tx_data[2]=(rex>>8);       //End Col High
  trans[1].tx_data[3]=(rex)&0x00ff;     //End Col Low
  trans[2].tx_data[0]=0x2B;           //Page address set
  trans[3].tx_data[0]=(sy>>8);        //Start page high
  trans[3].tx_data[1]=sy&0x00ff;      //start page low
  trans[3].tx_data[2]=(sy>>8);    //end page high
  trans[3].tx_data[3]=(ey)&0x00ff;  //end page low
  trans[4].tx_data[0]=0x2C;           //memory write
  trans[5].tx_buffer=linedata;        //finally send the line data
  trans[5].length=(ex-sx)*2*8*(ey-sy);   //Data length, in bits
  trans[5].flags=0; //undo SPI_TRANS_USE_TXDATA flag

  //Queue all transactions.
  for (x=0; x<6; x++) {
    ret=spi_device_queue_trans(spi, &trans[x], portMAX_DELAY);
    assert(ret==ESP_OK);
  }
  
  //When we are here, the SPI driver is busy (in the background) getting the transactions sent. That happens
  //mostly using DMA, so the CPU doesn't have much to do here. We're not going to wait for the transaction to
  //finish because we may as well spend the time calculating the next line. When that is done, we can call
  //send_line_finish, which will wait for the transfers to be done and check their status.
}

//slow and unoptimized
void blit_rect(uint16_t *data, int sx, int sy, int ex, int ey) {
  //we perform no checks.
  //we cannot malloc enough space for the entire screen in DMA capable memory
  //printf("got this far - blit_rect\n");
  int total_size = (ey-sy)*(ex-sx);
  if (total_size > MAX_RECT){
    //well .. foobar
    //send in alternating lines? Blocks? Probably a quick heuristic could go here

  }
  //safely copy to a single buffer.
  else {
    //    printf("%ls, total_size %d\n", lines[0], total_size);    
    memcpy(lines[0],data, sizeof(uint16_t)*total_size);
    send_lines(spi_handle, sx, sy,ex,ey, lines[0]);

  }
   


}

static void send_line_finish(spi_device_handle_t spi)
{
    spi_transaction_t *rtrans;
    esp_err_t ret;
    //Wait for all 6 transactions to be done and get back the results.
    for (int x=0; x<6; x++) {
        ret=spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
        assert(ret==ESP_OK);
        //We could inspect rtrans now if we received any info back. The LCD is treated as write-only, though.
    }
}



//Simple routine to generate some patterns and send them to the LCD. Don't expect anything too
//impressive. Because the SPI driver handles transactions in the background, we can calculate the next line
//while the previous one is being sent.
static void display_pretty_colors(spi_device_handle_t spi)
{
    uint16_t *lines[2];
    
    int frame=0;
   //Indexes of the line currently being sent to the LCD and the line we're calculating.
    int sending_line=-1;
    int calc_line=0;
    int sy = 0;
    int sx = 0;
    int ey = 50;
    int ex = 50;
	uint64_t time = xTaskGetTickCount() ;
    for  (int p=0; p<50*50;p++) {
       lines[0][p] = 0xe102;
	    lines[1][p]=0x0000;
       }
    while(1) {
        frame++;
        for (int y=0; y<240/50; y++) {
            //Calculate a line.
            //pretty_effect_calc_lines(lines[calc_line], y, frame, PARALLEL_LINES);
	  for (int i=0; i<320/50;i++) {
	      
	    //Finish up the sending process of the previous line, if any
            if (sending_line!=-1) send_line_finish(spi);
            //Swap sending_line and calc_line
	    

	        sending_line=calc_line;
            calc_line=(calc_line==1)?0:1;
	    
            //Send the line we currently calculated.
    	    sx=i*50;
	    ex=sx+50;
	    if(ex > 320){ex=50;sx=0;}
	    sy=y*50;
	    ey=sy+50;
	    if(ey > 240){ey=50;sy=0;}

	    
	    send_lines(spi, sx,sy,ex,ey, lines[sending_line]);
	    //if (frame%30==0)
	    for (int yy=0; yy<3; yy++){
	      for (int zz=0; zz< 50*50;zz++){
		lines[calc_line][zz] = lines[calc_line][zz] + 0x0821;
		lines[calc_line][zz] = lines[calc_line][zz] + 0x0821;
		lines[calc_line][zz] = lines[calc_line][zz] + 0x0821;
	       
		}
	    }
	    //calc new co-ords
	    }
            //The line set is queued up for sending now; the actual sending happens in the
            //background. We can go on to calculate the next line set as long as we do not
            //touch line[sending_line]; the SPI sending process is still reading from that.	    
	      
        }
	//vTaskDelay(100 / portTICK_RATE_MS);
	
	if (frame%5000==0){printf("Frame %d\n", frame);
	 
	  printf("ticks elapsed:%lld,\n", xTaskGetTickCount()-time);
	  time = xTaskGetTickCount();
	  sending_line=calc_line;
            calc_line=(calc_line==1)?0:1;
 	    //	    sending_line=calc_line;
	    //            calc_line=(calc_line==1)?0:1;
	    
	  }
			 			  
    }
}

void put_text_at(spi_device_handle_t spi_handle ,uint8_t row, uint8_t col, char *text, uint8_t len)
{
  uint16_t *flip = lines[0];
  uint16_t *flop = lines[1];
  uint16_t *tmp;
  int line_length = 19;
  uint8_t multi_line = 0;
  uint8_t last_line_len;
  //single line vs. multi line send
  if ((col + len) < line_length){
    multi_line = 0;    
  }
  else {
    multi_line = len - (line_length - col);
    last_line_len = multi_line % line_length;
    multi_line = (multi_line / line_length);
  }

  if (!multi_line){
    fill_line(flip, text, len);
    // void send_lines(spi_device_handle_t spi, uint16_t sx,uint16_t sy, uint16_t ex, uint16_t ey, uint16_t *linedata)
    //    printf("
    send_lines(spi_handle, col*FONT_SIZE, row*FONT_SIZE, (col+len)*FONT_SIZE,row*(FONT_SIZE+FONT_SIZE), flip);
    send_line_finish(spi_handle);
    return;
  }
  //  send_lines(spi_handle, 0 , (16*y), line_length*16, (16*y)+16, flop);
  /*    for (int x = 0; x< line_length; x++) {
    
    //blit_rect(flip, (16*x)+140,(16*y),(16*x)+15+140,(16*y)+16);
    int glyph = (k+y)%91 + 0x20; 
    uint16_t *char_loc = ((uint16_t *) gfx_high) + (glyph * FONT_SIZE*FONT_SIZE)-FONT_SIZE;
  }
  send_line_finish(spi_handle);
  tmp = flip;
  flip = flop;
  flop = flip;
  */
  
}


void spi_init()
{
    esp_err_t ret;
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=16*320*2
    };
    spi_device_interface_config_t devcfg={
#ifdef CONFIG_LCD_OVERCLOCK
        .clock_speed_hz=40*1000*1000,           //Clock out at 26 MHz
#else
        .clock_speed_hz=40*1000*1000,           //Clock out at 10 MHz
#endif
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
	.flags=SPI_DEVICE_NO_DUMMY
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi_handle);
    ESP_ERROR_CHECK(ret);
    //Initialize the LCD
    lcd_init(spi_handle);


    //Initialize the effect displayed
//    ret=pretty_effect_init();
    ESP_ERROR_CHECK(ret);
//Allocate memory for the pixel buffers
    //for (int i=0; i<2; i++) {
    //    lines[i]=heap_caps_malloc(MAX_RECT*sizeof(uint16_t), MALLOC_CAP_DMA);
    //    assert(lines[i]!=NULL);
    //}
    start_text_mode();
    //get_char(lines[0], 0x20);
    uint16_t *flip = lines[0];
    uint16_t *flop = lines[1];
    uint16_t *tmp;
    int line_length = 19;
    /*
        for (int k = 0; k < 91; k++){    
	  for (int y = 0; y < 15; y++){
	    send_lines(spi_handle, 0 , (16*y), line_length*16, (16*y)+16, flop);
	    for (int x = 0; x< line_length; x++) {
	      
	      //blit_rect(flip, (16*x)+140,(16*y),(16*x)+15+140,(16*y)+16);
	      int glyph = (k+y)%91 + 0x20; 
	      uint16_t *char_loc = ((uint16_t *) gfx_high) + (glyph * FONT_SIZE*FONT_SIZE)-FONT_SIZE;
	      for (int j = 0; j < 16; j++)
	    memcpy(flop + (j+j*((line_length)*16) + (x*16)),  char_loc+(j*16), sizeof(uint16_t) * FONT_SIZE);
	}
	send_line_finish(spi_handle);
	tmp = flip;
	flip = flop;
	flop = flip;
      }
      }*/
    //	send_line_finish(spi_handle);
    printf("Sending text\n");
    put_text_at(spi_handle,1,1,"Hello World",11);
    //Go do nice stuff.
    //display_pretty_colors(spi);
}
