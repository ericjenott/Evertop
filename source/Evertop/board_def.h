//#pragma message "Using board: MY_LILYGO_T8_171"
#define EPD_SCLK      (18)    
#define EPD_CS        (-1)   //EPD is the only device on vspi, can I hard wire EPD's CS to low and use GPIO 4 for something else?  
#define EPD_RSET      (12)   
#define EPD_DC        (19)    
#define EPD_BUSY      (35)   
#define EPD_MOSI      (5)    
#define EPD_MISO      (2)   

#define ELINK_BUSY 35
#define ELINK_RESET 12
#define ELINK_DC 19
#define ELINK_SS -1

#define SDCARD_SS 13
#define SDCARD_CS     (13)
#define SDCARD_MOSI   (15)
#define SDCARD_MISO   (2)
#define SDCARD_SCLK   (14)


//#define _HAS_ADC_DETECTED_
//#define _HAS_LED_
//#define _HAS_SPEAKER_
//#define _BUILTIN_DAC_
//#define _USE_SHARED_SPI_BUS_
//#define _HAS_SDCARD_
