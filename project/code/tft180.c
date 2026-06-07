#include "tft180.h"
#include "zf_common_headfile.h"

#define IPS200_TYPE     (IPS200_TYPE_SPI)                                 // 并口两寸屏 这里宏定义填写 IPS200_TYPE_PARALLEL8

void tft_init(void){
    tft180_set_dir(TFT180_PORTAIT);
    tft180_set_font(TFT180_8X16_FONT);
    tft180_set_color(RGB565_WHITE, RGB565_BLACK);
    tft180_init();
    interrupt_global_enable(0);
		tft180_clear();
}
