#include "menu.h"
#include "zf_common_headfile.h"

extern struct keys key[4];
#define key_up                  (key[1].single_flag)
#define key_down                (key[0].single_flag)
#define key_left                (key[2].single_flag)
#define key_right               (key[3].single_flag)

static int num = 0,interface = 0;

void menu(void){

	while(1){
		key_read();
		
		switch (interface){
		
			case 0:{
				tft180_show_string(0,num * 16,"->");
				tft180_show_string(16,0,"start");
				tft180_show_string(16,16,"FL");
				tft180_show_string(16,32,"FR");
				tft180_show_string(16,48,"BL");
				tft180_show_string(16,64,"BR");
				tft180_show_string(16,80,"YAW");
				break;
			}
			case 1:{
			
				break;
			}
			
		}
		
		if (key_up){
			num++;
			if (num > 5) num = 5;
			key_up = 0;
			tft180_clear();
		}
		if (key_down){
			num--;
			if (num < 0) num = 0;
			key_down = 0;
			tft180_clear();
		}
		if (key_right){
			if (num == 0){
				tft180_clear();
				tft180_show_string(16,32,"RUN!!!");
				return;
			}
			key_right = 0;
			tft180_clear();
		}
		
		system_delay_ms(10);
	}
	
}
