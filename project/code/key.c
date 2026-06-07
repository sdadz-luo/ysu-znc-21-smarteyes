#include "key.h"
#include "zf_common_headfile.h"

struct keys key[4] = {0,0,0,0,0,0,0};

void my_key_init(void){
	
	gpio_init(C30, GPI, 1, GPI_PULL_UP);
	gpio_init(C31, GPI, 1, GPI_PULL_UP);
	gpio_init(C28, GPI, 1, GPI_PULL_UP);
	gpio_init(C29, GPI, 1, GPI_PULL_UP);
	
}

void key_read(void){
		key[0].key_sta = gpio_get_level(C30);
		key[1].key_sta = gpio_get_level(C29);
		key[2].key_sta = gpio_get_level(C31);
		key[3].key_sta = gpio_get_level(C28);
		
    for(int i=0;i<4;i++){
        switch (key[i].value){
            case 0:{
                if (key[i].key_sta == 0){
                    key[i].value = 1;
                }
                break;
            }
            case 1:{
                if (key[i].key_sta == 0){
                    key[i].single_flag = 1;
                    key[i].value = 2;
                }
                break;
            }
            case 2:{
                if (key[i].key_sta == 1){
                    key[i].value = 0;
                }
            }
        }
    }
}
