#include "my_uart.h"
#include "zf_common_headfile.h"


void my_uart_init(void){
	
	wireless_uart_init();
	seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART);
	
}

void my_uart_write(int num,float data){
	
	switch (num){
		case 0:seekfree_assistant_oscilloscope_data.data[0] = data;break;
		case 1:seekfree_assistant_oscilloscope_data.data[1] = data;break;
		case 2:seekfree_assistant_oscilloscope_data.data[2] = data;break;
		case 3:seekfree_assistant_oscilloscope_data.data[3] = data;break;
		case 4:seekfree_assistant_oscilloscope_data.data[4] = data;break;
		case 5:seekfree_assistant_oscilloscope_data.data[5] = data;break;
		case 6:seekfree_assistant_oscilloscope_data.data[6] = data;break;
		case 7:seekfree_assistant_oscilloscope_data.data[7] = data;break;
	}
	
}


void my_uaer_send(int sum){
	seekfree_assistant_oscilloscope_data.channel_num = sum;
	seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data); 
}
