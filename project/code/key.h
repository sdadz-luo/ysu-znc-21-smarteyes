#ifndef _CODE_KEY_h_
#define _CODE_KEY_h_

#include "stdbool.h"

struct keys{
	char value;
	bool key_sta;
	int time;
	int double_time;
	bool single_flag;
};

extern struct keys key[4];

void my_key_init(void);
void key_read(void);

#endif
