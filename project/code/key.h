#ifndef _CODE_KEY_h_
#define _CODE_KEY_h_

#include <stdbool.h>

// ================= 按键状态机定义 =================
enum {
    KEY_STATE_IDLE    = 0,  // 等待按下
    KEY_STATE_PRESSED = 1,  // 已按下，等待释放
    KEY_STATE_DONE    = 2,  // 已释放确认，等待回到空闲
};

#define KEY_COUNT 4

// ================= 按键数据结构 =================
struct keys {
    char  value;         // 状态机当前状态 (KEY_STATE_xxx)
    bool  key_sta;       // 当前电平 (0=按下, 1=释放)
    int   time;          // 长按计时
    int   double_time;   // 双击间隔计时
    bool  single_flag;   // 单击事件标志
};

extern struct keys key[KEY_COUNT];

void my_key_init(void);
void key_read(void);

#endif
