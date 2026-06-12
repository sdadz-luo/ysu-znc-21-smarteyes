#include "key.h"
#include "zf_common_headfile.h"

// ================= 按键引脚映射表 =================
// key[0]→C30, key[1]→C29, key[2]→C31, key[3]→C28
static const gpio_pin_enum key_pins[KEY_COUNT] = { C30, C29, C31, C28 };

struct keys key[KEY_COUNT] = { {0, 0, 0, 0, 0} };

void my_key_init(void) {
    for (int i = 0; i < KEY_COUNT; i++) {
        gpio_init(key_pins[i], GPI, 1, GPI_PULL_UP);
    }
}

void key_read(void) {
    // 读取所有按键电平
    for (int i = 0; i < KEY_COUNT; i++) {
        key[i].key_sta = gpio_get_level(key_pins[i]);
    }

    // 按键状态机：消抖 + 单击检测
    for (int i = 0; i < KEY_COUNT; i++) {
        switch (key[i].value) {
        case KEY_STATE_IDLE:
            if (key[i].key_sta == 0) {
                key[i].value = KEY_STATE_PRESSED;
            }
            break;

        case KEY_STATE_PRESSED:
            if (key[i].key_sta == 0) {
                key[i].single_flag = 1;
                key[i].value = KEY_STATE_DONE;
            }
            break;

        case KEY_STATE_DONE:
            if (key[i].key_sta == 1) {
                key[i].value = KEY_STATE_IDLE;
            }
            break;
        }
    }
}
