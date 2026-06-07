#include "menu.h"
#include "zf_common_headfile.h"

extern struct keys key[4];
#define key_up                  (key[2].single_flag)
#define key_down                (key[3].single_flag)
#define key_left                (key[0].single_flag)
#define key_right               (key[1].single_flag)

typedef enum {
    start,
    PID,
    ORIGIN,
    CORR,
    datasave
} menu_State;

static menu_State interface = start;  /* 当前菜单界面 */
static int8_t num = 0,mode = 0;

/* 获取当前菜单的选项数量 */
static int get_menu_item_count(void){
    switch (interface)
    {
        case start:  return 5;  /* start, PID, ORIGIN, CORR, datasave */
        case PID:  return 9;    /* PID */
        case ORIGIN:  return 4; /* ORIGIN */
        case CORR:  return 3;   /* CORR */
        default: return 0;
    }
}

/* 数据保存 */
static void Data_save(void){
    tft180_clear();
    tft180_show_string(16, 32, "Saving...");
    system_delay_ms(800);  
}

/* 显示当前菜单内容 */
static void show_menu(void){
    switch (interface){
        /* ========== 主菜单 ========== */
        case start:{
            tft180_show_string(0,  num * 16, "->");
            tft180_show_string(16, 0,  "start");
            tft180_show_string(16, 16, "PID");
            tft180_show_string(16, 32, "ORIGIN");
            tft180_show_string(16, 48, "CORR");
            tft180_show_string(16, 64, "datasave");
            break;
        }

        /* ========== PID 子菜单 ========== */
        case PID:{
            tft180_show_string(0, 0,  "PID Setting");
            tft180_show_string(0,  16 + num * 16, "->");
            tft180_show_string(16, 16, "x_P");
            tft180_show_string(16, 32, "x_I");
            tft180_show_string(16, 48, "x_D");
            tft180_show_string(16, 64, "x_speed");
            tft180_show_string(16, 80, "y_P");
            tft180_show_string(16, 96, "y_I");
            tft180_show_string(16, 112, "y_D");
            tft180_show_string(16, 128, "y_speed");
            tft180_show_string(16, 144, "BACK");
            break;
        }

        /* ========== ORIGIN 子菜单 ========== */
        case ORIGIN:{
            tft180_show_string(0, 0,  "Origin Setting");
            tft180_show_string(0,  16 + num * 16, "->");
            tft180_show_string(16, 16, "X");
            tft180_show_string(16, 32, "Y");
            tft180_show_string(16, 48, "YAW");
            tft180_show_string(16, 64, "BACK");
            break;
        }

        /* ========== CORR 子菜单 ========== */
        case CORR:{
            tft180_show_string(0, 0,  "Correct Setting");
            tft180_show_string(0,  16 + num * 16, "->");
            tft180_show_string(16, 16, "X_enc");
            tft180_show_string(16, 32, "Y_enc");
            tft180_show_string(16, 48, "BACK");
            break;
        }

        default:break;
    }
}

void menu(void){
    num = 0;
    interface = start;

    while (1){
        key_read();

        show_menu();

        /* ---- 上键 ---- */
        if (key_up){
            num++;
            if (num > get_menu_item_count() - 1) num = 0;
            key_up = 0;
            tft180_clear();
        }

        /* ---- 下键 ---- */
        if (key_down){
            num--;
            if (num < 0) num = get_menu_item_count() - 1;
            key_down = 0;
            tft180_clear();
        }

        /* ---- 右键（确认/进入） ---- */
        if (key_right){
            key_right = 0;

            switch (interface){
                /* -------- 主菜单 -------- */
                case start:{
                    if (num == 0)   /* start: 直接开始 */
                    {
                        tft180_clear();
                        tft180_show_string(16, 32, "RUN!!!");
                        return;
                    }
                    else if (num == 1) interface = PID;     /* 进入 PID */
                    else if (num == 2) interface = ORIGIN;  /* 进入 原点 */
                    else if (num == 3) interface = CORR;    /* 进入 修正值 */
                    else if (num == 4) Data_save();         /* 数据保存 */
                    num = 0;
                    tft180_clear();
                    break;
                }

                /* -------- PID 子菜单 -------- */
                case PID:{
                    if (num == get_menu_item_count() - 1)       /* BACK */
                    {
                        interface = start;
                        num = 1;
                    }
                    else
                    {
                        /* 选中 P/I/D，显示提示 */
                        tft180_clear();
                        tft180_show_string(16, 32, "PID Adjusted!");
                        system_delay_ms(800);
                    }
                    tft180_clear();
                    break;
                }

                /* -------- ORIGIN 子菜单 -------- */
                case ORIGIN:{
                    if (num == get_menu_item_count() - 1)       /* BACK */
                    {
                        interface = start;
                        num = 2;
                    }
                    else
                    {
                        tft180_clear();
                        tft180_show_string(16, 32, "Origin Set!");
                        system_delay_ms(800);
                    }
                    tft180_clear();
                    break;
                }

                /* -------- CORR 子菜单 -------- */
                case CORR:{
                    if (num == get_menu_item_count() - 1)       /* BACK */
                    {
                        interface = start;
                        num = 3;
                    }
                    else
                    {
                        tft180_clear();
                        tft180_show_string(16, 32, "Correct Set!");
                        system_delay_ms(800);
                    }
                    tft180_clear();
                    break;
                }

                default:break;
            }
        }

        /* ---- 左键（退出修改状态） ---- */
        if (key_left)
        {
            key_left = 0;
        }

        system_delay_ms(20);
    }
}
