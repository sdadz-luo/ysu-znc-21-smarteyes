#include "menu.h"
#include "zf_common_headfile.h"

#define FLASH_SECTION_INDEX       (127)                                         // 存储数据用的扇区 倒数第一个扇区
#define FLASH_PAGE_INDEX          (FLASH_PAGE_3)                                // 存储数据用的页码 倒数第一个页码

float pid_x_p = 2.0, pid_x_i = 0.005, pid_x_d = 5.0;
float pid_y_p = -2.0, pid_y_i = -0.005, pid_y_d = -5.0;
uint8_t pid_x_speed = 80, pid_y_speed = 80;
float origin_x_enc = 0.75, origin_y_enc = 0.73, origin_yaw = 0.00025;
uint8_t origin_x_cam = 0, origin_y_cam = 0;
uint8_t corr_x_enc = 2, corr_y_enc = 6;

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
        case ORIGIN:  return 6; /* ORIGIN */
        case CORR:  return 3;   /* CORR */
        default: return 0;
    }
}

/* 数据初始化*/
void data_init(void){ 
    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX); 
    pid_x_p = flash_union_buffer[0].float_type;
    pid_x_i = flash_union_buffer[1].float_type;
    pid_x_d = flash_union_buffer[2].float_type;
    pid_x_speed = flash_union_buffer[3].uint8_type;
    pid_y_p = flash_union_buffer[4].float_type;
    pid_y_i = flash_union_buffer[5].float_type;
    pid_y_d = flash_union_buffer[6].float_type;
    pid_y_speed = flash_union_buffer[7].uint8_type;
    origin_x_enc = flash_union_buffer[8].float_type;
    origin_y_enc = flash_union_buffer[9].float_type;
    origin_x_cam = flash_union_buffer[10].uint8_type;
    origin_y_cam = flash_union_buffer[11].uint8_type;
    origin_yaw = flash_union_buffer[12].float_type;
    corr_x_enc = flash_union_buffer[13].uint8_type;
    corr_y_enc = flash_union_buffer[14].uint8_type;
}

/* 数据保存 */
static void Data_save(void){
    tft180_clear();
    tft180_show_string(16, 32, "Saving...");
    flash_buffer_clear();
    flash_union_buffer[0].float_type = pid_x_p;
    flash_union_buffer[1].float_type = pid_x_i;
    flash_union_buffer[2].float_type = pid_x_d;
    flash_union_buffer[3].uint8_type = pid_x_speed;
    flash_union_buffer[4].float_type = pid_y_p;
    flash_union_buffer[5].float_type = pid_y_i;
    flash_union_buffer[6].float_type = pid_y_d;
    flash_union_buffer[7].uint8_type = pid_y_speed;
    flash_union_buffer[8].float_type = origin_x_enc;
    flash_union_buffer[9].float_type = origin_y_enc;
    flash_union_buffer[10].uint8_type = origin_x_cam;
    flash_union_buffer[11].uint8_type = origin_y_cam;
    flash_union_buffer[12].float_type = origin_yaw;
    flash_union_buffer[13].uint8_type = corr_x_enc;
    flash_union_buffer[14].uint8_type = corr_y_enc;
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
    system_delay_ms(500);  
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
            tft180_show_float(80,16,pid_x_p,2,2);
            tft180_show_float(80,32,pid_x_i,1,3);
            tft180_show_float(80,48,pid_x_d,2,2);
            tft180_show_int(80,64,pid_x_speed,3);
            tft180_show_float(72,80,pid_y_p,2,2);
            tft180_show_float(72,96,pid_y_i,1,3);
            tft180_show_float(72,112,pid_y_d,2,2);
            tft180_show_int(80,128,pid_y_speed,3);
            break;
        }

        /* ========== ORIGIN 子菜单 ========== */
        case ORIGIN:{
            tft180_show_string(0, 0,  "Origin Setting");
            tft180_show_string(0,  16 + num * 16, "->");
            tft180_show_string(16, 16, "X_enc");
            tft180_show_string(16, 32, "Y_enc");
            tft180_show_string(16, 48, "X_cam");
            tft180_show_string(16, 64, "Y_cam");
            tft180_show_string(16, 80, "YAW");
            tft180_show_string(16, 96, "BACK");
            tft180_show_float(64,16, origin_x_enc, 1, 2);
            tft180_show_float(64,32, origin_y_enc, 1, 2);
            tft180_show_int(64,48, origin_x_cam, 2);
            tft180_show_int(64,64, origin_y_cam, 1);
            tft180_show_float(64,80, origin_yaw, 1, 5);
            break;
        }

        /* ========== CORR 子菜单 ========== */
        case CORR:{
            tft180_show_string(0, 0,  "Correct Setting");
            tft180_show_string(0,  16 + num * 16, "->");
            tft180_show_string(16, 16, "X_enc");
            tft180_show_string(16, 32, "Y_enc");
            tft180_show_string(16, 48, "BACK");
            tft180_show_int(64,16, corr_x_enc, 2);
            tft180_show_int(64,32, corr_y_enc, 2);
            break;
        }

        default:break;
    }
}

void menu(void){
    num = 0;
    interface = start;

    data_init(); /* 初始化数据 */

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
