#include "menu.h"
#include "zf_common_headfile.h"

/*===========================================================================
 * Flash 存储地址定义
 *===========================================================================*/
#define FLASH_SECTION_INDEX     (127)           /* 倒数第一个扇区 */
#define FLASH_PAGE_INDEX        (FLASH_PAGE_3)  /* 倒数第一个页码 */

/*===========================================================================
 * 全局变量 —— PID 参数
 *===========================================================================*/
float   pid_x_p     =  2.0;
float   pid_x_i     =  0.005;
float   pid_x_d     =  5.0;
uint8_t pid_x_speed =  80;
float   pid_y_p     = -2.0;
float   pid_y_i     = -0.005;
float   pid_y_d     = -5.0;
uint8_t pid_y_speed =  80;

/*===========================================================================
 * 全局变量 —— 原点参数
 *===========================================================================*/
uint8_t origin_x_enc = 2;
uint8_t origin_y_enc = 6;

/*===========================================================================
 * 全局变量 —— 修正值
 *===========================================================================*/
float  corr_x_enc = 0.75;
float  corr_y_enc = 0.73;
int8_t corr_x_cam = 0;
int8_t corr_y_cam = 0;
float   corr_yaw   = 0.00025;

/*===========================================================================
 * 全局变量 —— 加减速控制
 *===========================================================================*/
float x_acc = 0.02;
float x_dec = 0.05;
float y_acc = 0.02;
float y_dec = 0.05;

/*===========================================================================
 * 按键宏定义 (key[0]=左, key[1]=右, key[2]=上, key[3]=下)
 *===========================================================================*/
extern struct keys key[4];
#define KEY_UP      (key[2].single_flag)
#define KEY_DOWN    (key[3].single_flag)
#define KEY_LEFT    (key[0].single_flag)
#define KEY_RIGHT   (key[1].single_flag)

/*===========================================================================
 * 菜单状态枚举
 *===========================================================================*/
typedef enum {
    MENU_START,
    MENU_PID,
    MENU_ORIGIN,
    MENU_CORR,
    MENU_CONTROL,
    MENU_DATASAVE
} menu_state_t;

/*===========================================================================
 * 静态变量
 *===========================================================================*/
static menu_state_t current_menu = MENU_START;  /* 当前菜单界面       */
static int8_t       cursor       = 0;           /* 光标位置           */
static int8_t       edit_mode    = 0;           /* 0=导航, 1=编辑模式  */

/*===========================================================================
 * 各数据项编辑步长
 *===========================================================================*/
#define STEP_PID_P       0.1f
#define STEP_PID_I       0.001f
#define STEP_PID_D       0.1f
#define STEP_PID_SPEED   1
#define STEP_ORIGIN_ENC  1
#define STEP_CORR_YAW  0.00001f
#define STEP_CORR_ENC    0.01f
#define STEP_CORR_CAM    1
#define STEP_CONTROL     0.01f


/*===========================================================================
 * apply_edit —— 对当前选中的数据项进行加减操作
 *===========================================================================*/
static void apply_edit(menu_state_t menu, int8_t item, bool increment)
{
    switch (menu) {

        /* -------- PID 参数 -------- */
        case MENU_PID:
            switch (item) {
                case 0: if (increment) pid_x_p += STEP_PID_P;
                        else            pid_x_p -= STEP_PID_P;   break;
                case 1: if (increment) pid_x_i += STEP_PID_I;
                        else            pid_x_i -= STEP_PID_I;   break;
                case 2: if (increment) pid_x_d += STEP_PID_D;
                        else            pid_x_d -= STEP_PID_D;   break;
                case 3: if (increment) { if (pid_x_speed < 255) pid_x_speed += STEP_PID_SPEED; }
                        else            { if (pid_x_speed > 0)   pid_x_speed -= STEP_PID_SPEED; } break;
                case 4: if (increment) pid_y_p += STEP_PID_P;
                        else            pid_y_p -= STEP_PID_P;   break;
                case 5: if (increment) pid_y_i += STEP_PID_I;
                        else            pid_y_i -= STEP_PID_I;   break;
                case 6: if (increment) pid_y_d += STEP_PID_D;
                        else            pid_y_d -= STEP_PID_D;   break;
                case 7: if (increment) { if (pid_y_speed < 255) pid_y_speed += STEP_PID_SPEED; }
                        else            { if (pid_y_speed > 0)   pid_y_speed -= STEP_PID_SPEED; } break;
                default: break;
            }
            break;

        /* -------- 原点参数 -------- */
        case MENU_ORIGIN:
            switch (item) {
                case 0: if (increment) { if (origin_x_enc < 255) origin_x_enc += STEP_ORIGIN_ENC; }
                        else            { if (origin_x_enc > 0)   origin_x_enc -= STEP_ORIGIN_ENC; } break;
                case 1: if (increment) { if (origin_y_enc < 255) origin_y_enc += STEP_ORIGIN_ENC; }
                        else            { if (origin_y_enc > 0)   origin_y_enc -= STEP_ORIGIN_ENC; } break;
                default: break;
            }
            break;

        /* -------- 修正值 -------- */
        case MENU_CORR:
            switch (item) {
                case 0: if (increment) corr_x_enc += STEP_CORR_ENC;
                        else            corr_x_enc -= STEP_CORR_ENC;
                        if (corr_x_enc < 0) corr_x_enc = 0;  break;
                case 1: if (increment) corr_y_enc += STEP_CORR_ENC;
                        else            corr_y_enc -= STEP_CORR_ENC;
                        if (corr_y_enc < 0) corr_y_enc = 0;  break;
                case 2: if (increment) { if (corr_x_cam < 127)  corr_x_cam += STEP_CORR_CAM; }
                        else            { if (corr_x_cam > -128) corr_x_cam -= STEP_CORR_CAM; } break;
                case 3: if (increment) { if (corr_y_cam < 127)  corr_y_cam += STEP_CORR_CAM; }
                        else            { if (corr_y_cam > -128) corr_y_cam -= STEP_CORR_CAM; } break;
                case 4: if (increment) corr_yaw += STEP_CORR_YAW;
                        else            corr_yaw -= STEP_CORR_YAW; break;
                default: break;
            }
            break;

        /* -------- 加减速控制 -------- */
        case MENU_CONTROL:
            switch (item) {
                case 0: if (increment) { if (x_acc < 255) x_acc += STEP_CONTROL; }
                        else            { if (x_acc > 0)   x_acc -= STEP_CONTROL; } break;
                case 1: if (increment) { if (x_dec < 255) x_dec += STEP_CONTROL; }
                        else            { if (x_dec > 0)   x_dec -= STEP_CONTROL; } break;
                case 2: if (increment) { if (y_acc < 255) y_acc += STEP_CONTROL; }
                        else            { if (y_acc > 0)   y_acc -= STEP_CONTROL; } break;
                case 3: if (increment) { if (y_dec < 255) y_dec += STEP_CONTROL; }
                        else            { if (y_dec > 0)   y_dec -= STEP_CONTROL; } break;
                default: break;
            }
            break;

        default: break;
    }
}


/*===========================================================================
 * get_menu_item_count —— 获取当前菜单的选项数量
 *===========================================================================*/
static int get_menu_item_count(void)
{
    switch (current_menu) {
        case MENU_START:    return 6;   /* start, PID, ORIGIN, CORR, control, datasave */
        case MENU_PID:      return 9;   /* 8 数据项 + BACK                            */
        case MENU_ORIGIN:   return 3;   /* 2 数据项 + BACK                            */
        case MENU_CORR:     return 6;   /* 5 数据项 + BACK                            */
        case MENU_CONTROL:  return 5;   /* 4 数据项 + BACK                            */
        default:            return 0;
    }
}


/*===========================================================================
 * data_init —— 从 Flash 读取所有参数
 *===========================================================================*/
void data_init(void)
{
    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);

    /* PID */
    pid_x_p     = flash_union_buffer[0].float_type;
    pid_x_i     = flash_union_buffer[1].float_type;
    pid_x_d     = flash_union_buffer[2].float_type;
    pid_x_speed = flash_union_buffer[3].uint8_type;
    pid_y_p     = flash_union_buffer[4].float_type;
    pid_y_i     = flash_union_buffer[5].float_type;
    pid_y_d     = flash_union_buffer[6].float_type;
    pid_y_speed = flash_union_buffer[7].uint8_type;

    /* 原点 */
    origin_x_enc = flash_union_buffer[13].uint8_type;
    origin_y_enc = flash_union_buffer[14].uint8_type;
    corr_yaw   = flash_union_buffer[12].float_type;

    /* 修正值 */
    corr_x_enc = flash_union_buffer[8].float_type;
    corr_y_enc = flash_union_buffer[9].float_type;
    corr_x_cam = flash_union_buffer[10].int8_type;
    corr_y_cam = flash_union_buffer[11].int8_type;

    /* 加减速控制 */
    x_acc = flash_union_buffer[15].float_type;
    x_dec = flash_union_buffer[16].float_type;
    y_acc = flash_union_buffer[17].float_type;
    y_dec = flash_union_buffer[18].float_type;
}


/*===========================================================================
 * data_save —— 将所有参数保存到 Flash
 *===========================================================================*/
static void data_save(void)
{
    tft180_clear();
    tft180_show_string(16, 32, "Saving...");

    flash_buffer_clear();

    /* PID */
    flash_union_buffer[0].float_type  = pid_x_p;
    flash_union_buffer[1].float_type  = pid_x_i;
    flash_union_buffer[2].float_type  = pid_x_d;
    flash_union_buffer[3].uint8_type  = pid_x_speed;
    flash_union_buffer[4].float_type  = pid_y_p;
    flash_union_buffer[5].float_type  = pid_y_i;
    flash_union_buffer[6].float_type  = pid_y_d;
    flash_union_buffer[7].uint8_type  = pid_y_speed;

    /* 原点 */
    flash_union_buffer[13].uint8_type = origin_x_enc;
    flash_union_buffer[14].uint8_type = origin_y_enc;
    flash_union_buffer[12].float_type = corr_yaw;

    /* 修正值 */
    flash_union_buffer[8].float_type  = corr_x_enc;
    flash_union_buffer[9].float_type  = corr_y_enc;
    flash_union_buffer[10].int8_type  = corr_x_cam;
    flash_union_buffer[11].int8_type  = corr_y_cam;

    /* 加减速控制 */
    flash_union_buffer[15].float_type = x_acc;
    flash_union_buffer[16].float_type = x_dec;
    flash_union_buffer[17].float_type = y_acc;
    flash_union_buffer[18].float_type = y_dec;

    flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
    system_delay_ms(500);
}


/*===========================================================================
 * show_menu —— 绘制当前菜单界面
 *===========================================================================*/
static void show_menu(void)
{
    switch (current_menu) {

        /* ======== 主菜单 ======== */
        case MENU_START:
            tft180_show_string(0,  cursor * 16, edit_mode ? ">>" : "->");
            tft180_show_string(16, 0,  "start");
            tft180_show_string(16, 16, "PID");
            tft180_show_string(16, 32, "ORIGIN");
            tft180_show_string(16, 48, "CORR");
            tft180_show_string(16, 64, "control");
            tft180_show_string(16, 80, "datasave");
            break;

        /* ======== PID 子菜单 ======== */
        case MENU_PID:
            tft180_show_string(0, 0, "PID Setting");
            tft180_show_string(0,  16 + cursor * 16, edit_mode ? ">>" : "->");
            tft180_show_string(16, 16,  "x_P");
            tft180_show_string(16, 32,  "x_I");
            tft180_show_string(16, 48,  "x_D");
            tft180_show_string(16, 64,  "x_spd");
            tft180_show_string(16, 80,  "y_P");
            tft180_show_string(16, 96,  "y_I");
            tft180_show_string(16, 112, "y_D");
            tft180_show_string(16, 128, "y_spd");
            tft180_show_string(16, 144, "BACK");
            tft180_show_float(72, 16,  pid_x_p,     2, 2);
            tft180_show_float(72, 32,  pid_x_i,     1, 4);
            tft180_show_float(72, 48,  pid_x_d,     2, 2);
            tft180_show_int  (72, 64,  pid_x_speed, 3);
            tft180_show_float(64, 80,  pid_y_p,     2, 2);
            tft180_show_float(64, 96,  pid_y_i,     1, 4);
            tft180_show_float(64, 112, pid_y_d,     2, 2);
            tft180_show_int  (72, 128, pid_y_speed, 3);
            break;

        /* ======== ORIGIN 子菜单 ======== */
        case MENU_ORIGIN:
            tft180_show_string(0, 0, "Origin Setting");
            tft180_show_string(0,  16 + cursor * 16, edit_mode ? ">>" : "->");
            tft180_show_string(16, 16, "X_enc");
            tft180_show_string(16, 32, "Y_enc");
            tft180_show_string(16, 48, "BACK");
            tft180_show_int  (64, 16, origin_x_enc, 3);
            tft180_show_int  (64, 32, origin_y_enc, 3);
            break;

        /* ======== CORR 子菜单 ======== */
        case MENU_CORR:
            tft180_show_string(0, 0, "Correct Setting");
            tft180_show_string(0,  16 + cursor * 16, edit_mode ? ">>" : "->");
            tft180_show_string(16, 16, "X_enc");
            tft180_show_string(16, 32, "Y_enc");
            tft180_show_string(16, 48, "X_cam");
            tft180_show_string(16, 64, "Y_cam");
            tft180_show_string(16, 80, "YAW");
            tft180_show_string(16, 96, "BACK");
            tft180_show_float(64, 16, corr_x_enc, 1, 3);
            tft180_show_float(64, 32, corr_y_enc, 1, 3);
            tft180_show_int  (64, 48, corr_x_cam, 3);
            tft180_show_int  (64, 64, corr_y_cam, 3);
            tft180_show_float(56, 80, corr_yaw,   1, 6);
            break;

        /* ======== CONTROL 子菜单 ======== */
        case MENU_CONTROL:
            tft180_show_string(0, 0, "Control Setting");
            tft180_show_string(0,  16 + cursor * 16, edit_mode ? ">>" : "->");
            tft180_show_string(16, 16, "X_acc");
            tft180_show_string(16, 32, "X_dec");
            tft180_show_string(16, 48, "Y_acc");
            tft180_show_string(16, 64, "Y_dec");
            tft180_show_string(16, 80, "BACK");
            tft180_show_float(72, 16, x_acc, 1, 3);
            tft180_show_float(72, 32, x_dec, 1, 3);
            tft180_show_float(72, 48, y_acc, 1, 3);
            tft180_show_float(72, 64, y_dec, 1, 3);
            break;

        default: break;
    }
}


/*===========================================================================
 * handle_navigate —— 处理导航模式按键
 *===========================================================================*/
static bool handle_navigate(void)
{
    /* ---- 上键：光标上移 ---- */
    if (KEY_UP) {
        cursor++;
        if (cursor > get_menu_item_count() - 1) cursor = 0;
        KEY_UP = 0;
        tft180_clear();
    }

    /* ---- 下键：光标下移 ---- */
    if (KEY_DOWN) {
        cursor--;
        if (cursor < 0) cursor = get_menu_item_count() - 1;
        KEY_DOWN = 0;
        tft180_clear();
    }

    /* ---- 右键：确认 / 进入子菜单 / 进入编辑 ---- */
    if (KEY_RIGHT) {
        KEY_RIGHT = 0;

        switch (current_menu) {

            case MENU_START:
                if (cursor == 0) {
                    tft180_clear();
                    tft180_show_string(16, 32, "RUN!!!");
                    system_delay_ms(500);
                    return false;                   /* 退出菜单，开始运行 */
                } else if (cursor == 1) {
                    current_menu = MENU_PID;
                } else if (cursor == 2) {
                    current_menu = MENU_ORIGIN;
                } else if (cursor == 3) {
                    current_menu = MENU_CORR;
                } else if (cursor == 4) {
                    current_menu = MENU_CONTROL;
                } else if (cursor == 5) {
                    data_save();
                }
                cursor = 0;
                tft180_clear();
                break;

            case MENU_PID:
                if (cursor == get_menu_item_count() - 1) {
                    current_menu = MENU_START;      /* BACK */
                    cursor = 1;
                } else {
                    edit_mode = 1;                  /* 进入编辑模式 */
                }
                tft180_clear();
                break;

            case MENU_ORIGIN:
                if (cursor == get_menu_item_count() - 1) {
                    current_menu = MENU_START;      /* BACK */
                    cursor = 2;
                } else {
                    edit_mode = 1;                  /* 进入编辑模式 */
                }
                tft180_clear();
                break;

            case MENU_CORR:
                if (cursor == get_menu_item_count() - 1) {
                    current_menu = MENU_START;      /* BACK */
                    cursor = 3;
                } else {
                    edit_mode = 1;                  /* 进入编辑模式 */
                }
                tft180_clear();
                break;
            case MENU_CONTROL:
                if (cursor == get_menu_item_count() - 1) {
                    current_menu = MENU_START;      /* BACK */
                    cursor = 4;
                } else {
                    edit_mode = 1;
                }
                tft180_clear();
                break;
            default: break;
        }
    }

    /* ---- 左键 ---- */
    if (KEY_LEFT) {
        KEY_LEFT = 0;
    }

    return true;    /* 继续运行 */
}


/*===========================================================================
 * handle_edit —— 处理编辑模式按键
 *===========================================================================*/
static void handle_edit(void)
{
    /* ---- 上键：数据增加 ---- */
    if (KEY_DOWN) {
        apply_edit(current_menu, cursor, true);
        KEY_DOWN = 0;
        tft180_clear();
    }

    /* ---- 下键：数据减少 ---- */
    if (KEY_UP) {
        apply_edit(current_menu, cursor, false);
        KEY_UP = 0;
        tft180_clear();
    }

    /* ---- 左键：退出编辑，返回导航模式 ---- */
    if (KEY_LEFT) {
        edit_mode = 0;
        KEY_LEFT = 0;
        tft180_clear();
    }

    /* ---- 右键：编辑模式下忽略 ---- */
    if (KEY_RIGHT) {
        KEY_RIGHT = 0;
    }
}


/*===========================================================================
 * menu —— 菜单主循环
 *===========================================================================*/
void menu(void)
{
    cursor       = 0;
    edit_mode    = 0;
    current_menu = MENU_START;

    data_init();

    while (1) {
        key_read();
        show_menu();

        if (edit_mode == 0) {
            if (!handle_navigate()) break;
        } else {
            handle_edit();
        }

        system_delay_ms(20);
    }
}
