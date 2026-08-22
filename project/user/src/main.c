#include "zf_common_headfile.h"

#define PIT_CH                  (PIT_CH0 )                                      // PIT通道选择 注意：中断服务函数请在 isr.c 中编写
#define PIT_PRIORITY            (PIT_IRQn)                                      // PIT中断优先级 

#define x_cam_uint                   16.2
#define y_cam_uint                   15.8
#define x_enc_uint					 20
#define y_enc_uint					 20
#define END_ROTATION_SPEED_LIMIT     45.0f
#define END_ROTATION_ANGLE           360.0f

// ================= 状态机定义 =================
typedef enum {
    NoGame,     // 空闲/未开始
    GameMap,    // 地图识别/定位初始化
    Waiting,    // 等待中 (路径规划完成等待执行)
    Look,       // 观察/识别ID (摄像头识别车位ID)
    Run,        // 运动状态 (执行路径跟踪)
    GameOver,   // 游戏结束/停车入位
    End         // 完全结束
} Car_State;

static volatile Car_State car_state = NoGame;   // 初始状态
// ================= 路径结构体 =================
static Path path_car = {0};          		    // 正常行驶路径
static Path_Look path_look_car = {0}; 		    // 观察路径
static Path_Start path_start_car = {0}; 	    // 起始路径
static Path path_boom_car = {0};			    // 炸弹破局路径

// ================= 函数声明 =================
static void Init(void);
static void state_judgment(void);
static void path_process(void);
static void uart_updata(void);
static void cam2_process(void);

// ================= 外部变量声明  =================
extern float encoder_data_FL, encoder_data_FR, encoder_data_BL, encoder_data_BR; 	// 四个轮子编码器数据
extern volatile float yaw;            												// 小车当前角度 (航向角)
extern volatile float yaw_continuous;  												// 解包后的连续偏航角
extern pid pid_FL, pid_FR, pid_BL, pid_BR; 										    // 四个轮子的速度环 PID
extern pid pid_gyro,pid_yaw,pid_x, pid_y;          							        // 角速度环，角度环, X/Y位置环 PID

// ================= 全局变量 =================
static uint32_t time = 0;             											// 时间计数器 (每 10ms 加1)
static float vx = 0, vy = 0, vz = 0; 											// 底盘合成速度
static volatile float x_target = 10, y_target = 130; 							// 目标位置坐标
static float x_cam = 0, y_cam = 0, x_cam_last = 0, y_cam_last = 0; 				// 摄像头检测到的车位坐标及上次值
static uint8_t has_2 = 0, has_3 = 0, has_6 = 0;
static uint8_t map_has_car = 0;
static volatile float yaw_target = 0; 						                	// 目标角度
static volatile float end_rotation_target = 0.0f;
static volatile uint8_t end_rotation_started = 0;
static volatile uint8_t end_rotation_completed = 0;
static int FL = 0, FR = 0, BL = 0, BR = 0; 										// 四个电机的 PWM 占空比
static int step = 0;             												// 路径跟踪的当前步数
static volatile CAMDATA car_data;         										// 摄像头数据结构体
static uint8_t type = 0;             										    // 车位类型
static int id = -1;          												    // 识别到的 ID

// 流程控制标志
static uint8_t map_process_flag = 1; // 地图处理标志
static uint8_t game_count = 0;       // 游戏次数计数
static uint8_t game_over_flag = 0;   // 游戏结束标志
static uint8_t game_mode = 0;        // 游戏模式: 0=Start, 1=Normal Run, 2=Look, 3=ID Run  
static uint8_t look_flag = 0;        // 观察模式标志
static uint8_t id_flag = 0;          // 识别模式标志
static uint8_t boom_flag = 0;        // 炸弹破局标志
static uint8_t turn_mode_flag = 0;   // 模式转换标志
static uint8_t grid[MAP_ROWS][MAP_COLS] = {0};

int main(void){
    clock_init(SYSTEM_CLOCK_600M);  // 初始化系统时钟为 600MHz
    debug_init();                   // 初始化调试串口
    system_delay_ms(300);           // 延时等待稳定
    
    Init();                         // 硬件初始化
    
    interrupt_global_enable(0);     // 开启全局中断

	while(1){ 

        // 状态机判断与状态转换处理
		state_judgment();

	}
}

/**
 * @brief PIT 中断服务函数 (周期性执行)
 * @note PIT_CH0: 5ms 执行一次运动控制
 * @note PIT_CH1: 10ms 执行一次数据接收
 */
void PIT_IRQHandler(void){
	
    // --- PIT Channel 0: 运动控制周期 (5ms) ---
    if(pit_flag_get(PIT_CH0)){
        time++;
        // 1. 获取传感器数据
        encoder_get(); // 读取编码器数据
        imu_get();     // 读取 IMU 数据
        distance(yaw); // 航迹推算

        // 2. 位置环 PID 计算
        if (time % 2 == 0 && (car_state == Run || car_state == GameOver)){
            pid_position_target(x_target, y_target); // 更新位置环目标值
            // 计算 X 和 Y 方向的速度输出
            vx = pid_location(&pid_x, x_enc);
            vy = pid_location(&pid_y, y_enc);
        }

        // 3. 角度环 PID 计算 
        if (time % 2 == 0){
            if (car_state == End && end_rotation_started && !end_rotation_completed){
                pid_change(&pid_yaw, 5, 0.001f, 30);
                pid_target(&pid_yaw, end_rotation_target);
                vz = pid_location(&pid_yaw, yaw_continuous);
                if (vz > END_ROTATION_SPEED_LIMIT) vz = END_ROTATION_SPEED_LIMIT;
                else if (vz < -END_ROTATION_SPEED_LIMIT) vz = -END_ROTATION_SPEED_LIMIT;

                if (yaw_continuous >= end_rotation_target){
                    end_rotation_completed = 1;
                    vz = 0.0f;
                }
            } else if (car_state != End){
                if (car_state == Run) pid_change(&pid_yaw, 10, 0.001f, 30);
                else pid_change(&pid_yaw, 5, 0.001f, 30);

                float yaw_error = yaw_target - yaw;
                if (yaw_error > 180.0f)       yaw_error -= 360.0f;
                else if (yaw_error < -180.0f) yaw_error += 360.0f;

                pid_target(&pid_yaw, yaw + yaw_error);
                vz = pid_location(&pid_yaw, yaw); 
            } else {
                vz = 0.0f;
            }
        } 
        
        // 4. 运动学解算: 将合成速度 (vx, vy, vz) 分解到四个轮子
        motor_solution(vx, vy, yaw, vz);

        // 5. 速度环/增量式 PID 计算得到最终 PWM
        if (car_state == End && end_rotation_completed){
            pid_wheel_target(0, 0, 0, 0);
            motor_duty(0, 0, 0, 0);
        } else {
            FL = pid_increm(&pid_FL, encoder_data_FL / 4.0f);
            FR = pid_increm(&pid_FR, encoder_data_FR / 4.0f);
            BL = pid_increm(&pid_BL, encoder_data_BL / 4.0f);
            BR = pid_increm(&pid_BR, encoder_data_BR / 4.0f);

            motor_duty(FL, FR, BL, BR); // 输出电机PWM
        }
        
        pit_flag_clear(PIT_CH0);
    }
    
    // --- PIT Channel 1: 数据接收周期 (10ms) ---
    if(pit_flag_get(PIT_CH1)){
        car_data = cam_uart1_read(); 	// 读取摄像头1数据
        uart_updata();          		// 处理串口数据
        pit_flag_clear(PIT_CH1);
    }
	__DSB();
}


static void Init(void){
    tft_init();
    tft180_clear();
    my_key_init();
    flash_init();
    menu();                                 // 进入菜单界面选择模式
    system_delay_ms(1000);
    tft180_show_string(0, 0, "Init... ");
    encoder_init(); 						// 编码器初始化
    imu_init();     						// IMU 初始化
    // my_uart_init(); 						// 串口初始化
    motor_init();   						// 电机 PWM 初始化
    pid_init();     						// PID 参数初始化
    cam_uart_init();						// 摄像头串口初始化
    cam1_uart_send(0);
    reset_planning_system();
    // 初始化PIT定时器
    pit_ms_init(PIT_CH, 5);     // CH0: 5ms
    pit_ms_init(PIT_CH1, 10);   // CH1: 10ms
	tft180_clear();
}

/**
 * @brief 在摄像头栅格数据中查找指定数值
 * @return 1 找到, 0 未找到
 */
static uint8_t find_in_grid(const volatile CAMDATA *data, uint8_t val1, uint8_t val2){
    for (uint8_t row = 0; row < 12; row++) {
        for (uint8_t col = 0; col < 16; col++) {
            uint8_t v = data->grid[row][col];
            if (v == val1 || v == val2) {
                return 1;
            }
        }
    }
    return 0;
}

//状态机判断与转换
static void state_judgment(void){
    switch (car_state){

        case NoGame:{
            // 检测是否进入开始区域
            if (x_cam >= 0 && x_cam <= 2.5 * x_cam_uint && y_cam >= 5 * y_cam_uint && y_cam <= 7 * y_cam_uint){
                system_delay_ms(300);
                // 根据摄像头坐标初始化车辆位置
                x_enc = (x_cam / x_cam_uint) * x_enc_uint;
                y_enc = (y_cam / y_cam_uint) * y_enc_uint;
                car_state = GameMap;
                return;
            }
            break;
        }
        
        case GameMap:{
            if (x_cam >= 0 * x_cam_uint && x_cam <= 2.5 * x_cam_uint && y_cam >= 5 * y_cam_uint && y_cam <= 7 * y_cam_uint){
                // 设置初始目标位置
                x_target = (origin_x_enc + 0.5) * x_enc_uint;
                y_target = (origin_y_enc + 0.5) * y_enc_uint;
            }
            car_state = Run;
            return;
        }
        
        case Look:{
            vx = 0;vy = 0;
            // 在 Look 状态下进行摄像头识别处理
            cam2_process();
            break;
        }

        case Run:{
            // 判断是否到达目标点，到达则转入 Waiting 状态等待下一步
            if (fabsf(x_enc - x_target) <= 1 && fabsf(y_enc - y_target) <= 1){
                car_state = Waiting;
                return;
            }
            break;
        }
        
        case Waiting:{
            vx = 0;vy = 0;
            path_process(); // 路径规划与跟踪处理
            break;
        }
        
        case GameOver:{
            if (game_over_flag){
                // 停车入位: 到达停车位
                if (fabsf(x_enc - x_target) <= 2 && fabsf(y_enc - y_target) <= 2){
                    vx = 0;vy = 0;
                    system_delay_ms(500);
                    if (find_in_grid(&car_data, 3, 6)) system_delay_ms(3000);
                    game_over_flag = 0;
                }
            }else{
                vx = 0;vy = 0;
                game_count++;
                if (game_count >= 3) {
                    car_state = End; // 3次后完全结束
                    return;
                } else {
                    // 重置系统
                    reset_planning_system();
                    pid_reset();
                    game_over_flag = 1;
                    game_mode = 0;
                    map_process_flag = 1;
                    step = 0;
					memset(&path_start_car, 0, sizeof(path_start_car));
					memset(&path_look_car, 0, sizeof(path_look_car));
                    memset(&path_car, 0, sizeof(path_car));
                    memset(&path_boom_car, 0, sizeof(path_boom_car));
                    car_state = NoGame; // 回到初始状态
                    return;
                } 
            }
            break;
        }
        
        case End:{
            vx = 0;vy = 0;
            if (!end_rotation_started){
                end_rotation_target = yaw_continuous + END_ROTATION_ANGLE;
                end_rotation_started = 1;
                end_rotation_completed = 0;
                pid_reset();
            }
            if (end_rotation_completed) vz = 0;
            break;
        }
    }
}

//串口数据更新处理
static void uart_updata(void){
    // 获取摄像头检测到的车位中心坐标
    x_cam = car_data.car_cx - corr_x_cam;
    y_cam = car_data.car_cy - corr_y_cam;        // 减去摄像头偏移量

    // 对摄像头数据进行低通滤波平滑处理
    x_cam = x_cam * 0.8 + x_cam_last * 0.2;
    y_cam = y_cam * 0.8 + y_cam_last * 0.2;

    x_cam_last = x_cam;
    y_cam_last = y_cam;
}

/**
 * @brief 模式1和模式3的公共路径跟踪逻辑
 * @param path  路径数据指针
 * @param end_x 终点栅格X坐标
 * @param end_y 终点栅格Y坐标
 */
static void process_normal_path(const Path *path){
    static float end_x = 1, end_y = 6;
    if (fabsf(x_enc - x_target) <= 1 && fabsf(y_enc - y_target) <= 1) {
        step++;
        if (step > path->len) {
            x_target = end_x * x_enc_uint;
            y_target = end_y * y_enc_uint;
            pid_position_speed();
            game_over_flag = 1;
            car_state = GameOver;
            return;
        }else{
            x_target = (path->x[step] + 0.5f) * x_enc_uint;
            y_target = (path->y[step] + 0.5f) * y_enc_uint;
            car_state = Run;
            return;
        }
    }
}

//路径规划与跟踪处理
static void path_process(void){
    // 1. 扫描地图数据 (检测是否有车位2, 障碍3, 车辆6)
    if (map_process_flag == 1) {
        system_delay_ms(600);
        if (game_count == 0) game_mode = 1;
        else game_mode = 2;
        for (uint8_t row = 0; row < 12; row++) {
            for (uint8_t col = 0; col < 16; col++) {
                uint8_t val = car_data.grid[row][col];
                if (val == 2) has_2 = 1;
                if (val == 3) has_3 = 1;
                if (val == 6) has_6 = 1;
                if (val == 7) game_mode = 4; // 特殊标志位7表示炸弹破局模式
            } 
        } 
        map_has_car = has_2 && has_3 && has_6;
        if (map_has_car) memcpy(grid,(const void *)car_data.grid,sizeof(grid));
    }
    // 2. 根据当前游戏模式调用相应的路径规划算法
    if ((map_process_flag && map_has_car) || boom_flag == 1 || look_flag == 1 || id_flag == 1) {
        if (game_mode == 0) path_start_car = path_start_calculation(grid);      // 模式0: 起始路径规划
        else if (game_mode == 1) path_car = path_calculation(grid);     		// 模式1: 正常行驶
        else if (game_mode == 2) path_look_car = path_look_calculation(grid); 	// 模式2: ID识别路径
        else if (game_mode == 3) path_car = path_id_calculation(); 		        // 模式3: ID行驶
        else if (game_mode == 4) path_boom_car = path_boom_calculation(grid);   // 模式4: 炸弹破局路径
        map_process_flag = 0; 		
        boom_flag = 0;look_flag = 0;id_flag = 0;
        has_2 = 0;has_3 = 0;has_6 = 0;
        step = 0;             													// 重置路径步数
        if (game_mode == 4 && path_boom_car.len == 0){
            game_mode = 2;
            boom_flag = 1;
        }
        if (((game_mode == 1 || game_mode == 3) && path_car.len == 0) || (game_mode == 2 && path_look_car.len == 0)){
            x_target = 1 * x_enc_uint;
            y_target = 6 * y_enc_uint;
            pid_position_speed();
            game_over_flag = 1;
            car_state = GameOver;
            return;
        }
    }

    // 3. 路径跟踪执行 (根据不同模式)
    if (game_mode == 1 && !map_process_flag && path_car.len > 0){
        process_normal_path(&path_car);
        return;
    }else if (game_mode == 0 && !map_process_flag && path_start_car.len > 0){
        if (fabsf(x_enc - x_target) <= 1 && fabsf(y_enc - y_target) <= 1) {            
            step++;
            if (step >= path_start_car.len) {
                yaw_target = path_start_car.angle; // 设置目标角度
                type = path_start_car.type;
                grid[path_start_car.y[0]][path_start_car.x[0]] = 0;
                grid[path_start_car.y[path_start_car.len - 1]][path_start_car.x[path_start_car.len - 1]] = 2;
                look_flag = 1;
                // 起始路径走完，转入 Look 状态识别ID
                car_state = Look;
                return;
            }else{
                x_target = (path_start_car.x[step] + 0.5f) * x_enc_uint;
                y_target = (path_start_car.y[step] + 0.5f) * y_enc_uint;
                car_state = Run;
                return;
            }
        }
    }else if (game_mode == 2 && !map_process_flag && path_look_car.len > 0){
        if (fabsf(x_enc - x_target) <= 1 && fabsf(y_enc - y_target) <= 1) {
            if (turn_mode_flag){
                turn_mode_flag = 0;
                id_input(id);
                id = -1;
                step = 1;
                x_target = (path_look_car.x[step] + 0.5f) * x_enc_uint;
                y_target = (path_look_car.y[step] + 0.5f) * y_enc_uint;
                car_state = Run;
                return;
            }          
            if (path_look_car.is_look[step] == 1 && step < path_look_car.len){
                vx = 0;vy = 0;
                yaw_target = path_look_car.angle[step]; // 设置观察角度
                type = path_look_car.type[step];
                // 需要观察的点，转入 Look 状态
                car_state = Look;
                return;
            }else step++;
            if (step >= path_look_car.len) {
                game_mode = 3; // 切换到模式3
                id_flag = 1; // 设置ID识别标志
                step = 0;
                car_state = Waiting;
                return;
            }else{
                x_target = (path_look_car.x[step] + 0.5f) * x_enc_uint;
                y_target = (path_look_car.y[step] + 0.5f) * y_enc_uint;
                car_state = Run;
                return;
            }
        }
    }else if (game_mode == 3 && !map_process_flag && path_car.len > 0){
        process_normal_path(&path_car);
        return;
    }else if (game_mode == 4 && !map_process_flag && path_boom_car.len > 0){
        if (fabsf(x_enc - x_target) <= 1 && fabsf(y_enc - y_target) <= 1) {
            if (path_boom_car.is_push[step] == 1) system_delay_ms(1500); // 每颗炸弹引爆后等待爆炸完成
            step++;
            // 检查是否走完整个路径
            if (step >= path_boom_car.len) {
                map_boom_out(grid); 
                grid[origin_y_enc][origin_x_enc] = 0;
                grid[path_boom_car.y[path_boom_car.len - 1]][path_boom_car.x[path_boom_car.len - 1]] = 2;
                boom_flag = 1;
                game_mode = 2;
                car_state = Waiting;
                return;
            }else{
                // 路径点坐标转换为实际坐标 (+0.5 是为了定位到格子中心)
                x_target = (path_boom_car.x[step] + 0.5f) * x_enc_uint;
                y_target = (path_boom_car.y[step] + 0.5f) * y_enc_uint;
                car_state = Run;
                return;
            }
        }
    }
}

static void cam2_process(void){
    // 角度差归一化到 [-180, 180]，处理角度环绕。
    float yaw_diff = yaw_target - yaw;
    if (yaw_diff > 180.0f)       yaw_diff -= 360.0f;
    else if (yaw_diff < -180.0f) yaw_diff += 360.0f;
    if (fabsf(yaw_diff) > 1) return;

    if (game_mode == 0) {
        if (id == -1) {
            system_delay_ms(300);
            cam1_uart_send(yaw_target);
            if (yaw_target == 0) turn_mode_flag = 1;
            cam_uart2_write(type);
            while (id == -1) id = cam_uart2_read();
        } else {
            yaw_target = 0;
            if (yaw_target == 0 && fabsf(yaw) <= 1){
                cam1_uart_send(yaw_target); // 发送目标角度
                step = 0;
                if (id == 10){ 
                    game_mode = 1;      // 切换到正常行驶模式
                    id = -1;
                    car_state = Waiting;
                    return;
                }else{
                    game_mode = 2;      // 切换到ID识别模式
                    if (turn_mode_flag == 0) id = -1;
                    car_state = Waiting;
                    return;
                }
            }
        }
    } else if (game_mode == 2) {
        if (id == -1) {
            system_delay_ms(300);
            cam1_uart_send(yaw_target);
            cam_uart2_write(type);
            while (id == -1) id = cam_uart2_read();
        } else {
            yaw_target = 0;
            if (fabsf(yaw) <= 1) {
                cam1_uart_send(yaw_target);
                id_input(id);
                id = -1;
                if (step < path_look_car.len) {
                    step++;
                    if (step == path_look_car.len) {
                        car_state = Waiting;
                        return;
                    }
                    x_target = (path_look_car.x[step] + 0.5f) * x_enc_uint;
                    y_target = (path_look_car.y[step] + 0.5f) * y_enc_uint;
                    car_state = Run;
                }
            }
        }
    }
}
