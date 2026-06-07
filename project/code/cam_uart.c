#include "zf_common_headfile.h"
#include <string.h>

#define cam_uint					13.8
#define enc_uint					20

// 串口1定义（接收OpenMV数据）
#define UART1             (UART_3)
#define UART1_BAUDRATE    (115200)
#define UART_TX1          (UART3_TX_B22)
#define UART_RX1          (UART3_RX_B23)
#define UART1_PRIORITY    (LPUART3_IRQn)

// 串口2定义（用于调试回显）
#define UART2             (UART_1)
#define UART2_BAUDRATE    (115200)
#define UART_TX2          (UART1_TX_B12)
#define UART_RX2          (UART1_RX_B13)
#define UART2_PRIORITY    (LPUART1_IRQn)

static uint8 uart1_get_data[512];                   // 串口接收数据缓冲区
static uint8 fifo1_get_data[512];                   // fifo 输出读出缓冲区
static uint8 get1_data = 0;                         // 接收数据变量
static uint32 fifo1_data_count = 0;                 // fifo 数据个数
static fifo_struct uart1_data_fifo;
static uint8 uart1_rx_buf[512];
static uint16 uart1_rx_len = 0;

static uint8 uart2_get_data[512];                   // 串口接收数据缓冲区
static uint8 fifo2_get_data[512];                   // fifo 输出读出缓冲区
static uint8 get2_data = 0;                         // 接收数据变量
static uint32 fifo2_data_count = 0;                 // fifo 数据个数
static fifo_struct uart2_data_fifo;


static CAMDATA cam_uart_data;

uint8_t error_matrix[12][16][2] = {
    {{0,0},{0,0},{0,1},{0,1},{0,1},{0,1},{0,0},{0,1},{0,1},{0,1},{-1,1},{-1,2},{-1,2},{-1,2},{-1,2},{-1,2}},
    {{0,0},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,2},{0,2},{-1,2},{-1,2},{0,2}},
    {{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,1},{0,1},{0,1},{-1,1},{-1,1},{-1,1},{0,1}},
    {{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{-1,0},{-1,0},{-1,0}},
    {{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{-1,0},{-1,0},{-1,1}},
    {{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,1},{-1,1},{0,1},{0,1},{0,1},{-1,1},{-1,1}},
    {{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{-2,0},{-1,0},{0,1},{-1,1},{-1,0},{0,1},{0,1},{-1,1},{-1,1}},
    {{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{-1,0},{0,0},{0,0},{-1,0},{-1,0},{0,0},{-1,0},{-2,0},{-2,0}},
    {{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{-1,0},{-1,0},{-1,0},{-1,0},{-2,0},{-2,0}},
    {{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{-1,0},{-1,0},{-1,0},{-2,0},{-2,0},{-2,0}},
    {{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{-1,0},{-1,0},{-2,0},{-2,0},{-2,0},{-2,0}},
    {{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{-1,0},{-1,0},{-1,0},{-2,0},{-2,0},{-2,0},{-2,0},{-3,0}}
};

static int16 find_substring(const uint8 *buf, uint16 len, const uint8 *sub, uint16 sub_len) {
    for (uint16 i = 0; i <= len - sub_len; i++) {
        if (memcmp(&buf[i], sub, sub_len) == 0) {
            return i;
        }
    }
    return -1;
}

static char* my_strtok(char **str, const char *delim) {
    if (*str == NULL) return NULL;
    // 跳过前导分隔符
    char *start = *str;
    while (*start && strchr(delim, *start)) start++;
    if (*start == '\0') {
        *str = NULL;
        return NULL;
    }
    // 查找分隔符
    char *end = start;
    while (*end && !strchr(delim, *end)) end++;
    if (*end) {
        *end = '\0';
        *str = end + 1;
    } else {
        *str = NULL;
    }
    return start;
}


void push_cam_data(uint8_t *data) {
    memset(&cam_uart_data, 0, sizeof(CAMDATA));
    
    char *ptr = (char*)data;
    
    // 1. 解析小车坐标
    char *car_start = strstr(ptr, "car:");
    if (car_start) {
        car_start += 4;  // 跳过 "car:"
        // 提取到 ';' 或字符串结尾
        char *car_end = strchr(car_start, ';');
        if (!car_end) car_end = car_start + strlen(car_start);
        
        // 临时复制坐标部分
        char coord_buf[64] = {0};
        size_t len = car_end - car_start;
        if (len < sizeof(coord_buf) - 1) {
            memcpy(coord_buf, car_start, len);
            sscanf(coord_buf, "%f,%f", &cam_uart_data.car_cx, &cam_uart_data.car_cy);
        }
    }
    
    // 2. 解析地图数据
    char *map_start = strstr(ptr, "map:");
    if (!map_start) return;
    map_start += 4;  // 跳过 "map:"
    
    int row = 0, col = 0;
    // 遍历所有字符，只提取数字
    for (char *p = map_start; *p != '\0' && row < 12; p++) {
        if (*p >= '0' && *p <= '9') {
            cam_uart_data.grid[row][col] = *p - '0';
            col++;
            if (col >= 16) {
                col = 0;
                row++;
            }
        }
        // 其他所有字符（包括 '\n', '\r', '|', 空格等）均忽略
    }
}

// 初始化函数
void cam_uart_init(void)
{
    fifo_init(&uart1_data_fifo, FIFO_DATA_8BIT, uart1_get_data, 512);
    uart_init(UART1, UART1_BAUDRATE, UART_TX1, UART_RX1);
    uart_rx_interrupt(UART1, ZF_ENABLE);
    interrupt_set_priority(UART1_PRIORITY, 0);

    fifo_init(&uart2_data_fifo, FIFO_DATA_8BIT, uart2_get_data, 512);
    uart_init(UART2, UART2_BAUDRATE, UART_TX2, UART_RX2);
    uart_rx_interrupt(UART2, ZF_ENABLE);
    interrupt_set_priority(UART2_PRIORITY, 0);
}


// 中断服务函数
void cam_uart_isc_1(void)
{
    uart_query_byte(UART1, &get1_data);                                     // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
    fifo_write_buffer(&uart1_data_fifo, &get1_data, 1);                           // 将数据写入 fifo 中
}

void cam_uart_isc_2(void)
{
    uart_query_byte(UART2, &get2_data);                                     // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
    fifo_write_buffer(&uart2_data_fifo, &get2_data, 1);                           // 将数据写入 fifo 中
}


CAMDATA cam_uart1_read(void) {

    fifo1_data_count = fifo_used(&uart1_data_fifo);
    if (fifo1_data_count != 0) {
        if (uart1_rx_len + fifo1_data_count > sizeof(uart1_rx_buf)) uart1_rx_len = 0;

        fifo_read_buffer(&uart1_data_fifo, &uart1_rx_buf[uart1_rx_len], &fifo1_data_count, FIFO_READ_AND_CLEAN);
        uart1_rx_len += fifo1_data_count;
    }

    const uint8 frame_head[] = "start";
    const uint8 frame_tail[] = "end";
    const uint16 head_len = sizeof(frame_head) - 1;
    const uint16 tail_len = sizeof(frame_tail) - 1;

    int16 head_pos = find_substring(uart1_rx_buf, uart1_rx_len, frame_head, head_len);
    if (head_pos >= 0) {

        int16 tail_pos = find_substring(uart1_rx_buf + head_pos + head_len,
                                        uart1_rx_len - head_pos - head_len,
                                        frame_tail, tail_len);
        if (tail_pos >= 0) {

            uint16 frame_start = head_pos + head_len;           // 帧内容的起始位置
            uint16 frame_end   = head_pos + head_len + tail_pos; // 帧尾的起始位置
            uint16 frame_len   = frame_end - frame_start;       // 帧内容的长度


            uint8 frame_content[512]; 
            if (frame_len < sizeof(frame_content)) {
                memcpy(frame_content, &uart1_rx_buf[frame_start], frame_len);
                frame_content[frame_len] = '\0'; 

                // 调用解析函数
                push_cam_data(frame_content);
							
            }

            uint16 processed_len = frame_end + tail_len; 
            if (processed_len < uart1_rx_len) {
                memmove(uart1_rx_buf, &uart1_rx_buf[processed_len], uart1_rx_len - processed_len);
                uart1_rx_len -= processed_len;
            } else {
                uart1_rx_len = 0;
            }
        }
    }

    return cam_uart_data;
}

void cam1_uart_send(float angle){
    static uint8_t tx = 0;
    if (angle == 0.0f) tx = 1;
    else if (angle == 90.0f) tx = 2;
    else if (angle == -90.0f) tx = 3;
    else if (angle == 180.0f) tx = 4;
		else tx = 1;
    uint8_t tx_buf[32];
    int len = sprintf((char*)tx_buf, "start%02dend\r\n", tx);
    if (len > 0) uart_write_buffer(UART1, tx_buf, len);
}


int cam_uart2_read(void){
    static uint8 uart2_rx_buf[512];
    static uint16 uart2_rx_len = 0;
    
    fifo2_data_count = fifo_used(&uart2_data_fifo);
    if (fifo2_data_count != 0) {
        if (uart2_rx_len + fifo2_data_count > sizeof(uart2_rx_buf)) {
            uart2_rx_len = 0;
        }
        fifo_read_buffer(&uart2_data_fifo, &uart2_rx_buf[uart2_rx_len], &fifo2_data_count, FIFO_READ_AND_CLEAN);
        uart2_rx_len += fifo2_data_count;
        
        const uint8 frame_head[] = "start";
        const uint8 frame_tail[] = "end";
        const uint16 head_len = sizeof(frame_head) - 1;
        const uint16 tail_len = sizeof(frame_tail) - 1;
        
        int16 head_pos = find_substring(uart2_rx_buf, uart2_rx_len, frame_head, head_len);
        if (head_pos >= 0) {
            int16 tail_pos = find_substring(uart2_rx_buf + head_pos + head_len,
                                            uart2_rx_len - head_pos - head_len,
                                            frame_tail, tail_len);
            if (tail_pos >= 0) {
                uint16 frame_start = head_pos + head_len;
                uint16 frame_end = head_pos + head_len + tail_pos;
                uint16 frame_len = frame_end - frame_start;
                
                if (frame_len == 2) {
                    uint8 num_buf[3] = {0};
                    memcpy(num_buf, &uart2_rx_buf[frame_start], frame_len);
                    
                    int result = (num_buf[0] - '0') * 10 + (num_buf[1] - '0');
                    
                    uint16 processed_len = frame_end + tail_len;
                    if (processed_len < uart2_rx_len) {
                        memmove(uart2_rx_buf, &uart2_rx_buf[processed_len], uart2_rx_len - processed_len);
                        uart2_rx_len -= processed_len;
                    } else {
                        uart2_rx_len = 0;
                    }
                    return result;
                }
                
                uint16 processed_len = frame_end + tail_len;
                if (processed_len < uart2_rx_len) {
                    memmove(uart2_rx_buf, &uart2_rx_buf[processed_len], uart2_rx_len - processed_len);
                    uart2_rx_len -= processed_len;
                } else {
                    uart2_rx_len = 0;
                }
            }
        }
    }
    return -1;
}

void cam_uart2_write(uint8_t id){
    uint8_t tx_buf[32];
    int len = sprintf((char*)tx_buf, "start%02dend\r\n", id);
    if (len > 0) uart_write_buffer(UART2, tx_buf, len);
}

float enc_cam(float x,float y,int x_target,int y_target,int mode){

	int x_location = x_target /enc_uint - 0.5;
	int y_location = y_target /enc_uint - 0.5;
	
	if (!mode){
		float x_enc_cam = x / enc_uint * cam_uint + error_matrix[x_location][y_location][0];
		return x_enc_cam;
	}else{
		float y_enc_cam = y / enc_uint * cam_uint + error_matrix[x_location][y_location][1];
		return y_enc_cam;
	}
}
