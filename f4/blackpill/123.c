#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <stdio.h>
#include <string.h>


int _close(int fd) { (void)fd; return -1; }
int _fstat(int fd, void *buf) { (void)fd; (void)buf; return -1; }
int _getpid(void) { return 1; }
int _isatty(int fd) { (void)fd; return 1; }
int _kill(int pid, int sig) { (void)pid; (void)sig; return -1; }
int _lseek(int fd, int ptr, int dir) { (void)fd; (void)ptr; (void)dir; return -1; }
int _read(int fd, char *ptr, int len) { (void)fd; (void)ptr; (void)len; return -1; }
int _write(int fd, char *ptr, int len) { (void)fd; (void)ptr; (void)len; return -1; }

#define ACCEL_UART USART2
#define ACCEL_BAUDRATE 9600

uint8_t accel_rx_buf[6];
float x, y, z;

void accel_uart_init(void) {
    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART2_RX);
    usart_set_baudrate(ACCEL_UART, ACCEL_BAUDRATE);
    usart_set_databits(ACCEL_UART, 8);
    usart_set_stopbits(ACCEL_UART, USART_STOPBITS_1);
    usart_set_parity(ACCEL_UART, USART_PARITY_NONE);
    usart_set_flow_control(ACCEL_UART, USART_FLOWCONTROL_NONE);
    usart_set_mode(ACCEL_UART, USART_MODE_TX_RX);
    usart_enable(ACCEL_UART);
}

void accel_init(void) {
    uint8_t init_cmd[] = {0xAA, 0x20, 0x47};
    for(int i = 0; i < sizeof(init_cmd); i++) {
        usart_send_blocking(ACCEL_UART, init_cmd[i]);
        for(volatile int j = 0; j < 1000; j++);
    }
}

void accel_read_data(void) {
    usart_send_blocking(ACCEL_UART, 0xA8);
    for(int i = 0; i < 6; i++) {
        accel_rx_buf[i] = usart_recv_blocking(ACCEL_UART);
    }
    int16_t x_raw = (int16_t)((accel_rx_buf[1] << 8) | accel_rx_buf[0]);
    int16_t y_raw = (int16_t)((accel_rx_buf[3] << 8) | accel_rx_buf[2]);
    int16_t z_raw = (int16_t)((accel_rx_buf[5] << 8) | accel_rx_buf[4]);
    x = x_raw / 16384.0f;
    y = y_raw / 16384.0f;
    z = z_raw / 16384.0f;
}

void debug_uart_init(void) {
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);
    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_enable(USART1);
}

void debug_print(const char *msg) {
    while(*msg) {
        usart_send_blocking(USART1, *msg++);
    }
}

int main(void) {
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
    
    debug_uart_init();
    accel_uart_init();
    
    debug_print("Initializing accelerometer...\r\n");
    accel_init();
    for(volatile int i = 0; i < 500000; i++);
    
    debug_print("Reading accelerometer data:\r\n");

    while(1) {
        accel_read_data();
        char buf[64];
        snprintf(buf, sizeof(buf), "Raw: %02X %02X %02X %02X %02X %02X | X: %+.2fg Y: %+.2fg Z: %+.2fg\r\n",
                accel_rx_buf[0], accel_rx_buf[1], accel_rx_buf[2],
                accel_rx_buf[3], accel_rx_buf[4], accel_rx_buf[5],
                x, y, z);
        debug_print(buf);
        for(volatile int i = 0; i < 500000; i++);
    }

    return 0;
}