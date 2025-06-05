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

uint8_t accel_rx_buf[34];
uint16_t total_voltage;


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
    uint8_t init_cmd[] = {0xDD,0xA5,0x03,0x00,0xFF,0xFD,0x77};
    for(int i = 0; i < 7; i++) {
        usart_send_blocking(ACCEL_UART, init_cmd[i]);
        for(volatile int j = 0; j < 1000; j++);

    }
}

void accel_read_data(void) {;
    for(int i = 0; i < 6; i++) {
        accel_rx_buf[i] = usart_recv_blocking(ACCEL_UART);
        uint16_t total_voltage =  (((uint16_t)accel_rx_buf[2]) << 8 + accel_rx_buf[3]);
    }
    


    
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
    
    
    
    

    while(1) {
        accel_init();
        accel_read_data();
        char buf[64];
        snprintf(buf, sizeof(buf), "%02X, %02X  total voltage: %D \r\n",
                accel_rx_buf[0], accel_rx_buf[1], total_voltage 
                );
        debug_print(buf);
        for(volatile int i = 0; i < 1500000; i++);

    }

    return 0;
}