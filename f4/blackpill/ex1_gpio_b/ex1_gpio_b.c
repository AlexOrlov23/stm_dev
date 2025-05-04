#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include "ex1_gpio_blink.h"
 
static void clock_setup(void)
{
        rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_96MHZ]);
        rcc_periph_clock_enable(RCC_GPIOC);
}       
 
int main(void)
{
        clock_setup();
        gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13);
        while(1){
                int i;
                for (i = 0; i < 15000000; i++) {
                        __asm__("nop");
                }
                gpio_toggle(GPIOC, GPIO13);
        }
}
