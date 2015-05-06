#include <rcc/rcc.h>
#include <rcc/flash.h>
#include <gpio/gpio.h>
#include <pwr/pwr.h>
#include <os/time.h>

Pin led_green = GPIOB[7];
Pin led_blue  = GPIOB[6];

void clock_setup(void) {
	// Enable HSI
	RCC.CR |= 1;
	while (!RCC.CR & 0x2);

	// set range 1 for full speed
	RCC.enable(RCC.PWR);
	PWR.CR = (1 << 11);

	flash_init();

	// pll * 6 / 3, source = HSI
	RCC.CFGR = (2 << 22) | (2 << 18);
	RCC.CR |= (1<<24);  // PLL on
	while (!(RCC.CR & (1<<25)));
	RCC.CFGR |= 3; // switch to pll
	while ((RCC.CFGR & (3<<2)) != (3<<2)); // Wait for pll source

}

int main() {
	// Initialize system timer.
	//STK.LOAD = 2000000 / 2 / 1000; // 1000 Hz.
	//STK.CTRL = 0x03;
	
	RCC.enable(RCC.GPIOB);
	led_green.set_mode(Pin::Output);
	led_blue.set_mode(Pin::Output);

	led_blue.on();
	clock_setup();
	led_blue.off();
	
	while(1) {
		// This doesn't work on l1 yet?
		//Time::sleep(500);
		
		led_green.toggle();
		for (int i = 0; i < 1000000; i++) {
			__asm__("nop");
		}

	}
}
