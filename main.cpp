#include <rcc/rcc.h>
#include <gpio/gpio.h>
#include <os/time.h>

Pin led_green = GPIOC[8];
Pin led_blue  = GPIOC[9];

int main() {
	// Initialize system timer.
	STK.LOAD = 8000000 / 8 / 1000; // 1000 Hz.
	STK.CTRL = 0x03;
	
	RCC.enable(RCC.GPIOC);
	
	led_green.set_mode(Pin::Output);
	
	while(1) {
		Time::sleep(500);
		
		led_green.toggle();
	}
}
