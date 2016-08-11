#include <rcc/rcc.h>
#include <gpio/gpio.h>
#include <os/time.h>
#include <usb/usb.h>
#include <usb/descriptor.h>

auto dev_desc = device_desc(0x200, 0, 0, 0, 64, 0x1234, 0x5678, 0, 0, 0, 0, 1);
auto conf_desc = configuration_desc(2, 1, 0, 0xc0, 0,
	// CDC ACM control
	interface_desc(0, 0, 1, 0x02, 0x02, 0x01, 0,
		endpoint_desc(0x82, 0x03, 16, 255),
		cdc_header_desc(0x110),
		cdc_call_management_desc(0, 1),
		cdc_acm_desc(2),
		cdc_union_desc(0, 1)
	),
	// CDC ACM data
	interface_desc(1, 0, 2, 0x0a, 0x00, 0x00, 0,
		endpoint_desc(0x81, 0x02, 64, 0), // IN
		endpoint_desc(0x01, 0x02, 64, 0)  // OUT
	)
);

desc_t dev_desc_p = {sizeof(dev_desc), (void*)&dev_desc};
desc_t conf_desc_p = {sizeof(conf_desc), (void*)&conf_desc};

typedef GPIO_t::Pin Pin;

#if defined(STM32F1)
// Maple mini.

Pin usb_disc = GPIOB[9];
Pin usb_dm   = GPIOA[11];
Pin usb_dp   = GPIOA[12];

Pin led1 = GPIOB[1];

USB_f1 usb(USB, dev_desc_p, conf_desc_p);

#elif defined(STM32F3)
// STM32F3DISCOVERY.

Pin usb_dm   = GPIOA[11];
Pin usb_dp   = GPIOA[12];

Pin led1 = GPIOA[15];

USB_f1 usb(USB, dev_desc_p, conf_desc_p);

#elif defined(STM32F4)
// Generic F4.

Pin usb_vbus = GPIOA[9];
Pin usb_dm   = GPIOA[11];
Pin usb_dp   = GPIOA[12];

Pin led1 = GPIOA[4];

USB_otg usb(OTG_FS, dev_desc_p, conf_desc_p);

#endif

class USB_CDC_ACM : public USB_class_driver {
	private:
		USB_generic& usb;
		
		uint32_t buf[16];
	
	public:
		USB_CDC_ACM(USB_generic& usbd) : usb(usbd) {
			usb.register_driver(this);
		}
	
	protected:
		virtual SetupStatus handle_setup(uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength) {
			if(bmRequestType == 0x21 && bRequest == 0x20) {
				return SetupStatus::Ok;
			}
			
			if(bmRequestType == 0x21 && bRequest == 0x22) {
				usb.write(0, nullptr, 0);
				return SetupStatus::Ok;
			}
			
			return SetupStatus::Unhandled;
		}
		
		virtual void handle_set_configuration(uint8_t configuration) {
			if(configuration) {
				usb.register_out_handler(this, 1);
				usb.hw_conf_ep(0x01, EPType::Bulk, 64);
				usb.hw_conf_ep(0x81, EPType::Bulk, 64);
			}
		}
		
		virtual void handle_out(uint8_t ep, uint32_t len) {
			if(ep == 0) {
				usb.write(0, nullptr, 0);
			} else if(ep == 1) {
				uint32_t r_len = usb.read(ep, buf, len);
				if(r_len) {
					led1.toggle();
					usb.write(1, buf, r_len);
				}
			}
		}
};

USB_CDC_ACM usb_cdc_acm(usb);

int main() {
	rcc_init();
	#if defined(STM32F1)
	// Initialize system timer.
	STK.LOAD = 72000000 / 8 / 1000; // 1000 Hz.
	STK.CTRL = 0x03;
	
	RCC.enable(RCC.AFIO);
	RCC.enable(RCC.GPIOA);
	RCC.enable(RCC.GPIOB);
	
	led1.set_mode(Pin::Output);
	
	usb_dm.set_mode(Pin::AF);
	usb_dp.set_mode(Pin::AF);
	usb_disc.set_mode(Pin::Output);
	usb_disc.off();
	
	RCC.enable(RCC.USB);
	#elif defined(STM32F3)
	// Initialize system timer.
	STK.LOAD = 72000000 / 8 / 1000; // 1000 Hz.
	STK.CTRL = 0x03;
	
	RCC.enable(RCC.GPIOA);
	
	usb_dm.set_mode(Pin::AF);
	usb_dm.set_af(14);
	usb_dp.set_mode(Pin::AF);
	usb_dp.set_af(14);
	
	RCC.enable(RCC.USB);
	#elif defined(STM32F4)
	// Initialize system timer.
	STK.LOAD = 168000000 / 8 / 1000; // 1000 Hz.
	STK.CTRL = 0x03;
	
	RCC.enable(RCC.GPIOA);
	
	led1.set_mode(Pin::Output);
	
	usb_vbus.set_mode(Pin::Input);
	usb_dm.set_mode(Pin::AF);
	usb_dm.set_af(10);
	usb_dp.set_mode(Pin::AF);
	usb_dp.set_af(10);
	
	RCC.enable(RCC.OTGFS);
	#endif
	
	usb.init();
	
	while(1) {
		usb.process();
	}
}
