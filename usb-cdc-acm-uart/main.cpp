#include <rcc/rcc.h>
#include <gpio/gpio.h>
#include <usart/usart.h>
#include <os/time.h>
#include <usb/usb.h>
#include <usb/descriptor.h>

auto dev_desc = device_desc(0x200, 0, 0, 0, 64, 0xcafe, 0x5678, 0, 0, 0, 0, 1);
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

Pin led1 = GPIOD[12];

USB_otg usb(OTG_FS, dev_desc_p, conf_desc_p);

#endif

USART_t sport = USART2;
Pin sport_tx = GPIOA[2];
Pin sport_rx = GPIOA[3];

//----- USB CSC PSTN120.pdf Table 17: Line Coding Structure
struct usb_cdc_line_coding {
	uint32_t dwDTERate;
	uint8_t bCharFormat;
	uint8_t bParityType;
	uint8_t bDataBits;
} __attribute__((packed));

enum usb_cdc_line_coding_bCharFormat {
	Stop1	= 0,
	Stop1_5	= 1,
	Stop2	= 2,
};

enum usb_cdc_line_coding_bParityType {
	None	= 0,
	Odd	= 1,
	Even	= 2,
	Mark	= 3,
	Space	= 4,
};
//----- from usb class specs ----

class USB_CDC_ACM : public USB_class_driver {
	private:
		USB_generic& usb;
		USART_t& port;
		
		uint32_t buf[16];
		enum Requests {
			None,
			SetLineCoding = 0x20,
			GetLineCoding = 0x21,
			SetLineControlState = 0x22,
			SendBreak = 0x23
		};

		Requests request;
	
	public:
		USB_CDC_ACM(USB_generic& usbd, USART_t& port) : usb(usbd), port(port) {
			usb.register_driver(this);
		}
		void process(void) {
			// No... let RXNE interrupt into a rxfifo, and drain it here...
			if (sport.reg.SR & (1<<5)) {
				uint32_t ch = sport.reg.DR;
				usb.write(1, &ch, 1);
			}
		}
	
	protected:
		virtual SetupStatus handle_setup(uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength) {
			request = Requests::None;
			if(bmRequestType == 0x21 && bRequest == Requests::SetLineCoding) {
				request = Requests::SetLineCoding;
				// Better save wIndex here if you ever want to have multiple serial ports!
				return SetupStatus::Ok;
			}
			if(bmRequestType == 0x21 && bRequest == Requests::SetLineControlState) {
				request = Requests::SetLineControlState;
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

		virtual void handle_out_control(uint32_t len) {
			bool res = false;
			switch (request) {
			case Requests::SetLineCoding: {
				struct usb_cdc_line_coding coding;
				if (usb.read(0, (uint32_t*)&coding, len) != sizeof(coding)) {
					usb_rblog.log("failed to read coding?");
				} else {
					// FIXME - handle parity, stop bits and word length
					sport.set_baudrate(coding.dwDTERate);
					res = true;
				}
				break;
			case Requests::SetLineControlState:
				// FIXME - set DTR/RTS here based on wValue that you saved...
				res = true;
				break;
			}
			default:
				break;
			}
			if (res) {
				usb.write(0, nullptr, 0);
			} else {
				usb.hw_set_stall(0);
			}
		}
		
		virtual void handle_out(uint8_t ep, uint32_t len) {
			if(ep == 0) {
				handle_out_control(len);
			} else if(ep == 1) {
				uint32_t r_len = usb.read(ep, buf, len);
				// FIXME - yeah, no, need to put into our txbuffer here...
				if(r_len) {
					led1.toggle();
					port.reg.DR = buf[0];
				}
			}
		}
};

USB_CDC_ACM usb_cdc_acm(usb, sport);

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
	RCC.enable(RCC.GPIOD);
	
	led1.set_mode(Pin::Output);
	
	usb_vbus.set_mode(Pin::Input);
	usb_dm.set_mode(Pin::AF);
	usb_dm.set_af(10);
	usb_dp.set_mode(Pin::AF);
	usb_dp.set_af(10);
	
	RCC.enable(RCC.OTGFS);
	#endif

	RCC.enable(RCC.USART2);
	sport_rx.set_mode(Pin::AF);
	sport_rx.set_af(7);
	sport_tx.set_mode(Pin::AF);
	sport_tx.set_af(7);
	sport.set_baudrate(115200);
	sport.reg.CR1 = (1<<13) | (0x3<<2); // enable+transmit+rx
	
	usb.init();
	
	while(1) {
		usb.process();
		usb_cdc_acm.process();
	}
}