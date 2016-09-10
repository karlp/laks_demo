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
// cheap f103c8 china board

Pin usb_dm   = GPIOA[11];
Pin usb_dp   = GPIOA[12];

Pin led1 = GPIOC[13];

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

USART_t sport_uart = USART2;
Pin sport_tx = GPIOA[2];
Pin sport_rx = GPIOA[3];

template < int ABuffer_Len >
class ABuffer {
private:
	uint8_t buf[ABuffer_Len];
	volatile int idx_r;
	volatile int idx_w;
public:
	ABuffer() : idx_r(0), idx_w(0) {}
	inline bool empty() {
		return (idx_r == idx_w);
	}

	int depth() {
		return ((unsigned int)(ABuffer_Len + idx_w - idx_r) % ABuffer_Len);
	}

	int get() {
		if (empty()) {
			return -1;
		}
		uint8_t c = buf[idx_r];
		idx_r = (idx_r + 1) % ABuffer_Len;
		return c;
	}

	bool put(uint8_t c) {
		int next = (idx_w + 1) % ABuffer_Len;
		if (next != idx_r) {
			buf[idx_w] = c;
			idx_w = next;
			return true;
		}
		return false;
	}
};

class USART_Buffered {
	friend class USB_CDC_ACM;

	private:
		USART_t &port;
		// just needs to be deep enough to max baud in the loop time to make a new usb frame. (1ms)
		ABuffer<64> rx_ring;
		// needs to allow two full out USB packets.  (We _could_ set MPS to small, but... 64 it is)
		ABuffer<128> tx_ring;
	public:
		USART_Buffered(USART_t &sport) : port(sport) {}
		void init() {
			port.set_baudrate(115200);
			// enable, rxnei, te, re
			port.reg.CR1 = (1<<13) | (1<<5) | (0x3<<2);
			Interrupt::enable(Interrupt::USART2);
		}
		void irq() {
			if (port.reg.SR & 1<<5) { // RXNE
				if (rx_ring.put(port.reg.DR & 0xff)) {  // FIXME no parity yet
					// good
				} else {
					usb_rblog.log("fatal -> rx ring full!");
					while(1);
				}
			}
			if (port.reg.SR & (1<<7)) { // TXE
				if (tx_ring.empty()) {
					port.reg.CR1 &= ~(1<<7); // Disable txe
				} else {
					port.reg.DR = tx_ring.get();
				}
			}
			// TODO - TXC for rs485 control...
		}

		void set_baudrate(uint32_t baud) {
			port.set_baudrate(baud);
		}

		void push_tx(uint8_t *buf, int len) {
			for (int i = 0; i < len; i++) {
				if (!tx_ring.put(buf[i])) {
					// this causes the rblog to be truncated to just this entry?
					//usb_rblog.log("fatal -> tx ring full. flow control earlier!");
					while(1);
				}
				// turn on TXE, definitely got data to send!
				port.reg.CR1 |= (1<<7);
			}
		}
};


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
		USART_Buffered& port;
		
		uint32_t buf[16];
		enum Requests {
			None,
			SetLineCoding = 0x20,
			GetLineCoding = 0x21,
			SetLineControlState = 0x22,
			SendBreak = 0x23
		};

		Requests request;
		bool nakked = false;
	
	public:
		USB_CDC_ACM(USB_generic& usbd, USART_Buffered& port) : usb(usbd), port(port) {
			usb.register_driver(this);
		}
		void process(void) {
			// Re-enable OUT from host if we've got space for more data
			if (port.tx_ring.depth() < 64 && nakked) {
				usb.hw_set_nak(1, false);
				nakked = false;
			}

			// Drain RX ring
			uint8_t zero_copy_is_for_losers[64];
			int zci = 0;
			int c = port.rx_ring.get();
			while (c >= 0) {
				zero_copy_is_for_losers[zci++] = c;
				c = port.rx_ring.get();
			}

			if (zci) {
				usb.write(1, (uint32_t*)zero_copy_is_for_losers, zci);
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
				// No data stage for this, must handle here!
				request = Requests::SetLineControlState;
				// FIXME - set dtr/rts here
				usb_rblog.log("correctly setup ok SLCS");
				usb.write(0, nullptr, 0);
				return SetupStatus::Ok;
			}
			usb_rblog.log("setupfail: req: %x", bRequest);
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
					port.set_baudrate(coding.dwDTERate);
					usb_rblog.log("port set to %d", coding.dwDTERate);
					res = true;
				}
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
		
		virtual OutStatus handle_out(uint8_t ep, uint32_t len) {
			if(ep == 0) {
				handle_out_control(len);
			} else if(ep == 1) {
				uint32_t r_len = usb.read(ep, buf, len);
				if(r_len) {
					led1.toggle();
					port.push_tx((uint8_t*)buf, r_len);
					usb_rblog.log("depth now %d", ((uint32_t)port.tx_ring.depth()));
					if (port.tx_ring.depth() >= 64) {
						nakked = true;
						return OutStatus::NAK;
					}
				}
			}
			return OutStatus::Ok;
		}
};

USART_Buffered sport(USART2);
USB_CDC_ACM usb_cdc_acm(usb, sport);

template <>
void interrupt<Interrupt::USART2>() {
	sport.irq();
}

int main() {
	rcc_init();
	#if defined(STM32F1)
	// Initialize system timer.
	STK.LOAD = 72000000 / 8 / 1000; // 1000 Hz.
	STK.CTRL = 0x03;
	
	RCC.enable(RCC.AFIO);
	RCC.enable(RCC.GPIOA);
	RCC.enable(RCC.GPIOB);
	RCC.enable(RCC.GPIOC);
	
	led1.set_mode(Pin::Output);
	// hack to reenumerate
	usb_dp.set_mode(Pin::Output);
	usb_dp.off();
	Time::sleep(1);
	
	usb_dm.set_mode(Pin::AF);
	usb_dp.set_mode(Pin::AF);
//	usb_disc.set_mode(Pin::Output);
//	usb_disc.off();
	sport_rx.set_mode(Pin::InputPull);
	sport_tx.set_mode(Pin::AF);
	
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
	sport_rx.set_mode(Pin::AF);
	sport_rx.set_af(7);
	sport_tx.set_mode(Pin::AF);
	sport_tx.set_af(7);

	#endif

	RCC.enable(RCC.USART2);

	sport.init();
	usb.init();
	
	while(1) {
		usb.process();
		usb_cdc_acm.process();
	}
}
