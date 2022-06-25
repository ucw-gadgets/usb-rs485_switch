/*
 *	USB-RS485 Switch -- Interface Definitions
 *
 *	(c) 2022 Martin Mareš <mj@ucw.cz>
 */

#define USB_RS485_USB_VENDOR 0x4242
#define USB_RS485_USB_PRODUCT 0x000b
#define USB_RS485_USB_VERSION 0x0100

/*
 *	Endpoints:
 *
 *	0x00 = control endpoint, accepts URS485_CONTROL_xxx vendor-defined requests
 *
 *	0x01 = bulk endpoint, accepts a stream of MODBUS request messages
 *
 *	0x82 = bulk endpoint, produces a stream of MODBUS reply messages
 *		(broadcast messages produce a synthetic reply with no data)
 *
 *	All structure fields are little-endian.
 */

struct urs485_message {
	byte port;			// 0-7
	byte slave_address;		// 0 for broadcast
	byte function_code;
	byte data_size;
	u16 message_id;			// used to match replies with requests
	// data follow
};

enum urs485_control_request {
	URS485_CONTROL_GET_CONFIG,	// sends struct urs485_config
	URS485_CONTROL_SET_PORT_PARAMS,	// accepts struct urs485_port_params (wIndex=port number)
	URS485_CONTROL_GET_PORT_STATUS,	// sends struct urs485_port_status (wIndex=port number)
	URS485_CONTROL_GET_POWER_STATUS,	// sends struct urs485_power_status
};

struct urs485_config {
	u16 max_in_flight;		// maximum number of in-flight MODBUS messages
};

struct urs485_port_params {
	u32 baud_rate;
	byte parity;			// 0=none (2 stop bits), 1=odd, 2=even
	byte powered;			// 0=off, 1=on
	u16 request_timeout;		// in milliseconds
};

struct urs485_port_status {
	u16 current_sense;		// FIXME
	// FIXME: Counters
};

struct urs485_power_status {
	// Raw 12-bit ADC values
	u16 reference;			// 1.2V reference
	u16 power_supply;		// VCC from power supply
	u16 reg_5v;			// 5V from internal regulator
	u16 port_current_sense[8];	// FIXME
};
