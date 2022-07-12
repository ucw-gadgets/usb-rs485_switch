/*
 *	USB-RS485 Switch -- Interface Definitions
 *
 *	(c) 2022 Martin Mareš <mj@ucw.cz>
 */

#define URS485_USB_VENDOR 0x4242
#define URS485_USB_PRODUCT 0x000b
#define URS485_USB_VERSION 0x0100

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

/*
 *	Flow control:
 *
 *	The device has a limited number of slots for in-flight messages,
 *	reported in the	urs485_config. However, when the client starts,
 *	some slots can be busy from the previous client. So the device
 *	opens up client's send window by sending synthetic replies marked
 *	by port == 0xff.
 */

#define MODBUS_MAX_DATA_SIZE 252

struct urs485_message {
	byte port;			// 0-7 (0xff for window open message)
	byte frame_size;
	u16 message_id;			// used to match replies with requests
	/*
	 *  Internally, the frame has the following structure:
	 *  	byte	slave_address;	// 0 for broadcast
	 *  	byte	function_code;
	 *  	byte	data[];
	 *  	byte	crc[2];		// reserved for CRC, never sent over USB
	 *
	 *  Only frame_size bytes are sent over USB.
	 */
	byte frame[2 + MODBUS_MAX_DATA_SIZE + 2];
};

#define URS485_MSGHDR_SIZE offsetof(struct urs485_message, frame)

enum urs485_control_request {
	URS485_CONTROL_GET_CONFIG,	// in: sends struct urs485_config
	URS485_CONTROL_SET_PORT_PARAMS,	// out: accepts struct urs485_port_params (wIndex=port number)
	URS485_CONTROL_GET_PORT_STATUS,	// in: sends struct urs485_port_status (wIndex=port number)
	URS485_CONTROL_GET_POWER_STATUS,	// in: sends struct urs485_power_status
};

struct urs485_config {
	u16 max_in_flight;		// maximum number of in-flight MODBUS messages
};

struct urs485_port_params {
	u32 baud_rate;			// 1200 to 115200
	byte parity;			// URS485_PARITY_xxx
	byte powered;			// 0=off, 1=on
	u16 request_timeout;		// in milliseconds
};

enum urs485_parity {
	URS485_PARITY_NONE = 0,
	URS485_PARITY_ODD = 1,
	URS485_PARITY_EVEN = 2,		// recommended by MODBUS standard
};

struct urs485_port_status {
	u16 current_sense;		// FIXME
	u16 rfu;
	u32 cnt_broadcasts;		// Broadcast requests sent
	u32 cnt_unicasts;		// Unicast requests sent and replies received
	u32 cnt_frame_errors;		// Reply contains framing errors
	u32 cnt_oversize_errors;	// Reply is oversized
	u32 cnt_undersize_errors;	// Reply is undersized
	u32 cnt_crc_errors;		// Reply has incorrect CRC
	u32 cnt_mismatch_errors;	// Reply does not match request
	u32 cnt_timeouts;		// Did not receive reply in time
};

struct urs485_power_status {
	// Raw 12-bit ADC values
	u16 reference;			// 1.2V reference
	u16 power_supply;		// VCC from power supply
	u16 reg_5v;			// 5V from internal regulator
	u16 port_current_sense[8];	// FIXME
};
