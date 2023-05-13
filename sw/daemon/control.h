/*
 *	USB-RS485 Switch Daemon -- Control Protocol
 *
 *	(c) 2022--2023 Martin Mares <mj@ucw.cz>
 */

/*
 *	The switch can be managed by sending messages on its control bus.
 *	Switch ports correspond to device addresses 1 to 8 on the control bus.
 */

/*
 *	Input registers: current port state and statistics
 *
 *	Counters are 32-bit, split to 2 MODBUS registers (the first register
 *	contains the lower 16 bits).
 *
 *	An atomic snapshot of the statistics can be obtained by reading
 *	multiple registers in one MODBUS transaction.
 */

enum urs485_input_register {
	URS485_IREG_CURRENT_SENSE = 1,			// Not fully implemented yet
	URS485_IREG_CNT_BROADCASTS = 2,			// Successfully completed broadcast transactions
	URS485_IREG_CNT_BROADCASTS_HI,
	URS485_IREG_CNT_UNICASTS = 4,			// Successfully completed unicast transactions
	URS485_IREG_CNT_UNICASTS_HI,
	// Counters for failed transactions of various kinds:
	URS485_IREG_CNT_FRAME_ERRORS = 6,		// Parity errors, receiver overruns etc.
	URS485_IREG_CNT_FRAME_ERRORS_HI,
	URS485_IREG_CNT_OVERSIZE_ERRORS = 8,		// Reply exceeds maximum frame size
	URS485_IREG_CNT_OVERSIZE_ERRORS_HI,
	URS485_IREG_CNT_UNDERSIZE_ERRORS = 10,		// Reply too short
	URS485_IREG_CNT_UNDERSIZE_ERRORS_HI,
	URS485_IREG_CNT_CRC_ERRORS = 12,		// Reply has wrong CRC
	URS485_IREG_CNT_CRC_ERRORS_HI,
	URS485_IREG_CNT_MISMATCH_ERRORS = 14,		// Reply does not match request
	URS485_IREG_CNT_MISMATCH_ERRORS_HI,
	URS485_IREG_CNT_TIMEOUTS = 16,			// Timeout when waiting for reply
	URS485_IREG_CNT_TIMEOUTS_HI,
	URS485_IREG_MAX,
};

/*
 *	Holding registers: port settings
 */

enum urs485_holding_register {
	URS485_HREG_BAUD_RATE = 1,			// Baud rate divided by 100 (12 to 1152)
	URS485_HREG_PARITY = 2,				// Parity mode: 0=none, 1=odd, 2=even
	URS485_HREG_POWERED = 3,			// Deliver power to the port: 0=off, 1=on
	URS485_HREG_TIMEOUT = 4,			// Timeout when waiting for reply [ms]
	URS485_HREG_DESCRIPTION_1 = 5,			// Port description (ASCII, big-endian, space-padded)
	URS485_HREG_DESCRIPTION_2 = 6,
	URS485_HREG_DESCRIPTION_3 = 7,
	URS485_HREG_DESCRIPTION_4 = 8,
	URS485_HREG_CONFIG_MAX,
	URS485_HREG_RESET_STATS = 0x1000,		// Write 0xdead to reset port statistics
};
