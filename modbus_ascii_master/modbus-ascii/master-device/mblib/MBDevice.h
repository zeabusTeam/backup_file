#ifndef MBDEVICE_H
#define MBDEVICE_H

#include <stdint.h>
#include <errno.h>
#include <termios.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */

#include "MBDatatypes.h"
#include "MBDatalink.h"


class MBSlave{
private:
	uint8_t _slave_address;
	int _fd;
	const char* _port_name;
	const char* _link_name;
	int _interchar_timeout;
	int _lrc_correct_count;
	int _lrc_incorrect_count;
	int _transaction_count;
	int _transaction_success_count;
	int _show_debug_info;
	int _baud;
	int _bits;
	int _parity;
	int _stopbits;
	struct timeval _frame_timeout;
public:
	MBSlave(uint8_t slave_address, const char* port_name, const char* link_name, int baud, int bits, int parity, int stopbits);
	MBSlave(uint8_t slave_address);
	int Init(int fd, int speed, int bits, int parity, int stopbits);
	int Init(int fd);
	/* Modbus standard function */
	unsigned int Read_coils(uint16_t start_address, uint16_t quantity, uint8_t mb_bit_stream[]);
	// Return 0 if failed, else, return number of bits read, and update mb_bit_stream[] 

	unsigned int Read_discrete_inputs( uint16_t start_address, uint16_t quantity, uint8_t mb_bit_stream[]);
	// Return 0 if failed, else, return number of bits read, and update mb_bit_stream[] 

	unsigned int Read_holding_registers(uint16_t start_address, uint16_t quantity, uint16_t registers[]);
	// Return 0 if failed, else, return number of register read, and update registers[] 

	unsigned int Read_input_registers(uint16_t start_address, uint16_t quantity, uint16_t registers[]);
	// Return 0 if failed, else, return number of register read, and update registers[] 

	unsigned int Write_single_coil(uint16_t write_address, uint16_t write_val);
	// Writes 0 to slave if write_val is zero, writes 1 to slave if write_val is non-zero
	// Return 0 if failed, else, return 1 

	unsigned int Write_single_register(uint16_t write_address, uint16_t write_val);
	// Return 0 if failed, else, return 1 

	unsigned int Write_multiple_coils(uint16_t start_address, uint16_t quantity, uint8_t mb_bit_stream[]);
	// Return 0 if failed, else, return number of bits wrote.

	unsigned int Write_multiple_registers(uint16_t start_address, uint16_t quantity, uint16_t registers[]);
	// Return 0 if failed, else, return number of registers wrote.

	unsigned int Read_write_multiple_registers(uint16_t read_start_address, uint16_t read_quantity, uint16_t read_registers[], uint16_t write_start_address, uint16_t write_quantity, uint16_t write_registers[] );
	// Return 0 if failed, else, return number of registers read and update read_registers[].

	unsigned int ConnectionTest();
};
#endif
