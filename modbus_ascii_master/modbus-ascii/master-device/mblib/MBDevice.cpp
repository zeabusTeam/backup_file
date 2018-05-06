#include "MBDevice.h"
MBSlave::MBSlave(uint8_t slave_address, const char* port_name, const char* link_name, int baud, int bits, int parity, int stopbits){
	_slave_address = slave_address;
	_fd = -1;
	_port_name = port_name;
	_link_name = link_name;
	_baud = baud;
	_bits = bits;
	_parity = parity;
	_stopbits = stopbits;
	
}
MBSlave::MBSlave(uint8_t slave_address){
	_slave_address = slave_address;
	_fd = -1;
	_port_name = "";
	_link_name = "";
	_baud = 115200;
	_bits = 8;
	_parity = 0;
	_stopbits = 1;
	
}
int MBSlave::Init(int fd, int speed, int bits, int parity, int stopbits)
{
	int tmp1, tmp2;
	_fd = fd;
	tmp2 = Set_interface_attribs (&_fd, speed, bits, parity, stopbits, 0);  
	if (tmp1 & tmp1) return 1;  // Return success
	return 0;
}
int MBSlave::Init(int fd)
{
	int tmp;
	_fd = fd;
	return 1;
}
unsigned int MBSlave::Read_coils(uint16_t start_address, uint16_t quantity, uint8_t mb_bit_stream[])
{
	if (quantity > 0x70D0 || quantity == 0) return 0;
	pdu_t req_pdu, resp_pdu;
	uint16_t mb_bitstream_size = (quantity+7)/8;
	uint16_t req_pdu_len =  5;	
	req_pdu.function = FC_READ_COILS;
	req_pdu.data[0] = start_address >> 8;
	req_pdu.data[1] = start_address % 256;
	req_pdu.data[2] = quantity >> 8;
	req_pdu.data[3] = quantity % 256;
	int resp_pdu_len = Modbus_request(&_fd, _slave_address, &req_pdu, &resp_pdu, req_pdu_len);
	if (resp_pdu_len > 0 && resp_pdu.data[0] == (quantity+7)/8 && resp_pdu.function == FC_READ_COILS) {
		memcpy(mb_bit_stream, (resp_pdu.data)+1, mb_bitstream_size);
		return quantity;
	}else{
		return 0;
	}
}

unsigned int MBSlave::ConnectionTest()
{
	pdu_t req_pdu, resp_pdu;
	uint16_t req_pdu_len =  5;	
	req_pdu.function = FC_READ_COILS;
	req_pdu.data[0] = 0;
	req_pdu.data[1] = 0;
	req_pdu.data[2] = 0;
	req_pdu.data[3] = 1;
	int resp_pdu_len = Modbus_request(&_fd, _slave_address, &req_pdu, &resp_pdu, req_pdu_len);
	if (resp_pdu_len > 0 && ( resp_pdu.function == FC_READ_COILS || resp_pdu.function == FC_READ_COILS + 0x80) ) {
		return 1;
	}else{
		return 0;
	}
}

unsigned int MBSlave::Read_discrete_inputs(uint16_t start_address, uint16_t quantity, uint8_t mb_bit_stream[])
{
	if (quantity > 0x70D0 || quantity == 0) return 0;
	pdu_t req_pdu, resp_pdu;
	uint16_t mb_bitstream_size = (quantity+7)/8;
	uint16_t req_pdu_len =  5;	
	req_pdu.function = FC_READ_DISCRETE_INPUTS;
	req_pdu.data[0] = start_address >> 8;
	req_pdu.data[1] = start_address % 256;
	req_pdu.data[2] = quantity >> 8;
	req_pdu.data[3] = quantity % 256;
	int resp_pdu_len = Modbus_request(&_fd, _slave_address, &req_pdu, &resp_pdu, req_pdu_len);
	if (resp_pdu_len > 0 && resp_pdu.data[0] == (quantity+7)/8 && resp_pdu.function == FC_READ_DISCRETE_INPUTS) {
		memcpy(mb_bit_stream, (resp_pdu.data)+1, mb_bitstream_size);
		return quantity;
	}else{
		return 0;
	}
}
unsigned int MBSlave::Write_multiple_coils(uint16_t start_address, uint16_t quantity, uint8_t mb_bit_stream[])
{
	if (quantity > 0x07B0 || quantity == 0) return 0;
	pdu_t req_pdu, resp_pdu;
	uint16_t mb_bitstream_size = (quantity+7)/8;
	uint16_t req_pdu_len =  5+1+(quantity+7)/8;	
	req_pdu.function = FC_WRITE_MULTIPLE_COILS;
	req_pdu.data[0] = start_address >> 8;
	req_pdu.data[1] = start_address % 256;
	req_pdu.data[2] = quantity >> 8;
	req_pdu.data[3] = quantity % 256;
	req_pdu.data[4] = (quantity+7)/8;
	memcpy((req_pdu.data)+5, mb_bit_stream, mb_bitstream_size);
	int resp_pdu_len = Modbus_request(&_fd, _slave_address, &req_pdu, &resp_pdu, req_pdu_len);
	if (resp_pdu_len > 0  && resp_pdu.function == FC_WRITE_MULTIPLE_COILS) {
		return quantity;
	}else{
		return 0;
	}
}
unsigned int MBSlave::Read_holding_registers(uint16_t start_address, uint16_t quantity, uint16_t registers[])
{
	if (quantity > 0x007D || quantity == 0) return 0;
	pdu_t req_pdu, resp_pdu;
	uint16_t req_pdu_len =  5;	
	req_pdu.function = FC_READ_HOLDING_REGISTERS;
	req_pdu.data[0] = start_address >> 8;
	req_pdu.data[1] = start_address % 256;
	req_pdu.data[2] = quantity >> 8;
	req_pdu.data[3] = quantity % 256;
	int resp_pdu_len = Modbus_request(&_fd, _slave_address, &req_pdu, &resp_pdu, req_pdu_len);
	if (resp_pdu_len > 0 && resp_pdu.data[0] == quantity*2 && resp_pdu.function == FC_READ_HOLDING_REGISTERS) {
		int i;		
		for (i=0;i<resp_pdu.data[0]/2;i++)
		{
			registers[i] = resp_pdu.data[1+2*i]*256 + resp_pdu.data[2+2*i]; 
			//printf(" %.04x ", registers[i]);
			
		}
		return resp_pdu.data[0]/2;
	}else{
		return 0;
	}
}

unsigned int MBSlave::Read_input_registers(uint16_t start_address, uint16_t quantity, uint16_t registers[])
{
	if (quantity > 0x007D || quantity == 0) return 0;
	pdu_t req_pdu, resp_pdu;
	uint16_t req_pdu_len =  5;	
	req_pdu.function = FC_READ_INPUT_REGISTERS;
	req_pdu.data[0] = start_address >> 8;
	req_pdu.data[1] = start_address % 256;
	req_pdu.data[2] = quantity >> 8;
	req_pdu.data[3] = quantity % 256;
	int resp_pdu_len = Modbus_request(&_fd, _slave_address, &req_pdu, &resp_pdu, req_pdu_len );
	if (resp_pdu_len > 0 && resp_pdu.data[0] == quantity*2 && resp_pdu.function == FC_READ_INPUT_REGISTERS) {
		int i;		
		for (i=0;i<resp_pdu.data[0]/2;i++)
		{
			registers[i] = resp_pdu.data[1+2*i]*256 + resp_pdu.data[2+2*i]; 
			//printf(" %.04x ", registers[i]);
			
		}
		return resp_pdu.data[0]/2;
	}else{
		return 0;
	}
}

unsigned int MBSlave::Write_single_coil(uint16_t write_address, uint16_t write_val)
{
	write_val = (write_val)?0xFF00:0x0000;
	pdu_t req_pdu, resp_pdu;
	uint16_t req_pdu_len =  5;	
	req_pdu.function = FC_WRITE_SINGLE_COIL;
	req_pdu.data[0] = write_address >> 8;
	req_pdu.data[1] = write_address % 256;
	req_pdu.data[2] = write_val >> 8;
	req_pdu.data[3] = write_val % 256;
	int resp_pdu_len = Modbus_request(&_fd, _slave_address, &req_pdu, &resp_pdu, req_pdu_len);
	if (resp_pdu_len > 0) {
		return 1;
	}else{
		return 0;
	}
}

unsigned int MBSlave::Write_single_register(uint16_t write_address, uint16_t write_val)
{
	pdu_t req_pdu, resp_pdu;
	uint16_t req_pdu_len =  5;	
	req_pdu.function = FC_WRITE_SINGLE_REGISTER;
	req_pdu.data[0] = write_address >> 8;
	req_pdu.data[1] = write_address % 256;
	req_pdu.data[2] = write_val >> 8;
	req_pdu.data[3] = write_val % 256;
	int resp_pdu_len = Modbus_request(&_fd, _slave_address, &req_pdu, &resp_pdu, req_pdu_len);
	if (resp_pdu_len > 0) {
		return 1;
	}else{
		return 0;
	}
}

unsigned int MBSlave::Write_multiple_registers(uint16_t start_address, uint16_t quantity, uint16_t registers[])
{
	if (quantity > 0x007B || quantity == 0) return 0;
	pdu_t req_pdu, resp_pdu;
	uint16_t req_pdu_len =  quantity*2+5+1;	
	req_pdu.function = FC_WRITE_MULTIPLE_REGISTERS;
	req_pdu.data[0] = start_address >> 8;
	req_pdu.data[1] = start_address % 256;
	req_pdu.data[2] = quantity >> 8;
	req_pdu.data[3] = quantity % 256;
	req_pdu.data[4] = quantity*2; // byte count
	int i;
	for (i=0;i<quantity;i++)
	{
		req_pdu.data[2*i+5] = registers[i] >> 8;
		req_pdu.data[2*i+6] = registers[i] %256;
	}
	int resp_pdu_len = Modbus_request(&_fd, _slave_address, &req_pdu, &resp_pdu, req_pdu_len);
	if (resp_pdu_len > 0 && resp_pdu.function == FC_WRITE_MULTIPLE_REGISTERS) 
	{
		return quantity;
	}else{
		return 0;
	}
}
unsigned int MBSlave::Read_write_multiple_registers(uint16_t read_start_address, uint16_t read_quantity, uint16_t read_registers[], uint16_t write_start_address, uint16_t write_quantity, uint16_t write_registers[] )
{
	if (read_quantity > 0x007D || read_quantity == 0 || write_quantity > 0x0079 || write_quantity == 0) return 0;
	pdu_t req_pdu, resp_pdu;
	uint16_t req_pdu_len =  write_quantity*2+10;	
	req_pdu.function = FC_READ_WRITE_MULTIPLE_REGISTERS;
	req_pdu.data[0] = read_start_address >> 8;
	req_pdu.data[1] = read_start_address % 256;
	req_pdu.data[2] = read_quantity >> 8;
	req_pdu.data[3] = read_quantity % 256;
	req_pdu.data[4] = write_start_address >> 8;
	req_pdu.data[5] = write_start_address % 256;
	req_pdu.data[6] = write_quantity >> 8;
	req_pdu.data[7] = write_quantity % 256;
	req_pdu.data[8] = write_quantity*2; // byte count
	int i;
	for (i=0;i<write_quantity;i++)
	{
		req_pdu.data[2*i+9] = write_registers[i] >> 8;
		req_pdu.data[2*i+10] = write_registers[i] %256;
	}
	int resp_pdu_len = Modbus_request(&_fd, _slave_address, &req_pdu, &resp_pdu, req_pdu_len);
	if (resp_pdu_len > 0 && resp_pdu.data[0]/2 == read_quantity && resp_pdu.function == FC_READ_WRITE_MULTIPLE_REGISTERS) {
		int i;		
		for (i=0;i<read_quantity;i++)
		{
			read_registers[i] = resp_pdu.data[1+2*i]*256 + resp_pdu.data[2+2*i]; 
			//printf(" %.04x ", read_registers[i]);
			
		}
		return resp_pdu.data[0]/2;
	}else{
		return 0;
	}
}
