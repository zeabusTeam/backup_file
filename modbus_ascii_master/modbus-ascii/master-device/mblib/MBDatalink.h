#ifndef MBDATALINK_H
#define MBDATALINK_H

#include <sys/ioctl.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <termios.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/time.h>
#include "MBDatatypes.h"
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */


#define ILLEGAL_FUNCTION				1
#define ILLEGAL_DATA_ADDRESS			2
#define ILLEGAL_DATA_VALUE				3
#define SLAVE_DEVICE_FAILURE			4
#define ACKNOWLEDGE						5
#define SLAVE_DEVICE_BUSY				6
#define	MEMORY_PARITY_ERROR

#define FC_READ_COILS						0x01
#define FC_READ_DISCRETE_INPUTS				0x02
#define FC_WRITE_SINGLE_COIL				0x05
#define FC_WRITE_MULTIPLE_COILS				0x0F
#define FC_READ_INPUT_REGISTERS				0x04

#define FC_READ_HOLDING_REGISTERS			0x03
#define FC_WRITE_SINGLE_REGISTER			0x06
#define FC_WRITE_MULTIPLE_REGISTERS			0x10
#define FC_READ_WRITE_MULTIPLE_REGISTERS 	0x17
#define FC_MASK_WRITE_REGISTER				0x16

#define FC_READ_FIFO_QUEUE					0x18
#define FC_READ_FILE_RECORD					0x14
#define FC_WRITE_FILE_RECORD 				0x15
#define FC_READ_EXCEPTION_STATUS			0x07
#define FC_DIAGNOSTIC						0x08

#define FC_GET_COM_EVENT_COUNTER			0x0B
#define FC_GET_COM_EVENT_LOG				0x1C
#define	FC_REPORT_SLAVE_ID					0x11
#define FC_READ_DEVICE_IDENTIFICATION		0x2B

#define ERROR_RECEPTION_FAIL				-1
#define ERROR_LRC_INCORRECT					-2

/* Module's parameter options. */
#define INTERCHAR_TIMEOUT   1  // In tens of seconds.
#define RESPONSE_TIMEOUT   8000U  //In microseconds.
#define ENABLE_DEBUG		1  // 1:Enable 0:Disable
#define MAX_RETRY			1   // Retry n times if transaction fail. [1-5]

extern int total_request_count ;
extern int lrc_correct_count ;
extern int lrc_incorrect_count ;
extern int request_success_count ;
extern int show_debug_info;

/* Backend fuctions' prototype */
int Serial_read(int* fd, uint8_t* buf, int size_to_read);
int Open_port(int* fd, const char* portname);
int Open_port(int* fd, const char* portname, int baud);
int Open_port(int* fd, const char* portname, int baud, int bits, int parity, int stopbits, int should_block);
void Close_port(int* fd);
int Set_interface_attribs (int* fd, int speed, int bits, int parity, int stopbits, int should_block);
//void Set_blocking (int* fd, int should_block);
int Modbus_request(int* fd, uint8_t slave_address, pdu_t* req_pdu, pdu_t* resp_pdu, uint16_t pdu_len);
uint16_t Modbus_comm(int* fd, frame_buffer* tx_frame_buffer, frame_buffer* rx_frame_buffer); // return 0 if error, else return frame size.

void Hexstring_to_bin_frame(bin_frame_t* bin_frame, frame_buffer* hexstring_frame, uint16_t raw_frame_size);
uint8_t Hexstring_to_bin(uint8_t high, uint8_t low);
void Bin_to_hexstring_frame(bin_frame_t* bin_frame, frame_buffer* hexstring_frame);
void Bin_to_hexstring(uint8_t* bin, uint8_t* high, uint8_t* low);
void Lrc_calc(bin_frame_t* frame);
/* Aux functions */
//int Serial_print(char* data, int size);
void Print_raw_frame(frame_buffer* frame);
void Print_bin_frame(bin_frame_t* frame);
double time_diff(struct timeval x , struct timeval y);


#endif
