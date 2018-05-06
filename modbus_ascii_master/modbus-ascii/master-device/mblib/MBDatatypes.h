#ifndef MBDATATYPES_H
#define MBDATATYPES_H
#define SERIAL_BUFFER_SIZE  261
#define PDU_MAX_SIZE (SERIAL_BUFFER_SIZE-7)/2

#include <stdint.h>
#include <stdlib.h>

typedef uint16_t word_t;
union bytepair_t{
	// Little endian form of a byte pair
	word_t raw;
	uint8_t cbyte[2];
};

struct pdu_t{
	uint8_t function;
	uint8_t data[PDU_MAX_SIZE-1];
};

struct bin_frame_t{
	uint8_t address;
	pdu_t pdu;
	uint8_t lrc;
	uint8_t pdu_len;
};

struct frame_buffer{
	uint8_t raw_data[SERIAL_BUFFER_SIZE];
	volatile uint16_t pos;
	volatile uint16_t len;
	volatile uint8_t transmitting;
	volatile uint8_t read_ready;
	volatile uint8_t write_ready;
};

#endif
