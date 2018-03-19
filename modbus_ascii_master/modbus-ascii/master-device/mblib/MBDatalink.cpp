#include "MBDatalink.h"

#define SHOWDEBUG if(show_debug_info)printf
#define SHOWDEBUG_PREFIX if(show_debug_info)printf("[MODBUS-ASCII-%05d]",total_request_count);

int read_timeout = INTERCHAR_TIMEOUT;  		// Interchar timeout tens of second.
struct timeval timeout;
struct timeval select_timeout = {RESPONSE_TIMEOUT/1000000, RESPONSE_TIMEOUT%1000000};
int total_request_count = 0;
int lrc_correct_count = 0;
int lrc_incorrect_count = 0;
int request_success_count = 0;
int show_debug_info = 1;
int max_retry = MAX_RETRY;


const uint8_t map_ascii_to_binary[128] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //0 - 9
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //10 - 19
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //20 - 29
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //30 - 39
	0, 0, 0, 0, 0, 0, 0, 0, 0, 1, //40 - 49
	2, 3, 4, 5, 6, 7, 8, 9, 0, 0, //50 - 59
	0, 0, 0, 0, 0, 10, 11, 12, 13, 14, //60 - 69
	15, 0, 0, 0, 0, 0, 0, 0, 0, 0, //70 - 79
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //80 - 89
	0, 0, 0, 0, 0, 0, 0, 10, 11, 12, //90 - 99
	13, 14, 15, 0, 0, 0, 0, 0, 0, 0, //100 - 109
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //110 - 119
	0, 0, 0, 0, 0, 0, 0, 0		  //120 - 127
};
const uint8_t map_binary_to_ascii[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

int Set_interface_attribs (int* fd, int speed, int bits, int parity, int stopbits, int should_block)
{

    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (*fd, &tty) != 0)
    {
    		SHOWDEBUG_PREFIX;  SHOWDEBUG("error %d from tcgetattr\n", errno);
            *fd = -1;
            return -1;
    }
    SHOWDEBUG_PREFIX; SHOWDEBUG("Read-Serial-Attributes(i o c l flags):[0x%.02x][0x%.02x][0x%.02x][0x%.02x]\n", tty.c_iflag, tty.c_oflag, tty.c_cflag, tty.c_lflag);
    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);
    if (bits == 7) tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS7;     // 7-bit chars 
    else tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars

    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,	
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    
    tty.c_cc[VMIN]  = should_block ? 1 : 0;	// Set blocking.
    tty.c_cc[VTIME] = read_timeout;     //Set interchar timeout.

    tty.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | IGNCR | ICRNL); // shut off xon/xoff ctrl
    																// turn off cr to lf auto repalce.

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading


    if (stopbits != 2) tty.c_cflag &= ~CSTOPB; 		// one stopbit
    else tty.c_cflag |= CSTOPB; 					// two stopbit

    if (parity) tty.c_cflag |= PARENB;  		// Enable even parity    
    else tty.c_cflag &= ~PARENB;  				// Disable even parity

    tty.c_cflag &= ~CRTSCTS;		// No RTS CTS control signal
    tty.c_cflag &= ~HUPCL;			// Prevent restarting in arduino.
    if (tcsetattr (*fd, TCSANOW, &tty) != 0)
    {
    		SHOWDEBUG_PREFIX; SHOWDEBUG("error %d from tcsetattr\n", errno);
    		*fd = -1;
    		return 0;
    }
    SHOWDEBUG_PREFIX; SHOWDEBUG("Write-Serial-Attributes(i o c l flags):[0x%.02x][0x%.02x][0x%.02x][0x%.02x]\n", tty.c_iflag, tty.c_oflag, tty.c_cflag, tty.c_lflag);
    return 1;
}
	
uint16_t Modbus_comm(int* fd, frame_buffer* tx_frame_buffer, frame_buffer* rx_frame_buffer){
	//Open_port();
	if (*fd<0){
		SHOWDEBUG_PREFIX; SHOWDEBUG("Port not opened\n");
		return 0;
	}
	int i, wr, rr, ret, frame_ended;
	uint16_t count;
	double dif;
	struct timeval before , after;

	fd_set fds;
	FD_ZERO(&fds);
	FD_SET(*fd, &fds);

	tx_frame_buffer->raw_data[tx_frame_buffer->len] = '\0';		// Null terminator, in case of unexpected invalid frame printing.
	
	gettimeofday(&before , NULL);								//Starting time stamping.
	wr = write (*fd, tx_frame_buffer->raw_data, tx_frame_buffer->len); 	// Transmit a frame.
	
	if (wr!=tx_frame_buffer->len) // Write attemp 2 times.
	{
		SHOWDEBUG_PREFIX;	SHOWDEBUG("Transmission failed. retrying\n");
		wr = write (*fd, tx_frame_buffer->raw_data, tx_frame_buffer->len);
		if (wr!=tx_frame_buffer->len) 
		{
			SHOWDEBUG_PREFIX;	SHOWDEBUG("Transmission error, transaction terminated.\n");
			close(*fd);
			return 0;
		}
		
	}else{
		SHOWDEBUG_PREFIX;	SHOWDEBUG("Transmitted (%d bytes) : ",wr);
		Print_raw_frame(tx_frame_buffer);          			// Print transmitted frame.
	}

	timeout = select_timeout;  								// Reset select timeout.
	ret =select(*fd+1, &fds, NULL, NULL, &timeout);
	 /* ret == 0 means timeout, ret == 1 means descriptor is ready for reading,
	 ret == -1 means error (check errno) */
	if (ret ==1){		
		//int rr = read (fd, rx_frame_buffer->raw_data, SERIAL_BUFFER_SIZE);	// Read
		int rr = Serial_read (fd, rx_frame_buffer->raw_data, SERIAL_BUFFER_SIZE); // N

		/* Roundtrip time measurement */
		gettimeofday(&after , NULL);
		dif = time_diff(before , after);
		/*-----------------------------*/

		/* Constructs received data into a frame */
		count=0;
		frame_ended = 0;
		SHOWDEBUG_PREFIX;	SHOWDEBUG("Received response (%d bytes) ", rr);
		for (i=0;i<rr;i++)
		{
			SHOWDEBUG("%c",rx_frame_buffer->raw_data[i]);
			if(rx_frame_buffer->raw_data[i]=='\n')
			{
				frame_ended = 1;
				count++;
				break;
			}else{
				count++;
			}
			
		}
		
		if (frame_ended!=1)
		{
			SHOWDEBUG("\nInvalid frame [total:%d bytes], last bytes is '%c' [%d].\n", count, rx_frame_buffer->raw_data[count-1], rx_frame_buffer->raw_data[count-1]);
			count = 0;
		}
		rx_frame_buffer->len = count; 
		SHOWDEBUG_PREFIX;	SHOWDEBUG("Roundtrip time %.0lf usec\n", dif);
		return count;
		/*--------------------------------------*/
	}
	else if(ret == 0) {
		SHOWDEBUG_PREFIX;	SHOWDEBUG("timeout error fd=%d\n",*fd);
	}
	else {
		SHOWDEBUG_PREFIX;	SHOWDEBUG("some other error.\n");
	}
	//close(fd);
	return 0;
}

int Modbus_request(int* fd, uint8_t slave_address, pdu_t* req_pdu, pdu_t* resp_pdu, uint16_t pdu_len)
{
	//SHOWDEBUG("============ Transaction started =============================================================\n");
	SHOWDEBUG("\n");
	uint16_t received_frame_size;
	uint8_t lrc;
	int retry_count = 0;

	frame_buffer tx_frame_buffer;
	frame_buffer rx_frame_buffer;
	bin_frame_t tx_bin_frame;
	bin_frame_t rx_bin_frame;

	tx_bin_frame.address = slave_address;
	tx_bin_frame.pdu = *req_pdu;
	total_request_count++;

	Lrc_calc(&tx_bin_frame); 
	tx_bin_frame.pdu_len = pdu_len;
	//Print_bin_frame(&tx_bin_frame);
	Bin_to_hexstring_frame(&tx_bin_frame, &tx_frame_buffer);
	// MB raw frame ready to transmit
	received_frame_size = Modbus_comm(fd, &tx_frame_buffer, &rx_frame_buffer);

	// Initial max retry
	if (max_retry<0) max_retry = 0;
	else if (max_retry>5) max_retry = 5; 
    
	while (received_frame_size<7 && retry_count < max_retry)
	{ 
	    
		SHOWDEBUG_PREFIX;	SHOWDEBUG("Reception failed. %d attemps\n", retry_count+1);
		retry_count++;
		received_frame_size = Modbus_comm(fd, &tx_frame_buffer, &rx_frame_buffer);
		//return 0;
	}
	if (received_frame_size<7)
	{
		SHOWDEBUG_PREFIX;	SHOWDEBUG("Reception failed, transaction aborted (total %d retries).\n", retry_count+1);
		return ERROR_RECEPTION_FAIL;
	}
	Hexstring_to_bin_frame(&rx_bin_frame, &rx_frame_buffer, received_frame_size);

	lrc = rx_bin_frame.lrc;
	Lrc_calc(&rx_bin_frame);

	if (rx_bin_frame.lrc == lrc)
	{
		SHOWDEBUG_PREFIX;	SHOWDEBUG("LRC Correct [%.02x]\n", rx_bin_frame.lrc);
		lrc_correct_count++;
	}
	else{
		SHOWDEBUG_PREFIX;	SHOWDEBUG("LRC Incorrect [%.02x] expected [%.02x] \n" , rx_bin_frame.lrc, lrc);
		lrc_incorrect_count++;
		return ERROR_LRC_INCORRECT;
	}
	*resp_pdu = rx_bin_frame.pdu;
	request_success_count++;
	return rx_bin_frame.pdu_len; 

}

void Hexstring_to_bin_frame(bin_frame_t* bin_frame, frame_buffer* hexstring_frame, uint16_t raw_frame_size){
	bin_frame->address = Hexstring_to_bin(hexstring_frame->raw_data[1], hexstring_frame->raw_data[2]);
	bin_frame->pdu.function = Hexstring_to_bin(hexstring_frame->raw_data[3],hexstring_frame->raw_data[4]);
	uint8_t i;
	for (i=5;i<hexstring_frame->len-4;i+=2)
	{
		bin_frame->pdu.data[(i-5)/2] = Hexstring_to_bin(hexstring_frame->raw_data[i],hexstring_frame->raw_data[i+1]);
	}
	bin_frame->lrc = Hexstring_to_bin(hexstring_frame->raw_data[hexstring_frame->len-4],hexstring_frame->raw_data[hexstring_frame->len-3]);
	bin_frame->pdu_len = (hexstring_frame->len-7)/2;
}


uint8_t Hexstring_to_bin(uint8_t high, uint8_t low)
{
	/*
	uint8_t tmp_h, tmp_l;
	if (high >='0' && high <= '9')
	{
		tmp_h = high - '0';
	}else 	if (high >= 'A' && high <= 'F')
			{
				tmp_h = high - 'A' + 10;
			}
	else tmp_h = 0;
	if (low >= '0' && low <= '9')
	{
		tmp_l = low - '0';
	}else 	if (low >= 'A' && low <= 'F')
			{
				tmp_l = low - 'A' + 10;
			}
	else tmp_l = 0;
	return (tmp_h << 4) | tmp_l;
	*/
	return (map_ascii_to_binary[high] << 4) | map_ascii_to_binary[low];
}

void Bin_to_hexstring_frame(bin_frame_t* bin_frame, frame_buffer* hexstring_frame)
{
	hexstring_frame->raw_data[0] = ':';
	Bin_to_hexstring(&(bin_frame->address), &(hexstring_frame->raw_data[1]), &(hexstring_frame->raw_data[2]) );
	Bin_to_hexstring(&(bin_frame->pdu.function), &(hexstring_frame->raw_data[3]), &(hexstring_frame->raw_data[4]) );
	uint8_t i;
	for (i=0;i<bin_frame->pdu_len-1;i++)
	{
		Bin_to_hexstring( &(bin_frame->pdu.data[i]), &(hexstring_frame->raw_data[2*i+5]), &(hexstring_frame->raw_data[2*i+6]) );
	}
	Lrc_calc(bin_frame);
	Bin_to_hexstring( &(bin_frame->lrc), &(hexstring_frame->raw_data[2*i+5]), &(hexstring_frame->raw_data[2*i+6]) );
	hexstring_frame->raw_data[2*i+7] = '\r';
	hexstring_frame->raw_data[2*i+8] = '\n';
	hexstring_frame->pos = 0;
	hexstring_frame->len = (bin_frame->pdu_len)*2+7;
	
}

void Bin_to_hexstring(uint8_t* bin, uint8_t* high, uint8_t* low)
{
	/*
	uint8_t tmp = (*bin) >> 4;
	*high = (tmp >= 0 && tmp <= 9)?(tmp+'0'):(tmp+'A'-10);
	tmp = (*bin) & 15;
	*low = (tmp >= 0 && tmp <= 9)?(tmp+'0'):(tmp+'A'-10);
	*/
	uint8_t tmp = (*bin) >> 4;
	*high = map_binary_to_ascii[tmp];
	tmp = (*bin) & 15;
	*low = map_binary_to_ascii[tmp];
}

void Lrc_calc(bin_frame_t* frame)
{
	uint8_t sum=0;
	uint8_t lrc_valid=0;
	sum += frame->address;
	sum += frame->pdu.function;
	uint8_t i;
	for (i=0;i<frame->pdu_len-1;i++)	
	{
		sum += frame->pdu.data[i];			
	}
	frame->lrc = (~sum) + 1; 
}
void Print_raw_frame(frame_buffer* frame)
{
	int i;
	for(i=0;i<frame->len;i++)
	{
		SHOWDEBUG("%c",(char)frame->raw_data[i]);
	}
}
void Print_bin_frame(bin_frame_t* frame)
{
	SHOWDEBUG("\\x%.02X",(int)frame->address);
	SHOWDEBUG("\\x%.02X",(int)frame->pdu.function);
	int i;
	for (i=0;i<frame->pdu_len-1;i++)
	{
		SHOWDEBUG("\\x%.02X",frame->pdu.data[i]);
	}
	SHOWDEBUG("\\x%.02X",(int)frame->lrc);
	SHOWDEBUG("   (len=%d)\n",frame->pdu_len);
}
/*void Set_portname(const char* _portname)
{
	portname = _portname;
}*/
void Set_select_timeout(int usec)
{
	select_timeout.tv_sec = usec/1000000;
	select_timeout.tv_usec = usec%1000000;
}
int Open_port(int* fd, const char* portname)
{
	if (*fd <= 0) 
	{
		*fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
		Set_interface_attribs (fd, B1000000, 7, 0, 2, 0);
	}
	if (*fd <= 0)
	{
			SHOWDEBUG_PREFIX;     SHOWDEBUG ("error %d opening %s: %s\n", errno, portname, strerror (errno));
	        close(*fd);
	        return 0;
	}
	SHOWDEBUG_PREFIX;	SHOWDEBUG("Openned port (fd = %d)\n", *fd);	
	return 1;
}
int Open_port(int* fd, const char* portname, int baud)
{
	if (*fd <= 0) 
	{
		*fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
		Set_interface_attribs (fd, baud, 7, 0, 2, 0);
	}
	if (*fd <= 0)
	{
			SHOWDEBUG_PREFIX;     SHOWDEBUG ("error %d opening %s: %s\n", errno, portname, strerror (errno));
	        close(*fd);
	        return 0;
	}
	SHOWDEBUG_PREFIX;	SHOWDEBUG("Openned port (fd = %d)\n", *fd);	
	return 1;
}
int Open_port(int* fd, const char* portname, int baud, int bits, int parity, int stopbits, int should_block)
{
	if (*fd <= 0) 
	{
		*fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
		Set_interface_attribs (fd, baud, bits, parity, stopbits, 0);
	}
	if (*fd <= 0)
	{
			SHOWDEBUG_PREFIX;     SHOWDEBUG ("error %d opening %s: %s\n", errno, portname, strerror (errno));
	        close(*fd);
	        return 0;
	}
	SHOWDEBUG_PREFIX;	SHOWDEBUG("Openned port (fd = %d)\n", *fd);	

	return 1;
}
void Close_port(int* fd)
{
	*fd = -1;
	close(*fd);
}
double time_diff(struct timeval x , struct timeval y)
{
    double x_ms , y_ms , diff;
     
    x_ms = (double)x.tv_sec*1000000 + (double)x.tv_usec;
    y_ms = (double)y.tv_sec*1000000 + (double)y.tv_usec;
     
    diff = (double)y_ms - (double)x_ms;
     
    return diff;
}
int Serial_read(int* fd, uint8_t* buf, int size_to_read)
{
	int read_fd = *fd;
	int count = 0;
	while (read (read_fd, buf+count, 1)>0 && buf[count] != '\n' && count < size_to_read)
	{
		count++;
	}
	if (buf[count] == '\n') count++;
	//printf("<count %d [%c]>", count, buf[14]);
	return count;
}

/*int Serial_print(char* data, int size)
{
	Open_port();
	if (fd<0){
		SHOWDEBUG_PREFIX;SHOWDEBUG("Port not opened: %s \n", portname);
		return 0;
	}
	int wr = write (fd, data, size);
	if (wr)
	{
		SHOWDEBUG_PREFIX;SHOWDEBUG("Transmitted %d bytes. ", wr);
		int i;
		for (i=0;i<wr;i++)
		{
			SHOWDEBUG_PREFIX;SHOWDEBUG("%c",data[i]);
		}
		SHOWDEBUG("\n");
	}else{
	SHOWDEBUG_PREFIX;SHOWDEBUG("Send frame error.\n");
			return 0;
	}


}*/
