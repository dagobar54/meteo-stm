#pragma once
//Not necessary, but just in case.
#if ARDUINO > 22
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "Stream.h"

#define UT_BUFFER_SIZE 128
#define UT_MIN_MESS_SIZE 3
#define UT_HEADER1 0xb7
#define UT_HEADER2 0x07
#define UT_HEADER3 0x33

class UARTTransfer
{
public:
	UARTTransfer();
	~UARTTransfer();
	void begin(Stream *theStream);
	void sendData(uint8_t send_id, uint8_t size, uint8_t* data);
	bool receiveData();
	uint8_t getMessageID() { return mess_id;};
	uint8_t getDataLength() { return mess_data_size; };
	uint8_t getData(uint8_t* dest, uint8_t size);
	void clearMessage();
	Stream *_stream;
	
private:
	uint8_t rx_buf[UT_BUFFER_SIZE];
	uint8_t buf_ind = 0;
	uint16_t rx_time;
	uint16_t rx_timeout = 500;
	uint8_t  rx_stage =0;
	uint8_t head[3] = { UT_HEADER1 , UT_HEADER2, UT_HEADER3 };
	uint8_t mess_id, mess_data_size;


};

