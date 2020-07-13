#include "UARTTransfer.h"



UARTTransfer::UARTTransfer()
{
}



UARTTransfer::~UARTTransfer()
{
}

void UARTTransfer::begin(Stream *theStream) {
	_stream = theStream;

}

boolean UARTTransfer::receiveData() {
	uint8_t mess_len = 0;
	uint8_t in_char;

	if (rx_stage > 5) return false;
	if ((rx_stage) && ((millis() - rx_time) > rx_timeout)) {
		clearMessage();
		
	}
	if (!(_stream->available())) return false;
	else mess_len = _stream->available();

	while (mess_len) {
		in_char = _stream->read();
		mess_len--;
		if (rx_stage == 0) {
			
			while ((in_char != head[rx_stage]) && (mess_len)) {
				in_char = _stream->read();
				mess_len--;
			}
			if (in_char != head[rx_stage]) return false;
			rx_stage++;
			rx_time = millis();
			

		}
		else if (rx_stage < 3) {
			
			if (in_char != head[rx_stage]) {
				clearMessage();
				
			}
			else rx_stage++;
			
		}
		else {
			
			if (rx_stage == 3) {
				mess_id = in_char;
				rx_stage++;

			}
			else if (rx_stage == 4) {
				mess_data_size = in_char;
				if (mess_data_size < UT_MIN_MESS_SIZE) {
					clearMessage();
					
				}
				else rx_stage++;

			}
			else if (rx_stage == 5) {

				rx_buf[buf_ind++] = in_char;
				if (buf_ind == mess_data_size) {
					mess_data_size--;
					uint8_t calc_CS = mess_data_size;
					for (uint8_t i = 0; i < mess_data_size; i++) {
						calc_CS ^= rx_buf[i];
					}
					if (calc_CS == rx_buf[mess_data_size]) { //CS good
						rx_stage++;
						return true;
					}
					clearMessage();
				}

			}
			
		}
	}
	return false;
}

void UARTTransfer::sendData(uint8_t send_id, uint8_t size, uint8_t* data) {
	uint8_t CS = size;
	for (uint8_t i = 0; i < 3; i++) {
		_stream->write(head[i]);
	}
	_stream->write(send_id);
	_stream->write(size + 1);
	for (uint8_t i = 0; i < size; i++) {
		CS ^= *(data + i);
		_stream->write(*(data + i));
	}
	_stream->write(CS);
}

uint8_t UARTTransfer::getData(uint8_t* dest, uint8_t size) {
	memcpy(dest, rx_buf, size);
	return size;
}
void UARTTransfer::clearMessage() {
	rx_stage = 0;
	mess_id = 0;
	mess_data_size = 0;
	buf_ind = 0;
}


