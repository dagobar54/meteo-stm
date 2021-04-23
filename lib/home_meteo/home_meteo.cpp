#include "home_meteo.h"

//RF24EventStack::RF24EventStack()
//{
//	firstNo=0;
//	lastNo=0;
//}
void RF24EventStack::push(uint8_t pipeNo)
{
	stack[lastNo]=pipeNo;
	if (lastNo == sizeof(stack)-1){
		lastNo=0;
	}
	else
		lastNo++;

}
uint8_t RF24EventStack::pop()
{
	uint8_t pos = firstNo;
	if (firstNo==sizeof(stack)-1)
		firstNo=0;
	else
		firstNo++;
	return stack[pos];
}
bool RF24EventStack::isEmpty()
{
	return firstNo == lastNo;
}
void RF24EventStack::clear()
{
	firstNo = lastNo = 0;
}

void unixtimeToString(uint32_t unixT,char* str)
{
    //unixT -= 946681200;
	unixT -= 946684800;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;

    second = unixT % 60;
    unixT /= 60;

    minute = unixT % 60;
    unixT /= 60;

    hour = unixT % 24;

	sprintf(str,"%i:%i:%i",hour,minute,second);
	//Serial.println(str);
	//delay(50);
}