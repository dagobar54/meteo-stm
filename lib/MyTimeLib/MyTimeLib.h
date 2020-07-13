
#ifndef MYTIMELIB_h
#define MYTIMELIB_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


#ifndef DATETIME_STRUCT_H
#define DATETIME_STRUCT_H
struct DateTime
{
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t dayOfWeek;
};
#endif

uint32_t dtToUnixtime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second);
uint32_t DateTimeToUnixtime(DateTime dt);
DateTime UnixtimeToDateTime(uint32_t t);

void dateFormat(char* buffer,const char* dateFormat, uint32_t time);

char *strDayOfWeek(uint8_t dayOfWeek);
char *strMonth(uint8_t month);
char *strAmPm(uint8_t hour, bool uppercase);
char *strDaySufix(uint8_t day);

uint8_t hour12(uint8_t hour24);

long time2long(uint16_t days, uint8_t hours, uint8_t minutes, uint8_t seconds);
uint16_t date2days(uint16_t year, uint8_t month, uint8_t day);
uint8_t daysInMonth(uint16_t year, uint8_t month);
uint16_t dayInYear(uint16_t year, uint8_t month, uint8_t day);
bool isLeapYear(uint16_t year);
uint8_t dow(uint16_t y, uint8_t m, uint8_t d);

#endif