
#include "Arduino.h"

#include "MyTimeLib.h"

const uint8_t daysArray [] PROGMEM = { 31,28,31,30,31,30,31,31,30,31,30,31 };
const uint8_t dowArray[] PROGMEM = { 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };


uint32_t DateTimeToUnixtime(DateTime dt)
{
    uint32_t u;

    u = time2long(date2days(dt.year, dt.month, dt.day), dt.hour, dt.minute, dt.second);
    u += 946681200;
    return u;
}

uint32_t dtToUnixtime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second)
{
    uint32_t u;

    u = time2long(date2days(year, month, day), hour, minute, second);
    u += 946681200;

    return u;
}
DateTime UnixtimeToDateTime(uint32_t t)
{
    t -= 946684800;
    DateTime dt;

    dt.second = t % 60;
    t /= 60;

    dt.minute = t % 60;
    t /= 60;

    dt.hour = t % 24;
    uint16_t days = t / 24;
    uint8_t leap;

    for (dt.year = 0; ; ++dt.year)
    {
        leap = dt.year % 4 == 0;
        if (days < 365 + leap)
        {
            break;
        }
        days -= 365 + leap;
    }

    for (dt.month = 1; ; ++dt.month)
    {
        uint8_t daysPerMonth = pgm_read_byte(daysArray + dt.month - 1);

        if (leap && dt.month == 2)
        {
            ++daysPerMonth;
        }

        if (days < daysPerMonth)
        {
            break;
        }
        days -= daysPerMonth;
    }

    dt.day = days + 1;
    dt.dayOfWeek =dow(dt.year, dt.month, dt.day);
    return dt;
}


void dateFormat(char* buffer,const char* dateFormat, uint32_t time)
{
    DateTime  dt = UnixtimeToDateTime(time);

    char helper[11];

    while (*dateFormat != '\0')
    {
        switch (dateFormat[0])
        {
            // Day decoder
            case 'd':
                sprintf(helper, "%02d", dt.day); 
                strcat(buffer, (const char *)helper); 
                break;
            case 'j':
                sprintf(helper, "%d", dt.day);
                strcat(buffer, (const char *)helper);
                break;
            case 'l':
                strcat(buffer, (const char *)strDayOfWeek(dt.dayOfWeek));
                break;
            case 'D':
                strncat(buffer, strDayOfWeek(dt.dayOfWeek), 2);
                break;
            case 'N':
                sprintf(helper, "%d", dt.dayOfWeek);
                strcat(buffer, (const char *)helper);
                break;
            case 'w':
                sprintf(helper, "%d", (dt.dayOfWeek + 7) % 7);
                strcat(buffer, (const char *)helper);
                break;
            case 'z':
                sprintf(helper, "%d", dayInYear(dt.year, dt.month, dt.day));
                strcat(buffer, (const char *)helper);
                break;
            case 'S':
                strcat(buffer, (const char *)strDaySufix(dt.day));
                break;

            // Month decoder
            case 'm':
                sprintf(helper, "%02d", dt.month);
                strcat(buffer, (const char *)helper);
                break;
            case 'n':
                sprintf(helper, "%d", dt.month);
                strcat(buffer, (const char *)helper);
                break;
            case 'F':
                strcat(buffer, (const char *)strMonth(dt.month));
                break;
            case 'M':
                strncat(buffer, (const char *)strMonth(dt.month), 3);
                break;
            case 't':
                sprintf(helper, "%d", daysInMonth(dt.year, dt.month));
                strcat(buffer, (const char *)helper);
                break;

            // Year decoder
            case 'Y':
                sprintf(helper, "%d", dt.year); 
                strcat(buffer, (const char *)helper); 
                break;
            case 'y': sprintf(helper, "%02d", dt.year-2000);
                strcat(buffer, (const char *)helper);
                break;
            case 'L':
                sprintf(helper, "%d", isLeapYear(dt.year)); 
                strcat(buffer, (const char *)helper); 
                break;

            // Hour decoder
            case 'H':
                sprintf(helper, "%02d", dt.hour);
                strcat(buffer, (const char *)helper);
                break;
            case 'G':
                sprintf(helper, "%d", dt.hour);
                strcat(buffer, (const char *)helper);
                break;
            case 'h':
                sprintf(helper, "%02d", hour12(dt.hour));
                strcat(buffer, (const char *)helper);
                break;
            case 'g':
                sprintf(helper, "%d", hour12(dt.hour));
                strcat(buffer, (const char *)helper);
                break;
            case 'A':
                strcat(buffer, (const char *)strAmPm(dt.hour, true));
                break;
            case 'a':
                strcat(buffer, (const char *)strAmPm(dt.hour, false));
                break;

            // Minute decoder
            case 'i': 
                sprintf(helper, "%02d", dt.minute);
                strcat(buffer, (const char *)helper);
                break;

            // Second decoder
            case 's':
                sprintf(helper, "%02d", dt.second); 
                strcat(buffer, (const char *)helper); 
                break;

            // Misc decoder
            //case 'U': 
            //    sprintf(helper, "%lu", dt.unixtime);
            //    strcat(buffer, (const char *)helper);
            //    break;

            default: 
                strncat(buffer, dateFormat, 1);
                break;
        }
        dateFormat++;
    }

}


char* strDayOfWeek(uint8_t dayOfWeek)
{
    switch (dayOfWeek) {
        case 1:
            return "ПН";
            break;
        case 2:
            return "ВТ";
            break;
        case 3:
            return "СР";
            break;
        case 4:
            return "ЧТ";
            break;
        case 5:
            return "ПТ";
            break;
        case 6:
            return "СБ";
            break;
        case 7:
            return "ВС";
            break;
        default:
            return "Unknown";
    }
}

char* strMonth(uint8_t month)
{
    switch (month) {
        case 1:
            return "January";
            break;
        case 2:
            return "February";
            break;
        case 3:
            return "March";
            break;
        case 4:
            return "April";
            break;
        case 5:
            return "May";
            break;
        case 6:
            return "June";
            break;
        case 7:
            return "July";
            break;
        case 8:
            return "August";
            break;
        case 9:
            return "September";
            break;
        case 10:
            return "October";
            break;
        case 11:
            return "November";
            break;
        case 12:
            return "December";
            break;
        default:
            return "Unknown";
    }
}

char* strAmPm(uint8_t hour, bool uppercase)
{
    if (hour < 12)
    {
        if (uppercase)
        {
            return "AM";
        } else
        {
            return "am";
        }
    } else
    {
        if (uppercase)
        {
            return "PM";
        } else
        {
            return "pm";
        }
    }
}

char* strDaySufix(uint8_t day)
{
    if (day % 10 == 1)
    {
        return "st";
    } else
    if (day % 10 == 2)
    {
        return "nd";
    }
    if (day % 10 == 3)
    {
        return "rd";
    }

    return "th";
}

uint8_t hour12(uint8_t hour24)
{
    if (hour24 == 0)
    {
        return 12;
    }

    if (hour24 > 12)
    {
       return (hour24 - 12);
    }

    return hour24;
}

long time2long(uint16_t days, uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    return ((days * 24L + hours) * 60 + minutes) * 60 + seconds;
}

uint16_t dayInYear(uint16_t year, uint8_t month, uint8_t day)
{
    uint16_t fromDate;
    uint16_t toDate;

    fromDate = date2days(year, 1, 1);
    toDate = date2days(year, month, day);

    return (toDate - fromDate);
}

bool isLeapYear(uint16_t year)
{
    return (year % 4 == 0);
}

uint8_t daysInMonth(uint16_t year, uint8_t month)
{
    uint8_t days;

    days = pgm_read_byte(daysArray + month - 1);

    if ((month == 2) && isLeapYear(year))
    {
        ++days;
    }

    return days;
}

uint16_t date2days(uint16_t year, uint8_t month, uint8_t day)
{
    year = year - 2000;

    uint16_t days16 = day;

    for (uint8_t i = 1; i < month; ++i)
    {
        days16 += pgm_read_byte(daysArray + i - 1);
    }

    if ((month > 2) && isLeapYear(year))
    {
        ++days16;
    }

    return days16 + 365 * year + (year + 3) / 4 - 1;
}



uint8_t conv2d(const char* p)
{
    uint8_t v = 0;

    if ('0' <= *p && *p <= '9')
    {
        v = *p - '0';
    }

    return 10 * v + *++p - '0';
}

uint8_t dow(uint16_t y, uint8_t m, uint8_t d)
{
    uint8_t dow;

    y -= m < 3;
    dow = ((y + y/4 - y/100 + y/400 + pgm_read_byte(dowArray+(m-1)) + d) % 7);

    if (dow == 0)
    {
        return 7;
    }

    return dow;
}

