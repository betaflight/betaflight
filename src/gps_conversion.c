#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>


#define DIGIT_TO_VAL(_x)    (_x - '0')
uint32_t GPS_coord_to_degrees(char* coordinateString)
{
    char *fieldSeparator, *remainingString;
    uint8_t degress = 0, minutes = 0;
    uint16_t fractionalMinutes = 0;
    uint8_t digitIndex;

    // scan for decimal point or end of field
    for (fieldSeparator = coordinateString; isdigit((unsigned char)*fieldSeparator); fieldSeparator++) {
        if (fieldSeparator >= coordinateString + 15)
            return 0; // stop potential fail
    }
    remainingString = coordinateString;

    // convert degrees
    while ((fieldSeparator - remainingString) > 2) {
        if (degress)
            degress *= 10;
        degress += DIGIT_TO_VAL(*remainingString++);
    }
    // convert minutes
    while (fieldSeparator > remainingString) {
        if (minutes)
            minutes *= 10;
        minutes += DIGIT_TO_VAL(*remainingString++);
    }
    // convert fractional minutes
    // expect up to four digits, result is in
    // ten-thousandths of a minute
    if (*fieldSeparator == '.') {
        remainingString = fieldSeparator + 1;
        for (digitIndex = 0; digitIndex < 4; digitIndex++) {
            fractionalMinutes *= 10;
            if (isdigit((unsigned char)*remainingString))
                fractionalMinutes += *remainingString++ - '0';
        }
    }
    return degress * 10000000UL + (minutes * 1000000UL + fractionalMinutes * 100UL) / 6;
}
