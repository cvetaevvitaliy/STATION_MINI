#ifndef DS3231_H_
#define DS3231_H_

#include "stm32f1xx_hal.h"

#define DS3231_I2C 				hi2c1
#define DS3231_I2C_TIMEOUT 	100
#define DS3231_ADDRESS     0xD0

uint8_t decToBcd(uint8_t val);
uint8_t bcdToDec(uint8_t val);

void DS3231_Update(void);
uint8_t DS3231_getSec(void);
uint8_t DS3231_getMin(void);
uint8_t DS3231_getHrs(void);
uint8_t DS3231_getDay(void);
uint8_t DS3231_getDate(void);
uint8_t DS3231_getMonth(void);
uint8_t DS3231_getYear(void);
uint8_t DS3231_getAlarm1Sec(void);
uint8_t DS3231_getAlarm1Min(void);
uint8_t DS3231_getAlarm1Hour(void);
uint8_t DS3231_getAlarm1Day(void);
uint8_t DS3231_getAlarm1Date(void);
uint8_t DS3231_getAlarm2Min(void);
uint8_t DS3231_getAlarm2Hour(void);
uint8_t DS3231_getAlarm2Day(void);
uint8_t DS3231_getAlarm2Date(void);
float DS3231_getTemp(void);


#endif /* DS3231_H_ */
