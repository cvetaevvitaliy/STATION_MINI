#include "ds3231.h"

extern I2C_HandleTypeDef DS3231_I2C;

uint8_t rtcBuffer[19];

uint8_t decToBcd(uint8_t val) {	return ((val / 10 * 16) + (val % 10)); }

uint8_t bcdToDec(uint8_t val) {	return ((val / 16 * 10) + (val % 16)); }

void DS3231_Update(void)
{
	uint8_t cmd = 0;
	HAL_I2C_Master_Transmit(&hi2c1, DS3231_ADDRESS, &cmd, 1, DS3231_I2C_TIMEOUT);
	HAL_I2C_Master_Receive(&hi2c1, DS3231_ADDRESS, rtcBuffer, 19, DS3231_I2C_TIMEOUT);
}

uint8_t DS3231_getSec(void) {	return bcdToDec(rtcBuffer[0]); }

uint8_t DS3231_getMin(void) {	return bcdToDec(rtcBuffer[1]); }

uint8_t DS3231_getHrs(void) {	return bcdToDec(rtcBuffer[2]); }

uint8_t DS3231_getDay(void) { return bcdToDec(rtcBuffer[3]); }

uint8_t DS3231_getDate(void) { return bcdToDec(rtcBuffer[4]); }

uint8_t DS3231_getMonth(void) {	return bcdToDec(rtcBuffer[5]); }

uint8_t DS3231_getYear(void) { return bcdToDec(rtcBuffer[6]); }

uint8_t DS3231_getAlarm1Sec(void) { return bcdToDec(rtcBuffer[7]); }

uint8_t DS3231_getAlarm1Min(void) { return bcdToDec(rtcBuffer[8]); }

uint8_t DS3231_getAlarm1Hour(void) { return bcdToDec(rtcBuffer[9]); }

uint8_t DS3231_getAlarm1Day(void) { return bcdToDec(rtcBuffer[10]); }

uint8_t DS3231_getAlarm1Date(void) { return bcdToDec(rtcBuffer[11]); }

uint8_t DS3231_getAlarm2Min(void) { return bcdToDec(rtcBuffer[12]); }

uint8_t DS3231_getAlarm2Hour(void) { return bcdToDec(rtcBuffer[13]); }

uint8_t DS3231_getAlarm2Day(void) { return bcdToDec(rtcBuffer[14]); }

uint8_t DS3231_getAlarm2Date(void) { return bcdToDec(rtcBuffer[15]); }

float DS3231_getTemp(void) 
{
	uint8_t tempMSB = rtcBuffer[17];
	uint8_t tempLSB = rtcBuffer[18];
	float t = 0.0;
	tempLSB >>= 6;                  
	tempLSB &= 0x03;      
	t = ((float)tempLSB);  
	t *= 0.25;      
	t += tempMSB;          
	return t; 
}
