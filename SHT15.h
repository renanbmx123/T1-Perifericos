#ifndef __SHT15_H__
#define __SHT15_H__
#include "stm32f4xx_hal.h"

enum {TEMP, HUMI};

/* GPIO related macro definition */
#define SHT15_DATA_PIN        GPIO_PIN_2
#define SHT15_SCK_PIN         GPIO_PIN_3
#define SHT15_DATA_PORT       GPIOG
#define SHT15_SCK_PORT        GPIOG

#define SHT15_DATA_H        GPIOG->BSRR = (1<<2)                   				     				//Pull the DATA data line
#define SHT15_DATA_L        GPIOG->BSRR = 1<<(2+16)                           				//Pull down the DATA data line
#define SHT15_DATA_R()      HAL_GPIO_ReadPin(SHT15_DATA_PORT, SHT15_DATA_PIN)         //Read the DATA line

#define SHT15_SCK_H       GPIOG->BSRR = (1<<3)                       									 //Pull high SCK clock line
#define SHT15_SCK_L       GPIOG->BSRR = 1<<(3+16)                         						 //Pull down the SCK clock line

/* Sensor-related macro definitions */
#define noACK               0
#define ACK                 1
																				//addr       command       r/w

#define MEASURE_TEMP        0x03        //000         0001          1          measure temperature
#define MEASURE_HUMI        0x05        //000         0010          1          Measuring humidity
#define SOFTRESET           0x1E        //000         1111          0          Reset

void SHT15_Init(void);

void SHT15_TransStart(void);
void SHT15_DATAIn(void);
void SHT15_DATAOut(void);
uint8_t SHT15_ReadByte(uint8_t Ack);
uint8_t SHT15_WriteByte(uint8_t value);

uint8_t SHT15_Measure(uint16_t *p_value, uint8_t mode);
void SHT15_Calculate(uint16_t t, uint16_t rh,float *p_temperature, float *p_humidity);
float SHT15_CalcuDewPoint(float t, float h);


#endif
