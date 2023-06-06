#ifndef VL6180X_H
#define VL6180X_H
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "main.h"

/* Split 16-bit register address into two bytes and write the address + data via I2C */
void WriteByte(uint16_t reg, uint8_t data);

/* Split 16-bit register address into two bytes and write required register address to VL6180X and read the data back */
int ReadByte(uint16_t reg);

/* Special initialization sequence for VL6180x */
void VL_InitDevice(void);

int VL6180X_Init(void);

/* Start a range measurement in continuous mode */
int VL6180X_Start_Range(void);

/* poll for new sample ready ready */
int VL6180X_Poll_Range(void);

/* Read range result (mm) */
int VL6180X_Read_Range(void);

/* clear interrupts */
int VL6180X_Clear_Interrupts(void);


#ifdef __cplusplus
}
#endif
#endif /* VL6180X_H */
