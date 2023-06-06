/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directoLy of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bno055.h"
#include "string.h"
#include "queue.h"
#include "VL6180X.h"
#include <stdlib.h>

/* printf function */
#include <stdio.h>
//#ifdef __GNUC__
//#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
/*end of printf function*/
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Create an instance of the bno055_dev structure for each IMU
bno055_t imu1;
bno055_t imu2;
error_bno IMU1PASS = BNO_ERR_I2C;
error_bno IMU2PASS = BNO_ERR_I2C;
int VL6180X_ERR = 0;
//f32 volatile gyro1_x, gyro1_y, gyro1_z;
//f32 volatile gyro2_x, gyro2_y, gyro2_z;
f32 euler1_x, euler1_y, euler1_z;
f32 volatile init_euler1_x, init_euler1_y, init_euler1_z;
f32 volatile euler2_x, euler2_y, euler2_z;
f32 volatile init_euler2_x, init_euler2_y, init_euler2_z;
f32 volatile test_x, test_y, test_z;

//f32 volatile a1_x, a1_y, a1_z;
f32 volatile a2_x, a2_y, a2_z;
f32 volatile absx = 0;
f32 volatile absy = 0;
f32 volatile absz = 0;
f32 volatile change_x = 0;
f32 volatile change_y = 0;
f32 volatile change_z = 0;
f32 volatile accleration_mag_buffer = 0; //stoLe the total acceleration

uint16_t volatile readFSR1;
uint16_t volatile readFSR2;

f32 max_change = 0;
int rotate_xyz = 0; // x == 1, y == 2, z == 3

/* FSR related */
#define FSR_threshold 0x0A00
uint8_t FSR1_pressed = 0;
uint8_t FSR2_pressed = 0;
/* STATES */
#define IDLE 0x00
#define WAIT_ZOOM_IN 0x01
#define WAIT_ZOOM_OUT 0x02
// #define WAIT_SHIFT 0x03
#define WAIT_ROTATE 0x04
#define ZOOM_IN_2 0x05
#define ZOOM_IN_4 0x06
#define ZOOM_OUT_2 0x07
#define ZOOM_OUT_4 0x08
#define WAIT_SHIFT_L 0x09
#define WAIT_SHIFT_R 0x0a
#define WAIT_SHIFT_U 0x0b
#define WAIT_SHIFT_D 0x0c
#define WAIT_SHIFT_F 0x0d
#define WAIT_SHIFT_B 0x0e
#define WAIT_SHIFT_L_DONE 0x0f 
#define WAIT_SHIFT_R_DONE 0x10
#define WAIT_SHIFT_U_DONE 0x11
#define WAIT_SHIFT_D_DONE 0x12
#define WAIT_SHIFT_F_DONE 0x13
#define WAIT_SHIFT_B_DONE 0x14
#define ROTATE_X 0x2d
#define ROTATE_Y 0x2e
#define ROTATE_Z 0x2f
/* gesture command */
#define CMD_LEFT 0x00 
#define CMD_RIGHT 0x01
#define CMD_UP 0x02 
#define CMD_DOWN 0x03
#define CMD_FRONT 0x04
#define CMD_BACK 0x05
#define CMD_ROTATE_X 0x06
#define CMD_ROTATE_Y 0x07
#define CMD_ROTATE_Z 0x08
#define CMD_ZOOM 0x09
#define CMD_RESET 0x0a
#define CMD_NONE 0xff
/* gesture data */
#define DATA_SHIFT_SPEED_1 0x01
#define DATA_SHIFT_SPEED_2 0x02
#define DATA_SHIFT_SPEED_3 0x03
#define DATA_ZOOM_IN_2 0x02
#define DATA_ZOOM_IN_4 0x03
#define DATA_ZOOM_OUT_2 0x05
#define DATA_ZOOM_OUT_4 0x06
#define DATA_NO_ZOOM_IN_OUT 0x04
#define DATA_RESET 0x00
#define DATA_NONE 0xff
/* FSR behavior */
#define ROTATE_MODE (FSR1_pressed == 1 && FSR2_pressed == 1)
#define ZOOM_MODE (FSR1_pressed == 0 && FSR2_pressed == 1)
#define SHIFT_MODE (FSR1_pressed == 1 && FSR2_pressed == 0)
/* Shift speed threshold */
#define TH_SPEED_BASE 1
// #define TH_SPEED_1 10
#define TH_SPEED_2 10
#define TH_SPEED_3 15
#define DELAY_IN_DONE 1
#define SHIFT_STABILIZED ((absx<TH_SPEED_BASE)&(absy<TH_SPEED_BASE)&(absz<TH_SPEED_BASE))
// toleranace are in degrees
#define TH_GESTURE_START_TOL 25
#define TH_GESTURE_START_TOL_UD 50
#define TH_GESTURE_START_TOL_L 35
#define TH_GESTURE_START_TOL_L_Y 90
#define TH_GESTURE_START_TOL_R 35
#define TH_GESTURE_START_TOL_R_Y 90
#define TH_GESTURE_START_TOL_R_Z 55
#define TH_GESTURE_START_TOL_FB_Y 50
#define TH_GESTURE_START_TOL_FB_Z 85

#define TH_GESTURE_END_TOL 25 
#define TH_GESTURE_END_TOL_UD 90
#define TH_GESTURE_END_TOL_L 35
#define TH_GESTURE_END_TOL_L_Y 90
#define TH_GESTURE_END_TOL_R 35
#define TH_GESTURE_END_TOL_R_Y 90
#define TH_GESTURE_END_TOL_R_Z 55
#define TH_GESTURE_END_TOL_FB_Y 50
#define TH_GESTURE_END_TOL_FB_Z 85

/* Shift gesture definition*/ // should all be a very clean number, in 90 degrees
#define L_1X 0
#define L_1Y -90
#define L_1Z 270
#define L_2X 0
#define L_2Y 90
#define L_2Z 270
#define R_1X 0
#define R_1Y -90
#define R_1Z 90
#define R_2X 0
#define R_2Y 90
#define R_2Z 90
#define U_1X 90
#define U_1Y 0
#define U_1Z 0
#define U_2X -90
#define U_2Y 0
#define U_2Z 0
#define D_1X -90
#define D_1Y 0
#define D_1Z 0
#define D_2X 90
#define D_2Y 0
#define D_2Z 0
#define F_1X 0
#define F_1Y -90
#define F_1Z 0
#define F_2X 0
#define F_2Y 90
#define F_2Z 0
#define B_1X 0
#define B_1Y -90
#define B_1Z 0
#define B_2X 0
#define B_2Y 90
#define B_2Z 270
#define GESTURE_L_STARTING ((L_1X-TH_GESTURE_START_TOL_L<euler1_x)&(euler1_x<L_1X+TH_GESTURE_START_TOL_L)&(L_1Y-TH_GESTURE_START_TOL_L_Y<euler1_y)&(euler1_y<L_1Y+TH_GESTURE_START_TOL_L_Y)&(L_1Z-TH_GESTURE_START_TOL_L<euler1_z)&(euler1_z<L_1Z+TH_GESTURE_START_TOL_L)&(L_2X-TH_GESTURE_START_TOL_L<euler2_x)&(euler2_x<L_2X+TH_GESTURE_START_TOL_L)&(L_2Y-TH_GESTURE_START_TOL_L_Y<euler2_y)&(euler2_y<L_2Y+TH_GESTURE_START_TOL_L_Y)&(L_2Z-TH_GESTURE_START_TOL_L<euler2_z)&(euler2_z<L_2Z+TH_GESTURE_START_TOL_L))
#define GESTURE_R_STARTING ((R_1X-TH_GESTURE_START_TOL_R<euler1_x)&(euler1_x<R_1X+TH_GESTURE_START_TOL_R)&(R_1Y-TH_GESTURE_START_TOL_R_Y<euler1_y)&(euler1_y<R_1Y+TH_GESTURE_START_TOL_R_Y)&(R_1Z-TH_GESTURE_START_TOL_R_Z<euler1_z)&(euler1_z<R_1Z+TH_GESTURE_START_TOL_R_Z)&(R_2X-TH_GESTURE_START_TOL_R<euler2_x)&(euler2_x<R_2X+TH_GESTURE_START_TOL_R)&(R_2Y-TH_GESTURE_START_TOL_R_Y<euler2_y)&(euler2_y<R_2Y+TH_GESTURE_START_TOL_R_Y)&(R_2Z-TH_GESTURE_START_TOL_R_Z<euler2_z)&(euler2_z<R_2Z+TH_GESTURE_START_TOL_R_Z))
#define GESTURE_U_STARTING ((U_1X-TH_GESTURE_START_TOL_UD<euler1_x)&(euler1_x<U_1X+TH_GESTURE_START_TOL_UD)&(U_2X-TH_GESTURE_START_TOL_UD<euler2_x)&(euler2_x<U_2X+TH_GESTURE_START_TOL_UD))
#define GESTURE_D_STARTING ((D_1X-TH_GESTURE_START_TOL_UD<euler1_x)&(euler1_x<D_1X+TH_GESTURE_START_TOL_UD)&(D_2X-TH_GESTURE_START_TOL_UD<euler2_x)&(euler2_x<D_2X+TH_GESTURE_START_TOL_UD))
#define GESTURE_F_STARTING ((F_1X-TH_GESTURE_START_TOL<euler1_x)&(euler1_x<F_1X+TH_GESTURE_START_TOL)&(F_1Y-TH_GESTURE_START_TOL<euler1_y)&(euler1_y<F_1Y+TH_GESTURE_START_TOL)&((euler1_z < TH_GESTURE_START_TOL)||(euler1_z > 360-TH_GESTURE_START_TOL))&(F_2X-TH_GESTURE_START_TOL<euler2_x)&(euler2_x<F_2X+TH_GESTURE_START_TOL)&(F_2Y-TH_GESTURE_START_TOL<euler2_y)&(euler2_y<F_2Y+TH_GESTURE_START_TOL)&((euler2_z < TH_GESTURE_START_TOL)||(euler2_z > 360-TH_GESTURE_START_TOL)))
#define GESTURE_B_STARTING ((B_1X-TH_GESTURE_START_TOL<euler1_x)&(euler1_x<B_1X+TH_GESTURE_START_TOL)&(B_1Y-TH_GESTURE_START_TOL<euler1_y)&(euler1_y<B_1Y+TH_GESTURE_START_TOL)&((euler1_z < TH_GESTURE_START_TOL)||(euler1_z > 360-TH_GESTURE_START_TOL))&(B_2X-TH_GESTURE_START_TOL<euler2_x)&(euler2_x<B_2X+TH_GESTURE_START_TOL)&(B_2Y-TH_GESTURE_START_TOL_FB_Y<euler2_y)&(euler2_y<B_2Y+TH_GESTURE_START_TOL_FB_Y)&(B_2Z-TH_GESTURE_START_TOL_FB_Z<euler2_z)&(euler2_z<B_2Z+TH_GESTURE_START_TOL))

#define GESTURE_L_ENDING ((L_1X-TH_GESTURE_END_TOL_L<euler1_x)&(euler1_x<L_1X+TH_GESTURE_END_TOL_L)&(L_1Y-TH_GESTURE_END_TOL_L_Y<euler1_y)&(euler1_y<L_1Y+TH_GESTURE_END_TOL_L_Y)&(L_1Z-TH_GESTURE_END_TOL_L<euler1_z)&(euler1_z<L_1Z+TH_GESTURE_END_TOL_L)&(L_2X-TH_GESTURE_END_TOL_L<euler2_x)&(euler2_x<L_2X+TH_GESTURE_END_TOL_L)&(L_2Y-TH_GESTURE_END_TOL_L_Y<euler2_y)&(euler2_y<L_2Y+TH_GESTURE_END_TOL_L_Y)&(L_2Z-TH_GESTURE_END_TOL_L<euler2_z)&(euler2_z<L_2Z+TH_GESTURE_END_TOL_L))
#define GESTURE_R_ENDING ((R_1X-TH_GESTURE_END_TOL_R<euler1_x)&(euler1_x<R_1X+TH_GESTURE_END_TOL_R)&(R_1Y-TH_GESTURE_END_TOL_R_Y<euler1_y)&(euler1_y<R_1Y+TH_GESTURE_END_TOL_R_Y)&(R_1Z-TH_GESTURE_END_TOL_R_Z<euler1_z)&(euler1_z<R_1Z+TH_GESTURE_END_TOL_R_Z)&(R_2X-TH_GESTURE_END_TOL_R<euler2_x)&(euler2_x<R_2X+TH_GESTURE_END_TOL_R)&(R_2Y-TH_GESTURE_END_TOL_R_Y<euler2_y)&(euler2_y<R_2Y+TH_GESTURE_END_TOL_R_Y)&(R_2Z-TH_GESTURE_END_TOL_R_Z<euler2_z)&(euler2_z<R_2Z+TH_GESTURE_END_TOL_R_Z))
#define GESTURE_U_ENDING ((U_1X-TH_GESTURE_END_TOL_UD<euler1_x)&(euler1_x<U_1X+TH_GESTURE_END_TOL_UD)&(U_2X-TH_GESTURE_END_TOL_UD<euler2_x)&(euler2_x<U_2X+TH_GESTURE_END_TOL_UD))
#define GESTURE_D_ENDING ((D_1X-TH_GESTURE_END_TOL_UD<euler1_x)&(euler1_x<D_1X+TH_GESTURE_END_TOL_UD)&(D_2X-TH_GESTURE_END_TOL_UD<euler2_x)&(euler2_x<D_2X+TH_GESTURE_END_TOL_UD))
#define GESTURE_F_ENDING ((F_1X-TH_GESTURE_END_TOL<euler1_x)&(euler1_x<F_1X+TH_GESTURE_END_TOL)&(F_1Y-TH_GESTURE_END_TOL<euler1_y)&(euler1_y<F_1Y+TH_GESTURE_END_TOL)&((euler1_z < TH_GESTURE_END_TOL)||(euler1_z > 360-TH_GESTURE_END_TOL))&(F_2X-TH_GESTURE_END_TOL<euler2_x)&(euler2_x<F_2X+TH_GESTURE_END_TOL)&(F_2Y-TH_GESTURE_END_TOL<euler2_y)&(euler2_y<F_2Y+TH_GESTURE_END_TOL)&((euler2_z < TH_GESTURE_END_TOL)||(euler2_z > 360-TH_GESTURE_END_TOL)))
#define GESTURE_B_ENDING ((B_1X-TH_GESTURE_END_TOL<euler1_x)&(euler1_x<B_1X+TH_GESTURE_END_TOL)&(B_1Y-TH_GESTURE_END_TOL<euler1_y)&(euler1_y<B_1Y+TH_GESTURE_END_TOL)&((euler1_z < TH_GESTURE_END_TOL)||(euler1_z > 360-TH_GESTURE_END_TOL))&(B_2X-TH_GESTURE_END_TOL<euler2_x)&(euler2_x<B_2X+TH_GESTURE_END_TOL)&(B_2Y-TH_GESTURE_END_TOL_FB_Y<euler2_y)&(euler2_y<B_2Y+TH_GESTURE_END_TOL_FB_Y)&(B_2Z-TH_GESTURE_END_TOL_FB_Z<euler2_z)&(euler2_z<B_2Z+TH_GESTURE_END_TOL))

/* Rotate calibration */
#define X_SIGN_CALIBRATE 1
#define Y_SIGN_CALIBRATE 1
#define Z_SIGN_CALIBRATE -1
/* prevent BT overflow */
#define DELAY_AFTER_SEND 1000
/* Reset gesture */ // reset if total acceleration > TH_RESET
#define DECAY_FACTOR 0.8
#define TH_RESET 50 

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int volatile range = 0;
int volatile STATE = 0;
uint8_t sendBuffer[2];
HAL_StatusTypeDef ret;

int volatile BOOT_UP = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

	HAL_ADC_Start(&hadc1);
	HAL_Delay(10);
	HAL_ADC_Start(&hadc2);
	WriteByte(0x010, 0x01); // SET GPIO0 to 1
	HAL_Delay(10); 		  // Device BOOT Up, wait for 10ms
	VL6180X_ERR = VL6180X_Init();
	VL6180X_ERR = VL6180X_Start_Range();  // start single range measurement
	//Rotation detection IMU 0x29
	imu1 = (bno055_t){.i2c = &hi2c1, .addr = BNO_ADDR, .mode = BNO_MODE_IMU, ._temp_unit = 0};
	//Shifting detection IMU 0x28
	imu2 = (bno055_t){.i2c = &hi2c1, .addr = BNO_ADDR_ALT, .mode = BNO_MODE_IMU, ._temp_unit = 0};
	// Initialize the sensor
	IMU1PASS = bno055_init(&imu1);
	IMU2PASS = bno055_init(&imu2);
	
	//bno055_vec3_t gyroscope1;
	bno055_euler_t euler1;
	//bno055_vec3_t accelerometer1;
	//bno055_vec3_t gyroscope2;
	bno055_euler_t euler2;
	bno055_vec3_t accelerometer2;
	imu1.euler(&imu1, &euler1);
	init_euler1_x = euler1.roll;
	init_euler1_y = euler1.pitch;
	init_euler1_z = euler1.yaw;
	imu2.euler(&imu2, &euler2);
	init_euler2_x = euler2.roll;
	init_euler2_y = euler2.pitch;
	init_euler2_z = euler2.yaw;
	
	uint8_t gesture;
	uint8_t gestureData;
	uint8_t prevGesture;
	uint8_t prevGestureData;

	uint8_t rotationDataInt;

	/* Function prototype */
	// function that will send out the gesture | data
	void send2App(uint8_t gesture, uint8_t data, uint8_t* buffer);
	
	void flash(void);
	/* END OF Function prototype*/

	// INITIALIZATION symbol
	send2App(0xab, 0xcd, sendBuffer);
	if(BOOT_UP == 1){
		for(int i = 0; i < 10; i++){
			flash();
			HAL_Delay(50);
		}
		BOOT_UP = 0;
	}else{
	}
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1){
		
		/* BEGIN OF Update all */
		// button 1
		if (HAL_ADC_PollForConversion(&hadc1, 1000000) == HAL_OK){
			readFSR1 = HAL_ADC_GetValue(&hadc1);
			/* check threshold */
			if(readFSR1 < FSR_threshold){
				FSR1_pressed = 1;
			}else{
				FSR1_pressed = 0;
			}
		}
		// button 2
		if (HAL_ADC_PollForConversion(&hadc2, 1000000) == HAL_OK){
			readFSR2 = HAL_ADC_GetValue(&hadc2);
			/* check threshold */
			if(readFSR2 < FSR_threshold){
				FSR2_pressed = 1;
			}else{
				FSR2_pressed = 0;
			}
		}
		// IMU1
		IMU1PASS = imu1.euler(&imu1, &euler1);
		euler1_x = euler1.roll;
		euler1_y = euler1.pitch;
		euler1_z = euler1.yaw;
		change_x = (euler1_x - init_euler1_x);
		change_y = (euler1_y - init_euler1_y);
		change_z = (euler1_z - init_euler1_z);
		rotate_xyz = 0;
		// IMU2
		IMU2PASS = imu2.euler(&imu2, &euler2);
		euler2_x = euler2.roll;
		euler2_y = euler2.pitch;
		euler2_z = euler2.yaw;
		IMU2PASS = imu2.linear_acc(&imu2,&accelerometer2);
		a2_x = accelerometer2.x;
		a2_y = accelerometer2.y;
		a2_z = accelerometer2.z;
		absx = abs((int)a2_x);
		absy = abs((int)a2_y);
		absz = abs((int)a2_z);
		// VL6180X
		VL6180X_ERR = VL6180X_Poll_Range();			
		if(VL6180X_ERR){
			VL6180X_ERR = VL6180X_Init();
			VL6180X_ERR = VL6180X_Start_Range();
		}
		range = VL6180X_Read_Range(); // read range result
		VL6180X_Clear_Interrupts(); // clear the interrupt on VL6180X
		// Utility update 
		accleration_mag_buffer = DECAY_FACTOR*accleration_mag_buffer + absx + absy + absz;
		/* END OF Update all */
		
		/* BEGIN OF next state*/
		switch(STATE){
			case IDLE:
				if(ROTATE_MODE){
					STATE = WAIT_ROTATE;
					init_euler1_x = euler1_x;
					init_euler1_y = euler1_y;
					init_euler1_z = euler1_z;
				}else if(ZOOM_MODE){
					if (range < 40) {
						STATE = WAIT_ZOOM_IN;
					} else if (range > 100) {
						STATE = WAIT_ZOOM_OUT;
					}
				}else if(SHIFT_MODE){
					if(GESTURE_L_STARTING){
						STATE = WAIT_SHIFT_L;
						accleration_mag_buffer = 0;
					}else if(GESTURE_R_STARTING){
						STATE = WAIT_SHIFT_R;
						accleration_mag_buffer = 0;
					}else if(GESTURE_U_STARTING){
						STATE = WAIT_SHIFT_U;
						accleration_mag_buffer = 0;
					}else if(GESTURE_D_STARTING){
						STATE = WAIT_SHIFT_D;
						accleration_mag_buffer = 0;
					}else if(GESTURE_F_STARTING){
						STATE = WAIT_SHIFT_F;
						accleration_mag_buffer = 0;
					}else if(GESTURE_B_STARTING){
						STATE = WAIT_SHIFT_B;
						accleration_mag_buffer = 0;
					}
				}
				break;
			case WAIT_ZOOM_IN :
				if (ZOOM_MODE){
					if (range > 40) {
						STATE = ZOOM_IN_2;
					} else {
						STATE = WAIT_ZOOM_IN;
					}
				}else{
					STATE = IDLE;
				}
				break;
			case WAIT_ZOOM_OUT :
				if (ZOOM_MODE){
					if (range < 80 && range > 40) {
						STATE = ZOOM_OUT_2;
					} else {
						STATE = WAIT_ZOOM_OUT;
					}
				}else{
					STATE = IDLE;
				}
				break;
			case WAIT_ROTATE :
				if (ROTATE_MODE){
					// figure out the maximum change
					if((abs((int)change_x)>abs((int)change_y))&& (abs((int)change_x) > abs((int)change_z))){
						max_change = abs((int)change_x);
						rotate_xyz = 1;
					}else if((abs((int)change_y)>abs((int)change_z))&& (abs((int)change_y) > abs((int)change_x))){
						max_change = abs((int)change_y);
						rotate_xyz = 2;
					}else{
						max_change = abs((int)change_z);
						rotate_xyz = 3;
					}
					// choose x y or z to rotate
					if(max_change > 10){
						switch(rotate_xyz){
							case 1:
								STATE = ROTATE_X; 
								break;
							case 2:
								STATE = ROTATE_Y;
								break;
							case 3:
								STATE = ROTATE_Z;
								break;
						}
					}else{ //not enough change
						STATE = WAIT_ROTATE;
					}
				}else{
					STATE = IDLE;
				}
				break;
			case ZOOM_IN_2 :
				if (ZOOM_MODE){
					if (range > 80) {
						STATE = ZOOM_IN_4;
					} else if (range < 40) {
						STATE = WAIT_ZOOM_IN;
					} else {
						STATE = ZOOM_IN_2;
					}
				}else{
					STATE = IDLE;
				}
				break;
			case ZOOM_IN_4 :
				if (ZOOM_MODE){
					if (range < 80) {
						STATE = ZOOM_IN_2;
					} else {
						STATE = ZOOM_IN_4;
					}
				}else{
					STATE = IDLE;
				}
				break;
			case ZOOM_OUT_2 :
				if (ZOOM_MODE){
					if (range < 40) {
						STATE = ZOOM_OUT_4;
					} else if (range > 100) {
						STATE = WAIT_ZOOM_OUT;
					} else {
						STATE = ZOOM_OUT_2;
					}
				}else{
					STATE = IDLE;
				}
				break;
			case ZOOM_OUT_4 :
				if (ZOOM_MODE){
					if (range > 40) {
						STATE = ZOOM_OUT_2;
					} else {
						STATE = ZOOM_OUT_4;
					}
				}else{
					STATE = IDLE;
				}
				break;
			case ROTATE_X :
				if (ROTATE_MODE){
					STATE = WAIT_ROTATE;
				}else{
					STATE = IDLE;
				}
				break;
			case ROTATE_Y :
				if (ROTATE_MODE){
					STATE = WAIT_ROTATE;
				}else{
					STATE = IDLE;
				}
				break;
			case ROTATE_Z :
				if (ROTATE_MODE){
					STATE = WAIT_ROTATE;
				}else{
					STATE = IDLE;
				}
				break;
			case WAIT_SHIFT_L:
				if (SHIFT_MODE){
					if(GESTURE_R_ENDING){
						STATE = WAIT_SHIFT_R_DONE;
					}else{
						STATE = STATE;
					}
				}else{
					STATE = IDLE;
				}
				break;
			case WAIT_SHIFT_R:
				if (SHIFT_MODE){
					if(GESTURE_L_ENDING){
						STATE = WAIT_SHIFT_L_DONE;
					}else{
						STATE = STATE;
					}
				}else{
					STATE = IDLE;
				}
				break;
			case WAIT_SHIFT_U:
				if (SHIFT_MODE){
					if(GESTURE_D_ENDING){
						STATE = WAIT_SHIFT_D_DONE;
					}else{
						STATE = STATE;
					}
				}else{
					STATE = IDLE;
				}
				break;
			case WAIT_SHIFT_D:
				if (SHIFT_MODE){
					if(GESTURE_U_ENDING){
						STATE = WAIT_SHIFT_U_DONE;
					}else{
						STATE = STATE;
					}
				}else{
					STATE = IDLE;
				}
				break;
			case WAIT_SHIFT_F:
				if (SHIFT_MODE){
					if(GESTURE_B_ENDING){
						STATE = WAIT_SHIFT_B_DONE;
					}else{
						STATE = STATE;
					}
				}else{
					STATE = IDLE;
				}
				break;
			case WAIT_SHIFT_B:
				if (SHIFT_MODE){
					if(GESTURE_F_ENDING){
						STATE = WAIT_SHIFT_F_DONE;
					}else{
						STATE = STATE;
					}
				}else{
					STATE = IDLE;
				}
				break;
			case WAIT_SHIFT_L_DONE:
				STATE = IDLE;
				break;
			case WAIT_SHIFT_R_DONE:
				STATE = IDLE;
				break;
			case WAIT_SHIFT_U_DONE:
				STATE = IDLE;
				break;
			case WAIT_SHIFT_D_DONE:
				STATE = IDLE;
				break;
			case WAIT_SHIFT_F_DONE:
				STATE = IDLE;
				break;
			case WAIT_SHIFT_B_DONE:
				STATE = IDLE;
				break;
		
		}
		/* END OF next state*/

		/* BEGIN OF state behavior*/
		switch(STATE){
			case IDLE :
				GPIOB->ODR &= ~(1 << 12); //LED off
				if (accleration_mag_buffer > TH_RESET){
					gesture = CMD_RESET; 
					gestureData = DATA_RESET;
					accleration_mag_buffer = 0;
				}else {
					gesture = CMD_NONE;
					gestureData = DATA_NONE;
				}
				break;
			case WAIT_ZOOM_IN :
				GPIOB->ODR |= (1 << 12); // LED ON
				gesture = CMD_ZOOM;
				gestureData = DATA_NO_ZOOM_IN_OUT;
				break;
			case WAIT_ZOOM_OUT :
				GPIOB->ODR |= (1 << 12); // LED ON
				gesture = CMD_ZOOM;
				gestureData = DATA_NO_ZOOM_IN_OUT;
				break;
			case WAIT_ROTATE :
				GPIOB->ODR |= (1 << 12); // LED ON
				gesture = CMD_NONE;
				gestureData = DATA_NONE;
				break;
			case ZOOM_IN_2 :
				gesture = CMD_ZOOM;
				gestureData = DATA_ZOOM_IN_2;
				break;
			case ZOOM_IN_4 :
				gesture = CMD_ZOOM;
				gestureData = DATA_ZOOM_IN_4;
				break;
			case ZOOM_OUT_2 :
				gesture = CMD_ZOOM;
				gestureData = DATA_ZOOM_OUT_2;
				break;
			case ZOOM_OUT_4 :
				gesture = CMD_ZOOM;
				gestureData = DATA_ZOOM_OUT_4;
				break;
			case ROTATE_X :
				gesture = CMD_ROTATE_X;
				if(change_x > 0){
					rotationDataInt = change_x/10;
					init_euler1_x = init_euler1_x + rotationDataInt*10;
				}else{
					rotationDataInt = change_x/10 + 37; // 37 so it doesn't overturn
					init_euler1_x = init_euler1_x + rotationDataInt*10 -360;
				}	
				if(change_x*X_SIGN_CALIBRATE > 0){
					gestureData = change_x/10;
				}else{
					gestureData = change_x/10 + 37;
				}
				break;
			case ROTATE_Y :
				gesture = CMD_ROTATE_Y;
				if(change_y > 0){
					rotationDataInt = change_y/10;
					init_euler1_y = init_euler1_y + rotationDataInt*10;
				}else{
					rotationDataInt = change_y/10 + 37;
					init_euler1_y = init_euler1_y + rotationDataInt*10 -360;
				}
				if(change_y*Y_SIGN_CALIBRATE > 0){
					gestureData = change_y/10;
				}else{
					gestureData = change_y/10 + 37;
				}
				break;
			case ROTATE_Z :
				gesture = CMD_ROTATE_Z;
				if(change_z > 0){
					rotationDataInt = change_z/10;
					init_euler1_z = init_euler1_z + rotationDataInt*10;
				}else{
					rotationDataInt = change_z/10 + 37;
					init_euler1_z = init_euler1_z + rotationDataInt*10 -360;
				}	
				if(change_z*Z_SIGN_CALIBRATE > 0){
					gestureData = change_z/10;
				}else{
					gestureData = change_z/10 + 37;
				}
				break;
			case WAIT_SHIFT_L:
				flash();
				gesture = CMD_NONE;
				gestureData = DATA_NONE;
				break;
			case WAIT_SHIFT_R:
				flash();
				gesture = CMD_NONE;
				gestureData = DATA_NONE;
				break;
			case WAIT_SHIFT_U:
				flash();
				gesture = CMD_NONE;
				gestureData = DATA_NONE;
				break;
			case WAIT_SHIFT_D:
				flash();
				gesture = CMD_NONE;
				gestureData = DATA_NONE;
				break;
			case WAIT_SHIFT_F:
				flash();
				gesture = CMD_NONE;
				gestureData = DATA_NONE;
				break;
			case WAIT_SHIFT_B:
				flash();
				gesture = CMD_NONE;
				gestureData = DATA_NONE;
				break;
			case WAIT_SHIFT_L_DONE:
				GPIOB->ODR &= ~(1 << 12); //LED off
				gesture = CMD_LEFT;
				if(accleration_mag_buffer > TH_SPEED_3){
					gestureData = DATA_SHIFT_SPEED_3;
				}else if(accleration_mag_buffer > TH_SPEED_2){
					gestureData = DATA_SHIFT_SPEED_2;	
				}else if(accleration_mag_buffer > 0){
					gestureData = DATA_SHIFT_SPEED_1;	
				}
				accleration_mag_buffer = 0;
				break;
			case WAIT_SHIFT_R_DONE:
				GPIOB->ODR &= ~(1 << 12); //LED off
				gesture = CMD_RIGHT;
				if(accleration_mag_buffer > TH_SPEED_3){
					gestureData = DATA_SHIFT_SPEED_3;
				}else if(accleration_mag_buffer > TH_SPEED_2){
					gestureData = DATA_SHIFT_SPEED_2;	
				}else if(accleration_mag_buffer > 0){
					gestureData = DATA_SHIFT_SPEED_1;	
				}
				accleration_mag_buffer = 0;
				break;
			case WAIT_SHIFT_U_DONE:
				GPIOB->ODR &= ~(1 << 12); //LED off
				gesture = CMD_UP;
				if(accleration_mag_buffer > TH_SPEED_3){
					gestureData = DATA_SHIFT_SPEED_3;
				}else if(accleration_mag_buffer > TH_SPEED_2){
					gestureData = DATA_SHIFT_SPEED_2;	
				}else if(accleration_mag_buffer > 0){
					gestureData = DATA_SHIFT_SPEED_1;	
				}
				accleration_mag_buffer = 0;
				break;
			case WAIT_SHIFT_D_DONE:
				GPIOB->ODR &= ~(1 << 12); //LED off
				gesture = CMD_DOWN;
				if(accleration_mag_buffer > TH_SPEED_3){
					gestureData = DATA_SHIFT_SPEED_3;
				}else if(accleration_mag_buffer > TH_SPEED_2){
					gestureData = DATA_SHIFT_SPEED_2;	
				}else if(accleration_mag_buffer > 0){
					gestureData = DATA_SHIFT_SPEED_1;	
				}
				accleration_mag_buffer = 0;
				break;
			case WAIT_SHIFT_F_DONE:
				GPIOB->ODR &= ~(1 << 12); //LED off
				gesture = CMD_FRONT;
				if(accleration_mag_buffer > TH_SPEED_3){
					gestureData = DATA_SHIFT_SPEED_3;
				}else if(accleration_mag_buffer > TH_SPEED_2){
					gestureData = DATA_SHIFT_SPEED_2;	
				}else if(accleration_mag_buffer > 0){
					gestureData = DATA_SHIFT_SPEED_1;	
				}
				accleration_mag_buffer = 0;
				break;
			case WAIT_SHIFT_B_DONE:
				GPIOB->ODR &= ~(1 << 12); //LED off
				gesture = CMD_BACK;
				if(accleration_mag_buffer > TH_SPEED_3){
					gestureData = DATA_SHIFT_SPEED_3;
				}else if(accleration_mag_buffer > TH_SPEED_2){
					gestureData = DATA_SHIFT_SPEED_2;	
				}else if(accleration_mag_buffer > 0){
					gestureData = DATA_SHIFT_SPEED_1;	
				}
				accleration_mag_buffer = 0;
				break;
		}
		/* END OF state behavior*/
		
		/* BEGIN OF bluetooth output */
		if ( (prevGesture != gesture) || (prevGestureData != gestureData)){
			if (gesture != CMD_NONE){
				send2App(gesture, gestureData, sendBuffer);
				HAL_Delay(DELAY_AFTER_SEND);
			}
		}
		/* END OF bluetooth output*/

		/* BEGIN OF state update*/
		prevGesture = gesture;
		prevGestureData = gestureData;
		/* END OF state update*/
		
		
		//HAL_Delay(1000);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void send2App(uint8_t gesture, uint8_t data, uint8_t* buffer){
	//uint16_t result = (data<<8)|(gesture);
	buffer[0] = gesture;
	buffer[1] = data;
	HAL_UART_Transmit(&huart1, buffer, 2,100);
	//printf("%x,",gesture);
	//printf("%x\n",data);
}

void flash(void){
	GPIOB->ODR ^= (1 << 12);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
