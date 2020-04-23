#include "lsm303dlhc.h"
#include "main.h"
#include "math.h"
#include "stm32f1xx_hal.h"
#include "Fuzzy.h"
#include "stdio.h"
#include "lcd.h"
#include "PID_motor.h"
#include <stdlib.h>

#define PI  3.14159265

Vector From = { 0, -1, 0};
extern I2C_HandleTypeDef hi2c1;
extern uint8_t datan[6];
extern uint8_t datam[6];
extern MotorPulse_Couter Motorright;
extern MotorPulse_Couter Motorleft;
extern int i;

float MagX[70], MagY[70], MagZ[70];
char str[15];
float Angle_velo;

void lsm303dlhc_init_la(I2C_HandleTypeDef *i2c) {
    uint8_t init[2][2] = {
        {CTRL_REG1_A, 0x27},
        {CTRL_REG4_A, 0x00}
    };

    HAL_StatusTypeDef ret;

    ret = HAL_I2C_Master_Transmit(i2c, LA_ADDRESS, init[0], 2, 1000);

    if (ret != HAL_OK) {
        return;
    }

    ret = HAL_I2C_Master_Transmit(i2c, LA_ADDRESS, init[1], 2, 1000);

    if (ret != HAL_OK) {
        return;
    }
}

void lsm303dlhc_init_mf(I2C_HandleTypeDef *i2c) {
    uint8_t init[3][2] = {
        {CRB_REG_M, (uint8_t)0x20},
        {MR_REG_M, (uint8_t)0x00},
				{CRA_REG_M, (uint8_t)0x0C}
    };

    HAL_StatusTypeDef ret;

    ret = HAL_I2C_Master_Transmit(i2c, MF_ADDRESS, init[0], 2, 1000);
    if (ret != HAL_OK) {
        return;
    }

    ret = HAL_I2C_Master_Transmit(i2c, MF_ADDRESS, init[1], 2, 1000);
    if (ret != HAL_OK) {
        return;
		}
		ret = HAL_I2C_Master_Transmit(i2c, MF_ADDRESS, init[2], 2, 1000);
    if (ret != HAL_OK) {
        return;
    }

}


void lsm303dlhc_read_la_b(I2C_HandleTypeDef *i2c, uint8_t *buf) {
    uint8_t reg = OUT_X_L_A| 0x80;
	  //uint8_t *buf;

    HAL_StatusTypeDef ret;

//    ret = HAL_I2C_Master_Transmit(i2c, LA_ADDRESS, &reg, 1, 1000);
//    if (ret != HAL_OK) {
//        return;
//    }
	
	
//		ret = HAL_I2C_Mem_Read(&hi2c1, LA_ADDRESS, OUT_X_L_A, I2C_MEMADD_SIZE_8BIT, datan,6,1000);
//		if (ret != HAL_OK) {
//			i++;
//				return;
//		
//		}
//		 while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
//  {
//		i++;
//  } 
		//I2C_DMA();
    ReadSensorReg( LA_ADDRESS, OUT_X_L_A, datan, 6);
		A.x = (int16_t)(datan[1]<<8| datan[0]);
	  A.y = (int16_t)(datan[3]<<8| datan[2]);
	  A.z = (int16_t)(datan[5]<<8| datan[4]);
}

//void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
//{
//	if(hi2c->Instance==hi2c1.Instance)
//	{
//		A.x = (int16_t)(datan[1]<<8| datan[0]);
//	  A.y = (int16_t)(datan[3]<<8| datan[2]);
//	  A.z = (int16_t)(datan[5]<<8| datan[4]);
//	}
//}

void lsm303dlhc_read_mf_b(I2C_HandleTypeDef *i2c,uint8_t *buf) {
    uint8_t reg = OUT_X_H_M;
	  HAL_StatusTypeDef ret;

    ret = HAL_I2C_Master_Transmit(i2c, MF_ADDRESS, &reg, 1, 1000);
    if (ret != HAL_OK) {
        return;
    }

    ret = HAL_I2C_Master_Receive(i2c, MF_ADDRESS, buf, 6,1000);
    if (ret != HAL_OK) {
        return;
    }
	  M.x = (int16_t)(datam[0]<<8 | datam[1]);
	  M.y = (int16_t)(datam[2]<<8 | datam[3]);
	  M.z = (int16_t)(datam[4]<<8 | datam[5]);
}

float Vector_dot(Vector *a, Vector *b)
{
	return((a->x * b->x) + (a->y * b->y) + (a->z * b->z));
}

void Vector_cross(Vector *a, Vector *b, Vector *cross)
{
	 cross->x = (a->y * b->z) - (a->z * b->y);
   cross->y = (a->z * b->x) - (a->x * b->z);
   cross->z = (a->x * b->y) - (a->y * b->x);
	 
}

void Nomalize(Vector *a) {
	float length = sqrt(Vector_dot(a, a));
   a->x = (float)(a->x / length) * 100;
   a->y = (float)(a->y / length) * 100;
   a->z = (float)(a->z / length) * 100;
}

void Extreme_mag() {
	sprintf(str," CALIBING...");
	lcd_goto_XY(1,0);
	lcd_send_string(str);
	for( int i = 0; i < 70; i++) {
	lsm303dlhc_read_mf_b(&hi2c1, datam);
		MagX[i] = M.x; 
		MagY[i] = M.y;
		MagZ[i] = M.z;
		HAL_Delay(200);
	}
	lcd_clear_display();
	
	Arrange_maxtomin(70,MagX);
	Arrange_maxtomin(70,MagY);
	Arrange_maxtomin(70,MagZ);
	
	lcd_clear_display();
	HAL_Delay(2000);
}

float Angle() {
	 lsm303dlhc_read_mf_b(&hi2c1, datam);
	 lsm303dlhc_read_la_b(&hi2c1, datan);	

	 M.x = (float)(M.x - MagX[69]) / (MagX[0] - MagX[69]) * 2 - 1.0;
   M.y = (float)(M.y - MagY[69]) / (MagY[0] - MagY[69]) * 2 - 1.0;
   M.z = (float)(M.z - MagZ[69]) / (MagZ[0] - MagZ[69]) * 2 - 1.0;
	
   Vector_cross(&M, &A, &E);
	 Nomalize(&E);
	 Vector_cross(&A, &E, &N);
	 Nomalize(&N);
	 // compute heading
	 float angle = atan2(Vector_dot(&E, &From), Vector_dot(&N, &From)) * 180 / PI;
	 if (angle < 0) angle += 360;
	 return angle;
	
}

float Angle_Kalman(float x) {
	float anglek, kalman_gain, last_estimate;
	static float err_estimate = 2;
	
	kalman_gain = err_estimate / (err_estimate + 0.03); // 0.03 = err_measure
	anglek = last_estimate + kalman_gain * (x - last_estimate);
	err_estimate = (1.0 - kalman_gain) * err_estimate + fabs(last_estimate - anglek) * 0.002; // 0.002 = q
	last_estimate = anglek;
	return anglek;
}

//void Angle_veloc() {
//	Angle_velo += 3.25 * 2 * PI *(Motorleft.speed - Motorright.speed) / 26.5;
//}

void I2C_DMA () {

	I2C1 ->CR2 |= (1 << 11); 
	I2C1 ->CR2 |= (1 << 12); 
	I2C1 ->CR1 |= (1 << 8); 
	while ( I2C_SR1_SB != (I2C_SR1_SB & I2C1->SR1)) {
	}
  I2C1 ->SR1 ;
	I2C1 ->DR = LA_ADDRESS;
	 while(I2C_SR1_ADDR != (I2C_SR1_ADDR & I2C1->SR1))
  {
    /* Do nothing */
  }
	 (void)I2C1->SR1;

  /* Read SR2 */
  (void)I2C1->SR2;
	 while(I2C_SR1_ADDR != (I2C_SR1_ADDR & I2C1->SR1))
  {
    /* Do nothing */
  }
	I2C1 ->CR2 &= ~(1 << 11);
}

void DMA_Receive( uint8_t * pBuffer, uint8_t size)
{
  /* Check null pointers */
  if(NULL != pBuffer)
  {
    /* Wait until DMA1_Stream2 is disabled */
    while(DMA_CCR_EN == (DMA_CCR_EN & DMA1_Channel7 ->CCR))
    {
      /* Do nothing, the enable flag shall reset
       * when DMA transfer complete */
    }

    /* Set memory address */
    DMA1_Channel7 ->CMAR = (uint32_t)pBuffer;

    /* Set number of data items */
    DMA1_Channel7->CNDTR = size;

    /* Clear all interrupt flags */
    DMA1->IFCR |= 0x00000000;

    /* Reset DMA flag */
    //DMAStatus = IKS01A2_DMA_NOT_FINISHED;

    /* Enable DMA1_Stream2 */
    DMA1_Channel7-> CCR |= DMA_CCR_EN;
  }
  else
  {
    /* Null pointers, do nothing */
  }
}
	
void ReadSensorReg(uint8_t SensorAddr, uint8_t ReadAddr, uint8_t * pReadBuffer, uint16_t NumByteToRead)
{
  /* Generate START */
  I2C1->CR1 |= I2C_CR1_START;

  /* Wait SB flag is set */
  while(I2C_SR1_SB != (I2C_SR1_SB & I2C1->SR1))
  {
    /* Do nothing */
  }

  /* Read SR1 */
  (void)I2C1->SR1;

  /* Send slave address with write */
  I2C1->DR = (uint16_t) SensorAddr;

  /* Wait ADDR flag is set */
  while(I2C_SR1_ADDR != (I2C_SR1_ADDR & I2C1->SR1))
  {
    /* Do nothing */
  }

  /* Read SR1 */
  (void)I2C1->SR1;

  /* Read SR2 */
  (void)I2C1->SR2;

  /* Wait TXE flag is set */
  while(I2C_SR1_TXE != (I2C_SR1_TXE & I2C1->SR1))
  {
    /* Do nothing */
  }

  if(2 <= NumByteToRead)
  {
    /* Acknowledge enable */
    I2C1->CR1 |= I2C_CR1_ACK;

    /* Send register address to read with increment */
    I2C1->DR = (uint16_t) (ReadAddr | (uint8_t)0x80);
  }
  else
  {
    /* Acknowledge disable */
    I2C1->CR1 &= ~I2C_CR1_ACK;

    /* Send register address to read (single) */
    I2C1->DR = (uint16_t) ReadAddr;
  }



  /* Wait BTF flag is set */
  while(I2C_SR1_BTF != (I2C_SR1_BTF & I2C1->SR1))
  {
    /* Do nothing */
  }

  /* Generate ReSTART */
  I2C1->CR1 |= I2C_CR1_START;

  /* Wait SB flag is set */
  while(I2C_SR1_SB != (I2C_SR1_SB & I2C1->SR1))
  {
    /* Do nothing */
  }

  /* Read SR1 */
  (void)I2C1->SR1;

  /* Send slave address with read */
  I2C1->DR = (uint16_t) (SensorAddr | (uint8_t)0x01);

  /* Wait ADDR flag is set */
  while(I2C_SR1_ADDR != (I2C_SR1_ADDR & I2C1->SR1))
  {
    /* Do nothing */
  }

  /* Start DMA */
  DMA_Receive(pReadBuffer, NumByteToRead);

  /* Read SR1 */
  (void)I2C1->SR1;

  /* Read SR2 */
  (void)I2C1->SR2;
}


	
	
