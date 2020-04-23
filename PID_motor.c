#include "PID_motor.h"


PID PID_Motorleft;
PID PID_Motorright;
MotorPulse_Couter Motorright;
MotorPulse_Couter Motorleft;

void PID_position_left(int g1)
	{
		
		Motorleft.cnt = __HAL_TIM_GET_COUNTER(&htim3) + 65535*Motorleft.overflow;
		PID_Motorleft.E = g1 - Motorleft.cnt;
		
		if (Motorleft.cnt < g1) {
			
			PID_Motorleft.P = kp * PID_Motorleft.E;
			PID_Motorleft.I += ki * ( PID_Motorleft.E_last+ PID_Motorleft.E) * 1; 
			PID_Motorleft.D = kd * (PID_Motorleft.E - PID_Motorleft.E_last) / 1;
			PID_Motorleft.OUTPUT = PID_Motorleft.P + PID_Motorleft.I + PID_Motorleft.D;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
			
			if ((int)PID_Motorleft.OUTPUT > 2000) {
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 400); 
			}
			else if ( (int)PID_Motorleft.OUTPUT < 400) {
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,PID_Motorleft.OUTPUT); 
				
				if( PID_Motorleft.E <400) {
					PID_Motorleft.OUTPUT = 0;
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0); 
				}
			}
			PID_Motorleft.E_last = PID_Motorleft.E;
		}
		
	}
	
	
	void PID_position_right(int g2)
	{
		Motorright.cnt = __HAL_TIM_GET_COUNTER(&htim2) + 65535* Motorright.overflow;
		PID_Motorright.E = g2 - Motorright.cnt;
		
		if (Motorright.cnt < g2)
		{
			PID_Motorright.P = kp * PID_Motorright.E;
			PID_Motorright.I += ki * (PID_Motorright.E + PID_Motorright.E_last) * 1; 
			PID_Motorright.D = kd * (PID_Motorright.E - PID_Motorright.E_last) / 1;
			PID_Motorright.OUTPUT = PID_Motorright.P + PID_Motorright.I + PID_Motorright.D;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
			
				if ((int)PID_Motorright.OUTPUT > 2000) {
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 400); 
				}
				else if ( (int)PID_Motorright.OUTPUT < 400) {
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,PID_Motorright.OUTPUT); 
					if( PID_Motorright.E <500) {
						PID_Motorright.OUTPUT = 0;
						__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
					}
			}
			PID_Motorright.E_last = PID_Motorright.E;
		}
	}
	
	
	void PID_velocity_right(float speed)
	{
		PID_Motorright.E = speed - Motorright.speed;
		
			PID_Motorright.P = kp * PID_Motorright.E;
			PID_Motorright.I += ki * (PID_Motorright.E + PID_Motorright.E_last) * 0.15; 
			PID_Motorright.D = kd * (PID_Motorright.E - PID_Motorright.E_last) / 0.15;
			PID_Motorright.OUTPUT += PID_Motorright.P + PID_Motorright.I + PID_Motorright.D;

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,PID_Motorright.OUTPUT); 
		if(PID_Motorright.OUTPUT < 50)
		{
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,0); 
		}
			PID_Motorright.E_last = PID_Motorright.E;	
	}
	
	void PID_velocity_left(float speed)
	{
		PID_Motorleft.E = speed - Motorleft.speed;
		
			PID_Motorleft.P = kp * PID_Motorleft.E;
			PID_Motorleft.I += ki * (PID_Motorleft.E + PID_Motorleft.E_last) * 0.15; 
			PID_Motorleft.D = kd * (PID_Motorleft.E - PID_Motorleft.E_last) / 0.15;
			PID_Motorleft.OUTPUT += PID_Motorleft.P + PID_Motorleft.I + PID_Motorleft.D;

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,PID_Motorleft.OUTPUT); 
		if(PID_Motorleft.OUTPUT < 50)
		{
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,0); 
		}
			PID_Motorleft.E_last = PID_Motorleft.E;	
	} 
	
	