#include "stm32f1xx_hal.h"
#include "Fuzzy.h"
#include "PID_motor.h"
#include "math.h"
#include "lsm303dlhc.h"

#define PI  3.14159265

extern MotorPulse_Couter Motorright;
extern MotorPulse_Couter Motorleft;
extern float full1, yaw;
extern TIM_HandleTypeDef htim4;
	
Coordinates Current;
Coordinates Last;
Coordinates Goal;
extern int phuong, count;
	
float Distance;

void GetCoordinates_current()
{
	yaw = Angle() - full1;
	Current.Heading = Angle_Kalman(yaw);
	Current.Y = 0.5 * 0.2 * 2 * PI * (Motorright.speed * 3.25 + Motorleft.speed * 3.25) * cos(Current.Heading * PI/180) + Last.Y;
	Current.X = 0.5 * 0.2 * 2 * PI * (Motorright.speed * 3.25 + Motorleft.speed * 3.25) * sin(Current.Heading * PI/180) + Last.X;
	Last.X = Current.X;
	Last.Y = Current.Y;
}


void Arrange_maxtomin(int x, float *M) {
	float tg;
	for( int i=0; i < x -1; i++) {
		for( int j=i+1; j < x ; j++) {
			if(M[i] < M[j]) {
				tg = M[i];
				M[i] = M[j];
				M[j] = tg;
			}
		}
	}
}

float MAX(int x, float *M) {
	for( int i=1; i < x; i++) {
		M[0] = M[0] > M[i] ? M[0] : M[i];		
	}
	return M[0];
}
	
float GetVN_D(float x)
{
	float k_vn;
	if(x <= 20) k_vn = 1;
	else if ( 20 < x && x <= 60) k_vn = (60-x)/40;
	else k_vn = 0;
	return k_vn;	
}

float GetN_D(float x) {
	float k_m;
	if(20 < x && x <= 60) k_m = (x-20)/40;
	else if ( 60 < x && x <= 100) k_m = (100-x)/40;
	else k_m =0;
	return k_m;	
}

float GetFar_D(float x) {
	float k_bp;
	if(60 <= x && x < 100) k_bp = (x-60)/40;
	else if (100 <= x && x < 200) k_bp = (200-x)/100;
	else k_bp = 0;
	return k_bp;	
}

float GetVFar_D(float x) {
	float k_bp;
	if(100 <= x && x < 200) k_bp = (x-100)/100;
	else if (x > 200) k_bp = 1;
	else k_bp = 0;
	return k_bp;	
}

float GetBN_theta(float x)
{
	float k_bn;
	if(x <= -50) k_bn = 1;
	else if ( -50 < x && x <= -15) k_bn = (-15 - x)/35;
	else k_bn = 0;
	return k_bn;	
}

float GetN_theta(float x) {
	float k_m;

	if(-50 <= x && x < -15) k_m = (x+50)/35;
	else if ( -15 <= x && x < 15) k_m = (15-x)/30;
	else k_m =0;
	return k_m;	
}

float GetP_theta(float x) {
	float k_bp;
	if(-15 <= x && x < 15) k_bp = (x+15)/30;
	else if (15 <= x && x < 50) k_bp = (50 - x)/35;
	else k_bp = 0;
	return k_bp;	
}

float GetBP_theta(float x) {
	float k_bp;
	if(15 <= x && x < 50) k_bp = (x-15)/35;
	else if ( x >= 50) k_bp = 1;
	else k_bp = 0;
	return k_bp;	
}

float Getright(float X, float theta) {
	
	float speed;
	float rightslow[3], rightmid[7], rightfast[4], rightveryfast[2];
	float k_rightslow, k_rightmid, k_rightfast, k_rightvfast;
	
	rightslow[0] = GetVN_D(X) < GetBP_theta(theta) ? GetVN_D(X) : GetBP_theta(theta);
	rightslow[1] = GetVN_D(X) < GetP_theta(theta) ? GetVN_D(X) : GetP_theta(theta);
	rightslow[2] = GetN_D(X) < GetBP_theta(theta) ? GetN_D(X) : GetBP_theta(theta);
	
	rightmid[0] = GetVN_D(X) < GetBN_theta(theta) ? GetVN_D(X) : GetBN_theta(theta);
	rightmid[1] = GetVN_D(X) < GetN_theta(theta) ? GetVN_D(X) : GetN_theta(theta);
	rightmid[2] = GetN_D(X) < GetN_theta(theta) ? GetN_D(X) : GetN_theta(theta);
	rightmid[3] = GetN_D(X) < GetP_theta(theta) ? GetN_D(X) : GetP_theta(theta);
	rightmid[4] = GetFar_D(X) < GetP_theta(theta) ? GetFar_D(X) : GetP_theta(theta);
	rightmid[5] = GetFar_D(X) < GetBP_theta(theta) ? GetFar_D(X) : GetBP_theta(theta);
	rightmid[6] = GetVFar_D(X) < GetBP_theta(theta) ? GetFar_D(X) : GetBP_theta(theta);
	
	rightfast[0] = GetN_D(X) < GetBN_theta(theta) ? GetN_D(X) : GetBN_theta(theta);
	rightfast[1] = GetFar_D(X) < GetN_theta(theta) ? GetFar_D(X) : GetN_theta(theta);
	rightfast[2] = GetVFar_D(X) < GetN_theta(theta) ? GetVFar_D(X) : GetN_theta(theta);
	rightfast[3] = GetVFar_D(X) < GetP_theta(theta) ? GetVFar_D(X) : GetP_theta(theta);
	
	rightveryfast[0] = GetVFar_D(X) < GetBN_theta(theta) ? GetVFar_D(X) : GetBN_theta(theta);
	rightveryfast[1] = GetFar_D(X) < GetBN_theta(theta) ? GetFar_D(X) : GetBN_theta(theta);
	
	k_rightslow = MAX(3,rightslow);
	k_rightmid = MAX(6,rightmid);
	k_rightfast = MAX(4,rightfast);
	k_rightvfast = MAX(2,rightveryfast);
	
	if(k_rightfast == 0 && k_rightvfast == 0 ) {
		speed = (0* k_rightslow + 1*k_rightmid)/(k_rightmid + k_rightslow);
	}
	else if(k_rightslow == 0 && k_rightvfast == 0) {
		speed = (2* k_rightfast + 1*k_rightmid)/(k_rightmid + k_rightfast);
	}
	else if (k_rightslow == 0 && k_rightmid == 0) {
		speed = (2* k_rightfast + 3*k_rightvfast)/(k_rightvfast + k_rightfast);
	}
	else {
		speed = (0* k_rightslow + 1*k_rightmid + 2* k_rightfast + 3*k_rightvfast )/(k_rightmid + k_rightslow + k_rightvfast + k_rightfast);
	}
	return speed;	
}

float Getleft(float X, float theta) {
	
	float speed;
	float leftslow[3], leftmid[7], leftfast[4], leftveryfast[2];
	float k_leftslow, k_leftmid, k_leftfast, k_leftvfast;

	
	leftslow[0] = GetVN_D(X) < GetBN_theta(theta) ? GetVN_D(X) : GetBN_theta(theta);
	leftslow[1] = GetVN_D(X) < GetN_theta(theta) ? GetVN_D(X) : GetN_theta(theta);
	leftslow[2] = GetN_D(X) < GetBN_theta(theta) ? GetN_D(X) : GetBN_theta(theta);
	
	leftmid[0] = GetVN_D(X) < GetBP_theta(theta) ? GetVN_D(X) : GetBP_theta(theta);
	leftmid[1] = GetVN_D(X) < GetP_theta(theta) ? GetVN_D(X) : GetP_theta(theta);
	leftmid[2] = GetN_D(X) < GetN_theta(theta) ? GetN_D(X) : GetN_theta(theta);
	leftmid[3] = GetN_D(X) < GetP_theta(theta) ? GetN_D(X) : GetP_theta(theta);
	leftmid[4] = GetFar_D(X) < GetN_theta(theta) ? GetFar_D(X) : GetN_theta(theta);
	leftmid[5] = GetFar_D(X) < GetBN_theta(theta) ? GetFar_D(X) : GetBN_theta(theta);
	leftmid[6] = GetVFar_D(X) < GetBN_theta(theta) ? GetFar_D(X) : GetBN_theta(theta);
	
	leftfast[0] = GetN_D(X) < GetBP_theta(theta) ? GetN_D(X) : GetBP_theta(theta);
	leftfast[1] = GetFar_D(X) < GetP_theta(theta) ? GetFar_D(X) : GetP_theta(theta);
	leftfast[2] = GetVFar_D(X) < GetN_theta(theta) ? GetVFar_D(X) : GetN_theta(theta);
	leftfast[3] = GetVFar_D(X) < GetP_theta(theta) ? GetVFar_D(X) : GetP_theta(theta);
	
	leftveryfast[0] = GetVFar_D(X) < GetBP_theta(theta) ? GetVFar_D(X) : GetBP_theta(theta);
	leftveryfast[1] = GetFar_D(X) < GetBP_theta(theta) ? GetFar_D(X) : GetBP_theta(theta);
	
	k_leftslow = MAX(3,leftslow);
	k_leftmid = MAX(7,leftmid);
	k_leftfast = MAX(4,leftfast);
	k_leftvfast = MAX(2,leftveryfast);
	
	if(k_leftfast == 0 && k_leftvfast == 0 ) {
		speed = (0* k_leftslow + 1*k_leftmid)/(k_leftmid + k_leftslow);
	}
	else if(k_leftslow == 0 && k_leftvfast == 0) {
		speed = (2* k_leftfast + 1*k_leftmid)/(k_leftmid + k_leftfast);
	}
	else if (k_leftslow == 0 && k_leftmid == 0) {
		speed = (2* k_leftfast + 3*k_leftvfast)/(k_leftvfast + k_leftfast);
	}
	else {
		speed = (0* k_leftslow + 1*k_leftmid + 2* k_leftfast + 3*k_leftvfast )/(k_leftmid + k_leftslow + k_leftvfast + k_leftfast);
	}
	return speed;	
}


void Goal_coordinate_relative(float X, float Y) {
	Goal.X = (X - Current.X) * cos( Current.Heading * PI/180 ) - (Y - Current.Y) * sin(Current.Heading * PI/180);
	Goal.Y = (X - Current.X) * sin( Current.Heading * PI/180 ) + (Y - Current.Y) * cos(Current.Heading * PI/180);
	Goal.Heading = atan(Goal.X/Goal.Y) * 180/PI;
	Distance = sqrt((Goal.X * Goal.X) + (Goal.Y * Goal.Y));
	if( Distance < (float)10  && count == 1) {
		phuong = 5;
	}
	else if( Distance < (float)10 && count == 2) {
		phuong = 6;
	}
	else if( Distance < (float)10 && count == 3) {
		phuong = 7;
	}
}

