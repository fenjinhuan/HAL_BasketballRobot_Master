#include "control.h"
#include "get_info.h"

u16 motortime = 0;

RxPID rxPID;     //串口调节pid
BLOCKING Blocking;  //走位pid

float output[6];

struct RADAR Radar;
struct VISION Vision;

MOTOR Motor[4];
ROBOT BasketballRobot;
PID_t *pidAdjust;

void Control_Init(void)
{
	BasketballRobot.X = 0;       //机器人在坐标系中x坐标
	BasketballRobot.Y = 0;       //机器人在坐标系中y坐标
	BasketballRobot.ThetaD = 0;  //机器人正方向和y轴夹角 角度
	BasketballRobot.ThetaR = 0;  //机器人正方向和y轴夹角 弧度
	
	BasketballRobot.Vx = 0;      //机器人在坐标系x方向速度
	BasketballRobot.Vy = 0;      //机器人在坐标系y方向速度
	BasketballRobot.W = 0;       //机器人角速度，顺时针正方向
	
	BasketballRobot.w[0] = 0;    //第一个编码器速度
	BasketballRobot.w[1] = 0;     //第二个编码器速度
	
	BasketballRobot.encoderCount[0] = 0;  //第一个编编码器总计数
	BasketballRobot.encoderCount[1] = 0;  //第二个编编码器总计数
	
	BasketballRobot.LastTheta = 0;  //上一时刻，机器人theta角
	
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);  //开启解码器通道
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	
	//开启外设
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim5);

	HAL_UART_Receive_IT(&huart1,(u8 *)aRxBuffer1, 1);
	HAL_UART_Receive_IT(&huart2,(u8 *)aRxBuffer2, 1);
	HAL_UART_Receive_IT(&huart3,(u8 *)aRxBuffer3, 1);
	HAL_UART_Receive_IT(&huart4, &rxPID.pidReadBuf, 1);
	
	IMU_Init();  //陀螺仪初始化
	
	Set_MotorSpeed(0,0,0,0);
}

//设置电机的速度（控制电流）
void Set_MotorSpeed(float V1,float V2,float V3,float V4)
{
	Motor[0].SetSpeed = V1;
	Motor[1].SetSpeed = V2;
	Motor[2].SetSpeed = V3;
	Motor[3].SetSpeed = V4;
}

void AllPID_Init(void)
{
	u8 i;
	for (i = 0;i < 4;i ++)
	{
	    PID_Init(&(Motor[i].SpeedPID), POSITION_PID, 5000, 3000,
					 5,1,0);
//	    PID_Init(&(Motor[i].AnglePID), POSITION_PID, ANGLE_MAX, 5000,0.25,0,0);
		
		PID_Init(&Blocking.Vx_adjust, POSITION_PID, 2000, 1000, 900, 0, 20);
		PID_Init(&Blocking.Vy_adjust, POSITION_PID, 2000, 1000, 900, 0, 20);
	}
}
//底盘PID控制程序
//包含速度PID与位置PID控制
void Calc_MotorSpeed_pid(void)
{
	u8 i;
	
	for (i = 0;i < 4;i ++)
	    Motor[i].Single_LoopOutput = PID_Calc(&(Motor[i].SpeedPID),
		                                  Motor[i].Speed, Motor[i].SetSpeed);
}

//void Calc_MotorAngle_pid(void)
//{
//	u8 i;
//	DoubleLoop_Flag = 1;
//	
//	for (i = 0;i < 4;i ++)
//	{
//	    Motor[i].SetSpeed = PID_Calc(&(Motor[i].AnglePID),
//											     Motor[i].Angle, Motor[i].SetAngle);
//		DoubleLoop_Flag = 0;
//		
//	    Motor[i].Double_LoopOutput = PID_Calc(&(Motor[i].SpeedPID),
//											     Motor[i].Speed, Motor[i].SetSpeed);
//	}
//}

//给自身坐标系速度求得轮子的速度
//麦克纳姆轮在运动分解时注意每个轮子的正方向
//vx：自身坐标的x轴速度
//vy：自身坐标的y轴速度
//w:  机器人原地旋转的角速度
void GetMotorVelocity_Self(float vx, float vy, float w)
{
	Motor[0].SetSpeed = vx + vy + w * PLATFORM_L;
	Motor[1].SetSpeed = vx - vy + w * PLATFORM_L;
	Motor[2].SetSpeed = - vx - vy + w * PLATFORM_L;
	Motor[3].SetSpeed = - vx + vy  + w * PLATFORM_L;
	
	LCD_Show_setspeed();
}

//给定球场坐标速度求得轮子的速度
//vx：球场坐标的x轴速度
//vy：球场坐标的y轴速度
//w:机器人原地旋转的角速度
void GetMotorVelocity(float vx_a, float vy_a, float w_a)
{
    float vx,vy,w;
	
	vx = vx_a * cos(BasketballRobot.ThetaR) + vy_a * sin(BasketballRobot.ThetaR);
	vy = -vx_a * sin(BasketballRobot.ThetaR) + vy_a * cos(BasketballRobot.ThetaR);
	w = w_a;
	
	Motor[0].SetSpeed = vx + vy + w * PLATFORM_L;
	Motor[1].SetSpeed = vx - vy + w * PLATFORM_L;
	Motor[2].SetSpeed = - vx - vy + w * PLATFORM_L;
	Motor[3].SetSpeed = - vx + vy  + w * PLATFORM_L;
	
	LCD_Show_setspeed();
}

//获取红外开关状态
void GetInfraredState(void)
{
	while (1)
	{
		if (!INFRARED)
			break;
	}
}

//铲球电机状态
void ShoveMotor(shovemotor t)
{
	
}

//机械臂下降
void Robot_ArmDown(void)
{
	
}

//机械臂上升
void Robot_ArmUp(void)
{
	
}

//PD调整角速度
static float adjustAngleVw_PD(float D_Theta)
{
	float Vw = 0;
	
	if (D_Theta > 0 && D_Theta <= 180)
		Vw = -D_Theta * 30;
	else if (D_Theta > 180)
	{
		D_Theta = 360 - D_Theta;
		Vw = D_Theta * 30;
	}
	else if (D_Theta < 0 && D_Theta > -180)
		Vw = -D_Theta * 30;
	else if (D_Theta <= -180)
	{
		D_Theta = 360 + D_Theta;
		Vw = -D_Theta * 30;
	}
	
	if (Vw > 900)
		Vw = 900;
	else if (Vw < -900)
		Vw = -900;
	if (Vw > 0 && Vw < 90)
		Vw = 90;
	else if (Vw > -90 && Vw < 0)
		Vw = -90;
	
	return Vw;
}

//PD调整X轴速度
static float adjustVx_PD(float D_X)
{
	float sx,Now_DX;
	static float Last_DX;
	Now_DX = D_X;
	
	if (Now_DX > 0.05f)
	{
		sx = Now_DX * 1000 + 200;
		
		
		
		if (Now_DX < 1)
			sx =Now_DX * 1000 + 5 * (Now_DX - Last_DX) + 600;
		
		if (sx > 1500)
			sx = 1500;
	}
	
	else if (Now_DX < -0.05f)
	{
		sx = -Now_DX * 1000 - 200;
		
		if (Now_DX > -1)
			sx = -Now_DX * 1000 -  5 * (Now_DX - Last_DX) - 800;
		
		if (sx < -1500)
			sx = -1500;
	}
	
	else
		sx = 0;
	
	Last_DX = Now_DX;

/*
    float sx;
	
	if (fabs(D_X) > 0.05f)
	{
		Blocking.Vx_adjust.err[NOW] = D_X;
		sx = (err_PID_Calc(&Blocking.Vx_adjust,Blocking.Vx_adjust.err[NOW]) + 300);
	}
	else
		sx = 0;
	
	if (sx > 1500)
		sx = 1500;
	if (sx < -1500)
		sx = -1500;
*/	
	return sx;	
}
//PD调整Y轴速度
static float adjustVy_PD(float D_Y)
{
	float sy,Now_DY;
	static float Last_DY;
	Now_DY = D_Y ;
	
	if (Now_DY > 0.05f)
	{
		//sy = Now_DY * 1000 + 200;
		sy =(7.5- Now_DY) * 250  + 200;
		
		if (Now_DY < 1)//距离目标1m
			sy = Now_DY * 1000 + 5 * (Now_DY - Last_DY) + 600;
		
		if (sy > 1500)
			sy = 1500;
	}
	
	else if (Now_DY < -0.05f)
	{
		sy = Now_DY * 1000 - 200;
		
		if (Now_DY > -1)
			sy = -Now_DY * 1000 -  5 * (Now_DY - Last_DY) - 600;
		
		if (sy < -1500)
			sy = -1500;
	}
	
	else
		sy = 0;
	
	Last_DY = Now_DY;
/*
    float sy;
	
	if (fabs(D_Y) > 0.05f)
	{
		Blocking.Vy_adjust.err[NOW] = D_Y;
		sy = err_PID_Calc(&Blocking.Vy_adjust,Blocking.Vy_adjust.err[NOW]) + 300;
	}
	else
		sy = 0;
	
	if (sy > 1500)
		sy = 1500;
	if (sy < -1500)
		sy = -1500;
*/
	return sy;
}

//转一周时的角y度调节
//由于在 adjustAngleVw_PD 函数中不能实现转一周，所以自转一周单独实现
static float adjust_circle(float D_Theta)
{
	float Vw = 0;

	if (D_Theta > 0 && D_Theta <= 180)
		Vw = -D_Theta * 30;
	else if (D_Theta > 180)
	{
		D_Theta = 360 - D_Theta;
		Vw = -D_Theta * 30 - 90;
	}
	
	if (Vw < -900)
		Vw = -900;
	else if (Vw > -90 && Vw < 0)
		Vw = -90;
	
	return Vw;
}

//自转一周
void RobotRotate_circle(void)
{
	float D_Theta;
	float Vw = 0;  //w > 0，顺时针
	
	D_Theta = 360 - BasketballRobot.ThetaD;
	
	Vw = adjust_circle(D_Theta);

	while (D_Theta > 1 || D_Theta < -1)
	{
		Set_MotorSpeed(Vw,Vw,Vw,Vw);
		
		D_Theta = 360 - BasketballRobot.ThetaD -2;
		
		Vw = adjust_circle(D_Theta);
		
		LCD_Show_setspeed();
	}
	
	Set_MotorSpeed(0,0,0,0);
}

//自旋运动，根据误差角度，自动调节
void RobotRotate(float theta)
{
	float D_Theta;
	float Vw = 0;  //w > 0，顺时针
	
	while (theta < 0)
		theta = theta + 360;
	
	while (theta > 360)
		theta = theta - 360;
	
	D_Theta = theta - BasketballRobot.ThetaD;
	
	Vw = adjustAngleVw_PD(D_Theta);

	while (D_Theta > 1 || D_Theta < -1)
	{
		Set_MotorSpeed(Vw,Vw,Vw,Vw);
		
		D_Theta = theta - BasketballRobot.ThetaD + 2;
		
		Vw = adjustAngleVw_PD(D_Theta);
		
		LCD_Show_setspeed();
	}
	
	Set_MotorSpeed(0,0,0,0);
}

//行至目的地
//X_I:目的地坐标的X
//Y_I:目的地坐标的Y
//Theta_I:目的地坐标的角度
void RobotGoTo(float X_I,float Y_I,float Theta_I)
{
	float D_Theta,D_X,D_Y,Vw = 0,sx = 0,sy = 0;
	
	D_Theta = Theta_I - BasketballRobot.ThetaD;
	D_X = X_I - BasketballRobot.X;
	D_Y = Y_I - BasketballRobot.Y;
	while (fabs(D_Y) > 0.05f || fabs(D_X) > 0.05f)
	{
		sy = adjustVy_PD(D_Y);
		sx = adjustVx_PD(D_X);
		Vw = adjustAngleVw_PD(D_Theta);
		
		GetMotorVelocity(sx,sy,Vw);
		
		Set_MotorSpeed(Motor[0].SetSpeed,Motor[1].SetSpeed,
		               Motor[2].SetSpeed,Motor[3].SetSpeed);
		
		D_Theta = Theta_I - BasketballRobot.ThetaD;
	    D_X = X_I - BasketballRobot.X;
	    D_Y = Y_I - BasketballRobot.Y;
	}
	RobotRotate(Theta_I);
	
	Set_MotorSpeed(0,0,0,0);
}


//折线直行 前三个参数为目标点,后三个参数为中间点
void RobotGoBrokenLine(float X_I,float Y_I,float Theta_I,float pointX, float pointY,float pointTheta)
{
	float D_Theta, D_X, D_Y, Vw = 0, sx, sy = 0;
	int8_t flagX,flagY; // 判断走中间点是否超调

	D_Theta = pointTheta - BasketballRobot.ThetaD; //角度差
	//加快到中间点的速度
	D_X = (pointX - BasketballRobot.X) * 2;
	D_Y = (pointY - BasketballRobot.Y) * 2;

	if (D_X >= 0)
		flagX = 1;
	else
		flagX = -1;

	if (D_Y >= 0)
		flagY = 1;
	else
		flagY = -1;

	while (1)
	{	
		sy = adjustVy_PD(D_Y);
		sx = adjustVx_PD(D_X);
		Vw = adjustAngleVw_PD(D_Theta) / 2;

		GetMotorVelocity(sx, sy, Vw);
		Set_MotorSpeed(Motor[0].SetSpeed, Motor[1].SetSpeed, Motor[2].SetSpeed, Motor[3].SetSpeed);

		D_Theta = pointTheta - BasketballRobot.ThetaD; //角度差
		//加快到中间点的速度
		D_X = (pointX - BasketballRobot.X) * 2;
		D_Y = (pointY - BasketballRobot.Y) * 2;

		//走到点或超调后跳出
		if(fabs(D_Y) < 0.1f && fabs(D_X) < 0.1f)
			break;
		if (D_X * flagX <= 0 && D_Y * flagY <= 0)
			break;
	}

	RobotGoTo(X_I,Y_I,Theta_I);
}

//避障直行
//直行1m
void RobotGoAvoidance(void)
{/*
	//float w = 100;
	float theta = BasketballRobot.ThetaD, D_theta = 0;
	u8 time = 1;


	
	Radar.RX_STA = 0;
	
	do
	{
		while((Radar.RX_STA&0x8000) == 0);
		
		if (!GetRadarData())
		{
			if (time == 0)
			{
			}
			else if (time++ < 5)
			{
				Set_MotorSpeed(0, 0, 0, 0);
				continue;//其作用为结束本次循环，即跳过循环体中下面尚未执行的语句，然后进行下一次是否执行循环的判定。
			}
			else if (time != 0)
				time = 0;
		}
		else
			time = 1;

		//		if(Radar.Distance < 10)
		//			continue;
	
		if (time == 0)
		{
			GetMotorVelocity_Self(0, 300, 0);
			Set_MotorSpeed(BasketballRobot.Velocity[0], BasketballRobot.Velocity[1], BasketballRobot.Velocity[2],BasketballRobot.Velocity[3]);
			delay_ms(5000);
			Set_MotorSpeed(0, 0, 0,0);
			break;
		}
		
		
		if (Radar.Angle < RADAR_MID - 15)
		{
			GetMotorVelocity_Self(-20, 0, 0);
			Set_MotorSpeed(BasketballRobot.Velocity[0], BasketballRobot.Velocity[1], BasketballRobot.Velocity[2],BasketballRobot.Velocity[3]);
		}
		else if (Radar.Angle > RADAR_MID + 15)
		{
			GetMotorVelocity_Self(20, 0, 0);
			Set_MotorSpeed(BasketballRobot.Velocity[0], BasketballRobot.Velocity[1], BasketballRobot.Velocity[2],BasketballRobot.Velocity[3]);
		}
		else if (Radar.Distance > 500)
		{
			GetMotorVelocity_Self(0, 200, 0);
			Set_MotorSpeed(BasketballRobot.Velocity[0], BasketballRobot.Velocity[1], BasketballRobot.Velocity[2],BasketballRobot.Velocity[3]);
		}
		else if (Radar.Angle < RADAR_MID - 10)
		{
			GetMotorVelocity_Self(-15, 0, 0);
			Set_MotorSpeed(BasketballRobot.Velocity[0], BasketballRobot.Velocity[1], BasketballRobot.Velocity[2],BasketballRobot.Velocity[3]);
		}
		else if ((Radar.Angle > RADAR_MID + 10))
		{
			GetMotorVelocity_Self(15, 0, 0);
			Set_MotorSpeed(BasketballRobot.Velocity[0], BasketballRobot.Velocity[1], BasketballRobot.Velocity[2],BasketballRobot.Velocity[3]);
		}
		
		else if (Radar.Angle < RADAR_MID - 5)
		{
			GetMotorVelocity_Self(-10, 0, 0); //原来-80 0 0
			Set_MotorSpeed(BasketballRobot.Velocity[0], BasketballRobot.Velocity[1], BasketballRobot.Velocity[2],BasketballRobot.Velocity[3]);
		}
		else if (Radar.Angle > RADAR_MID + 5)
		{
			GetMotorVelocity_Self(10, 0, 0); //原来80 0 0
			Set_MotorSpeed(BasketballRobot.Velocity[0], BasketballRobot.Velocity[1], BasketballRobot.Velocity[2],BasketballRobot.Velocity[3]);
		}
		else
		{
			if(BasketballRobot.Y >= 2.5f)
				RobotGoTo(BasketballRobot.X,BasketballRobot.Y-0.5,BasketballRobot.ThetaD);
			else
				RobotGoTo(BasketballRobot.X,BasketballRobot.Y+0.5,BasketballRobot.ThetaD);
			
			GetMotorVelocity_Self(0, 300, 0);
			Set_MotorSpeed(BasketballRobot.Velocity[0], BasketballRobot.Velocity[1], BasketballRobot.Velocity[2],BasketballRobot.Velocity[3]);
			delay_ms(4000);
			Set_MotorSpeed(0, 0, 0, 0);
			break;
		}
		LCD_Show_getspeed();
	} while (1);

	//	GetMotorVelocity_Self(0, 100, 0);
	//	SetPWM(BasketballRobot.Velocity[0], BasketballRobot.Velocity[1], BasketballRobot.Velocity[2]);


	Set_MotorSpeed(0, 0, 0, 0 );
	LCD_Show_getspeed();*/
	
	//float w = 100;
	float theta = BasketballRobot.ThetaD, D_theta = 0;
	u8 time = 1;

	Set_MotorSpeed(0,0,0,0);
	LCD_Show_getspeed();
	
	Radar.RX_STA = 0;
	
	do
	{
		while((Radar.RX_STA&0x8000) == 0);//接收未完成
		
		if (!GetRadarData())
		{
			if (time == 0)
			{
			}
			else if (time++ < 5)
			{
				Set_MotorSpeed(0, 0, 0, 0);
				continue;
			}
			else if (time != 0)
				time = 0;
		}
		else
			time = 1;

		//		if(Radar.Distance < 10)
		//			continue;
	
		if (time == 0)
		{
			GetMotorVelocity_Self(0, 1000, 0);
			Set_MotorSpeed(Motor[0].SetSpeed, Motor[1].SetSpeed, Motor[2].SetSpeed,Motor[3].SetSpeed);
			delay_ms(5000);
			Set_MotorSpeed(0,0,0,0);
			break;
		}
		
		
		if (Radar.Angle < RADAR_MID - 15)
		{
			GetMotorVelocity_Self(-800, 0, 0);
			Set_MotorSpeed(Motor[0].SetSpeed, Motor[1].SetSpeed, Motor[2].SetSpeed,Motor[3].SetSpeed);
		}
		else if (Radar.Angle > RADAR_MID + 15)
		{
			GetMotorVelocity_Self(800, 0, 0);
			Set_MotorSpeed(Motor[0].SetSpeed, Motor[1].SetSpeed, Motor[2].SetSpeed,Motor[3].SetSpeed);
		}
		else if (Radar.Distance > 500)
		{
			GetMotorVelocity_Self(0, 800, 0);
			Set_MotorSpeed(Motor[0].SetSpeed, Motor[1].SetSpeed, Motor[2].SetSpeed,Motor[3].SetSpeed);
		}
		else if (Radar.Angle < RADAR_MID - 10)
		{
			GetMotorVelocity_Self(-500, 0, 0);
			Set_MotorSpeed(Motor[0].SetSpeed, Motor[1].SetSpeed, Motor[2].SetSpeed,Motor[3].SetSpeed);
		}
		else if ((Radar.Angle > RADAR_MID + 10))
		{
			GetMotorVelocity_Self(500, 0, 0);
			Set_MotorSpeed(Motor[0].SetSpeed, Motor[1].SetSpeed, Motor[2].SetSpeed,Motor[3].SetSpeed);
		}
		
		else if (Radar.Angle < RADAR_MID - 5)
		{
			GetMotorVelocity_Self(-300, 0, 0); //原来-80 0 0
			Set_MotorSpeed(Motor[0].SetSpeed, Motor[1].SetSpeed, Motor[2].SetSpeed,Motor[3].SetSpeed);
		}
		else if (Radar.Angle > RADAR_MID + 5)
		{
			GetMotorVelocity_Self(300, 0, 0); //原来80 0 0
			Set_MotorSpeed(Motor[0].SetSpeed, Motor[1].SetSpeed, Motor[2].SetSpeed,Motor[3].SetSpeed);
		}
		else
		{
			if(BasketballRobot.X>= 2.5f)
				RobotGoTo(BasketballRobot.X-0.55,BasketballRobot.Y,BasketballRobot.ThetaD);
			else
				RobotGoTo(BasketballRobot.X-0.55,BasketballRobot.Y,BasketballRobot.ThetaD);
			
			GetMotorVelocity_Self(0, 800, 0);
			Set_MotorSpeed(Motor[0].SetSpeed, Motor[1].SetSpeed, Motor[2].SetSpeed,Motor[3].SetSpeed);
			delay_ms(4000);
			Set_MotorSpeed(0,0,0,0);
			break;
		}
		LCD_Show_getspeed();
	} while (1);

	//	GetMotorVelocity_Self(0, 100, 0);
	//	SetPWM(BasketballRobot.Velocity[0], BasketballRobot.Velocity[1], BasketballRobot.Velocity[2]);


	Set_MotorSpeed(0,0,0,0);
	LCD_Show_getspeed();
}

//行至指定坐标
//X_I:目标坐标的X
//Y_I:目标坐标的Y
//Theta_I:目标坐标的角度
//angle:目标坐标和当前机器人坐标的角度
//先对准目标坐标，前行过程中判断是否有障碍，但是无法判别是篮球还是机器人，因此仅试用与球场中间空旷区域
void RobotGoToAvoid(float X_I, float Y_I, float Theta_I)
{
//	float D_Theta, D_X, D_Y, Vw = 0, sx, sy = 0,ObsDistance;
//	float Offest_theta, angle, standard = 350, Distance;

//	D_X = X_I - BasketballRobot.X;
//	D_Y = Y_I - BasketballRobot.Y;

//	angle = atan2(D_Y, D_X);

//	//if (angle > 0)
//		angle = PI / 2 - angle;

////	else
////		angle = -PI / 2 - angle;

//	RobotRotate(-angle / PI * 180);

//	D_Theta = Theta_I - BasketballRobot.ThetaD; //角度差

//	while (fabs(D_Y) > 0.05f || fabs(D_X) > 0.05f)
//	{

//		//while((Radar.RX_STA&0x8000) != 0);

//		GetRadarData();

//		if (Radar.Distance > 1500 || (Radar.State == 0))
//		{

//			sx = adjustVx_PD(D_X) / 2;

//			sy = adjustVy_PD(D_Y) / 2;

//			Vw = adjustAngleV_PD(angle - BasketballRobot.ThetaD + 360);

//			GetMotorVelocity(sx, sy, 0);

//			SetPWM(BasketballRobot.Velocity[0], BasketballRobot.Velocity[1], BasketballRobot.Velocity[2]);

//			D_Theta = Theta_I - BasketballRobot.ThetaD;
//			D_X = X_I - BasketballRobot.X;
//			D_Y = Y_I - BasketballRobot.Y;
//			//	RobotGoTo(X_I,Y_I,Theta_I);
//			//Radar.State = 0;
//		}
//		else
//		{

//			//				SetPWM(0,0,0);

//			Offest_theta = (Radar.Angle * 1.0f - RADAR_MID) * 1.0f * PI / 180;
//			Distance = Radar.Distance * sin(Offest_theta);
//			ObsDistance=0.5f+Radar.Distance * cos(Offest_theta)/1000;
//			if (fabs(Distance) > 400)
//			{
//				RobotGoTo(BasketballRobot.X - ObsDistance * sin(BasketballRobot.ThetaR), BasketballRobot.Y + ObsDistance * cos(BasketballRobot.ThetaR), BasketballRobot.ThetaD);
//				continue;
//			}

//			if (Offest_theta > 0)
//			{

//				if (Distance < 400)
//				{

//					GetMotorVelocity_Self((Distance - standard) / 5, 0, 0);
//					SetPWM(BasketballRobot.Velocity[0], BasketballRobot.Velocity[1], BasketballRobot.Velocity[2]);
//					while (((Radar.RX_STA & 0x8000) == 0));
//					//							Offest_theta=(Radar.Angle*1.0f-RADAR_MID)*PI/180;
//					//							Distance=Radar.Distance*sin(Offest_theta);
//				}
//			}
//			else
//			{
//				if (Distance > -600)
//				{
//					GetMotorVelocity_Self((Distance - standard) / 5, 0, 0);
//					SetPWM(BasketballRobot.Velocity[0], BasketballRobot.Velocity[1], BasketballRobot.Velocity[2]);

//					while (((Radar.RX_STA & 0x8000) == 0));
//					//						Offest_theta=(Radar.Angle*1.0f-RADAR_MID)*PI/180;
//					//						Distance=Radar.Distance*sin(Offest_theta);
//				}
//			}
//			//				RobotGoTo(BasketballRobot.X,Y_I,angle);
//			//				RobotGoTo(X_I,Y_I,Theta_I);
//			Radar.State = 0;
//		}
//	}
//	SetPWM(0, 0, 0);
//	delay_ms(1000);
//	RobotRotate(Theta_I);

	float D_Theta, D_X, D_Y, Vw = 0, sx, sy = 0,ObsDistance;
	float Offest_theta, angle, standard = 350, Distance;

	D_X = X_I - BasketballRobot.X;
	D_Y = Y_I - BasketballRobot.Y;

	angle = atan2(D_Y, D_X);

	//if (angle > 0)
		angle = PI / 2 - angle;

//	else
//		angle = -PI / 2 - angle;

	RobotRotate(-angle / PI * 180);

	D_Theta = Theta_I - BasketballRobot.ThetaD; //角度差

	while (fabs(D_Y) > 0.05f || fabs(D_X) > 0.05f)
	{

		//while((Radar.RX_STA&0x8000) != 0);

		GetRadarData();

		if (Radar.Distance > 1500 || (Radar.State == 0))
		{

			sx = adjustVx_PD(D_X) / 2;

			sy = adjustVy_PD(D_Y) / 2;

			Vw = adjustAngleVw_PD(angle - BasketballRobot.ThetaD + 360);

			GetMotorVelocity(sx, sy, 0);

			Set_MotorSpeed(Motor[0].SetSpeed, Motor[1].SetSpeed, Motor[2].SetSpeed, Motor[3].SetSpeed);

			D_Theta = Theta_I - BasketballRobot.ThetaD;
			D_X = X_I - BasketballRobot.X;
			D_Y = Y_I - BasketballRobot.Y;
			//	RobotGoTo(X_I,Y_I,Theta_I);
			//Radar.State = 0;
		}
		else
		{

			//				SetPWM(0,0,0);

			Offest_theta = (Radar.Angle * 1.0f - RADAR_MID) * 1.0f * PI / 180;
			Distance = Radar.Distance * sin(Offest_theta);
			ObsDistance=0.5f+Radar.Distance * cos(Offest_theta)/1000;
			if (fabs(Distance) > 400)
			{
				RobotGoTo(BasketballRobot.X - ObsDistance * sin(BasketballRobot.ThetaR), BasketballRobot.Y + ObsDistance * cos(BasketballRobot.ThetaR), BasketballRobot.ThetaD);
				continue;
			}

			if (Offest_theta > 0)
			{

				if (Distance < 400)
				{

					GetMotorVelocity_Self((Distance - standard) / 5, 0, 0);
					Set_MotorSpeed(Motor[0].SetSpeed, Motor[1].SetSpeed, Motor[2].SetSpeed, Motor[3].SetSpeed);
					while (((Radar.RX_STA & 0x8000) == 0));
					//							Offest_theta=(Radar.Angle*1.0f-RADAR_MID)*PI/180;
					//							Distance=Radar.Distance*sin(Offest_theta);
				}
			}
			else
			{
				if (Distance > -600)
				{
					GetMotorVelocity_Self((Distance - standard) / 5, 0, 0);
					Set_MotorSpeed(Motor[0].SetSpeed, Motor[1].SetSpeed, Motor[2].SetSpeed, Motor[3].SetSpeed);

					while (((Radar.RX_STA & 0x8000) == 0));
					//						Offest_theta=(Radar.Angle*1.0f-RADAR_MID)*PI/180;
					//						Distance=Radar.Distance*sin(Offest_theta);
				}
			}
			//				RobotGoTo(BasketballRobot.X,Y_I,angle);
			//				RobotGoTo(X_I,Y_I,Theta_I);
			Radar.State = 0;
		}
	}
	Set_MotorSpeed(0, 0, 0, 0);
	delay_ms(1000);
	RobotRotate(Theta_I);
}
