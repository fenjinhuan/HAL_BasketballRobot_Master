#ifndef _CONTROL_H
#define _CONTROL_H

#include "stm32f4xx_HAL.h"
#include "sys.h"
#include "pid.h"
#include <math.h>

#define CURRENT_LIM 1000				//������Сֵ
#define CURRENT_MAX 16000				//�������ֵ
#define PI 3.141592654f

#define PLATFORM_X 0.8f  //ƽ̨���ֳ��������ľ����һ��
#define PLATFORM_Y 0.5f  //ƽ̨���ֿ������ľ����һ��
#define WHEEL_R 0.02     //С�������ĵ��������ĵľ���

enum{
    SPEED = 0,
    CURRENT 	,
    POSITION 	,
};

typedef struct
{
	uint16_t Angle;		  //ת�ӻ�е�Ƕ�
	int16_t Speed;	      //ת��ת��
	int16_t Current;      //ת�ص���
	int16_t Temperature;  //����¶�
	int16_t SetSpeed;
	int16_t SetAngle;
	
	PID_t SpeedPID;
	PID_t AnglePID;
	
	int16_t SpeedOutput;
	int16_t AngleOutput;

}MOTOR;

typedef struct
{
    uint8_t Count;
	uint8_t Buf[20];
	u8		Sum;
	u8		pidReadBuf;
	PID_t* 	pidAdjust;
} RxPID;

typedef struct
{
	float X;        //������������ϵ��y����
	float Y;		//������������ϵ��y����
	float PX;		//���x����
	float PY;		//���y����
	float ThetaR;   //�������������x��н� ����
	float ThetaD;	//�������������y��н� �Ƕ�
	
	float Vx;		//������������ϵx�����ٶ�
	float Vy;		//������������ϵy�����ٶ�	
	float W;		//�����˽��ٶȣ�˳ʱ��������
	
}ROBOT;

//extern PID_t* pidAdjust;
extern MOTOR Motor[4];
extern ROBOT BasketballRobot;
extern RxPID rxPID;

void MOTOR_ControlInit(void);
void SetMotorSpeed_pid(void);
void SetMotorAngle_pid(void);
void GetMotorVelocity_Self(float vx, float vy, float w);
void GetMotorVelocity(float vx, float vy, float w);
void Set_MotorSpeed(float V1,float V2,float V3,float V4);

#endif
