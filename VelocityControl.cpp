//This program is developed to control the Panasonic AC servo motor
//with the position control mode, the program is ran on a Googol
//8-axis motion controller


#include <stdio.h>	//load head files
#include <process.h>
#include <windows.h>
#include "gts.h"   //load the head files of Googol motion controller

//using namespace std;

#define AXIS_1 1   //define the name of axis/motor
#define AXIS_2 2
#define AXIS_3 3
#define AXIS_4 4
#define AXIS_5 5
#define AXIS_6 6
#define AXIS_7 7

LARGE_INTEGER litmp;	//define time loop variables，which is a structure to store CPU clock frequency and current CPU counter
LONGLONG QPart1,QPart2;	//用于存储当前CPU计数值，QPart1用来存储一次循环开始时的CPU计数值，QPart2用来存储一次循环结束时的计数值；
double dfMinus, dfFreq, dfTim;	//dfMinus存放当前CPU时钟计数值和循环开始时的计数值的差值；dfFreq存放CPU频率；dfTim为经过之后的当前时间和循环开始时的时间差，单位是毫秒；

double deltaT_15=1500;	//the unit is in ms
double deltaT=2000;	//the unit is in ms
double count=0;	//count how many times the loop has performed

bool loopflag=FALSE;	//a flag that controls whether the loop can be performed or not

void InitPID(TPid *PID, double kp, double ki, double kd)	//initiate PID parameters for axis 1 and 2
{
	PID->kp=kp;
	PID->ki=ki;
	PID->kd=kd;
	PID->kvff=0;
	PID->kaff=0;
	PID->integralLimit=32767;
	PID->derivativeLimit=32767;
	PID->limit=32767;
}

void InitAxis()	//initiate all axes, turn on servo
{
	TJogPrm jog;	//define the struct to store parameters related with the jog mode
	TPid pid_1,pid_2;	//define the struct to store the PID parameters of the motor
	
	GT_Open();	//operations on googol motion controller
	GT_Reset();	//operations on googol motion controller
	GT_LoadConfig("GTS800-2.cfg");	//operations on googol motion controller
	GT_ClrSts(1,8);	//operations on googol motion controller
	
	for(short i=1;i<=7;i++)	//set motion of 7 axes as jog
	{
		GT_PrfJog(i);	//set the motor to jog mode
		GT_GetJogPrm(i, &jog);	//get parameters of current axis/motor in the jog mode
		jog.acc=1;	//acceleration of the motor in jog mode with the unit pulse/ms^2
		jog.dec=1;	//deceleration of the motor in jog mode with the unit pulse/ms^2
		GT_SetJogPrm(i, &jog);
	}
	
	InitPID(&pid_1,20,0,0);	//initiate PID for axis 1
	InitPID(&pid_2,5,0,0);	//initiate PID for axis 2
	
	GT_SetPid(AXIS_1,1,&pid_1); //set PID for axis 1
	GT_SetPid(AXIS_2,1,&pid_2);	//set PID for axis 2
	
	for (i=1;i<=7;i++)	//turn on servo
	{
		GT_AxisOn(i);
	}
}

void TurnOffAxis()	//turn off servo
{
	for(short i=1;i<=7;i++)	//apply zero speed on axes first
	{
		GT_SetVel(i,0);
		GT_Update(1<<(i-1));
	}
	Sleep(1000);	//pause 1000 ms to stabalize the motor
	for(i=1;i<=7;i++)	//turn off servo next
	{
		GT_AxisOff(i);
	}
	
}

unsigned int __stdcall ThreadFun(PVOID pM)		//thread function
{  
	InitAxis();
	
	double Mtr_spd_1=20;	//initiate motor speed for the first loop
	double Mtr_spd_2=20;
	double Mtr_spd_3=20;
	double Mtr_spd_4=20;
	double Mtr_spd_5=20;
	double Mtr_spd_6=20;
	double Mtr_spd_7=20;

	double enc_0[8],enc_15[8];	//put encoder data in these two arrays

	do 
	{
		QueryPerformanceCounter(&litmp);	//get the current CPU clock before the loop
		QPart1 = litmp.QuadPart;	//get current CPU clock


		GT_GetEncPos(1,&enc_0[0],8);	//get encoder data(current position, in palse)

		GT_SetVel(AXIS_1, 5);	//set velocity
		GT_Update(1<<(AXIS_1-1));	//update axis speed

		GT_SetVel(AXIS_2, 5);
		GT_Update(1<<(AXIS_2-1));

		GT_SetVel(AXIS_3, 5);
		GT_Update(1<<(AXIS_3-1));

		GT_SetVel(AXIS_4, 5);
		GT_Update(1<<(AXIS_4-1));

		GT_SetVel(AXIS_5, 10);
		GT_Update(1<<(AXIS_5-1));


		count++;
		

		

		do 
		{
			QueryPerformanceCounter(&litmp);	//get the current counter of the CPU and store it in QPart2
			QPart2 = litmp.QuadPart;
			dfMinus = (double)(QPart2-QPart1);	//calculate the difference between current value and the initial value
			                                    //to determine whether the time set by deltaT has been reached
			dfTim = dfMinus / dfFreq * 1000;
		} while (dfTim<deltaT_15);	//CPU will process this loop until 15ms
		                            //The while loop continues to check whether the time set by deltaT has been reached


		//Here add your work that you want the program to do between 15ms and 20ms in every cycle.

		GT_GetEncPos(1,&enc_15[0],8);

		GT_SetVel(AXIS_1, -15);
		GT_Update(1<<(AXIS_1-1));

		GT_SetVel(AXIS_2, -15);
		GT_Update(1<<(AXIS_2-1));

		GT_SetVel(AXIS_3, -15);
		GT_Update(1<<(AXIS_3-1));

		GT_SetVel(AXIS_4, -15);
		GT_Update(1<<(AXIS_4-1));

		GT_SetVel(AXIS_5, -30);
		GT_Update(1<<(AXIS_5-1));

		
		printf("\ncount:%lf\n",count);	//print encoder data on screen
		printf("enc_15[0]:%lf\n",enc_15[0]/10000*360/10);	//电机每转有10000个脉冲，/10000*360后转换为电机角度。再/减速比，得到实际输出角度。
		printf("enc_15[1]:%lf\n",enc_15[1]/10000*360/20);
		printf("enc_15[2]:%lf\n",enc_15[2]/10000*360/25);
		printf("enc_15[3]:%lf\n",enc_15[3]/10000*360/20);
		printf("enc_15[4]:%lf\n",enc_15[4]/10000*360/15);
		printf("enc_15[5]:%lf\n",enc_15[5]);
		printf("enc_15[6]:%lf\n",enc_15[6]);
		printf("enc_15[7]:%lf\n",enc_15[7]);

		

		do 
		{
			QueryPerformanceCounter(&litmp);
			QPart2 = litmp.QuadPart;
			dfMinus = (double)(QPart2-QPart1);
			dfTim = dfMinus / dfFreq * 1000;
		} while (dfTim<deltaT);	//CPU will process this loop until 20ms

	} while (loopflag);
	
	TurnOffAxis();
	
    return 0;  
}  

int main()
{

	printf("Press ENTER to turn off robot control process...\n");
	
	GT_ZeroPos(1, 8);	//set encoders of 7 axes to 0
	
	loopflag=TRUE;

	QueryPerformanceFrequency(&litmp);	//get CPU frequency
    dfFreq = (double)litmp.QuadPart;
	
    HANDLE handle;  
    handle= (HANDLE)_beginthreadex(NULL, 0, ThreadFun, NULL, 0, NULL);		//creat a new thread to process the thread function

	getchar();	//function will not continue until you put ENTER on keyboard
	
	loopflag=FALSE;

	WaitForSingleObject(handle,INFINITE);	//wait for the thread to be finished
	
	return 0;
}


