//This program is developed to control the Panasonic AC servo motor
//with the position control mode, the program is ran on a Googol
//8-axis motion controller


//load header files
#include <stdio.h>   
#include <process.h>
#include <windows.h>
#include "gts.h"
//using namespace std;

#define AXIS_1 1
#define AXIS_2 2
#define AXIS_3 3
#define AXIS_4 4
#define AXIS_5 5
#define AXIS_6 6
#define AXIS_7 7

//set a route for axes
double route[120][6]={
200	,	1000	,	4000	,	1000	,	4000	,	1000	,
400	,	3000	,	8000	,	3000	,	8000	,	3000	,
600	,	6000	,	12000	,	6000	,	12000	,	6000	,
800	,	10000	,	8000	,	10000	,	8000	,	10000	,
1000	,	13000	,	4000	,	13000	,	4000	,	13000	,
1200	,	15000	,	0	,	15000	,	0	,	15000	,
1400	,	16000	,	4000	,	16000	,	4000	,	16000	,
1600	,	15000	,	8000	,	15000	,	8000	,	15000	,
1800	,	13000	,	12000	,	13000	,	12000	,	13000	,
2000	,	10000	,	8000	,	10000	,	8000	,	10000	,
2200	,	6000	,	4000	,	6000	,	4000	,	6000	,
2400	,	3000	,	0	,	3000	,	0	,	3000	,
2600	,	1000	,	4000	,	1000	,	4000	,	1000	,
2800	,	0	,	8000	,	0	,	8000	,	0	,
3000	,	1000	,	12000	,	1000	,	12000	,	1000	,
3200	,	3000	,	8000	,	3000	,	8000	,	3000	,
3400	,	6000	,	4000	,	6000	,	4000	,	6000	,
3600	,	10000	,	0	,	10000	,	0	,	10000	,
3800	,	13000	,	4000	,	13000	,	4000	,	13000	,
4000	,	15000	,	8000	,	15000	,	8000	,	15000	,
4200	,	16000	,	12000	,	16000	,	12000	,	16000	,
4400	,	15000	,	8000	,	15000	,	8000	,	15000	,
4600	,	13000	,	4000	,	13000	,	4000	,	13000	,
4800	,	10000	,	0	,	10000	,	0	,	10000	,
5000	,	6000	,	4000	,	6000	,	4000	,	6000	,
5200	,	3000	,	8000	,	3000	,	8000	,	3000	,
5400	,	1000	,	12000	,	1000	,	12000	,	1000	,
5600	,	0	,	8000	,	0	,	8000	,	0	,
5800	,	1000	,	4000	,	1000	,	4000	,	1000	,
6000	,	3000	,	0	,	3000	,	0	,	3000	,
6200	,	6000	,	4000	,	6000	,	4000	,	6000	,
6400	,	10000	,	8000	,	10000	,	8000	,	10000	,
6600	,	13000	,	12000	,	13000	,	12000	,	13000	,
6800	,	15000	,	8000	,	15000	,	8000	,	15000	,
7000	,	16000	,	4000	,	16000	,	4000	,	16000	,
7200	,	15000	,	0	,	15000	,	0	,	15000	,
7400	,	13000	,	4000	,	13000	,	4000	,	13000	,
7600	,	10000	,	8000	,	10000	,	8000	,	10000	,
7800	,	6000	,	12000	,	6000	,	12000	,	6000	,
8000	,	3000	,	8000	,	3000	,	8000	,	3000	,
8200	,	1000	,	4000	,	1000	,	4000	,	1000	,
8400	,	0	,	0	,	0	,	0	,	0	,
8600	,	1000	,	4000	,	1000	,	4000	,	1000	,
8800	,	3000	,	8000	,	3000	,	8000	,	3000	,
9000	,	6000	,	12000	,	6000	,	12000	,	6000	,
9200	,	10000	,	8000	,	10000	,	8000	,	10000	,
9400	,	13000	,	4000	,	13000	,	4000	,	13000	,
9600	,	15000	,	0	,	15000	,	0	,	15000	,
9800	,	16000	,	4000	,	16000	,	4000	,	16000	,
10000	,	15000	,	8000	,	15000	,	8000	,	15000	,
10200	,	13000	,	12000	,	13000	,	12000	,	13000	,
10400	,	10000	,	8000	,	10000	,	8000	,	10000	,
10600	,	6000	,	4000	,	6000	,	4000	,	6000	,
10800	,	3000	,	0	,	3000	,	0	,	3000	,
11000	,	1000	,	4000	,	1000	,	4000	,	1000	,
11200	,	0	,	8000	,	0	,	8000	,	0	,
11400	,	1000	,	12000	,	1000	,	12000	,	1000	,
11600	,	3000	,	8000	,	3000	,	8000	,	3000	,
11800	,	6000	,	4000	,	6000	,	4000	,	6000	,
12000	,	10000	,	0	,	10000	,	0	,	10000	,
12200	,	13000	,	4000	,	13000	,	4000	,	13000	,
12400	,	15000	,	8000	,	15000	,	8000	,	15000	,
12600	,	16000	,	12000	,	16000	,	12000	,	16000	,
12800	,	15000	,	8000	,	15000	,	8000	,	15000	,
13000	,	13000	,	4000	,	13000	,	4000	,	13000	,
13200	,	10000	,	0	,	10000	,	0	,	10000	,
13400	,	6000	,	4000	,	6000	,	4000	,	6000	,
13600	,	3000	,	8000	,	3000	,	8000	,	3000	,
13800	,	1000	,	12000	,	1000	,	12000	,	1000	,
14000	,	0	,	8000	,	0	,	8000	,	0	,
14200	,	1000	,	4000	,	1000	,	4000	,	1000	,
14400	,	3000	,	0	,	3000	,	0	,	3000	,
14600	,	6000	,	4000	,	6000	,	4000	,	6000	,
14800	,	10000	,	8000	,	10000	,	8000	,	10000	,
15000	,	13000	,	12000	,	13000	,	12000	,	13000	,
15200	,	15000	,	8000	,	15000	,	8000	,	15000	,
15400	,	16000	,	4000	,	16000	,	4000	,	16000	,
15600	,	15000	,	0	,	15000	,	0	,	15000	,
15800	,	13000	,	4000	,	13000	,	4000	,	13000	,
16000	,	10000	,	8000	,	10000	,	8000	,	10000	,
16200	,	6000	,	12000	,	6000	,	12000	,	6000	,
16400	,	3000	,	8000	,	3000	,	8000	,	3000	,
16600	,	1000	,	4000	,	1000	,	4000	,	1000	,
16800	,	0	,	0	,	0	,	0	,	0	,
17000	,	1000	,	4000	,	1000	,	4000	,	1000	,
17200	,	3000	,	8000	,	3000	,	8000	,	3000	,
17400	,	6000	,	12000	,	6000	,	12000	,	6000	,
17600	,	10000	,	8000	,	10000	,	8000	,	10000	,
17800	,	13000	,	4000	,	13000	,	4000	,	13000	,
18000	,	15000	,	0	,	15000	,	0	,	15000	,
18200	,	16000	,	4000	,	16000	,	4000	,	16000	,
18400	,	15000	,	8000	,	15000	,	8000	,	15000	,
18600	,	13000	,	12000	,	13000	,	12000	,	13000	,
18800	,	10000	,	8000	,	10000	,	8000	,	10000	,
19000	,	6000	,	4000	,	6000	,	4000	,	6000	,
19200	,	3000	,	0	,	3000	,	0	,	3000	,
19400	,	1000	,	4000	,	1000	,	4000	,	1000	,
19600	,	0	,	8000	,	0	,	8000	,	0	,
19800	,	1000	,	12000	,	1000	,	12000	,	1000	,
20000	,	3000	,	8000	,	3000	,	8000	,	3000	,
20200	,	6000	,	4000	,	6000	,	4000	,	6000	,
20400	,	10000	,	0	,	10000	,	0	,	10000	,
20600	,	13000	,	4000	,	13000	,	4000	,	13000	,
20800	,	15000	,	8000	,	15000	,	8000	,	15000	,
21000	,	16000	,	12000	,	16000	,	12000	,	16000	,
21200	,	15000	,	8000	,	15000	,	8000	,	15000	,
21400	,	13000	,	4000	,	13000	,	4000	,	13000	,
21600	,	10000	,	0	,	10000	,	0	,	10000	,
21800	,	6000	,	4000	,	6000	,	4000	,	6000	,
22000	,	3000	,	8000	,	3000	,	8000	,	3000	,
22200	,	1000	,	12000	,	1000	,	12000	,	1000	,
22400	,	0	,	8000	,	0	,	8000	,	0	,
22600	,	0	,	4000	,	0	,	4000	,	0	,
22800	,	0	,	0	,	0	,	0	,	0	,
23000	,	0	,	4000	,	0	,	4000	,	0	,
23200	,	0	,	8000	,	0	,	8000	,	0	,
23400	,	0	,	12000	,	0	,	12000	,	0	,
23600	,	0	,	8000	,	0	,	8000	,	0	,
23800	,	0	,	4000	,	0	,	4000	,	0	,
24000	,	0	,	0	,	0	,	0	,	0	,

};

LARGE_INTEGER litmp;   //define time loop variables
LONGLONG QPart1,QPart2;
double dfMinus, dfFreq, dfTim;

double deltaT=20;   //cycle
double count=0;   //count how many times the loop has performed

bool loopflag=FALSE;   //a flag that controls whether the loop can be performed or not

void InitPID(TPid *PID, double kp, double ki, double kd)   //initiate PID parameters for axis 1 and 2
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

void InitAxis()   //initiate all axes, turn on servo
{
	//TJogPrm jog;
	TPid pid_1,pid_2;
	
	GT_Open();   //operations on googol motion controller
	GT_Reset();
	GT_LoadConfig("GTS800-2.cfg");
	GT_ClrSts(1,8);
	
	for(short i=1;i<=7;i++)   //set motion of 7 axes as PT
	{
		GT_PrfPt(i, PT_MODE_DYNAMIC);	//set the movement of each motor to PT dynamic mode
		GT_PtClear(i);	//clear the cache for trajectory of each motor
	}
	
	InitPID(&pid_1,20,0,0);   //initiate PID for axis 1
	InitPID(&pid_2,5,0,0);   //initiate PID for axis 2
	
	GT_SetPid(AXIS_1,1,&pid_1);   //set PID for axis 1
	GT_SetPid(AXIS_2,1,&pid_2);   //set PID for axis 2
	
	for (i=1;i<=7;i++)   //turn on servo
	{
		Sleep(20);
		GT_AxisOn(i);
	}
}

void TurnOffAxis()   //turn off servo
{
	Sleep(500);
	
	for(int i=1;i<=7;i++)
	{
		Sleep(20);
		GT_AxisOff(i);
	}
	
}

unsigned int __stdcall ThreadFun(PVOID pM)     //thread function
{  
	InitAxis();
	

	double enc_0[8];   //put encoder data in it

	short sRtn,space;	//space is used to store the acquired remaining space of the trajectory planning cache of a specific motor
	int i=0;
	short flag_start=0;	//label,indicating whether the movement has been initialized
	
	double pos;	//read position information of the planned trajectory, and send to the PT mode cache of the motor
	long time;	//read the temporal information from the planned trajectory and store it in time, send time to PT-mode cache of the motor
	
	long RouteLength=sizeof(route)/sizeof(route[0]);   //calculate how many steps the route contain


	do
	{
		GT_PtSpace(AXIS_5, &space);   //get how much free space the axis has to contain route data
		
		time=(long)route[i][0];	//read the temporal information from the planned trajectory and store it in time
		pos=route[i][1];	//read the position information from the planned trajectory and store it in pos
		sRtn = GT_PtData(AXIS_1, pos, time,1);   //push route data
		
		pos=route[i][2];
		sRtn = GT_PtData(AXIS_2, pos, time,1);

		pos=route[i][3];
		sRtn = GT_PtData(AXIS_3, pos, time,1);

		pos=route[i][4];
		sRtn = GT_PtData(AXIS_4, pos, time,1);

		pos=route[i][5];
		sRtn = GT_PtData(AXIS_5, pos, time,1);
		
		i++;
		space=space-1;
		
		if(i>=RouteLength)   //if all the route data has been sent to axis, then break this loop
			break;
		
	} while (space>0);
	sRtn = GT_PtStart(1<<(AXIS_1-1));   //start motion
	sRtn = GT_PtStart(1<<(AXIS_2-1));
	sRtn = GT_PtStart(1<<(AXIS_3-1));
	sRtn = GT_PtStart(1<<(AXIS_4-1));
	sRtn = GT_PtStart(1<<(AXIS_5-1));
	flag_start=1;
	
	//通过上面的while循环，快速把第一批轨迹点压入轴的轨迹寄存器，保证在用户启动运动后，电机能够尽快开始执行动作。
	//接着执行下面的while循环，不断查询各轴的轨迹寄存器是否有剩余空间，并将剩余的轨迹点压入。同时，下面的循环也带有显示各轴当前位置的功能。

	do 
	{
		QueryPerformanceCounter(&litmp);//before the loop, check the CPU clock value
		QPart1 = litmp.QuadPart;
		
		
		
		
		
		GT_GetEncPos(1,&enc_0[0],8);   //get encoder data(current position, in palse)
		
		
		sRtn = GT_PtSpace(AXIS_5, &space);   //get how much free space the axis has to contain route data
		if(space>0)   //if there is free space, push route data into axis
		{
			if (i<RouteLength)
			{
				
				time=(long)route[i][0];
				pos=route[i][1];
				sRtn = GT_PtData(AXIS_1, pos, time,1);   //push route data into axis
				
				pos=route[i][2];
				sRtn = GT_PtData(AXIS_2, pos, time,1);
				
				pos=route[i][3];
				sRtn = GT_PtData(AXIS_3, pos, time,1);
				
				pos=route[i][4];
				sRtn = GT_PtData(AXIS_4, pos, time,1);
				
				pos=route[i][5];
				sRtn = GT_PtData(AXIS_5, pos, time,1);
				
				i++;
			}
			
		}
		

		count++;
		
		printf("\ncount:%lf\n",count);   //print encoder data on screen
		printf("enc_0[0]:%lf\n",enc_0[0]/10000*360/10);
		printf("enc_0[1]:%lf\n",enc_0[1]/10000*360/20);
		printf("enc_0[2]:%lf\n",enc_0[2]/10000*360/25);
		printf("enc_0[3]:%lf\n",enc_0[3]/10000*360/20);
		printf("enc_0[4]:%lf\n",enc_0[4]/10000*360/15);
		printf("enc_0[5]:%lf\n",enc_0[5]);
		printf("enc_0[6]:%lf\n",enc_0[6]);
		printf("enc_0[7]:%lf\n",enc_0[7]);

		do 
		{
			QueryPerformanceCounter(&litmp);//get the current counter of CPU and store it in QPart2
			QPart2 = litmp.QuadPart;
			dfMinus = (double)(QPart2-QPart1);//calculate the difference between current value and the initial value
			                                  //to determine whether the time set by deltaT has been reached
			dfTim = dfMinus / dfFreq * 1000;
		} while (dfTim<deltaT);   //CPU will process this loop until 15ms. 
		                          //The while loop continues to check whether the time set by deltaT has been reached


	} while (loopflag);

	TurnOffAxis();
	
    return 0;  
}  

int main()
{

	printf("Press ENTER to turn off robot control process...\n");
	
	GT_ZeroPos(1, 8);   //set encoders of 7 axes to 0
	
	loopflag=TRUE;

	QueryPerformanceFrequency(&litmp);   //get CPU frequency
    dfFreq = (double)litmp.QuadPart;
	
    HANDLE handle;  
    handle= (HANDLE)_beginthreadex(NULL, 0, ThreadFun, NULL, 0, NULL);     //creat a new thread to process the thread function

	getchar();   //function will not continue until you put ENTER on keyboard
	
	loopflag=FALSE;

	WaitForSingleObject(handle,INFINITE);   //wait for the thread to be finished
	
	return 0;
}