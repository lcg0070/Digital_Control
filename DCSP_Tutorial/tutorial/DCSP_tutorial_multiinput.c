//
// Created by User on 2024-09-06.
//

#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <Windows.h>
#include <time.h>
#include "NIDAQmx.h"

double GetWindowTime(void)
{
	LARGE_INTEGER	liEndCounter, liFrequency;

	QueryPerformanceCounter(&liEndCounter);
	QueryPerformanceFrequency(&liFrequency);

	return(liEndCounter.QuadPart / (double)(liFrequency.QuadPart) * 1000.0);
};

#define   N_STEP		    (int)   ( FINAL_TIME*SAMPLING_FREQ )
#define   FINAL_TIME		(double)(				5.0 )
#define   SAMPLING_FREQ		(double)(               10 )  // 10Hz
#define   SAMPLING_TIME     (double)( 1.0/SAMPLING_FREQ )
#define   UNIT_PI			(double)( 3.14159265358979  )


enum LogicFlag{ NOK, YOK};

#define WRITE_SINGLE_CH_AO( task, value) DAQmxWriteAnalogScalarF64(taskAO, 1.0, 5.0, Vcmd, NULL);
#define INIT_BUF(buf)						memset(&buf, 0, sizeof(buf));

typedef struct
{
    double OutData[1000];
    double StopCondition;
}DEBUG;

DEBUG debug;
typedef double REAL;
typedef unsigned int COUNT;

typedef struct
{
    REAL prev;
    REAL curr;
}TIME;
TIME Time;

// ------------------------------------------------------------------------------------------

int main(void)
{
	TaskHandle	taskAI = 0;
	TaskHandle	taskAO = 0;

	float64		Vin[2] = {0.0};
	double		Vcmd[2] = {0.0};

	DAQmxCreateTask("", &taskAI);
	DAQmxCreateTask("", &taskAO);

	DAQmxCreateAIVoltageChan(taskAI, "Dev1/ai0:1", "", DAQmx_Val_RSE, -10.0, 10.0, DAQmx_Val_Volts, "");
	DAQmxCreateAOVoltageChan(taskAO, "Dev1/ao0:1", "", 0.0, 5.0, DAQmx_Val_Volts, "");

	DAQmxStartTask(taskAI);
	DAQmxStartTask(taskAO);

	do
	{
		Vcmd[0] = 5.0;
		Vcmd[1] = 3.0;
		// numSampsperChan - 몇 샘플링 시간동안 채널별로 값을 받아오는지
		DAQmxWriteAnalogF64(taskAO,1,1.0, 5.0, DAQmx_Val_GroupByChannel, Vcmd, NULL, NULL);
		// DAQmxWriteAnalogScalarF64(taskAO, 1.0, 5.0, Vcmd, NULL);
		DAQmxReadAnalogF64(taskAI, DAQmx_Val_Auto, 10.0, DAQmx_Val_GroupByChannel, Vin, 2, NULL, NULL);
	}while(1);
	DAQmxStopTask(taskAI);
	DAQmxStopTask(taskAO);

	printf("%f\n", Vin[0]);
	printf("%f\n", Vin[1]);


    return 0;
}
