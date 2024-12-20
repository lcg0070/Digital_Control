//
// Created by User on 2024-10-02.
//

#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <windows.h>
#include <time.h>
#include "NIDAQmx.h"

double GetWindowTime(void)
{
    LARGE_INTEGER	liEndCounter, liFrequency;

    QueryPerformanceCounter(&liEndCounter);
    QueryPerformanceFrequency(&liFrequency);

    return(liEndCounter.QuadPart / (double)(liFrequency.QuadPart) * 1000.0);
};


#define   SAMPLING_FREQ		(double)(                200)  // 10Hz //50, 100, 125, 150
#define   FINAL_TIME		(double)(                 5 )
#define   N_STEP		    (int)   ( FINAL_TIME*SAMPLING_FREQ )
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
    FILE*		pFile;
    memset( &Time, 0, sizeof(TIME));
    INIT_BUF(Time);

    // Time.curr = 0.0;
    // Time.prev = 0.0;

    double		time_curr = 0.0;
    double		time_init = 0.0;
    double		time;
    char		OutFileName[100] = { "" };

    double		Freq = 1.0;

    int			idx = 0;
    int			count = 0;

    unsigned	i = 0;
    double		Standard = 2.5;
    double		OutData[N_STEP] = { 0.0, };
    double		OutTime[N_STEP] = { 0.0, };
    double		OutVcmd[N_STEP] = { 0.0, };

    double		Vcmd = 0.0;
    float64		Vin = 0.0;

    TaskHandle	taskAI = 0;
    TaskHandle	taskAO = 0;

    DAQmxCreateTask("", &taskAI);
    DAQmxCreateTask("", &taskAO);

    DAQmxCreateAIVoltageChan(taskAI, "Dev1/ai0", "", DAQmx_Val_RSE, -10.0, 10.0, DAQmx_Val_Volts, "");
    DAQmxCreateAOVoltageChan(taskAO, "Dev1/ao0", "", 0.0, 5.0, DAQmx_Val_Volts, "");

    DAQmxStartTask(taskAI);
    DAQmxStartTask(taskAO);

    printf("Press any key to start the program.... \n");
    getchar();

    time_init = GetWindowTime();

    // main loop
    do
    {
        time = SAMPLING_TIME * count;

        /* DAQ Reading */
//        DAQmxReadAnalogF64(taskAI, DAQmx_Val_Auto, 10.0, DAQmx_Val_GroupByChannel, Vin, 4, NULL, NULL);
        DAQmxReadAnalogScalarF64(taskAI, 10.0, &Vin, NULL);

        /* DAQ Writing : Analog Output */

        Vcmd = Vin + 2.5;
        WRITE_SINGLE_CH_AO(taskAO, Vcmd);


//        OutTime[count] = time;
//        OutVcmd[count] = Vcmd;
//        OutData[count] = Vin;

        /* check the simulation time and loop count */
        while (YOK)
        {
            time_curr = GetWindowTime();

            if (time_curr - (time_init + SAMPLING_TIME * (double)count * 1000.0) >= (SAMPLING_TIME * 1000.0))	break;
        }
    } while (1);

    DAQmxStopTask(taskAI);
    DAQmxStopTask(taskAO);

    /* Data Print */

    sprintf(OutFileName, "%1.1f", SAMPLING_FREQ);

    pFile = fopen(strcat(OutFileName, "_data.out"), "w+t");

    for (idx = 0; idx < N_STEP; idx++)
    {
        fprintf(pFile, "%20.10f %20.10f %20.10f \n", OutTime[idx],OutVcmd[idx] ,OutData[idx]);
//        fprintf(pFile, "%20.10f %20.10f\n", OutTime[idx], OutData[idx]);
    }

    fclose(pFile);
//     fcloseall();

}