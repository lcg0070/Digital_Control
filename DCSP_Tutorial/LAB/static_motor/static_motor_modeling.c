//
// Created by User on 2024-11-06.
//

#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <Windows.h>
#include <time.h>
#include "NIDAQmx.h"

#include "DAQ_include.h"

//double GetWindowTime(void){
//    LARGE_INTEGER	liEndCounter, liFrequency;
//
//    QueryPerformanceCounter(&liEndCounter);
//    QueryPerformanceFrequency(&liFrequency);
//
//    return(liEndCounter.QuadPart / (double)(liFrequency.QuadPart) * 1000.0);
//};


#define   FINAL_TIME		(double)(				10.0 )
#define   SAMPLING_FREQ		(double)(               200  )  // 200Hz
#define   N_STEP		    (int)   ( FINAL_TIME*SAMPLING_FREQ )
#define   HALT_STEP		    (int)   ( 5.0*SAMPLING_FREQ )
#define   SAMPLING_TIME     (double)( 1.0/SAMPLING_FREQ )
#define   UNIT_PI			(double)( 3.14159265358979  )


//enum LogicFlag{ NOK, YOK};

//#define WRITE_SINGLE_CH_AO( task, value) DAQmxWriteAnalogScalarF64(taskAO, 1.0, 5.0, Vcmd, NULL);
//#define INIT_BUF(buf)						memset(&buf, 0, sizeof(buf));



typedef double REAL;

//typedef struct
//{
//    REAL prev;
//    REAL curr;
//}TIME;
//TIME Time;

// ------------------------------------------------------------------------------------------

int main(void)
{
    TaskHandle	taskAI = 0;
    TaskHandle	taskAO = 0;

    float64		Vin = 0.0;
    double		Vcmd[2] = {0.0};

    DAQmxCreateTask("", &taskAI);
    DAQmxCreateTask("", &taskAO);

    DAQmxCreateAIVoltageChan(taskAI, "Dev1/ai2", "", DAQmx_Val_Diff, -10.0, 10.0, DAQmx_Val_Volts, "");
    DAQmxCreateAOVoltageChan(taskAO, "Dev1/ao0:1", "", 0.0, 5.0, DAQmx_Val_Volts, "");

    DAQmxStartTask(taskAI);
    DAQmxStartTask(taskAO);



    double		time_curr = 0.0;
    double		time_init = 0.0;
    double		time;
    char		OutFileName[100] = { "" };


    int			idx = 0;
    int			count = 0;

    unsigned	i = 0;
    double		OutData[N_STEP] = { 0.0, };
    double		OutTime[N_STEP] = { 0.0, };
    double		OutVcmd[N_STEP] = { 0.0, };

    Vcmd[0] = 5.0;
    Vcmd[1] = 0.0;



    double rotate_time = 7.;
    int temp_step   = 0;
    double step = 0.025;
    double Vin_data = 1.0;

    while(1){
        rotate_time = 7. - fabs(Vin_data - 2.5);
        temp_step   = (int)(rotate_time * SAMPLING_FREQ);
        count = 0;

        DAQmxWriteAnalogF64(taskAO,1,1.0, 5.0, DAQmx_Val_GroupByChannel, Vcmd, NULL, NULL);
        do{
            time = SAMPLING_TIME * count;

            DAQmxReadAnalogScalarF64(taskAI, 10.0, &Vin, NULL);

            /* Memory Write */
            OutTime[count] = time;
            OutVcmd[count] = Vcmd[1];
            OutData[count] = Vin;

            while (1)
            {
                time_curr = GetWindowTime();

                if (time_curr - (time_init + SAMPLING_TIME * (double)count * 1000.0) >= (SAMPLING_TIME * 1000.0))	break;
            }

        } while (count++ < temp_step - 1);
//        } while (count++ < N_STEP - 1);

        time_init = GetWindowTime();

        // halt
        count = 0;
        Vcmd[0] = 0.f;
        DAQmxWriteAnalogF64(taskAO,1,1.0, 5.0, DAQmx_Val_GroupByChannel, Vcmd, NULL, NULL);
        do{
            while (1){
                time_curr = GetWindowTime();
                if (time_curr - (time_init + SAMPLING_TIME * (double)count * 1000.0) >= (SAMPLING_TIME * 1000.0))	break;
            }
        } while (count++ < HALT_STEP - 1);

        time_init = GetWindowTime();


        // file save
        FILE*		pFile;

        sprintf(OutFileName, "../LAB/static_motor/static_motor_data_final/%2.3f", Vcmd[1]);

        pFile = fopen(strcat(OutFileName, "_data.out"), "w+t");

        for (idx = 0; idx < temp_step - 1; idx++)
        {
            fprintf(pFile, "%20.10f %20.10f %20.10f\n", OutTime[idx], OutVcmd[idx], OutData[idx]);
        }
        printf("save finished\n");
//        file_save(OutFileName, OutTime, OutVcmd, OutData, N_STEP);

        fclose(pFile);

        Vin_data += step;
        Vcmd[0] = 5.f;
        Vcmd[1] = Vin_data;
        if(Vin_data > 4.) break;
    }


    Vcmd[0] = 5.f;
    Vcmd[1] = 2.5;
    DAQmxWriteAnalogF64(taskAO,1,1.0, 5.0, DAQmx_Val_GroupByChannel, Vcmd, NULL, NULL);
    DAQmxStopTask(taskAI);
    DAQmxStopTask(taskAO);

    return 0;
}