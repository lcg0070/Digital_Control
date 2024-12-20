////
//// Created by User on 2024-11-06.
////


#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <Windows.h>
#include <time.h>
#include "NIDAQmx.h"

#include "DAQ_include.h"


//#define   FINAL_TIME		(double)(				5.0 )
//#define   SAMPLING_FREQ		(double)(               200  )  // 200Hz
//#define   N_STEP		    (int)   ( FINAL_TIME*SAMPLING_FREQ )
//#define   HALT_STEP		    (int)   ( 5.0*SAMPLING_FREQ )
//#define   SAMPLING_TIME     (double)( 1.0/SAMPLING_FREQ )
//#define   UNIT_PI			(double)( 3.14159265358979  )
#define   CW                (int)(0)
#define   CCW               (int)(1)

//double GetWindowTime(void){
//    LARGE_INTEGER	liEndCounter, liFrequency;
//
//    QueryPerformanceCounter(&liEndCounter);
//    QueryPerformanceFrequency(&liFrequency);
//
//    return(liEndCounter.QuadPart / (double)(liFrequency.QuadPart) * 1000.0);
//};

double voltage2degree_potential_meter(double voltage, int direction);


int main(void)
{
    TaskHandle	taskAI = 0;
    TaskHandle	taskAO = 0;

    float64		Vin[2] = {0.0};
    double		Vcmd[2] = {0.0};

    DAQmxCreateTask("", &taskAI);
    DAQmxCreateTask("", &taskAO);

    DAQmxCreateAIVoltageChan(taskAI, "Dev1/ai2:3", "", DAQmx_Val_Diff, -10.0, 10.0, DAQmx_Val_Volts, "");
    DAQmxCreateAOVoltageChan(taskAO, "Dev1/ao0:1", "", 0.0, 5.0, DAQmx_Val_Volts, "");

    DAQmxStartTask(taskAI);
    DAQmxStartTask(taskAO);



    char		OutFileName[100] = { "" };
    int			idx = 0;

    unsigned	i = 0;

    double		OutTime[N_STEP] = { 0.0, };
    double		OutVcmd[N_STEP] = { 0.0, };
    double		OutGyro[N_STEP] = { 0.0, };
    double		OutPoten[N_STEP] = { 0.0, };

//    GYRO OFFSET
    double z = 0.0;
    double time = 0.0;
    double gyro_bias = 0.0;
    double time_init = 0.0;
    double time_curr = 0.0;

    int count = 0;
    int bias_count = 0;

    Vcmd[0] = 5.0;
    Vcmd[1] = 2.5;

    DAQmxWriteAnalogF64(taskAO,1,1.0, 5.0, DAQmx_Val_GroupByChannel, Vcmd, NULL, NULL);
    time_init = GetWindowTime();
    do{
        time = SAMPLING_TIME * count;

        DAQmxReadAnalogF64(taskAI, DAQmx_Val_Auto, 10.0, DAQmx_Val_GroupByChannel, Vin, 2, NULL, NULL);
        bias_count++;
        z = Vin[0];
        gyro_bias = (1.0 - 1.0 / bias_count) * gyro_bias + z / bias_count;

        while (YOK)
        {
            time_curr = GetWindowTime();

            if (time_curr - (time_init + SAMPLING_TIME * (double)count * 1000.0) >= (SAMPLING_TIME * 1000.0))   break;
        }
    } while (count++ < HALT_STEP - 1);

    printf("gyro_bias is : %f\n", gyro_bias);
    printf("Press any key to start the program.... \n");
    getchar();

    double degree_data = 0.0;
    double velocity_data = 0.0;

    double Kp = 63.3218386532083e-003;
    double Kd = 2.36026341463415e-003;
    double k1 = Kp / Kd;
    double k2 = Kd;
    count = 0;

    int direction = CW;
    time_init = GetWindowTime();

    double desire_degree = 0.;

    while(1){

        time = SAMPLING_TIME * count;

        Vcmd[0] = 5.f;

        DAQmxReadAnalogF64(taskAI, DAQmx_Val_Auto, 10.0, DAQmx_Val_GroupByChannel, Vin, 2, NULL, NULL);

        // potential meter to degree
        degree_data = voltage2degree_potential_meter(Vin[1], direction);    // [deg]
        // gyro to angular velocity
        velocity_data = (Vin[0] - gyro_bias) * 1000.0 / 0.67;                 // [deg/s]

        if (desire_degree - degree_data > 0) direction = CW;
        else direction = CCW;

        Vcmd[1] = ((desire_degree - degree_data) * k1 - velocity_data) * k2 + 2.5;    // Vc [v]
        Vcmd[1] = vc2vs_gyro(Vcmd[1]);                                            // Vs [v]

        OutTime[count]  = time;
        OutVcmd[count]  = ((desire_degree - degree_data) * k1 - velocity_data) * k2 + 2.5;
        OutGyro[count]  = Vin[0];
        OutPoten[count] = Vin[1];

        DAQmxWriteAnalogF64(taskAO,1,1.0, FINAL_TIME, DAQmx_Val_GroupByChannel, Vcmd, NULL, NULL);
        while (YOK)
        {
            time_curr = GetWindowTime();
            if (time_curr - (time_init + SAMPLING_TIME * (double)count * 1000.0) >= (SAMPLING_TIME * 1000.0))	break;
        }
        if (count++ >= N_STEP - 1) break;
//        if (fabs(degree_data - desire_degree) < 0.2) break;
    }
    Vcmd[0] = 0;
    Vcmd[1] = 2.5;
    DAQmxWriteAnalogF64(taskAO,1,1.0, FINAL_TIME, DAQmx_Val_GroupByChannel, Vcmd, NULL, NULL);
    close_DAQ();

    FILE*		pFile;
//    sprintf(OutFileName, "../LAB/static_motor/controller_valid_data/%2.1f", desire_degree);
    sprintf(OutFileName, "../LAB/static_motor/controller_valid_test/%2.1f", desire_degree);

    pFile = fopen(strcat(OutFileName, "_data.out"), "w+t");

    for (idx = 0; idx < N_STEP; idx++)
    {
        fprintf(pFile, "%20.10f %20.10f %20.10f %20.10f\n", OutTime[idx], OutVcmd[idx], OutGyro[idx], OutPoten[idx]);
    }
    printf("save finished\n");

    return 0;
}



double voltage2degree_potential_meter(double voltage, int direction)
{
    double slope_CW = 14.7389393466144e-003;
    double offset_CW = 2.47186910750064e+000;

    double slope_CCW = 14.7609223735546e-003;
    double offset_CCW = 2.54496753089874e+000;

    if (direction == CW)
    {
        return (voltage - offset_CW) / slope_CW;
    }
    else if (direction == CCW)
    {
        return (voltage - offset_CCW) / slope_CCW;
    }else{
        return 0;
    }
}
