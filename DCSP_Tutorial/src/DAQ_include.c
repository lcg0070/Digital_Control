//
// Created by User on 2024-11-06.
//

#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>
#include <Windows.h>
#include "NIDAQmx.h"
#include <math.h>
#include "DAQ_include.h"
#include <string.h>
#include <time.h>

FILE*		pFile;

TaskHandle   taskAI = 0;
TaskHandle   taskAO = 0;
FILE *pFile;

double      OutData[4400] = { 0.0, };
double      OutTime[4400] = { 0.0, };
double      OutVcmd[4400] = { 0.0, };

void init_DAQ(void) {

    DAQmxCreateTask("", &taskAI);
    DAQmxCreateTask("", &taskAO);

    DAQmxCreateAIVoltageChan(taskAI, "Dev1/ai3", "", DAQmx_Val_Diff, -10.0, 10.0, DAQmx_Val_Volts, "");
    DAQmxCreateAOVoltageChan(taskAO, "Dev1/ao0:1", "", 0.0, 5.0, DAQmx_Val_Volts, "");

    DAQmxStartTask(taskAI);
    DAQmxStartTask(taskAO);
}

void close_DAQ(void) {
    DAQmxStopTask(taskAI);
    DAQmxStopTask(taskAO);
}


double vc2vs_potential(double vc){
    if(vc <= 2.2){
//        213.390387562850e-003   -1.08727860794984e+000    2.46570577643673e+000   -136.360362763351e-003
        return 213.390387562850e-003 * vc * vc * vc + -1.08727860794984e+000 * vc * vc +
               2.46570577643673e+000 * vc + -136.360362763351e-003;
    }
    if(vc >= 2.8){
//       191.834372640679e-003   -1.90506521587090e+000    6.93016235911180e+000   -5.93095303034089e+000
        return 191.834372640679e-003 * vc * vc * vc + -1.90506521587090e+000 * vc * vc +
               6.93016235911180e+000 * vc + -5.93095303034089e+000;
    }
    return 2.5;
}




double Vc2Vs_conversion(double vc) {
    if(vc <= 2.45){
        double temp = vc + 0.05;
//        149.784376194705e-003   -860.787494165033e-003    2.14461500954046e+000    22.9890464318206e-003      // fitted first
//        151.542000047116e-003   -870.171877504343e-003    2.15779603802559e+000    18.4842378214636e-003      // fitted last
        return 151.542000047116e-003 * temp * temp * temp + -870.171877504343e-003 * temp * temp +
               2.15779603802559e+000 * temp + 18.4842378214636e-003;
    }
    if(vc >= 2.55){
        double temp = vc + 0.25;
//       235.609241826375e-003   -2.39579584199515e+000    8.66601036822973e+000   -7.85130840197436e+000      // fitted first
//        216.848370210713e-003   -2.15091763097387e+000    7.62201809556707e+000   -6.32481201545038e+000     // fitted last
        return 216.848370210713e-003 * temp * temp * temp + -2.15091763097387e+000 * temp * temp +
               7.62201809556707e+000 * temp + -6.32481201545038e+000  - 0.15;
    }
    return 2.5;
}

double bias_calculate(void) {
    double z = 0.0;
    double time = 0.0;
    double Vcmd[2] = {0.0};
    double GYRO_BIAS = 0.0;
    double time_init = 0.0;
    double time_curr = 0.0;
    float64      Vin = 0.0;
    int count = 0;
    int bias_count = 0;

    Vcmd[0] = 5.0;
    Vcmd[1] = 2.5;

    DAQmxWriteAnalogF64(taskAO,1,1.0, 5.0, DAQmx_Val_GroupByChannel, Vcmd, NULL, NULL);

    time_init = GetWindowTime();

    do{
        time = SAMPLING_TIME * count;

        DAQmxReadAnalogScalarF64(taskAI, 5.0, &Vin, NULL);

        bias_count++;

        z = Vin;
        GYRO_BIAS = (1.0 - 1.0/bias_count) * GYRO_BIAS + z / bias_count;

        while (YOK)
        {
            time_curr = GetWindowTime();

            if (time_curr - (time_init + SAMPLING_TIME * (double)count * 1000.0) >= (SAMPLING_TIME * 1000.0))   break;
        }
    } while (count++ < N_STEP - 1);

    return GYRO_BIAS;
}

void save_static_angular_velocity(double Vin_data, double GYRO_BIAS) {
    double time = 0.0;
    double time_init = 0.0;
    double time_curr = 0.0;
    float64      Vin = 0.0;
    int count = 0;
    double Vcmd[2] = {5.0, 0.0};

    // Vc2Vs_conversion return val: Vcmd[1]
    Vcmd[1] = Vc2Vs_conversion(Vin_data);

    count = 0;
    time_init = GetWindowTime();

    do{
        DAQmxWriteAnalogF64(taskAO,1,1.0, FINAL_TIME, DAQmx_Val_GroupByChannel, Vcmd, NULL, NULL);

        time = SAMPLING_TIME * count;

        DAQmxReadAnalogScalarF64(taskAI, 10.0, &Vin, NULL);

        /* Memory Write */
        OutTime[count] = time;
        OutVcmd[count] = Vcmd[1];
        OutData[count] = Vin - GYRO_BIAS;

        while (YOK)
        {
            time_curr = GetWindowTime();

            if (time_curr - (time_init + SAMPLING_TIME * (double)count * 1000.0) >= (SAMPLING_TIME * 1000.0))   break;
        }
    } while (count++ < N_STEP - 1);
}

void save_dynamic_angular_velocity(double mag, double Hz, double GYRO_BIAS){
    double time = 0.0;
    double time_init = 0.0;
    double time_curr = 0.0;
    double Vin_data = 0.0;
    float64      Vin = 0.0;
    int count = 0;
    double Vcmd[2] = {5.0, 0.0};

    count = 0;
    time_init = GetWindowTime();

    do{
        time = SAMPLING_TIME * count;

        Vin_data = 2.5 + mag * sin(Hz * 2 * UNIT_PI * time);

        Vcmd[1] = Vc2Vs_conversion(Vin_data);

        DAQmxWriteAnalogF64(taskAO,1,1.0, FINAL_TIME, DAQmx_Val_GroupByChannel, Vcmd, NULL, NULL);

        DAQmxReadAnalogScalarF64(taskAI, 10.0, &Vin, NULL);

        /* Memory Write */
        OutTime[count] = time;
        OutVcmd[count] = Vin_data;
        OutData[count] = Vin - GYRO_BIAS;

        while (YOK)
        {
            time_curr = GetWindowTime();

            if (time_curr - (time_init + SAMPLING_TIME * (double)count * 1000.0) >= (SAMPLING_TIME * 1000.0))   break;
        }
    } while (count++ < 400 + SAMPLING_FREQ * 2.0 / Hz - 1);
}

void motor_rest(void) {
    double time_init = 0.0;
    double time_curr = 0.0;
    int count = 0;
    double Vcmd[2] = {0.0};

    time_init = GetWindowTime();
    count = 0;
    Vcmd[0] = 0.f;
    DAQmxWriteAnalogF64(taskAO,1,1.0, HALT_STEP/SAMPLING_FREQ, DAQmx_Val_GroupByChannel, Vcmd, NULL, NULL);
    do{
        while (YOK){
            time_curr = GetWindowTime();
            if (time_curr - (time_init + SAMPLING_TIME * (double)count * 1000.0) >= (SAMPLING_TIME * 1000.0))   break;
        }
    } while (count++ < HALT_STEP - 1);
}

// void file_save(char OutFileName[], double OutTime[], double OutVcmd[], double OutData[]) {
void file_save(char OutFileName[], double Hz) {
    pFile = fopen(OutFileName, "w+t");

    if(Hz) for(int idx = 0; idx < 400 + (SAMPLING_FREQ * 2.0 / Hz); idx++) fprintf(pFile, "%20.10f %20.10f %20.10f\n", OutTime[idx], OutVcmd[idx], OutData[idx]);
    else for(int idx = 0; idx < N_STEP; idx++) fprintf(pFile, "%20.10f %20.10f %20.10f\n", OutTime[idx], OutVcmd[idx], OutData[idx]);

    fclose(pFile);
}

double GetWindowTime(void){
    LARGE_INTEGER   liEndCounter, liFrequency;

    QueryPerformanceCounter(&liEndCounter);
    QueryPerformanceFrequency(&liFrequency);

    return(liEndCounter.QuadPart / (double)(liFrequency.QuadPart) * 1000.0);
}



void move_motor(int direction, double step) {
    double Vcmd[2] = {5.0, 2.5};

    if (direction == POSITIVE){
        Vcmd[1] = vc2vs_gyro(2.35);
    }else if(direction == NEGATIVE){
        Vcmd[1] = vc2vs_gyro(2.60);
    }else if(direction == STOP){
        Vcmd[1] = vc2vs_gyro(2.5);
    }
    DAQmxWriteAnalogF64(taskAO, 1, 1.0, 5.0, DAQmx_Val_GroupByChannel, Vcmd, NULL, NULL);
}
int print_data(int direction){
    float64      Vin = 0.0;
    DAQmxReadAnalogScalarF64(taskAI, 5.0, &Vin, NULL);
    if (direction == NEGATIVE){
        if(Vin > 3.404) return 1;
    }else{
        if (Vin < 1.640) return 1;
    }
    printf("%f\n", Vin);
    return 0;
}



double vc2vs_gyro(double vc) {
    if(vc <= 2.45){
//        double temp = vc + 0.08;
        double temp = vc + 0.05;
        if (vc<0) temp = 0.00;
//        91.2509671571033e-003   -460.015457421307e-003    1.56479526085459e+000    45.8124348254995e-003
        // Vc 0~5 -> Vs 0~5
//        124.663079019440e-003   -587.931529891506e-003    2.02001822041579e+000   -513.984898431631e-003

        return 151.542000047116e-003 * temp * temp * temp + -870.171877504343e-003 * temp * temp +
               2.15779603802559e+000 * temp + 18.4842378214636e-003;

//        return 91.2509671571033e-003 * temp * temp * temp + -460.015457421307e-003 * temp * temp +
//                1.56479526085459e+000 * temp + 45.8124348254995e-003 - 0.23;
//        return 91.2509671571033e-003 * temp * temp * temp + -460.015457421307e-003 * temp * temp +
//               1.56479526085459e+000 * temp + 45.8124348254995e-003 ;
    }
    if(vc >= 2.55){
        double temp = vc + 0.10;
//        double temp = vc;
        if (vc > 5) temp = 5;
        // Vc 0~5 -> Vs 1~4
//        111.439748079985e-003   -1.17744140596400e+000    4.93833878495428e+000   -4.24018485324034e+000
        // Vc 0~5 -> Vs 0~5
//        73.7583752414959e-003   -727.823467313981e-003    3.44137665477092e+000   -2.92327437767961e+000

        return 216.848370210713e-003 * temp * temp * temp + -2.15091763097387e+000 * temp * temp +
               7.62201809556707e+000 * temp + -6.32481201545038e+000 ;

//        return 111.439748079985e-003 * temp * temp * temp +  -1.17744140596400e+000 * temp * temp +
//                4.93833878495428e+000  * temp + -4.24018485324034e+000  + 0.15;
//        return 111.439748079985e-003 * temp * temp * temp +  -1.17744140596400e+000 * temp * temp +
//               4.93833878495428e+000  * temp + -4.24018485324034e+000 ;
    }
    return 2.5;
}


double potential2degree(double potential_meter, int direction){
    if(direction == CW_direction){
        return (potential_meter - 2.47186910750064e+000) / 14.7389393466144e-003;
    }else if (direction == CCW_direction){
        return (potential_meter - 2.54496753089874e+000) / 14.7609223735546e-003;
    }return 0;
}





