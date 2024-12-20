////
//// Created by User on 2024-11-06.
////


#define _CRT_SECURE_NO_WARNINGS


#include <stdio.h>
#include "DAQ_include.h"

#define FREQ_START        0.1
#define FREQ_END          2.0
#define FREQ_STEP         0.1
#define SIN_AMPLITUDE     0.5
#define HZ2RAD            (2 * M_PI)



#include <stdio.h>

#include "DAQ_include.h."
#include "NIDAQmx.h"

int main(void)
{
    double       Vcmd[2] = {0.0};
    char        OutFileName[100] = { "" };

    Vcmd[0] = 5.0;
    Vcmd[1] = 0.0;

    double step = 0.1;
    double Vin_data = 0.00;
    double GYRO_BIAS = 0.0;
    double mag = 0.5;
    double Hz = 0.1;

    // init_DAQ
    init_DAQ();

    // bias_calculate return val: GYRO_BIAS
    GYRO_BIAS = bias_calculate();

    printf("GYRO_BIAS: %f\n", GYRO_BIAS);

    printf("Press any key to start the program.... \n");

    getchar();

    printf("start_save\n");

    while(1){

        Vcmd[0] = 5.f;
        if(Hz >= MAX_HZ) break;

        // data saving
        save_dynamic_angular_velocity(mag, Hz, GYRO_BIAS);

        // rest
        motor_rest();

        // file save

        sprintf(OutFileName, "../LAB/static_motor/static_sin_wave_data/%1.2f.out", Hz);
        file_save(OutFileName, Hz);

        Hz += step;
    }

    close_DAQ();

    return 0;
}
