//
// Created by User on 2024-12-04.
//


#include <stdio.h>
#include "DAQ_include.h"
#include <conio.h>

#define FREQ_START        0.1
#define FREQ_END          2.0
#define FREQ_STEP         0.1
#define SIN_AMPLITUDE     0.5
#define HZ2RAD            (2 * M_PI)




int main(void) {
    double Vcmd[2] = {0.0};
    char OutFileName[100] = {""};

    Vcmd[0] = 5.0;
    Vcmd[1] = 2.5;

    double GYRO_BIAS = 0.0;
    double mag = 0.5;
    int position = -60;


    // init_DAQ
    init_DAQ();

    // bias_calculate return val: GYRO_BIAS
//    GYRO_BIAS = bias_calculate();
//    printf("Press any key to start the program.... \n");
//    getchar();

    printf("start_save\n");

    while (1) {
//        // Check for key press
//        if (kbhit()) {
//            char key = getch();
//            if (key == 'a') {
//                move_motor(NEGATIVE, MOVE_STEP);
//            } else if (key == 'd') {
//                move_motor(POSITIVE, MOVE_STEP);
//            }else if (key == 's') {
//                move_motor(STOP, MOVE_STEP);
//            }else if (key == 'q') {
//                break; // Exit the loop if 'q' is pressed
//            }
//        }

        int direction = POSITIVE;
//        int direction = NEGATIVE;
//        move_motor(NEGATIVE, MOVE_STEP);
        move_motor(direction, MOVE_STEP);
        if(print_data(direction)) break;

        // data saving
//        save_dynamic_angular_velocity(mag, Hz, GYRO_BIAS);

        // rest
//        motor_rest();


        // file save
//        sprintf(OutFileName, "../LAB/static_motor/potential_meter_value_data/%2d.out", position);
//        file_save(OutFileName, 0);
    }
    move_motor(STOP, MOVE_STEP);

    close_DAQ();

    return 0;
}