//
// Created by User on 2024-11-06.
//

#ifndef DCSP_TUTORIAL_DAQ_HEADER_H
#define DCSP_TUTORIAL_DAQ_HEADER_H

#include "NIDAQmx.h"
#include <conio.h>


#define   FINAL_TIME          (double)(               2.0 )
#define   SAMPLING_FREQ       (double)(               200  )
#define   N_STEP              (int)( FINAL_TIME*SAMPLING_FREQ )
#define   HALT_STEP           (int)( 3.0*SAMPLING_FREQ )
#define   SAMPLING_TIME       (double)( 1.0/SAMPLING_FREQ )
#define   UNIT_PI             (double)( 3.14159265358979  )
#define   MAX_VCMD            (double)(5.025)
#define   MAX_HZ              (double)(2.1)
#define   MOVE_STEP           (double)(0.05)

// daq motor way
#define   NEGATIVE            (int)(0)
#define   POSITIVE            (int)(1)
#define   STOP                (int)(2)

// potential_meter
#define   CCW_direction       (int)(0)
#define   CW_direction        (int)(1)

#define   Hz2RAD            (double)( 2 * UNIT_PI )                 // [-]
#define   RAD2HZ            (double)( 1 / (2 * UNIT_PI) )           // [-]
#define   DEG2RAD           (double)( UNIT_PI / 180.0 )             // [-]
#define   RAD2DEG           (double)( 180.0 / UNIT_PI )             // [-]

enum LogicFlag{ NOK, YOK};

typedef double REAL;

typedef struct
{
    REAL prev;
    REAL curr;
}TIME;

TIME Time;

void init_DAQ(void);
void close_DAQ(void);
double Vc2Vs_conversion(double Vin_data);
double bias_calculate(void);
void save_static_angular_velocity(double Vin_data, double GYRO_BIAS);
void save_dynamic_angular_velocity(double mag, double Hz, double GYRO_BIAS);
void motor_rest(void);
void file_save(char OutFileName[], double Hz);
double GetWindowTime(void);

void move_motor(int direction, double step);
int print_data(int direction);

double vc2vs_potential(double vc);
double vc2vs_gyro(double vc);


double potential2degree(double potential_meter, int direction);

#endif //DCSP_TUTORIAL_DAQ_HEADER_H
