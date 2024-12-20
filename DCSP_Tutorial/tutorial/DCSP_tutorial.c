//
// Created by User on 2024-09-11.
//

#include <stdio.h>
#include "NIDAQmx.h"


int main()
{
    TaskHandle	taskAI = 0;
    TaskHandle	taskAO = 0;

    float64		Vin = 0.0;
    double		Vcmd = 0.0;

    DAQmxCreateTask("", &taskAI);
    DAQmxCreateTask("", &taskAO);

    DAQmxCreateAIVoltageChan(taskAI, "Dev1/ai0", "", DAQmx_Val_RSE, -10.0, 10.0, DAQmx_Val_Volts, "");
    DAQmxCreateAOVoltageChan(taskAO, "Dev1/ao0", "", 0.0, 5.0, DAQmx_Val_Volts, "");

    DAQmxStartTask(taskAI);
    DAQmxStartTask(taskAO);

    Vcmd = 5.0;
    DAQmxWriteAnalogScalarF64(taskAO, 1.0, 5.0, Vcmd, NULL);
    DAQmxReadAnalogScalarF64(taskAI, 10.0, &Vin, NULL);

    DAQmxStopTask(taskAI);
    DAQmxStopTask(taskAO);

    printf("%f\n", Vin);

    return 0;
}
