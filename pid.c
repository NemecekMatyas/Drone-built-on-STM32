/*#include "main.h"
#include "mpu6050.h"
#include "ibus.h"
#include "pid.h"
#include <stdio.h>
#include <stdint.h>
#include <math.h>
//#include "mpu6050.c"
extern MPU6050_t MPU6050;

// Constants for control
#define KP 1.5

// Function to constrain values
float constrain(float value, float min, float max) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

// Function for P-only control
float p_control(float setpoint, float measured) {
    float error = setpoint - measured;
    return KP * error;
}

void drone_control_update() {

	//while (MPU6050_Init(&hi2c1) == 1);

    MPU6050_Read_All(&hi2c1, &MPU6050);

    // Read user inputs from IBUS
    uint16_t mappedData = map(ibus_data[2], 1000, 2000, 50, 100);
    int16_t manual_roll = map(ibus_data[0], 1000, 2000, -15, 15);
    int16_t manual_pitch = map(ibus_data[1], 1000, 2000, -15, 15);
    int16_t manual_yaw = map(ibus_data[3], 1000, 2000, -15, 15);

    // Get pitch and roll values from Kalman filter
    float pitch = MPU6050.KalmanAngleX;
    float roll = MPU6050.KalmanAngleY;

    // Stabilization corrections
    float stabilization_pitch = p_control(0, pitch);
    float stabilization_roll = p_control(0, roll);

    // Combining manual control and stabilization
    int16_t final_roll = manual_roll + stabilization_roll;
    int16_t final_pitch = manual_pitch + stabilization_pitch;
    int16_t final_yaw = manual_yaw;

    // Set motor values with constrained limits
    PL_Data = constrain(mappedData + final_pitch - final_roll + final_yaw, 1000, 2000);
    PP_Data = constrain(mappedData + final_pitch + final_roll - final_yaw, 1000, 2000);
    ZL_Data = constrain(mappedData - final_pitch - final_roll - final_yaw, 1000, 2000);
    ZP_Data = constrain(mappedData - final_pitch + final_roll + final_yaw, 1000, 2000);

    // Apply motor values to PWM outputs
    TIM1->CCR1 = PL_Data;
    TIM1->CCR2 = ZP_Data;
    TIM1->CCR3 = ZL_Data;
    TIM1->CCR4 = PP_Data;

    // Debugging output
    printf("PL: %d, PP: %d, ZL: %d, ZP: %d\n", PL_Data, PP_Data, ZL_Data, ZP_Data);
}

*/
