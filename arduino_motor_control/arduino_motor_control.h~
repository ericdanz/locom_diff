#ifndef UDOO_MOTOR_CONTROL_H_
#define UDOO_MOTOR_CONTROL_H_

#include "PID_v1.h"

#define RX_PIN 158 // Yellow, goes to TX on Udoo
#define TX_PIN 159 // Orange, goes to RX on Udoo
#define WHEEL_DIAMETER 100 // 100mm
#define SERIAL_BUFFER_LEN 32

#define DIST_MODE 0
#define VEL_MODE 1


//CHRIS FIX THIS
/*const String dist_locomotion_mode = "dist_mode";
const String vel_locomotion_mode = "vel_mode";

String current_locomotion_mode = vel_locomotion_mode;
*/
int current_mode = VEL_MODE;

int PID_output_lower = 0;
int PID_output_upper = 255;


double PID1_input = 0;
double PID1_output = 0;
double PID1_target = 0;

double PID2_input = 0;
double PID2_output = 0;
double PID2_target = 0;

double Kp = 0.15;
double Ki = 0.05;
double Kd = 0.001;

PID enc1PID(&PID1_input, &PID1_output, &PID1_target, Kp, Ki, Kd, DIRECT);
PID enc2PID(&PID2_input, &PID2_output, &PID2_target, Kp, Ki, Kd, DIRECT);

// Contains the serial output
char serial1_buffer[SERIAL_BUFFER_LEN];
int serial1_buffer_position = 0;

#endif
