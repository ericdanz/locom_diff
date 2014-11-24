#ifndef ARDUINO_MOTOR_CONTROL_H_
#define ARDUINO_MOTOR_CONTROL_H_

#include "PID_v1.h"
#include "commands.h"

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

int PID_output_lower = 1;
int PID_output_upper = 255;

double enc1_PID_input = 0;
double enc1_PID_output = 0;
double enc1_PID_target = 0;

double enc2_PID_input = 0;
double enc2_PID_output = 0;
double enc2_PID_target = 0;

double Kp = 0.17;
double Ki = 0.07;
double Kd = 0.001;

PID enc1PID(&enc1_PID_input, &enc1_PID_output, &enc1_PID_target, Kp, Ki, Kd, DIRECT);
PID enc2PID(&enc2_PID_input, &enc2_PID_output, &enc2_PID_target, Kp, Ki, Kd, DIRECT);

// Contains the serial output
char serial1_buffer[SERIAL_BUFFER_LEN];
int serial1_buffer_position = 0;

#endif
