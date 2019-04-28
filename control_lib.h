#ifndef __CONTROL_LIB_H__
#define __CONTROL_LIB_H__
#include "arduino.h"
#include <MeOrion.h>      //Orion board
#include <TimerOne.h>


#define RAD2PULSE 509
#define ROTATEPULSE 3200 //1âÒì]ÇÃÉpÉãÉXêî
#define MAX_SPEED 15	//(=5rps)
#define MIN_SPEED 3.14e-3

#define LEFT 1
#define RIGHT 2

#define SPEED 0
#define POSITION 1

#define SERVO_PIN 12

void init_step_motor(int mode);
void drive_step_motor_s();
void drive_step_motor_p();
void update_step_speed(int ch,float speed);
float get_step_position(int ch);
float update_step_position(int ch, float position, float speed);
void update_servo_angle(int angle);

int init_ussensor();
int get_distance();

void comm_ussensor();

static char send_data[4]={0x22,0x00,0x00,0x22};


void output_data(int ch, float voltage);

#define MeControlDA	MeDCMotor
#endif