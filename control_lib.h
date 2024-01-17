#ifndef __CONTROL_LIB_H__
#define __CONTROL_LIB_H__
#include "arduino.h"
#include <MeOrion.h>      //Orion board
#include <TimerOne.h>


#define RAD2PULSE 509
#define ROTATEPULSE 3200 //1回転のパルス数
#define MAX_SPEED 15	//(=5rps)
#define MIN_SPEED 3.14e-3

#define LEFT 1
#define RIGHT 2

#define SPEED 0
#define POSITION 1

#define SERVO_PIN 13
#define SERVO_EN 0
#define SERVO_PORT 1

#define MAX_SERVO_CON	8	//マイコンボードの最大コネクタ数
#define	MAX_SERVO_SLOT	2	//各ポートごとのサーボスロット数

#ifdef	SOFT_US_SENSOR3
#include <SoftwareSerial.h>
SoftwareSerial softuart(13,12);
#endif

#ifdef	SOFT_US_SENSOR4
#include <SoftwareSerial.h>
SoftwareSerial softuart(2,8);
#endif

#ifdef	SOFT_US_SENSOR6
#include <SoftwareSerial.h>
SoftwareSerial softuart(A3,A2);
#endif

void (*call_control_process)(void);


void init_step_motor(int mode);
void drive_step_motor_s();
void drive_step_motor_p();
void update_step_speed(int ch,float speed);
float get_step_position(int ch);
float update_step_position(int ch, float position, float speed);
void update_servo_angle(int connector, int slot, int angle);
void nop_process(void);
int set_interrupt_function(void (*function)(void));
	
int init_ussensor();
int get_distance();
int get_temperature(void);
	
int init_ussensor_s();
int get_distance_s();
int get_temperature_s(void);

void comm_ussensor();

static char send_data[4]={0x22,0x00,0x00,0x22};
static char temp_send_data[4]={0x11,0x00,0x00,0x11};


void output_data(int ch, float voltage);

#define MeControlDA	MeDCMotor
#endif