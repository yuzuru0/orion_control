#include "arduino.h"
#include "control_lib.h"
#include <MeOrion.h>      //Orion board

int dirPin1 = mePort[PORT_1].s1;
int stpPin1 = mePort[PORT_1].s2;

int dirPin2 = mePort[PORT_2].s1;
int stpPin2 = mePort[PORT_2].s2;

int timer_counter;
int step_count[2];
int step_delay[2]={0x7FFF,0x7FFF};
int step_dir[2] = {0,0};
long step_position[2]={0,0};
long step_position_ref[2] ={0,0};
int servo_position_ref;
int servo_counter=0;


int state[2]={LOW,LOW};

void update_servo_angle(int angle)
{
	servo_position_ref = angle *70 /180 +20;
}

float update_step_position(int ch, float position, float speed)
{
/*
	float pos_now;
	
	pos_now = get_step_position(ch);
	
	if(position <pos_now)
		update_step_speed(ch, -speed);
	if(position >pos_now)
		update_step_speed(ch, speed);
	if(position == pos_now)
		update_step_speed(ch,0);
*/
	float pos_now;

	if(speed<0)
		speed = -speed;
	pos_now = get_step_position(ch);
	step_position_ref[ch-1] = position*ROTATEPULSE/3.14/2;
	
	if(step_position_ref[ch-1] <pos_now)
		update_step_speed(ch,-speed);

	else
		update_step_speed(ch,speed);
	
	return step_position_ref[ch-1] -pos_now;

}

float get_step_position(int ch)
{
	return (step_position[ch-1]*2*3.14/ROTATEPULSE);
//	return step_position[ch-1];
}

void update_step_speed(int ch,float speed)
{
  if(ch ==1)
  {
  	  if(speed<0)
  	  {
  	  	  digitalWrite(dirPin1,1);
  	  	  step_dir[0] =-1;
  	  	  speed= -speed;
  	  }
  	  else
  	  {
  	  	  digitalWrite(dirPin1,0);
  	  	  step_dir[0] =1;
  	  }
  	  
  	  if(speed<MIN_SPEED)
  	  	  step_delay[ch-1] =  0x7FFF;
  	  
  	  else if(speed>MAX_SPEED)
    	step_delay[ch-1] =  (int)(1.0e6/20/RAD2PULSE/MAX_SPEED);

	  else
    	step_delay[ch-1] =  (int)(1.0e6/20/RAD2PULSE/speed);
  }

  if(ch ==2)
  {
  	  if(speed<0)
  	  {
  	  	  digitalWrite(dirPin2,1);
  	  	  step_dir[1] = -1;
  	  	  speed= -speed;
  	  }
  	  else
  	  {
  	  	  digitalWrite(dirPin2,0);
  	  	  step_dir[1] =1;
  	  }
  	  
  	  if(speed<MIN_SPEED)
  	  	  step_delay[ch-1] =  0x7FFF;
  	  
  	  else if(speed>MAX_SPEED)
    	step_delay[ch-1] =  (int)(1.0e6/40/RAD2PULSE/MAX_SPEED);

	  else
    	step_delay[ch-1] =  (int)(1.0e6/40/RAD2PULSE/speed);
  }

}

void drive_step_motor_s()
{

  if(servo_counter >1000)
  {
  	  servo_counter =0;
  	  digitalWrite(SERVO_PIN,1);
  }
  
  if(servo_counter ==servo_position_ref)
  	  digitalWrite(SERVO_PIN, 0);

  servo_counter++;


	if(step_delay[0] != 0x7FFF)
	{
	  if(step_count[0] >= step_delay[0])
	  {
	    state[0] = !state[0];
	    digitalWrite(stpPin1, state[0]);
	    step_count[0] = 0;
	    step_position[0] += step_dir[0] * state[0];
	  }
	  step_count[0]++;
	}

	if(step_delay[1] != 0x7FFF)
	{
	  if(step_count[1] >= step_delay[1])
	  {
	    state[1] = !state[1];
	    digitalWrite(stpPin2, state[1]);
	    step_count[1] = 0;
	    step_position[1] += step_dir[1] * state[1];
	  }

	  step_count[1]++;
	}
  
  timer_counter++;
}


void drive_step_motor_p()
{
  if(servo_counter >1000)
  {
  	  servo_counter =0;
  	  digitalWrite(SERVO_PIN,1);
  }
  
  if(servo_counter ==servo_position_ref)
  	  digitalWrite(SERVO_PIN, 0);

  servo_counter++;

	if(step_delay[0] != 0x7FFF && step_position[0] != step_position_ref[0])
	{
	  if(step_count[0] >= step_delay[0])
	  {
	    state[0] = !state[0];
	    digitalWrite(stpPin1, state[0]);
	    step_count[0] = 0;
	    step_position[0] += step_dir[0] * state[0];
	  }
	  step_count[0]++;
	}

	if(step_delay[1] != 0x7FFF && step_position[1] != step_position_ref[1])
	{
	  if(step_count[1] >= step_delay[1])
	  {
	    state[1] = !state[1];
	    digitalWrite(stpPin2, state[1]);
	    step_count[1] = 0;
	    step_position[1] += step_dir[1] * state[1];
	  }

	  step_count[1]++;
	}
//  timer_counter++;
}

void init_step_motor(int mode)
{
  //  setting for stepping motor
  pinMode(dirPin1, OUTPUT);
  pinMode(stpPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stpPin2, OUTPUT);
  
  pinMode(SERVO_PIN,OUTPUT);

  Timer1.initialize(20);
  if(mode ==SPEED)
  	Timer1.attachInterrupt(drive_step_motor_s);
  if(mode ==POSITION)
  	Timer1.attachInterrupt(drive_step_motor_p);

}


int init_ussensor()
{
  int i;
   Serial.begin(9600);
   delay(10);

  for(i=0;i<4;i++)
    Serial.write(send_data[i]);
}

int get_distance()
{
  int read_data[255];
  int temp;
  int i;
  int count_data=0;
  int return_value=0;
  
  if(Serial.available())
  {
    while((temp = Serial.read()) != (int)-1)
    {
      read_data[count_data] = temp;
      count_data++;
//      delay(1);
    }

//    return_value = read_data[2];
    return_value = read_data[2] |((0x01&read_data[1])<<8);
//    return_value = (0x01&read_data[1])<<5;
  }
  else
    return_value=-1;
  
  if(read_data[0] != 0x22)
  	  return_value =-1;

  for(i=0;i<4;i++)
    Serial.write(send_data[i]);

    return return_value;
}

/*
void output_data(int ch, float voltage)
{
  if(voltage<0)
    voltage=0;
  if(voltage>12)
    voltage=12;
  if(ch ==1)
    da1.run(voltage*255/12);

  if(ch ==2)
    da2.run(voltage*255/12);

}
*/
