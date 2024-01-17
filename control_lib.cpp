#include "arduino.h"
#include "control_lib.h"
#include <MeOrion.h>      //Orion board

int dirPin1 = mePort[PORT_1].s1;
int stpPin1 = mePort[PORT_1].s2;

int dirPin2 = mePort[PORT_2].s1;
int stpPin2 = mePort[PORT_2].s2;

int timer_counter;		//�v���p50�ʃC���N��
int step_count[2];
int step_delay[2]={0x7FFF,0x7FFF};	//�X�e�b�s���O���[�^�̃p���X�Ԋu
int step_dir[2] = {0,0};			//��]���� ����]1 �t��]-1
long step_position[2]={0,0};		//���݈ʒu(�p���X)
long step_position_ref[2] ={0,0};	//�ʒu�w��(�p���X)
int servo_position_ref=8;			//�T�[�{�ʒu
int servo_counter=0;				//�T�[�{����p�J�E���^

int state[2]={LOW,LOW};				//�X�e�b�s���O���[�^����s��

void update_servo_angle(int angle)
{
	servo_position_ref = angle *32 /180 +10;
	Serial.print("");		// ��V���A�������Ȃ��ƂȂ��������Ȃ��̂��C��(��)
}

//	speed�̑��x��position�܂�ch�̃X�e�b�s���O���[�^���ʒu�w�ߒl�Z�b�g
float update_step_position(int ch, float position, float speed)
{
	float pos_now;

	if(speed<0)
		speed = -speed;

	pos_now = get_step_position(ch);
	step_position_ref[ch-1] = position*ROTATEPULSE/3.14/2;
	
	if(step_position_ref[ch-1] <pos_now*ROTATEPULSE/3.14/2)
		update_step_speed(ch,-speed);

	else
		update_step_speed(ch,speed);
	
	//	�߂�l�͖ڕW�l�܂ł̌덷
	return step_position_ref[ch-1] -pos_now;

}

float get_step_position(int ch)
{
	return (step_position[ch-1]*2*3.14/ROTATEPULSE);
}

//	ch�̃X�e�b�s���O���[�^�̑��x�w�ߒl��speed�ɃZ�b�g
void update_step_speed(int ch,float speed)
{
  if(ch ==1)
  {
  	  if(speed<0)
  	  {
  	  	  digitalWrite(dirPin1,1);	// 1�̎�����]
  	  	  step_dir[0] =-1;
  	  	  speed= -speed;
  	  }
  	  else
  	  {
  	  	  digitalWrite(dirPin1,0);	// 0�̎�����]
  	  	  step_dir[0] =1;
  	  }
  	  
  	  if(speed<MIN_SPEED)
  	  	  step_delay[ch-1] =  0x7FFF;
  	  
  	  else if(speed>MAX_SPEED)
    	step_delay[ch-1] =  (int)(1.0e6/100/RAD2PULSE/MAX_SPEED);

	  else
    	step_delay[ch-1] =  (int)(1.0e6/100/RAD2PULSE/speed);
  }

  if(ch ==2)
  {
  	  if(speed<0)
  	  {
  	  	  digitalWrite(dirPin2,1);	// 1�̎�����]
  	  	  step_dir[1] = -1;
  	  	  speed= -speed;
  	  }
  	  else
  	  {
  	  	  digitalWrite(dirPin2,0);	// 0�̎�����]
  	  	  step_dir[1] =1;
  	  }
  	  
  	  if(speed<MIN_SPEED)
  	  	  step_delay[ch-1] =  0x7FFF;
  	  
  	  else if(speed>MAX_SPEED)
    	step_delay[ch-1] =  (int)(1.0e6/100/RAD2PULSE/MAX_SPEED);

	  else
    	step_delay[ch-1] =  (int)(1.0e6/100/RAD2PULSE/speed);
  }

}

// �X�e�b�s���O���[�^�쓮�֐�(���x)
void drive_step_motor_s(void)
{

	//	�T�[�{���g��20Hz�ɂȂ�Ƃ���ŏo�͕ύX
  if(servo_counter >1000)	
  {
  	  servo_counter =0;
  	  digitalWrite(SERVO_PIN,1);
  }
  
  //�T�[�{����̃f���[�e�B�쐬
  if(servo_counter ==servo_position_ref)	
  	  digitalWrite(SERVO_PIN, 0);

  servo_counter++;

	// ��~��ԃX�e�[�^�X�Ŗ�����΃X�e�b�s���O���[�^�h���C�u
	if(step_delay[0] != 0x7FFF)
	{
	  if(step_count[0] >= step_delay[0])	//����̑��x���o��^�C�~���O��
	  {
	    state[0] = !state[0];				//�o�̓p���X���]
	    digitalWrite(stpPin1, state[0]);
	    step_count[0] = 0;
	    step_position[0] += step_dir[0] * state[0]; //�����ʒu���A�b�v�f�[�g
	  }
	  step_count[0]++;
	}

	// ��~��ԃX�e�[�^�X�Ŗ�����΃X�e�b�s���O���[�^�h���C�u
	if(step_delay[1] != 0x7FFF)
	{
	  if(step_count[1] >= step_delay[1])	//����̑��x���o��^�C�~���O��
	  {
	    state[1] = !state[1];				//�o�̓p���X���]
	    digitalWrite(stpPin2, state[1]);
	    step_count[1] = 0;
	    step_position[1] += step_dir[1] * state[1];	//�����ʒu���A�b�v�f�[�g
	  }

	  step_count[1]++;
	}
  
  timer_counter++;
  
  (*call_control_process)();
  
}

// �X�e�b�s���O���[�^�쓮�֐�(�ʒu)
void drive_step_motor_p(void)
{
	
  //	�T�[�{���g��20Hz�ɂȂ�Ƃ���ŏo�͕ύX
  if(servo_counter >1000)
  {
  	  servo_counter =0;
  	  digitalWrite(SERVO_PIN,1);
  }
  
  //�T�[�{����̃f���[�e�B�쐬
  if(servo_counter ==servo_position_ref)
  	  digitalWrite(SERVO_PIN, 0);

  servo_counter++;

	// ��~��ԃX�e�[�^�X�łȂ��A���w�߈ʒu�ɂ��Ȃ���΃X�e�b�s���O���[�^�h���C�u
	if(step_delay[0] != 0x7FFF && step_position[0] != step_position_ref[0])
	{
	  if(step_count[0] >= step_delay[0])	//����̑��x���o��^�C�~���O��
	  {
	    state[0] = !state[0];				//�o�̓p���X���]
	    digitalWrite(stpPin1, state[0]);
	    step_count[0] = 0;
	    step_position[0] += step_dir[0] * state[0];	//�����ʒu���A�b�v�f�[�g
	  }
	  step_count[0]++;
	}

	// ��~��ԃX�e�[�^�X�łȂ��A���w�߈ʒu�ɂ��Ȃ���΃X�e�b�s���O���[�^�h���C�u
	if(step_delay[1] != 0x7FFF && step_position[1] != step_position_ref[1])
	{
	  if(step_count[1] >= step_delay[1])	//����̑��x���o��^�C�~���O��
	  {
	    state[1] = !state[1];				//�o�̓p���X���]
	    digitalWrite(stpPin2, state[1]);
	    step_count[1] = 0;
	    step_position[1] += step_dir[1] * state[1];	//�����ʒu���A�b�v�f�[�g
	  }

	  step_count[1]++;
	}
  (*call_control_process)();

}

//���[�^�h���C�u������
void init_step_motor(int mode)
{
  //  setting for stepping motor
  pinMode(dirPin1, OUTPUT);
  pinMode(stpPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stpPin2, OUTPUT);
  
  pinMode(SERVO_PIN,OUTPUT);
  
  call_control_process = nop_process;

  Timer1.initialize(50);
  if(mode ==SPEED)
  	Timer1.attachInterrupt(drive_step_motor_s);
  if(mode ==POSITION)
  	Timer1.attachInterrupt(drive_step_motor_p);

}

void nop_process(void)
{
  	
}

int set_interrupt_function(void (*function)(void))
{
	call_control_process = function;
}

//�n�[�h�E�F�A��
//�����g�Z���T������
int init_ussensor(void)
{
  int i;
   Serial.begin(9600);
   delay(10);

  for(i=0;i<4;i++)
    Serial.write(send_data[i]);
}

int get_distance(void)
{
  int read_data[255];
  int temp;
  int i;
  int count_data=0;
  int return_value=0;
  
  // �O��̃f�[�^���͂��Ă����
  if(Serial.available())
  {
    while((temp = Serial.read()) != (int)-1)
    {
      read_data[count_data] = temp;
      count_data++;
    }

	// �V���A���f�[�^2�o�C�g������(��ʂ�1bit�ȊO�؂�̂�)
    return_value = read_data[2] |((0x01&read_data[1])<<8);

  }
  else
    return_value=-1;
  
  if(read_data[0] != 0x22)
  	  return_value =-1;

// ����p�̃Z���T�p���X���M
  for(i=0;i<4;i++)
    Serial.write(send_data[i]);

    return return_value;
}


int get_temperature(void)
{
  int read_data[255];
  int temp;
  int i;
  int count_data=0;
  int wait_count=0;
  int return_value=0;

  // �O��̃f�[�^���͂��Ă���Γǂݎ̂Ă�
  if(Serial.available())
  {
    while((temp = Serial.read()) != (int)-1)
    {
      read_data[count_data] = temp;

      count_data++;
    }
  }

// ���x�Z���T�p���X���M
  for(i=0;i<4;i++)
  {
    Serial.write(temp_send_data[i]);
    delay(1);
  }

    delay(5);


	count_data=0;
  // �f�[�^���͂��Ă���Γǂ�
  if(Serial.available())
  {
    while((temp = Serial.read()) != (int)-1)
    {
      read_data[count_data] = temp;

      count_data++;
//      delay(1);
    }


	// �V���A���f�[�^2�o�C�g������(��ʂ�1bit�ȊO�؂�̂�)
    return_value = read_data[2] |(read_data[1]<<8);
  }
  else
    return_value=-999;
  
  if(read_data[0] != 0x11)
  	  return_value =-999;


    return return_value;
}



//�\�t�g�E�F�A�V���A����
//�����g�Z���T������
int init_ussensor_s(void)
{
  int i;
  extern SoftwareSerial softuart;
   softuart.begin( 9600UL  );
   delay(10);

}

int get_distance_s(void)
{
  int read_data[255];
  int temp;
  int i;
  int count_data=0;
  int return_value=0;
    extern SoftwareSerial softuart;

  // �O��̃f�[�^���͂��Ă���Γǂݎ̂Ă�
  if(softuart.available())
  {
    while((temp = softuart.read()) != (int)-1)
    {
      read_data[count_data] = temp;
      Serial.print(" 0x");
      Serial.print( read_data[count_data],HEX);

      count_data++;
    }
  }

// �Z���T�p���X���M
  for(i=0;i<4;i++)
  {
    softuart.write(send_data[i]);
  }

    delay(5);

	count_data=0;

  // �f�[�^���͂��Ă����
  if(softuart.available())
  {
    while((temp = softuart.read()) != (int)-1)
    {
      read_data[count_data] = temp;

      count_data++;
      delay(1);
    }

	// �V���A���f�[�^2�o�C�g������(��ʂ�1bit�ȊO�؂�̂�)
    return_value = read_data[2] |((0x01&read_data[1])<<8);
  }
  else
    return_value=-1;
  
  if(read_data[0] != 0x22)
  	  return_value =-1;


    return return_value;
}


int get_temperature_s(void)
{
  int read_data[255];
  int temp;
  int i;
  int count_data=0;
  int wait_count=0;
  int return_value=0;
  extern SoftwareSerial softuart;

  // �O��̃f�[�^���͂��Ă���Γǂݎ̂Ă�
  if(softuart.available())
  {
    while((temp = softuart.read()) != (int)-1)
    {
      read_data[count_data] = temp;

      count_data++;
    }
  }

// ���x�Z���T�p���X���M
  for(i=0;i<4;i++)
  {
    softuart.write(temp_send_data[i]);
    delay(1);
  }

    delay(5);


	count_data=0;
  // �f�[�^���͂��Ă���Γǂ�
  if(softuart.available())
  {
    while((temp = softuart.read()) != (int)-1)
    {
      read_data[count_data] = temp;

      count_data++;
//      delay(1);
    }


	// �V���A���f�[�^2�o�C�g������(��ʂ�1bit�ȊO�؂�̂�)
    return_value = read_data[2] |(read_data[1]<<8);
  }
  else
    return_value=-999;
  
  if(read_data[0] != 0x11)
  	  return_value =-999;


    return return_value;
}
