#include "arduino.h"
#include "control_lib.h"
#include <MeOrion.h>      //Orion board

int dirPin1 = mePort[PORT_1].s1;
int stpPin1 = mePort[PORT_1].s2;

int dirPin2 = mePort[PORT_2].s1;
int stpPin2 = mePort[PORT_2].s2;

int timer_counter;		//計時用50μインクリ
int step_count[2];
int step_delay[2]={0x7FFF,0x7FFF};	//ステッピングモータのパルス間隔
int step_dir[2] = {0,0};			//回転方向 正回転1 逆回転-1
long step_position[2]={0,0};		//現在位置(パルス)
long step_position_ref[2] ={0,0};	//位置指令(パルス)
int servo_position_ref=8;			//サーボ位置
int servo_counter=0;				//サーボ制御用カウンタ

int state[2]={LOW,LOW};				//ステッピングモータ制御ピン

void update_servo_angle(int angle)
{
	servo_position_ref = angle *32 /180 +10;
	Serial.print("");		// 空シリアルを入れないとなぜか動かないのを修正(謎)
}

//	speedの速度でpositionまでchのステッピングモータを位置指令値セット
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
	
	//	戻り値は目標値までの誤差
	return step_position_ref[ch-1] -pos_now;

}

float get_step_position(int ch)
{
	return (step_position[ch-1]*2*3.14/ROTATEPULSE);
}

//	chのステッピングモータの速度指令値をspeedにセット
void update_step_speed(int ch,float speed)
{
  if(ch ==1)
  {
  	  if(speed<0)
  	  {
  	  	  digitalWrite(dirPin1,1);	// 1の時負回転
  	  	  step_dir[0] =-1;
  	  	  speed= -speed;
  	  }
  	  else
  	  {
  	  	  digitalWrite(dirPin1,0);	// 0の時正回転
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
  	  	  digitalWrite(dirPin2,1);	// 1の時負回転
  	  	  step_dir[1] = -1;
  	  	  speed= -speed;
  	  }
  	  else
  	  {
  	  	  digitalWrite(dirPin2,0);	// 0の時正回転
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

// ステッピングモータ駆動関数(速度)
void drive_step_motor_s(void)
{

	//	サーボ周波数20Hzになるところで出力変更
  if(servo_counter >1000)	
  {
  	  servo_counter =0;
  	  digitalWrite(SERVO_PIN,1);
  }
  
  //サーボ所定のデューティ作成
  if(servo_counter ==servo_position_ref)	
  	  digitalWrite(SERVO_PIN, 0);

  servo_counter++;

	// 停止状態ステータスで無ければステッピングモータドライブ
	if(step_delay[0] != 0x7FFF)
	{
	  if(step_count[0] >= step_delay[0])	//所定の速度が出るタイミングで
	  {
	    state[0] = !state[0];				//出力パルス反転
	    digitalWrite(stpPin1, state[0]);
	    step_count[0] = 0;
	    step_position[0] += step_dir[0] * state[0]; //内部位置情報アップデート
	  }
	  step_count[0]++;
	}

	// 停止状態ステータスで無ければステッピングモータドライブ
	if(step_delay[1] != 0x7FFF)
	{
	  if(step_count[1] >= step_delay[1])	//所定の速度が出るタイミングで
	  {
	    state[1] = !state[1];				//出力パルス反転
	    digitalWrite(stpPin2, state[1]);
	    step_count[1] = 0;
	    step_position[1] += step_dir[1] * state[1];	//内部位置情報アップデート
	  }

	  step_count[1]++;
	}
  
  timer_counter++;
  
  (*call_control_process)();
  
}

// ステッピングモータ駆動関数(位置)
void drive_step_motor_p(void)
{
	
  //	サーボ周波数20Hzになるところで出力変更
  if(servo_counter >1000)
  {
  	  servo_counter =0;
  	  digitalWrite(SERVO_PIN,1);
  }
  
  //サーボ所定のデューティ作成
  if(servo_counter ==servo_position_ref)
  	  digitalWrite(SERVO_PIN, 0);

  servo_counter++;

	// 停止状態ステータスでなく、かつ指令位置にいなければステッピングモータドライブ
	if(step_delay[0] != 0x7FFF && step_position[0] != step_position_ref[0])
	{
	  if(step_count[0] >= step_delay[0])	//所定の速度が出るタイミングで
	  {
	    state[0] = !state[0];				//出力パルス反転
	    digitalWrite(stpPin1, state[0]);
	    step_count[0] = 0;
	    step_position[0] += step_dir[0] * state[0];	//内部位置情報アップデート
	  }
	  step_count[0]++;
	}

	// 停止状態ステータスでなく、かつ指令位置にいなければステッピングモータドライブ
	if(step_delay[1] != 0x7FFF && step_position[1] != step_position_ref[1])
	{
	  if(step_count[1] >= step_delay[1])	//所定の速度が出るタイミングで
	  {
	    state[1] = !state[1];				//出力パルス反転
	    digitalWrite(stpPin2, state[1]);
	    step_count[1] = 0;
	    step_position[1] += step_dir[1] * state[1];	//内部位置情報アップデート
	  }

	  step_count[1]++;
	}
  (*call_control_process)();

}

//モータドライブ初期化
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

//ハードウェア版
//超音波センサ初期化
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
  
  // 前回のデータが届いていれば
  if(Serial.available())
  {
    while((temp = Serial.read()) != (int)-1)
    {
      read_data[count_data] = temp;
      count_data++;
    }

	// シリアルデータ2バイトを結合(上位は1bit以外切り捨て)
    return_value = read_data[2] |((0x01&read_data[1])<<8);

  }
  else
    return_value=-1;
  
  if(read_data[0] != 0x22)
  	  return_value =-1;

// 次回用のセンサパルス送信
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

  // 前回のデータが届いていれば読み捨てる
  if(Serial.available())
  {
    while((temp = Serial.read()) != (int)-1)
    {
      read_data[count_data] = temp;

      count_data++;
    }
  }

// 温度センサパルス送信
  for(i=0;i<4;i++)
  {
    Serial.write(temp_send_data[i]);
    delay(1);
  }

    delay(5);


	count_data=0;
  // データが届いていれば読む
  if(Serial.available())
  {
    while((temp = Serial.read()) != (int)-1)
    {
      read_data[count_data] = temp;

      count_data++;
//      delay(1);
    }


	// シリアルデータ2バイトを結合(上位は1bit以外切り捨て)
    return_value = read_data[2] |(read_data[1]<<8);
  }
  else
    return_value=-999;
  
  if(read_data[0] != 0x11)
  	  return_value =-999;


    return return_value;
}



//ソフトウェアシリアル版
//超音波センサ初期化
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

  // 前回のデータが届いていれば読み捨てる
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

// センサパルス送信
  for(i=0;i<4;i++)
  {
    softuart.write(send_data[i]);
  }

    delay(5);

	count_data=0;

  // データが届いていれば
  if(softuart.available())
  {
    while((temp = softuart.read()) != (int)-1)
    {
      read_data[count_data] = temp;

      count_data++;
      delay(1);
    }

	// シリアルデータ2バイトを結合(上位は1bit以外切り捨て)
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

  // 前回のデータが届いていれば読み捨てる
  if(softuart.available())
  {
    while((temp = softuart.read()) != (int)-1)
    {
      read_data[count_data] = temp;

      count_data++;
    }
  }

// 温度センサパルス送信
  for(i=0;i<4;i++)
  {
    softuart.write(temp_send_data[i]);
    delay(1);
  }

    delay(5);


	count_data=0;
  // データが届いていれば読む
  if(softuart.available())
  {
    while((temp = softuart.read()) != (int)-1)
    {
      read_data[count_data] = temp;

      count_data++;
//      delay(1);
    }


	// シリアルデータ2バイトを結合(上位は1bit以外切り捨て)
    return_value = read_data[2] |(read_data[1]<<8);
  }
  else
    return_value=-999;
  
  if(read_data[0] != 0x11)
  	  return_value =-999;


    return return_value;
}
