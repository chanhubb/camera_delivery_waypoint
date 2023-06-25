#ifndef NEWPLATFORMCONTROL_H
#define NEWPLATFORMCONTROL_H

#include <ros/ros.h>
#include <serial/serial.h>



/*
Cycle Time : 20 msec
Baud:115200, parity : None, Stop : 1

PC -> ERP42 (14 Byte)								ERP42 -> PC (18 Byte)
바이트 이름     /			값						바이트 이름	 	/			값
1	S				0x53						1	S					0x53
2	T				0x54						2	T					0x54
3	X				0x58						3	X					0x58
4	A or M			0x00 or 0x01				4	A or M				0x00 or 0x01
5	E-stop			0x00 or 0x01				5	E-stop				0x00 or 0x01
6	GEAR			0~2							6	GEAR				0~2
7	SPEED0┐			0~200						7	SPEED0┐				0~200
8	SPEED1┘										8	SPEED1┘
9	STEER0┐			-2000~2000					9	STEER0┐				-2000~2000
10	STEER1┘										10	STEER1┘
11	BRAKE			1~150						11	BRAKE				1~150
12	ALIVE			0~255						12	ENC0┐
13	ETX0			0x0D						13	ENC1│				-2^31~2^31
14	ETX1			0x0A						14	ENC2│
												15	ENC3┘
												16	ALIVE				0~255
												17	ETX0				0x0D
												18	ETX1				0x0A

시리얼에 들어가 있는 값들
AorM -> Auto or Manual -------- 0x00 : manual mode ,      0x01 : auto mode
ESTOP -> Emergency STOP -------- 0x00 : E-STOP Off,      0x01 : E-STOP On
GEAR -> 0x00 : forward drive,      0x01 : neutral,      0x02 : backward drive
SPEED -> actual speed (KPH) * 10
STEER -> actual steering dgree (dgree) * 71, 오차율 : 4%  (negative is left steer)
BRAKE -> 1 : no braking,      150 : full braking
ENC -> encoder counting
ALIVE -> increasing each one step


<Returned Serial data 변환>
speed = (speed_return0 + speed_return1 * 256) / 10) km/h
steer = ((int)(serial_input[8]) + (int)(serial_input[9]) * 256) / 71) degree
brake = 1~150
*/
using namespace std;

class PlatformConnector
{
private:
	serial::Serial ser_;
	ros::NodeHandle nh_;

	ros::Subscriber course_sub_;

	float input_speed_;
	float input_steer_;
	float cur_speed_;

	unsigned char alive_;
	unsigned char gear_;
	unsigned char speed0_;
	unsigned char speed1_;
	unsigned char steer0_;
	unsigned char steer1_;
	unsigned char front_brake_ = 0x01;
	unsigned char str_[14];

	size_t num_write_;

	// PID
	const float Kp_ = 0.5f;			// p_gain
	const float Ki_ = 0.5f;			// I_gain
	const float Kd_ = 0.5f;			// D_gain
	const float controltime_ = 0.1; // 10Hz -> 0.1 , 30Hz -> 0.033

	float prev_input_ = 0.0f;
	float prev_error_ = 0.0f;

public:
	PlatformConnector()
	{
		initSetup();
	}

	void initSetup()
	{

		course_sub_ = nh_.subscribe("course", 10, &PlatformConnector::CourseCallback, this);

		alive_ = 0x00;	// 0x00: manual mode
		gear_ = 0x00;	// 0x00: foward drive
		speed0_ = 0x00; //
		speed1_ = 0x00; // 0~200 actual speed (kph) * 10
		steer0_ = 0x00; // -2000~2000 actual steering angle (degree) * 71 +_ 4% negative: left
		steer1_ = 0x00;
		front_brake_ = 0x01; // 1 no 200 full brake
		num_write_ = 14;

		ROS_INFO("[PLATFORM_CONTROL] : INITIALIZED");
	}

	void readserial()
	{
		serialConnect();
		string serial_input;

		if (ser_.available())
		{
			serial_input = ser_.readline();
			alive_ = (unsigned char)serial_input[15];
			if (alive_ != 0xff)
				alive_++;
			else
				alive_ = 0x00;
			ser_.write(str_, num_write_);
		}
		else
			cout << "----------------NO ALIVE----------------" << endl;

		ser_.close();
	}

	void serialConnect()
	{
		try
		{
			// MRP-2000: /dev/ttyUSB0
			ser_.setPort("/dev/ttyUSB1");
			ser_.setBaudrate(115200);
			serial::Timeout to = serial::Timeout::simpleTimeout(20);
			ser_.setTimeout(to);
			ser_.open();
		}
		catch (serial::IOException &e)
		{
			ROS_ERROR_STREAM("UNABLE TO OPEN SERIAL PORT.");
		}

		if (!ser_.isOpen())
		{
			ROS_ERROR_STREAM("UNABLE TO INITIALIZE SERIAL PORT.");
			ros::shutdown();
		}
	}

	void inputSerial()
	{

		// set final_input_speed
		float pid = PID(input_speed_, cur_speed_);

		// Carculate Speed & Brake
		if (input_speed_ == 0) // 정지 명령시 full brake
		{
			speed0_ = 0X00;
			speed1_ = 0X00;
			front_brake_ = 150;
		}
		else
		{
			if (input_speed_ > 0 && input_speed_ < 200)
			{ // 전진
				// pid값 반영
				// input_speed_+=pid;
				gear_ = 0X00;
				speed0_ = 0X00;
				speed1_ = input_speed_ * 10;
				front_brake_ = 0;
			}
			else if (input_speed_ > -200 && input_speed_ < 0)
			{ // 후진
				gear_ = 0X02;
				speed0_ = 0X00;
				speed1_ = -input_speed_ * 10;
				front_brake_ = 0;
			}
			else
			{ // 예외처리 -> 필요없다고 판단되나 혹시나..
				gear_ = 0x01;
				speed0_ = 0x00;
				speed1_ = 0x00;
				front_brake_ = 150;
			}
		}

		if (speed1_ > 180)
			speed1_ = 180;

		if (input_steer_ > 28)
			input_steer_ = 28;
		else if (input_steer_ < -28)
			input_steer_ = -28;

		// Carculate Steer
		steer0_ = (int)(input_steer_ * 71) >> 8;
		steer1_ = (int)(input_steer_ * 71) & 0xff;

		// Input Serial
		str_[0] = 0x53;
		str_[1] = 0x54;
		str_[2] = 0x58;
		str_[3] = 0x01;
		str_[4] = 0x00;
		str_[5] = gear_;
		str_[6] = speed0_;
		str_[7] = speed1_;
		str_[8] = steer0_;
		str_[9] = steer1_;
		str_[10] = front_brake_;
		str_[11] = alive_;
		str_[12] = 0x0D;
		str_[13] = 0x0A;
	}

	void setFromWF(float wf_speed, float wf_steer)
	{
		this->input_speed_ = wf_speed * 3.6;
		this->input_steer_ = wf_steer;
	}

	float PID(float target_spd, float cur_spd)
	{
		float error = input_speed_ - cur_spd;

		float p_control = Kp_ * error;
		float i_control = 0.0f;
		i_control += Ki_ * error * controltime_;

		// float d_control = Kd_ * (input_spd - prev_input_)/controltime_; 아래 d제어가 작동하지 않으면
		float d_control = Kd_ * (error - prev_error_) / controltime_;

		float output = p_control + i_control + d_control;

		// prev_input_ = input_spd;
		prev_error_ = error;
		return output;
	} // input : cur_spd   Output : Output

	void CourseCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &course_msg)
	{
		cur_speed_ = course_msg->drive.speed; // kph
	}
};

#endif
