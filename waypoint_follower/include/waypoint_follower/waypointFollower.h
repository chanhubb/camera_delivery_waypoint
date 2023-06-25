#ifndef WAYPOINTFOLLOWER_H
#define WAYPOINTFOLLOWER_H

#include <ros/ros.h>
// #include <tf/tf.h>

// C, C++ header
#include <iostream>
#include <vector>

// msg OpenSource
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

// msgs from waypont_Loader
#include <waypoint_maker/Lane.h>
#include <waypoint_maker/Waypoint.h>
#include <waypoint_maker/State.h>

// msg for Big_Static mission
#include <static_big_gps/LidarMsg.h>

// msg for Delivery
#include <delivery_yolo/delivery_sign.h>

// msg for parking
#include <parking/ParkingMsg.h>

// msg for FMTC vision (23.04.01)
#include <vision_fmtc/vision_check.h>

// msg for FMTC LIDAR (23.04.01)
// #include "fmtc_lidar/lidar_discision.h"

// msg for FMTC IMU (23.04.01)
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <math.h>

//AD chanhee
#include <camera_delivery/CD.h>
//


using namespace std;

class WaypointFollower
{
private:
	/*
	필요없는 변수 후보

		int mission_state_;
		double ex_xpos_ = 0;
		double ex_ypos_ = 0;
		double xpos_;
		double ypos_;
		ros::Publisher ackermann_pub_;
		int waypoint_target_index_;
		int lane_final_;
		int ex_lane_final_;
		float lidar_obj_dist[4] = {0};
		bool parking_ok_[2] = {false};

	*/

	// parameters in header
	double lpf_course_;
	double prev_lpf;

	vector<waypoint_maker::Waypoint> waypoints_;
	int loader_number_;
	float max_search_dist_;
	int waypoint_min;

	// parameters for process
	bool is_pose_;
	bool is_course_;
	bool is_lane_;
	bool is_control_;

	int spd_state_;
	float dist_;

	int current_mission_state_;
	bool vision_check_;

	float input_speed_;
	float input_steer_;

	// parameters for pure pursuit
	geometry_msgs::PoseStamped cur_pose_;
	float minimum_ld = 2.0f;
	const float maximum_ld = 20.0f;
	const float ld_ratio = 0.4f;

	float lookahead_dist_;
	int waypoints_size_;
	int target_index_;
	float wheel_base_;
	// double dx;
	// double dy;
	double cur_course_;
	float cur_speed_;

	// parameters for Delivery Mission (in Pure pursuit)
	bool is_delivery_offset_;
	float delivery_offset_A1;
	float delivery_offset_B;
	bool delivery_stop_;
	bool delivery_ready_;

	// parameters for Big Static Mission (in Pure Pursuit)
	int closest_waypoint_;
	bool is_static_;
	bool is_obs_detect_;
	int obs_waypoint_index_;
	double obs_dist_;
	float offset_;

	// parameters for Parking (in Process & in Pure Pursuit)
	ros::Time start_;
	ros::Time end_;
	ros::Duration sec_;

	ros::Time start_2;
	ros::Time end_2;
	ros::Duration sec_2;

	int lane_number_;
	int parking_count_;
	bool is_parking_finished_;
	bool is_PurePursuit_;
	bool is_backward_;
	bool is_ready_;

	// parameters for Parking_Lidar (in Process)
	float GPS_parking_dist_[2] = {0};
	vector<float> lidar_obj_dist;
	float parking_spot_[3][2] = {
		{302479.162029569, 4123760.41933156},
		{302476.728706876, 4123756.06162277},
	};
	int donnot_parking_count_[2] = {0};

	int parking_candidate_;
	bool keep_search_;
	int best_parking_count_;

	int lazu_count_;
	bool detection_trigger_;

	// variable for CrossWalk mission
	bool is_cross_walk_finish1;
	bool is_cross_walk_finish2;
	bool walk1_;
	bool walk2_;

	// parameters for FMTC 4.1 vision
	bool vision_slow_;
	bool vision_stop_;
	bool vision_traffic_;

	// parameters for FMTC 4.1 lidar
	bool lidar_static_;
	bool lidar_dynamic_;
	bool lidar_warning_;
	float lidar_dist_;
	bool lidar_flag;

	// parameters for FMTC 4.1 IMU
	float fixed_cur_course_;

	// ROS
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

	// publisher

	// to waypoint_loader
	ros::Publisher lane_number_pub_;

	// subscriber

	// from nmea_parser
	ros::Subscriber course_sub_;

	// from utm_odometry_node
	ros::Subscriber odom_sub_;

	// from Waypoint_loader
	ros::Subscriber lane_sub_;
	ros::Subscriber state_sub_;

	// from static_big_gps
	ros::Subscriber static_lidar_sub_;

	// from traffic sign checker
	ros::Subscriber vision_sub_;

	// from delivery_yolo
	ros::Subscriber deliverygosign_sub_;

	// from Parking_Lidar
	ros::Subscriber parking_lidar_sub_;

	// 4.1 FMTC
	ros::Subscriber vision_traffic_sub_;
	ros::Subscriber vision_sign_sub_;
	ros::Subscriber lidar_discision_sub_;
	// from IMU sensor current pose (hee)
	ros::Subscriber imu_sub_;

	// msgs
	ackermann_msgs::AckermannDriveStamped ackermann_msg_;
	waypoint_maker::State lane_number_msg_;

	sensor_msgs::Imu imu_cur_course_;

	//AD
	
	ros::Subscriber CD_sub;
	
	int lane1_stop_cnt;
	int lane2_stop_cnt;
	int cam_lane_;
	bool cam_stop1_;
	bool cam_stop2_;
	//AD

public:
	WaypointFollower()
	{
		initSetup();
	}

	~WaypointFollower()
	{
		waypoints_.clear();
	}

	void initSetup()
	{

		// parameters in header
		lpf_course_ = 0;
		prev_lpf = 0;

		loader_number_ = 0;
		max_search_dist_ = 10.0f;
		waypoint_min = -1;

		// parameters for process
		is_pose_ = false;
		is_course_ = false;
		is_lane_ = false;
		is_control_ = false;

		spd_state_ = 0;
		dist_ = 100.0f;

		current_mission_state_ = -1;
		vision_check_ = true;

		input_speed_ = 0.0f;
		input_steer_ = 0.0f;

		// parameters for pure pursuit
		lookahead_dist_ = 6.0f;
		waypoints_size_ = 0;
		target_index_ = 0;
		wheel_base_ = 1.04f;
		cur_speed_ = 0.0f;
		cur_course_ = 0.0; // pure-pursuit 결과값

		// parameters for delivery Mission
		is_delivery_offset_ = false;
		delivery_offset_A1 = 0.0f;
		delivery_offset_B = 0.0f;
		delivery_stop_ = false;
		delivery_ready_ = false;

		// parameters for Big_Static Mission
		closest_waypoint_ = 0;
		is_static_ = false;
		is_obs_detect_ = false;
		obs_waypoint_index_ = 0;
		obs_dist_ = 0.0;
		offset_ = 2.0f;

		// varaible for Parking Mission
		lane_number_ = 0;
		private_nh_.getParam("/waypoint_loader_node/parking_count", parking_count_);
		is_parking_finished_ = false;
		is_backward_ = false;
		is_PurePursuit_ = true;
		is_ready_ = false;

		// parameters for Parking_Lidar (in Process)

		parking_candidate_ = 0;
		keep_search_ = true;
		best_parking_count_ = 0;

		lazu_count_ = 0;
		detection_trigger_ = false;

		// variable for CrossWalk Mission
		is_cross_walk_finish1 = false;
		is_cross_walk_finish2 = false;
		walk1_ = false;
		walk2_ = false;

		// parameters for FMTC 4.1 vision
		vision_slow_ = false;
		vision_stop_ = false;
		vision_traffic_ = false;

		// parameters for FMTC 4.1 lidar
		lidar_static_ = false;
		lidar_dynamic_ = false;
		lidar_warning_ = false; // false 이면 움직이면됨
		lidar_dist_ = -1;		// 초깃값 고민해보기 //보험
		lidar_flag = false;
		fixed_cur_course_ = 0;
	

		//AD
		cam_stop1_=false;
		cam_stop2_=false;
		cam_lane_=-1;
		lane1_stop_cnt=0;
		lane2_stop_cnt=0;
		
		//AD
		
		

		///

		// ROS
		// Publisher
		lane_number_pub_ = nh_.advertise<waypoint_maker::State>("lane_number_msg_", 1);

		// Subscriber
		course_sub_ = nh_.subscribe("course", 1, &WaypointFollower::CourseCallback, this);
		odom_sub_ = nh_.subscribe("odom", 1, &WaypointFollower::OdomCallback, this);
		lane_sub_ = nh_.subscribe("final_waypoints", 1, &WaypointFollower::LaneCallback, this);
		state_sub_ = nh_.subscribe("gps_state", 1, &WaypointFollower::StateCallback, this);
		static_lidar_sub_ = nh_.subscribe("/static_big_msgs", 10, &WaypointFollower::StaticLidarCallback, this); // big static
		vision_sub_ = nh_.subscribe("/traffic_light_judgement", 1, &WaypointFollower::TrafficSignCallback, this);
		deliverygosign_sub_ = nh_.subscribe("/delivery_offset", 10, &WaypointFollower::DeliverygosignCallback, this);
		parking_lidar_sub_ = nh_.subscribe("/parking_msg", 1, &WaypointFollower::ParkingLidarCallback, this);
		// Sub for 4.1 FMTC
		vision_traffic_sub_ = nh_.subscribe("vision_traffic", 1, &WaypointFollower::VisionTrafficCallback, this); // 신호등  std_msgs 형태로 넘어옴
		vision_sign_sub_ = nh_.subscribe("vision_sign", 1, &WaypointFollower::VisionSignCallback, this);		  //
		// lidar_discision_sub_ = nh_.subscribe("/lidar_discision", 1, &WaypointFollower::LidarDiscisionCallback, this); //
		imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/handsfree/imu", 10, &WaypointFollower::imuCourseCallback, this); // hee
		
		//AD
		CD_sub = nh_.subscribe("CD_topic", 10, &WaypointFollower::ADCallback, this);
		//AD
	}

	//AD
	void ADCallback(const camera_delivery::CD::ConstPtr &CD_msg_)
	{
		cam_lane_ = CD_msg_->cam_lane; 
		cam_stop1_ = CD_msg_->cam_stop1;
		cam_stop2_=CD_msg_->cam_stop2;
	}
	//AD

	double LPF(double heading)
	{
		float alpha = 0.3f;
		double lpf_course_;

		lpf_course_ = alpha * prev_lpf + (1 - alpha) * heading;

		prev_lpf = lpf_course_;

		return lpf_course_;
	}

	double calcPlaneDist(const geometry_msgs::PoseStamped pose1, const geometry_msgs::PoseStamped pose2)
	{
		return (double)(sqrt(pow(pose1.pose.position.x - pose2.pose.position.x, 2) + pow(pose1.pose.position.y - pose2.pose.position.y, 2)));
	}

	void OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
	{
		cur_pose_.header = odom_msg->header;
		cur_pose_.pose.position = odom_msg->pose.pose.position;
		is_pose_ = true;
	}

	void CourseCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &course_msg)
	{
		// imu ㅏ안쓸때
		cur_course_ = course_msg->drive.steering_angle;
		// imu 쓸때
		fixed_cur_course_ = weight(course_msg->drive.steering_angle, imu_cur_course_.orientation.z);

		cur_speed_ = course_msg->drive.speed;
		is_course_ = true;
	}

	void TrafficSignCallback(const std_msgs::Bool::ConstPtr &vision_msg)
	{
		vision_check_ = vision_msg->data;
	}

	void DeliverygosignCallback(const delivery_yolo::delivery_sign::ConstPtr &deliverygo_msg)
	{
		is_delivery_offset_ = deliverygo_msg->delivery_offset; // 트리거
		delivery_ready_ = deliverygo_msg->delivery_ready;
		delivery_stop_ = deliverygo_msg->delivery_stop;
	}

	void StaticLidarCallback(const static_big_gps::LidarMsg::ConstPtr &static_lidar_msg)
	{
		is_obs_detect_ = static_lidar_msg->is_obs;
		obs_dist_ = static_lidar_msg->obs_dist;
	}

	void StateCallback(const waypoint_maker::State::ConstPtr &state_msg)
	{
		dist_ = state_msg->dist;
		current_mission_state_ = state_msg->current_state;
		loader_number_ = state_msg->lane_number;
	}

	void LaneCallback(const waypoint_maker::Lane::ConstPtr &lane_msg)
	{
		waypoints_.clear();
		vector<waypoint_maker::Waypoint>().swap(waypoints_);
		waypoints_ = lane_msg->waypoints;
		waypoints_size_ = waypoints_.size();
		if (waypoints_size_ != 0)
		{
			is_lane_ = true;
		}
	}

	// Parking for Lidar
	void ParkingLidarCallback(const parking::ParkingMsg::ConstPtr &parking_msg)
	{
		lidar_obj_dist.clear();
		lidar_obj_dist = parking_msg->car_;
	}

	// for 4.1 FMTC vision

	void VisionTrafficCallback(const std_msgs::Bool::ConstPtr &vision_traffic)
	{
		vision_traffic_ = vision_traffic->data;
	}

	void VisionSignCallback(const vision_fmtc::vision_check::ConstPtr &vision_sign)
	{
		vision_slow_ = vision_sign->vision_slow;
		vision_stop_ = vision_sign->vision_stop;
	}
	// for 4.1 FMTC lidar
	/*
	void LidarDiscisionCallback(const fmtc_lidar::lidar_discision::ConstPtr &lidar_discision)
	{
		lidar_static_ = lidar_discision->static_;
		lidar_dynamic_ = lidar_discision->dynamic_;
		lidar_warning_ = lidar_discision->warning_;
		lidar_dist_ = lidar_discision->chandist_;
	}
	*/

	// for 4.1 IMU
	void imuCourseCallback(const sensor_msgs::Imu::ConstPtr &imu_msg)
	{
		// imu_cur_course_.header.frame_id=imu_msg->header.frame_id;
		// imu_cur_course_.orientation.x=imu_msg->orientation.x;
		// imu_cur_course_.orientation.y=imu_msg->orientation.y;
		// imu_cur_course_.orientation.z=imu_msg->orientation.z;
		// imu_cur_course_.orientation.w=imu_msg->orientation.w;

		// experiment

		// tf::Quaternion qua(imu_msg->orientation.x, imu_msg->orientation.y, -imu_msg->orientation.z,imu_msg->orientation.w);
		//  y axis plus minus convert

		// tf::Quaternion qua(imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z,imu_msg->orientation.w);

		// 1. y axis plus minus convert (-o.z) 2. //coordinates transform -90 degree -> x'=y, y'=-x
		// tf::Quaternion qua(imu_msg->orientation.y, -imu_msg->orientation.x, -imu_msg->orientation.z,imu_msg->orientation.w);

		// if this cannot be solove I use this
		tf::Quaternion qua(imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w);
		// y axis plus minus convert

		// ending
		tf::Matrix3x3 mat(qua);
		double roll, pitch, yaw;
		mat.getRPY(roll, pitch, yaw);

		imu_cur_course_.orientation.x = roll * 180 / M_PI;
		imu_cur_course_.orientation.y = -pitch * 180 / M_PI; // pitch값 반대
		imu_cur_course_.orientation.z = -yaw * 180 / M_PI;	 // yaw 값 반대
		imu_cur_course_.orientation.w = 1;

		if (imu_cur_course_.orientation.z < 0)
			imu_cur_course_.orientation.z += 360.0; // minus yaw degree to plus

		// if this cannot be solove I use this
		// imu_cur_course_.orientation.z+=180.0; //convert to -90 degree
		if (imu_cur_course_.orientation.z >= 360.0)
			imu_cur_course_.orientation.z -= 360.0;
		imu_cur_course_.orientation.z = imu_cur_course_.orientation.z - 8.95; // magnetic and true difference

		// cout<<"this is IMU SENSOR: "<<roll<<" // "<<pitch<<" // "<<yaw<<endl;
		cout << "IMU_orientation: " << imu_cur_course_.orientation.x << " // " << imu_cur_course_.orientation.y << " // " << imu_cur_course_.orientation.z << endl;
	}

	float weight(float current_gps_course, float current_imu_course)
	{
		float imu_weight = 0.0f;
		float gps_weight = 1.0f - imu_weight;
		float tmp;
		float weight_course= current_gps_course;
		//float weight_course = fabs(current_gps_course + current_imu_course) / 2;
		// double weight_course=current_gps_course+(360-tmp)/2;
		// double weight_course=current_gps_course+tmp/2;

		if (weight_course == 360.0)
			weight_course = 0;

		// 350 10 -> 360=0, 340 5
		//cout << "weight_course_angle: " << weight_course << endl;
		// double weight_course=imu_weight*current_imu_course+gps_weight*current_gps_course;
		return weight_course;
	}

	void getClosestWaypoint(geometry_msgs::PoseStamped current_pose)
	{ // 큰 정적 미션용 getClosestWaypoint

		if (!waypoints_.empty())
		{
			float dist_min = max_search_dist_;
			for (int i = 0; i < waypoints_.size(); i++)
			{
				float dist = calcPlaneDist(current_pose, waypoints_[i].pose);
				if (dist < dist_min)
				{
					dist_min = dist;
					waypoint_min = i;
				}
			}
			closest_waypoint_ = waypoint_min;
		}
		else
			cout << "------ NO CLOSEST WAYPOINT -------" << endl;
	}

	float calcSteeringAngle(); // 함수 명 선언
	float delivery_calcSteeringAngle(); // 함수 명 선언
	void process();
	float getSpeed() { return input_speed_; }
	float getSteer() { return input_steer_; }
};

#endif









