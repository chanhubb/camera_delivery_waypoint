#include <ros/ros.h>

//msgs
#include <geometry_msgs/PoseStamped.h> 
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <waypoint_maker/Lane.h> //For Parking Mission
#include <waypoint_maker/Waypoint.h> // Waypoints
#include <waypoint_maker/State.h> //For Mission (미션에 필요한 msg 헤더 모음)

#include <vector>
#include <string>

// For CSV file loading
#include <fstream>
#include <sys/types.h> // 시스템 자료형 타입 정리용 헤더
#include <dirent.h> // 디렉토리 다루기 위한 헤더


//AD
#include <camera_delivery/CD.h>

//


using namespace std;

/*
	waypoint loader algorithm : 

	1. initsetup()
		1-1 get_csvs_fromDirectory() : save all csv file's name in vector.
		1-2 get_new_waypooints() : open CSV files and save waypoints in vector.
	2. subscribe the Current_pose and the Lane_number
	3. poseprocess()
		3-1 getClosesttWaypoint() : find the closest waypoint 
		3-2 getFinalWaypoint() : decide 50 waypoints to publish.
		3-3 mission_state_ 설정
		3-4 state & waypoint publish
	4. publish waypoints

*/

class WaypointLoader {
private:

	int state_inspection_;	
	int current_mission_state_;

//ROS
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

	// publisher	
	ros::Publisher waypoint_pub_;
	ros::Publisher state_pub_;
	
	// subscriber
	ros::Subscriber pose_sub_;
	ros::Subscriber lane_number_sub_;

	//msg
	geometry_msgs::PoseStamped current_pose;
	waypoint_maker::Lane lane_msg_;
	waypoint_maker::State state_msg_;

//Functions
	//get_csvs_fromDirectory() & getNewWaypoints()
	std::vector<std::string> all_csv_;
	ifstream is_;
	vector< vector<waypoint_maker::Waypoint> > all_new_waypoints_;
	vector<waypoint_maker::Waypoint> new_waypoints_;
	vector<int> lane_size_;  
	int size_;

	vector< vector<int> > all_state_index_;

	//getClosestWaypoint()
	vector<int> closest_waypoint_candidates;
	float max_search_dist_;
	float min_search_dist_;
	float dist_;
	int closest_waypoint_;
	int ex_closest_waypoint_;
	
	//getFinalWaypoint()
	int final_size_;
	vector<waypoint_maker::Waypoint> final_waypoints_;
	//최종적으로 쏴주는 펍 웨포



//Mission
	//Parking mission
	int parking_state_;
	int lane_number_=0;
	int ex_lane_number_;
	bool is_onlane_;
	
	bool is_state_;
	
	//AD
	int cam_lane_in_maker_;
	bool cam_stop1_in_maker_;
	bool cam_stop2_in_maker_;
	int gps_lane_;
	int lane1_cnt;
	int lane2_cnt;
	ros::Subscriber CD_sub;
	//AD

public:	
	WaypointLoader() {
		initSetup();
		ROS_INFO("WAYPOINT LOADER INITIALIZED.");
	}
	
	~WaypointLoader() {
		ROS_INFO("WAYPOINT LOADER TERMINATED.");
	}
	
	void initSetup() {
		
		private_nh_.getParam("/waypoint_loader_node/state_inspection", state_inspection_);
		
		private_nh_.getParam("/waypoint_loader_node/parking_count", parking_state_); 
	
		waypoint_pub_ = nh_.advertise<waypoint_maker::Lane>("final_waypoints", 1);
		state_pub_ = nh_.advertise<waypoint_maker::State>("gps_state",1);

		pose_sub_ = nh_.subscribe("odom", 10, &WaypointLoader::poseCallback, this);
		//lane_number_sub_ = nh_.subscribe("lane_number_msg_", 1, &WaypointLoader::lane_number_Callback, this);

		//AD chanhee
		CD_sub = nh_.subscribe("CD_topic", 10, &WaypointLoader::CDCallback, this);
		//AD
		final_size_ = 50;

		max_search_dist_ = 10.0f;
		min_search_dist_ = 0.5f;
		current_mission_state_ = state_inspection_; 

		closest_waypoint_ = 0;
		
		lane_number_ = 0;
		ex_lane_number_ = 0;

		is_onlane_ = false;

		is_state_ = false;

		//AD chanhee
		
		cam_lane_in_maker_=-1;
		cam_stop1_in_maker_=false;
		cam_stop2_in_maker_=false;
		lane1_cnt=0;
		lane2_cnt=0;
		gps_lane_=0;
		//

		get_csvs_fromDirectory();
		getNewWaypoints();
	}
	
//Functions in initSetup
	void get_csvs_fromDirectory() // /home/kuuve/data/ 안에 있는 여러개의 csv파일 이름을 벡터에 저장
	{
	
		DIR* dirp = opendir("/home/kuuve/data/"); //디렉토리 저장
		
		if (dirp == NULL){
			perror("UNABLE TO OPEN FOLDER");
			return;
		}

		struct dirent* dp; 
		while((dp = readdir(dirp)) != NULL){ //디렉토리 안의 파일 이름 저장
			string address("/home/kuuve/data/");

			string filename (dp->d_name);

			address.append(filename);
			if (filename.size() > 2){ //디렉토리 open시 ..과 .를 막기 위한 방어코드
				all_csv_.emplace_back(address);
				ROS_INFO("SAVED CSV");
			}
		} 
		
		sort(all_csv_.begin(),all_csv_.end()); //정렬
		closedir(dirp);
	}
	//AD
	void AD_delivery()
	{	
		if(lane_number_==0){	
			new_waypoints_.clear();
			new_waypoints_.assign(all_new_waypoints_[lane_number_].begin(), all_new_waypoints_[lane_number_].end());
			size_ = lane_size_[lane_number_];
			ex_lane_number_ = lane_number_;
		}
		else if (lane_number_ ==1)
		{		
				new_waypoints_.clear();
				new_waypoints_.assign(all_new_waypoints_[lane_number_].begin(), all_new_waypoints_[lane_number_].end());
				size_ = lane_size_[lane_number_];
				ex_lane_number_ = lane_number_;
			}
		else if (lane_number_ ==2)
		{		
				new_waypoints_.clear();
				new_waypoints_.assign(all_new_waypoints_[lane_number_].begin(), all_new_waypoints_[lane_number_].end());
				size_ = lane_size_[lane_number_];
				ex_lane_number_ = lane_number_;
			}
	}
	//AD

	void getNewWaypoints() { // 가져온 여러개의 CSV내 데이터를 벡터에 저장
	/*
		정리하고자 하는 벡터
		all_new_waypoints_ : Waypoint 모음 벡터
		all_state_index_ : mission state 모음 벡터
		lane_size_ : Vector 길이 모음 벡터
	*/

		string str_buf;
		int pos;
		vector<int> state_index_;
		vector<waypoint_maker::Waypoint> temp_new_waypoints;
		waypoint_maker::Waypoint temp_waypoint;

		for(auto i = 0; i < all_csv_.size(); i++){ //파일 갯수만큼

			state_index_.emplace_back(5); // current_mission_state 0번 시작 : 반드시 5 
										  // 이유 : getClosestWaypoint()의 조사 시작 범위를 위해
										  // ctrl+f --> void getClosestWaypoint()
						
			is_.open(all_csv_[i]); //파일 열기

			cout << "OPEN CSV" << all_csv_[i] << endl;
			
			temp_waypoint.mission_state = 0;
			// 이 부분 내용 알아보기
			while(getline(is_, str_buf)) {//파일 내용을 str_buf에 저장

				if(str_buf != "") { // temp_waypoint = [index, x, y, mission_state]
					pos = str_buf.find(",");
					temp_waypoint.waypoint_index = stoi(str_buf.substr(0, pos));

					str_buf = str_buf.substr(++pos);
					pos = str_buf.find(","); 
					temp_waypoint.pose.pose.position.x = stof(str_buf.substr(0, pos));

					str_buf = str_buf.substr(++pos);
					pos = str_buf.find(",");
					temp_waypoint.pose.pose.position.y = stof(str_buf.substr(0, pos));

					str_buf = str_buf.substr(++pos);
					pos = str_buf.find(",");

					if(temp_waypoint.mission_state != stof(str_buf.substr(0, pos))) //mission state 변화시 따로 저장
						state_index_.emplace_back(temp_waypoint.waypoint_index);
					temp_waypoint.mission_state = stof(str_buf.substr(0, pos));
					
					temp_new_waypoints.emplace_back(temp_waypoint);

				}
			}
			is_.close();
	
			size_ = temp_new_waypoints.size();
			lane_size_.emplace_back(size_); //lane_size 정리
				
			ROS_INFO("%d WAYPOINTS HAVE BEEN SAVED.", size_);

			all_new_waypoints_.emplace_back(temp_new_waypoints); //all_new_waypoints_ 정리
			all_state_index_.emplace_back(state_index_);//all_state_index_ 정리
		
			temp_new_waypoints.clear();
			state_index_.clear();
		}

		//0번 행 : 전체 경로
		//1번 행 ~  : 주차 경로
		new_waypoints_.assign(all_new_waypoints_[0].begin(), all_new_waypoints_[0].end());
		size_ = lane_size_[0];		
	}
	
	void poseProcess(){
		final_waypoints_.clear();
		
		//AD chanhee
		
		if(current_mission_state_==0)
		{
			if(cam_lane_in_maker_==2) lane_number_=1;
			if(cam_stop1_in_maker_==true) lane1_cnt=1; //왜냐면 cam_stop1_in_maker값은 한번 true이후 다시 false
			if(lane1_cnt==1) lane_number_=0;
		}

		else if(current_mission_state_==1)
		{	
			lane1_cnt=0; // 3.csv에 있는 0번 미션구간 때메 한 번 털리는걸 초기화
			lane_number_=0;
		}
		else if(current_mission_state_==2)
		{
			if(cam_lane_in_maker_==3) lane_number_=2;
			if(cam_stop2_in_maker_==true) lane2_cnt=1; 
			if(lane2_cnt==1) lane_number_=0;
		}

		// if(cam_lane_in_maker_==2){ 
		// lane_number_=1;
		// }
		// else if(cam_lane_in_maker_==3){ 
		// lane_number_=2;
		// }
		// else if(cam_lane_in_maker_==-1)
		// lane_number_=0;
		
		// if(cam_stop1_in_maker_==true) lane1_cnt=1;
		// else if(cam_stop2_in_maker_==true) lane2_cnt=1;
		// if(lane1_cnt==1 || lane2_cnt==1) lane_number_=0;
		cout<<"final my lane number is : "<<lane_number_<<endl;
		AD_delivery();
		//AD

		//parkingLaneSet();
		getClosestWaypoint(current_pose);//내 위치 기준 closest waypoint 찾기
		getFinalWaypoint();//최종 50개의 waypoint 결정
		/*
			parking_state_(default : -3)

			-2 부터 주차 시작, 주차시 mission_state는 parking_state_에 의해 결정
			mission_state_ = parking_state_ + 18 이 되도록 설정

			parking_state_: -3 ~ 1			mission_state_
			-3 : 평시 주행							15
			-2 : 주차공간 인지		   				 16
			-1 : 감속하며 주차 시작	    			  17
			 0 : 후진 주차					  		18
			 1 : 복귀 전진 					 		19
		*/

		private_nh_.getParam("/waypoint_loader_node/parking_count",parking_state_);//주차용 변수 설정

		//current_mission_state_ 결정
		if(parking_state_ >= -2){ //주차시 mission_state_
			current_mission_state_ = parking_state_ + 18;
		}
		else{//평상시
			current_mission_state_ = final_waypoints_[0].mission_state;
		}
		

		if(!is_state_ && closest_waypoint_candidates.empty()){
		    current_mission_state_ = state_inspection_;
		}//중간에서 실행시 current_mission_state_ 수신하지 못하는 오류 방어 코드 -> 원인 아직 파악 못함 -> 로직 수정 필요
		closest_waypoint_candidates.clear();
	

		//dist_ 결정
		dist_ = calcPlaneDist(current_pose, new_waypoints_[all_state_index_[lane_number_][current_mission_state_+1]].pose);
		//현위치와 다음 mission_state 간의 거리 (각종 미션에 사용)
			

		state_msg_.dist = dist_;
		state_msg_.current_state = current_mission_state_;
		state_msg_.lane_number = lane_number_;
		state_pub_.publish(state_msg_);//미션 관련 정보 publish
		

		lane_msg_.waypoints = final_waypoints_;
		lane_msg_.onlane = is_onlane_;
		
		waypoint_pub_.publish(lane_msg_);//waypoint publish
		
		int final_waypoints_size = lane_msg_.waypoints.size();
		ROS_INFO("FINAL WAYPOINTS NUMBER=%d PUBLISHED.", final_waypoints_size);	
			
	}

	//Functions in poseProcess
	/*
		parkingLaneSet()
		주차를 위한 경로 교체
		default lane : 0
		if lane_number changed, change the new_waypoints_
	*/
	void parkingLaneSet(){

		ROS_INFO("parkingLaneSet INITIALIZED.");

		private_nh_.getParam("/waypoint_loader_node/parking_count",parking_state_);

		if(lane_number_ != ex_lane_number_){
			if(parking_state_ >= -2){//주차 공간 확정시 경로 변경
				private_nh_.setParam("/waypoint_follower_node/lane_final", lane_number_);
				new_waypoints_.clear();
				new_waypoints_.assign(all_new_waypoints_[lane_number_].begin(),all_new_waypoints_[lane_number_].end());
				size_ = lane_size_[lane_number_];
				ex_lane_number_ = lane_number_;
			}
		}
	}

	/*
		getClosestWaypoint()
		1. 후보군 선정
		2. 최종 cloest_waypoint 선정
	*/
	void getClosestWaypoint(geometry_msgs::PoseStamped current_pose) {
		
		//1. 후보군 선정
		//조사 시작 범위 : (현 mission state 시작점) - 4  ~ (현 mission state + 2)번 구역까지
		//조사시 mission_state 
		/*  ex)
			current mission state 3번 : 67 ~ 190 이라면
			조사 시작 범위 = 63 ~

			예외)
			current mission state 0번 : 0 ~ 
			조사 시작 범위 = -4 ~
			-> 오류 발생

			해결 : 	state_index_.emplace_back(5)
				-> current mission state 0번 : 5 ~
				   조사 시작 범위 : 1 ~
		*/
		int t_index = all_state_index_[lane_number_][current_mission_state_]-4;
		cout << "search_startIndex : " << t_index << endl;

		for(int i = t_index; i < size_; i++) {
			float dist = calcPlaneDist(current_pose, new_waypoints_[i].pose);
			int t_state_check = new_waypoints_[i].mission_state - current_mission_state_;
			//주차시
			if(parking_state_ >= -2){
				if(dist < max_search_dist_ && (new_waypoints_[i].mission_state == parking_state_+18)) {
					closest_waypoint_candidates.emplace_back(i);
				}
			}//평시
			else if (dist < max_search_dist_ && (t_state_check == 2 || t_state_check == 1 || t_state_check == 0)){
				closest_waypoint_candidates.emplace_back(i);
			}
			
			if(dist > max_search_dist_ && t_state_check > 2) break;
		}
		
		//2. 후보 중 최종 cloest_waypoint 선정
		if(!closest_waypoint_candidates.empty()) {
			int waypoint_min = -1;
			float dist_min = max_search_dist_; 
			for(int i=0;i<closest_waypoint_candidates.size();i++) {
				float dist = calcPlaneDist(current_pose, new_waypoints_[closest_waypoint_candidates[i]].pose);
				if(dist < dist_min) {
					dist_min = dist;
					waypoint_min = closest_waypoint_candidates[i];
				}
			}

			closest_waypoint_ = waypoint_min;
			
			ROS_INFO("CLOSEST WAYPOINT INDEX=%d, X=%f, Y=%f", closest_waypoint_, new_waypoints_[closest_waypoint_].pose.pose.position.x, new_waypoints_[closest_waypoint_].pose.pose.position.y);
			is_state_ = true;//middle start
		}
		else ROS_INFO("THERE IS NO CLOSEST WAYPOINT CANDIDATE.");

		
	}

	// getFinalWaypoint()
	//최종 50개의 waypoint 선정
	void getFinalWaypoint(){ 
		if((size_ - closest_waypoint_ - 1) < final_size_) final_size_ = size_ - closest_waypoint_ - 1;
		
	 	for(int i=0;i<final_size_;i++) {
			final_waypoints_.emplace_back(new_waypoints_[closest_waypoint_ + i]);
		}
	}
	
	//Function in getClosetWaypoint()
	float calcPlaneDist(const geometry_msgs::PoseStamped pose1, const geometry_msgs::PoseStamped pose2) {
		return sqrtf(powf(pose1.pose.position.x - pose2.pose.position.x, 2) + powf(pose1.pose.position.y - pose2.pose.position.y, 2));
	}  

	//Callback function
	void poseCallback(const nav_msgs::Odometry::ConstPtr &msg) {
		
		current_pose.pose.position = msg->pose.pose.position;
		lane_msg_.header = msg->header;	
	
	}

	//AD
	void CDCallback(const camera_delivery::CD::ConstPtr &CD_msg_)
	{
		cam_lane_in_maker_ = CD_msg_->cam_lane; // 트리거
		cam_stop1_in_maker_ = CD_msg_->cam_stop1;
		cam_stop2_in_maker_=CD_msg_->cam_stop2;
	}
	//
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "waypoint_loader"); //노드명 -> 생성자 void init setup
	WaypointLoader wl;//생성자 void init setup
	ros::Rate loop_rate(10);
	int cnt=0;
	int finish=0;
	
	while(ros::ok()) {

		ros::spinOnce();
		wl.poseProcess(); 
		loop_rate.sleep();	
	}
	return 0;
}
