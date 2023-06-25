#include <ros/ros.h>
#include <waypointFollower.h>
#include <new_platform_control.h>

/*
	Process Algorithm

	1. 위치 수신, Heading 수신, 경로 수신 확인
	2. mission_state별 제어
	3. 주차 미션시 제어
	4. 제어에 따른 속도 및 조향각 결정
*/

void WaypointFollower::process()
{

	// 1. 위치 수신, Heading 수신, 경로 수신 확인
	if (is_pose_ && is_course_ && is_lane_)
	{
		is_control_ = true; // control 작동
		cout<<"cam_lane_sub: "<<cam_lane_<<"  cam stop1 flag: "<<cam_stop1_<<"cam stop2 flag: "
		<<cam_stop2_<<endl;
		// 2. mission_state별 제어
		switch (current_mission_state_) // csv에서 받아오는 mision_state
		{
		case 0: // 배달
			spd_state_ = 5;
			//AD
			if(cam_lane_==2 && lane1_stop_cnt==0) spd_state_=3;
			//AD
			
			// spd_state_ = (dist_ >= 20.0f) ? 10 : (!vision_check_ && dist_ < 3.0f) ? 0 : 4;
			break;
		case 1: // 
			spd_state_ = 6;
		
		case 2: // 배달
			spd_state_ = 5;
			//AD
			if(cam_lane_==3 && lane2_stop_cnt==0) spd_state_=3;

			break;
			// 일단 신호 들어오면 저속주행
		// 	if (lidar_static_ && lidar_dynamic_)
		// 	{
		// 		spd_state_ = 3;
		// 	}
		// 	if (lidar_dynamic_ && lidar_warning_)
		// 	{
		// 		spd_state_ = 0;
		// 	}
		// 	if (lidar_static_ && lidar_warning_)
		// 	{
		// 		spd_state_ = 2;
		// 	}
		// 	break;
		// case 2: // 마지막 전 코너 도는 곳
		// 	spd_state_ = 3;
		// 	break;
		// case 3: // 직진 정동적
		// 	spd_state_ = 4;
		// 	// 일단 신호 들어오면 저속주행
		// 	if (lidar_static_ && lidar_dynamic_)
		// 	{
		// 		spd_state_ = 3;
		// 	}
		// 	if (lidar_dynamic_ && lidar_warning_)
		// 	{
		// 		spd_state_ = 0;
		// 	}
		// 	if (lidar_static_ && lidar_warning_)
		// 	{
		// 		spd_state_ = 2;
		// 	}
		// 	break;
		// case 4: // 마무리
		// 	spd_state_ = 3;
		// 	break;
		/*

		case 1:
			spd_state_ = 3;
			// 첫번째 신호등 ~ 두번째 신호등
		case 13:	// 고속 주행 ~ 신호등
		case 14:	//신호등 ~ 신호등
			spd_state_ = (dist_ >= 20.0f) ? 9 : (!vision_check_ && dist_ < 3.0f) ? 0 : 4;
			break;
		case 2:		//두번째 신호등 ~ 큰정적 진입 전
		case 20:	//달리기
			spd_state_ = 9;
			break;
		case 3:		//큰정적 진입 ~ 큰 정적 이후 신호등
			spd_state_ = (dist_ >= 20.0f) ? 5 : (!vision_check_ && dist_ < 3.0f) ? 0 : 4;
			break;
		case 4:
			spd_state_ = delivery_stop_ ? 0 : is_delivery_offset_ ? 2 : delivery_ready_ ? 4 : 8;
			break;
		case 5:		//배달 A 횡단보도 전 ~ 차선 번경
			spd_state_ = (dist_ >= 20.0f) ? 6 : 4;
			break;
		case 6: // 차선 변경 ~ 교차로 신호등
		case 8:	// U턴 전 신호등
			spd_state_ = (dist_ >= 20.0f) ? 6 : (!vision_check_ && dist_ < 3.0f) ? 0 : 4;
			break;
		case 7:		// 교차로 신호등 이후 코너
			spd_state_ = 6;
			break;
		case 9:		//U턴
			spd_state_ = 4;
			break;
		case 10:	// 횡단보도 정지 1구역
			if(dist_ < 3.0f && !is_cross_walk_finish1)
			{
				spd_state_ = 0;
				if(!walk1_)
				{
					ros::Time::init();
					start_ = ros::Time::now();
					walk1_ = true;
					is_PurePursuit_ = false;
				}
				else
				{
					end_ = ros::Time::now();
					sec_ = end_ - start_;
					is_cross_walk_finish1 = is_PurePursuit_ = sec_.toSec() > 5.0;
				}
			}
			else
				spd_state_ = 4;
			break;
		case 11:	 // 횡단보도 정지 2구역
			if(dist_ < 20.0f)
			{
				if(dist_ < 3.0f && !is_cross_walk_finish2)
				{
					spd_state_ = 0;
					if(!walk2_)
					{
						ros::Time::init();
						start_2 = ros::Time::now();
						walk2_ = true;
						is_PurePursuit_ = false;
					}
					else
					{
						end_2 = ros::Time::now();
						sec_2 = end_2 - start_2;
						is_cross_walk_finish2 = is_PurePursuit_ = sec_2.toSec() > 5.0;
					}
				}
				else
					spd_state_ = 4;
			}
			else
				spd_state_ = 6;
			break;
		case 12:	//배달 B 구역 ~ 신호등
			spd_state_ = ((!vision_check_ && dist_ < 3.0f) || delivery_stop_) ? 0 : is_delivery_offset_ ? 2 : (delivery_ready_ || dist_ < 20.0f) ? 4 : 6;
			break;
		case 15:	//신호등 ~ 주차_LIDAR
			spd_state_ = 9;
			if(dist_<=3.0)
				parking_count_ = -2;
			break;
		case 16:	// 주차_LIDAR 시작
			private_nh_.setParam("/waypoint_loader_node/parking_count", parking_count_);

			if(dist_ <= 19.0)
			{
				spd_state_ = 3;
				if(dist_ <= 14.0)//장애물 인지 시작
				{
					lazu_count_++;
					detection_trigger_ = true; 	//인지하는동안 속도 & 조향 제어권 박탈

					for(int i = 0; i < 2; i++)
						GPS_parking_dist_[i] = sqrt(pow(parking_spot_[i][0]-cur_pose_.pose.position.x,2) + pow(parking_spot_[i][1]-cur_pose_.pose.position.y,2));


					for(int i = 0; i < lidar_obj_dist.size(); i++)
					{
						for(int k = 0; k < 2; k++)
						{
							if(abs(GPS_parking_dist_[k] - lidar_obj_dist[i]) < 0.75)
							{
								donnot_parking_count_[k] = donnot_parking_count_[k] + 1;
								break;
							}
						}
					}
					//주차칸의 차량 유무에 따라 주차공간 판단 -> 기본값 3번칸
					if(keep_search_){ // 해당 주차칸의 차량의 유무에 따른 lane_change 판단 로직 ---- 디폴트 3번째 주차칸

						if(lazu_count_ >= 15) { // 첫번째 두번째칸 장애물을 합쳐서 15번 이상 탐지하면
							if(donnot_parking_count_[0] > 10 && donnot_parking_count_[1] > 7 )// 둘 다 차있으면
								parking_candidate_=0;
							else if (donnot_parking_count_[0] <=3 && donnot_parking_count_[1] <= 3)
								parking_candidate_ = 1; // 둘 다 비어있으면 -> 첫번째 칸에
							else {
								if(donnot_parking_count_[0] < donnot_parking_count_[1])
									parking_candidate_ = 1;	// 첫번째 칸이 두번째 칸보다 적게 감지되면 -> 첫번째 칸 비어있으면 -> 첫번째 칸에
								else parking_candidate_ = 2;
							}
						}
						keep_search_ = false;
					}
					else
					{
						lane_number_msg_.lane_number = parking_candidate_;
						lane_number_pub_.publish(lane_number_msg_);
					}

					if(loader_number_ == parking_candidate_ && lazu_count_ >= 15)
					{
						parking_count_ = -1;
						private_nh_.setParam("/waypoint_loader_node/parking_count", parking_count_);
						detection_trigger_ = false;
					}
				}
			}
			else
				spd_state_ = 4;
			break;
		case 17:	// 감속하면서 주차하러 들어가기
		case 18:	// 후진
		case 19:	// 복귀 전진
			break;*/
		default:
			spd_state_ = 4;
			break;
		}
	}
	else
	{
		cout << "process DIED" << endl;
	}
	/*
		parking_count_(default : -3)

		-2 부터 주차 시작
		mission_state_ = parking_state_ + 18 이 되도록 설정

		parking_count_: -3 ~ 1			mission_state_
		-3 : 평시 주행							15
		-2 : 주차공간 인지		   				 16
		-1 : 감속하며 주차 시작	    			  17
		 0 : 후진 주차					  		18
		 1 : 복귀 전진 					 		19
	*/

	// 3. 주차 미션시 제어
	switch (parking_count_)
	{
	case -1:
		// 거리 조건 충족시 is_backward_ 활성화
		// is_backward_ : 후진 시작시 한번만 활성화되는 변수, pure pursuit 참고
		if (dist_ <= 3.5f && current_mission_state_ == 17)
		{
			parking_count_ = 0;
			private_nh_.setParam("/waypoint_loader_node/parking_count", parking_count_);
			is_backward_ = true;
		}
		break;
	case 0: // 후진
		if (is_backward_)
			is_backward_ = false;

		if (dist_ < 5.5f)
		{
			is_control_ = false;
			if (dist_ <= 2.75f) // 후진 도중 주차 위치 도달시
			{
				input_speed_ = 0.0; // 정지
				ros::Time::init();
				start_ = ros::Time::now();
				is_PurePursuit_ = false; // 멈췄을 때 조향 제어 박탈
				parking_count_ = 1;
				private_nh_.setParam("/waypoint_loader_node/parking_count", parking_count_);
			}
			else // 후진
			{
				input_speed_ = -0.5f;
				input_steer_ = -25.0;
			}
		}
		break;
	case 1: // 12초간 정지 후 재출발
		end_ = ros::Time::now();
		sec_ = end_ - start_;

		if (sec_.toSec() <= 12.0)
		{
			input_speed_ = 0.0;
			input_steer_ = 0.0;
			is_control_ = false;
			is_PurePursuit_ = false;
		}
		else if (sec_.toSec() > 12.0 && sec_.toSec() <= 16.0)
		{
			input_speed_ = 2.0;
			input_steer_ = -28.0;
		}
		else if (!is_parking_finished_ && !is_ready_)
		{
			is_parking_finished_ = true;
			is_ready_ = true;
		}
		else
		{
			if (is_ready_)
				is_parking_finished_ = false;
			is_PurePursuit_ = true;
			if (dist_ <= 2.5f)
			{
				parking_count_ = -3;
				private_nh_.setParam("/waypoint_loader_node/parking_count", parking_count_);
			}
		}
		break;
	default:
		break;
	}

	// 4. 제어에 따른 속도 및 조향각 결정
	if (is_control_)
	{
		// 주차 후진할 때 천천히, 주차 재출발시 천천히
		input_speed_ = (parking_count_ == -1) ? 1.5f : (parking_count_ == 0) ? -0.5f
												   : (parking_count_ == 1)	 ? 2.0f
																			 : (spd_state_ * 0.8f);

		if (is_PurePursuit_ && !detection_trigger_)
		{	
			//AD
			if (cam_lane_==2 || cam_lane_==3)
				input_steer_ = delivery_calcSteeringAngle();
			else
				input_steer_ = calcSteeringAngle();
			//AD
		}
		// 인지하는동안 속도 & 조향 제어권 박탈
		if (detection_trigger_)
		{
			input_speed_ = 0.0f;
			input_steer_ = 0.0f;
		}

		//AD
		if(cam_stop1_==true && lane1_stop_cnt==0 && cam_lane_==2)
		{
		input_speed_ = 0.0;
		input_steer_ = 0.0;
		lane1_stop_cnt++;
		cout<<lane1_stop_cnt<<" <-1번레인 카운트 횟수, 멈출게요~"<<endl;					
		ros::Duration(3.0).sleep();	
		}
		if(cam_stop2_==true && lane2_stop_cnt==0 && cam_lane_==2)
		{
		input_speed_ = 0.0;
		input_steer_ = 0.0;
		lane2_stop_cnt++;						
		cout<<lane2_stop_cnt<<" <-2번레인 카운트 횟수, 멈출게요~"<<endl;					
		ros::Duration(3.0).sleep();	
		} 	
		//AD	
	}
	is_pose_ = false;
	is_course_ = false;
}
		
