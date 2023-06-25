#include <ros/ros.h>
#include <waypointFollower.h>
#include <new_platform_control.h>



/*
	pure pursuit 알고리즘

	1. 목표 추종점 찾기
		1-1 50개의 waypoint와 cur_pose 간의 거리 계산
		1-2 dist > lookahead_dist_ 될 때의 waypoint를 추종점으로
		1-3 계산에 사용할 최종 ld 설정

	*  후진에서 전진 or 전진에서 후진 전환시 heading 방향 반대로 돌려주기

	2. alpha 구하기
		2-1 atan2로 temp_theta 계산 (temp_theta = 목표 heading, atan2 좌표계)
		2-2 atan2-절대좌표계 통일
		2-3 alpha 계산 (목표 heading - 현재 heading)

	3. mission 별 코드
		3-1 큰 정적 미션
		3-2 배달 미션
		3-3 주차 미션

	4. 최종조향각

*/

float WaypointFollower::calcSteeringAngle()
{
	getClosestWaypoint(cur_pose_);

	lookahead_dist_ = cur_speed_ * ld_ratio + 1.0;

	// 1. 목표추종점 찾기
	float final_ld;
	for (int i = 0; i < waypoints_size_; i++)
	{
		float dist = calcPlaneDist(cur_pose_, waypoints_[i].pose);
		if (dist > lookahead_dist_)
		{
			target_index_ = i;
			final_ld = dist; // 최종으로 사용할 ld
			break;
		}
	}

	// 후진에서 전진 or 전진에서 후진 전환시 heading 방향 반대로 돌려주기
	// 전환시 최대 조향각이 들어가는 오류 방지
	if (is_backward_ || is_parking_finished_)
		cur_course_ += cur_course_ < 180.0f ? 180.0f : -180.0f;






	// 2. alpha 구하기
	// temp_theta 계산
	
	double target_x = waypoints_[target_index_].pose.pose.position.x;
	double target_y = waypoints_[target_index_].pose.pose.position.y;
	double cur_x = cur_pose_.pose.position.x;
	double cur_y = cur_pose_.pose.position.y;
	double dx = (target_x - cur_x);
	double dy = (target_y - cur_y);

	float temp_theta = ((float)(atan2(dy, dx))) * 180.0f / M_PI; // temp theta : 동쪽 기준, 반시계 방향, -π ~ π

	// atan2 - 절대좌표계 통일 -> temp theta : 진북 기준, 시계 방향, 0 ~ 2π
	temp_theta = temp_theta <= 90.0f ? 90.0f - temp_theta : 450.0f - temp_theta;

	// alpha 계산
	float deg_alpha = (temp_theta - cur_course_);
	deg_alpha += deg_alpha <= -180.0f ? 360.0f : deg_alpha > 180.0f ? -360.0f
																	: 0;

	float alpha = deg_alpha * M_PI / 180.0f;

	// 3. 미션별 코드
	// 3-1 큰 정적 미션
	if (current_mission_state_ == 3)
	{
		float min_dist = 12.0f;
		if (!is_static_ && is_obs_detect_)
		{
			for (int i = 0; i < waypoints_size_; i++)
			{
				double temp_dist = calcPlaneDist(cur_pose_, waypoints_[i].pose);
				double gap = abs(obs_dist_ - temp_dist);
				if (gap < min_dist)
				{
					min_dist = gap;
					obs_waypoint_index_ = waypoints_[i].waypoint_index;
				}
			}
			is_static_ = true;
		}
		offset_ = 2.4f;
		if (waypoints_[closest_waypoint_].waypoint_index - obs_waypoint_index_ < 9 && waypoints_[closest_waypoint_].waypoint_index - obs_waypoint_index_ > -16 && is_static_)
			alpha = atanf((lookahead_dist_ * sinf(alpha) + offset_) / (lookahead_dist_ * cosf(alpha))); // 8 -18
	}

	// 3-2 배달 미션
	delivery_offset_A1 = 1.65f;
	delivery_offset_B = 1.65f;
	if (current_mission_state_ == 4)
	{
		if (is_delivery_offset_)
		{
			alpha = atanf((lookahead_dist_ * sinf(alpha) + delivery_offset_A1) / (lookahead_dist_ * cosf(alpha)));
		}
	}

	else if (current_mission_state_ == 12)
	{
		if (is_delivery_offset_)
		{
			alpha = atanf((lookahead_dist_ * sinf(alpha) + delivery_offset_B) / (lookahead_dist_ * cosf(alpha)));
		}
	}

	// 3-3 주차 미션
	// 주차 미션에 알맞는 ld값 재설정
	if (17 <= current_mission_state_ && current_mission_state_ <= 19)
	{
		minimum_ld = 1.5f;
		final_ld = cur_speed_ * 0.55;
	}
	else if (current_mission_state_ >= 20)
	{
		minimum_ld = 3.0f;
		final_ld = cur_speed_ * 0.55;
	}

	// 4. 최종 조향각
	if (final_ld > maximum_ld)
		final_ld = maximum_ld;
	if (final_ld < minimum_ld)
		final_ld = minimum_ld;

	float cur_steer = atan2f(2.0f * wheel_base_ * sinf(alpha) / (final_ld), 1.0f);
	cur_steer = cur_steer * 180.0f / M_PI;

	
	
	// 	if (cur_steer >= 14.99f)

	
	if (cur_steer >= 29.99f)
	cur_steer = 29.99f;
	if(cur_steer<=-29.99f)
	cur_steer = -29.99f;

	return cur_steer;
}
