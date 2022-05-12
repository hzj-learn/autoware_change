/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/console.h>

#include "autoware_config_msgs/ConfigLaneStop.h"
#include "autoware_msgs/TrafficLight.h"
#include "autoware_msgs/LaneArray.h"

#include <lane_planner/lane_planner_vmap.hpp>

namespace {

bool config_manual_detection = true;

ros::Publisher traffic_pub;

autoware_msgs::LaneArray current_red_lane;
autoware_msgs::LaneArray current_green_lane;

const autoware_msgs::LaneArray *previous_lane = &current_red_lane;


//note-tianyu 根据不同的信号灯情况来切换不同的lane
void select_current_lane(const autoware_msgs::TrafficLight& msg)
{
  const autoware_msgs::LaneArray *current;
  
  switch (msg.traffic_light) {
  case lane_planner::vmap::TRAFFIC_LIGHT_RED:
    current = &current_red_lane;
    break;
  case lane_planner::vmap::TRAFFIC_LIGHT_GREEN:
    current = &current_green_lane;
    break;
  case lane_planner::vmap::TRAFFIC_LIGHT_UNKNOWN:
    current = previous_lane; // if traffic light state is unknown, keep previous state
    break;
  default:
    ROS_ERROR_STREAM("undefined traffic light");
    return;
  }

  if (current->lanes.empty()) {
    ROS_ERROR_STREAM("empty lanes");
    return;
  }

  traffic_pub.publish(*current);

  previous_lane = current;
}

void receive_auto_detection(const autoware_msgs::TrafficLight& msg)
{
  if (!config_manual_detection)
    select_current_lane(msg);//根据交通灯切换不同衍射的路线
}

void receive_manual_detection(const autoware_msgs::TrafficLight& msg)
{
  if (config_manual_detection)
    select_current_lane(msg);
}

void cache_red_lane(const autoware_msgs::LaneArray& msg)
{
  current_red_lane = msg;
}

void cache_green_lane(const autoware_msgs::LaneArray& msg)
{
  current_green_lane = msg;
}

void config_parameter(const autoware_config_msgs::ConfigLaneStop& msg)
{
  config_manual_detection = msg.manual_detection;
}

} // namespace

///light_color相当于是一个 开关，切换red_waypoints_array和green_waypoints_array
int main(int argc, char **argv)
{
  ros::init(argc, argv, "lane_stop");

  ros::NodeHandle n;

  int sub_light_queue_size;
  n.param<int>("/lane_stop/sub_light_queue_size", sub_light_queue_size, 1);
  int sub_waypoint_queue_size;
  n.param<int>("/lane_stop/sub_waypoint_queue_size", sub_waypoint_queue_size, 1);
  int sub_config_queue_size;
  n.param<int>("/lane_rule/sub_config_queue_size", sub_config_queue_size, 1);
  int pub_waypoint_queue_size;
  n.param<int>("/lane_stop/pub_waypoint_queue_size", pub_waypoint_queue_size, 1);
  bool pub_waypoint_latch;
  n.param<bool>("/lane_stop/pub_waypoint_latch", pub_waypoint_latch, true);
  n.param<bool>("/lane_stop/manual_detection", config_manual_detection, true);

//发布要走的路径
  traffic_pub = n.advertise<autoware_msgs::LaneArray>("/traffic_waypoints_array", pub_waypoint_queue_size,
                pub_waypoint_latch);

//订阅交通灯的信息
  ros::Subscriber light_sub = n.subscribe("/light_color", sub_light_queue_size, receive_auto_detection);
  ros::Subscriber light_managed_sub = n.subscribe("/light_color_managed", sub_light_queue_size,
              receive_manual_detection);
//订阅红灯和绿灯要走的不同速度的轨迹
  ros::Subscriber red_sub = n.subscribe("/red_waypoints_array", sub_waypoint_queue_size, cache_red_lane);
  ros::Subscriber green_sub = n.subscribe("/green_waypoints_array", sub_waypoint_queue_size, cache_green_lane);
  ros::Subscriber config_sub = n.subscribe("/config/lane_stop", sub_config_queue_size, config_parameter);

  ros::spin();

  return EXIT_SUCCESS;
}
