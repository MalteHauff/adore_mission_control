/********************************************************************************
 * Copyright (c) 2025 Contributors to the Eclipse Foundation
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#include "mission_control_node.hpp"

#include <type_traits>
using namespace std::chrono_literals;

namespace adore
{

MissionControlNode::MissionControlNode( const rclcpp::NodeOptions& options ) :
  Node( "mission_control", options )
{
  load_parameters();
  road_map = std::make_shared<map::Map>( map::MapLoader::load_from_file( map_file_location ) );
  create_publishers();
  create_subscribers();
}

void
MissionControlNode::create_publishers()
{
  route_publisher         = create_publisher<RouteAdapter>( "route", 10 );
  local_map_publisher     = create_publisher<MapAdapter>( "local_map", 10 );
  goal_reached_publisher  = create_publisher<std_msgs::msg::Bool>( "goal_reached", 10 );
  publisher_caution_zones = create_publisher<adore_ros2_msgs::msg::CautionZone>( "caution_zones", 10 );
  drive_back_subscriber = create_subscription<std_msgs::msg::Bool>(
  "mission/drive_back_to_start", 10,
  std::bind(&MissionControlNode::drive_back_to_start_callback, this, std::placeholders::_1));

}

void MissionControlNode::drive_back_to_start_callback(const std_msgs::msg::Bool& msg)
{
  if (msg.data && start_goal.has_value())
  {
    Goal drive_back_goal;
    drive_back_goal.x = start_goal->x;
    drive_back_goal.y = start_goal->y;
    drive_back_goal.label = "drive back to start";
    goals.push_front(drive_back_goal);
    current_route = std::nullopt; // Force route recalculation to the new goal
    RCLCPP_INFO(get_logger(), "Received drive back command. Added goal to drive back to start point at (%.2f, %.2f)", drive_back_goal.x, drive_back_goal.y);
  }
}


void
MissionControlNode::update_route()
{
  if( current_route.has_value() && latest_vehicle_state.has_value() )
  {
    if( current_route->get_length() - current_route->get_s( latest_vehicle_state.value() ) < 0.5 )
      reach_goal();
  }
  if( !current_route && latest_vehicle_state && !goals.empty() && road_map )
  {
    auto route = map::Route( latest_vehicle_state.value(), goals.front(), road_map );
    if( !route.reference_line.empty() )
      current_route = route;
  }
}

void
MissionControlNode::reach_goal()
{
  std_msgs::msg::Bool reached;
  reached.data = true;
  goal_reached_publisher->publish( reached );
  if( !goals.empty() )
    goals.pop_front();
  current_route = std::nullopt;
}

void
MissionControlNode::create_subscribers()
{
  keep_moving_subscriber = create_subscription<adore_ros2_msgs::msg::GoalPoint>( "mission/goal_request", 10,
                                                                                 std::bind( &MissionControlNode::keep_moving_callback, this,
                                                                                            std::placeholders::_1 ) );

  vehicle_state_subscriber = create_subscription<StateAdapter>( "vehicle_state_dynamic", 10,
                                                                std::bind( &MissionControlNode::vehicle_state_callback, this,
                                                                           std::placeholders::_1 ) );

  clicked_point_subscriber = create_subscription<geometry_msgs::msg::PointStamped>( "/clicked_point/goal_position", 10,
                                                                                    std::bind( &MissionControlNode::clicked_point_callback,
                                                                                               this, std::placeholders::_1 ) );

  main_timer = create_wall_timer( 100ms, std::bind( &MissionControlNode::timer_callback, this ) );
}

void
MissionControlNode::load_parameters()
{
  // Load parameters directly
  Goal initial_goal;

  local_map_size     = declare_parameter<double>( "local_map_size", 50.0 );
  initial_goal.x     = declare_parameter<double>( "goal_position_x", 0.0 );
  initial_goal.y     = declare_parameter<double>( "goal_position_y", 0.0 );
  initial_goal.label = "goal from launch file";
  goals.push_back( initial_goal );

  map_file_location = declare_parameter<std::string>( "map file", "" );

  std::vector<double> ra_polygon_values; // request assistance polygon
  ra_polygon_values = declare_parameter( "request_assistance_polygon", ra_polygon_values );

  // Convert the parameter into a Polygon2d
  if( ra_polygon_values.size() >= 6 ) // minimum 3 x, 3 y
  {
    adore::math::Polygon2d polygon( ra_polygon_values );
    caution_zones["Request Assistance"] = polygon;
  }
}

void
MissionControlNode::timer_callback()
{
  update_route();
  publish_local_map();
  publish_caution_zones();
}

void
MissionControlNode::publish_local_map()
{
  if( !road_map || !latest_vehicle_state.has_value() )
    return;

  auto local_map_ptr = std::make_shared<map::Map>( road_map->get_submap( latest_vehicle_state.value(), local_map_size, local_map_size ) );
  local_map_publisher->publish( *local_map_ptr );

  if( current_route.has_value() )
  {
    auto local_route = current_route; // copy your optional (as you do now)
    local_route->map = local_map_ptr; // share, don’t copy
    route_publisher->publish( *local_route );
  }
  else
  {
    // send empty route anyway
    map::Route empty;
    route_publisher->publish( empty );
  }
}

void
MissionControlNode::keep_moving_callback( const adore_ros2_msgs::msg::GoalPoint& msg )
{
  Goal keep_moving_goal;
  keep_moving_goal.label = "keep moving goal";
  keep_moving_goal.x     = msg.x_position;
  keep_moving_goal.y     = msg.y_position;
  if( !goals.empty() )
    goals.front() = keep_moving_goal;
  else
    goals.push_front( keep_moving_goal );

  current_route = std::nullopt;
}

void
MissionControlNode::clicked_point_callback( const geometry_msgs::msg::PointStamped& msg )
{
  Goal keep_moving_goal;
  keep_moving_goal.label = "custom set goal";
  keep_moving_goal.x     = msg.point.x;
  keep_moving_goal.y     = msg.point.y;
  goals.push_front( keep_moving_goal );
  
}

void
MissionControlNode::vehicle_state_callback( const dynamics::VehicleStateDynamic& msg )
{
  latest_vehicle_state = msg;
  if(!start_goal.has_value())
  {
    Goal initial_goal;
    initial_goal.x     = msg.x;
    initial_goal.y     = msg.y;
    initial_goal.label = "initial position";
    start_goal = initial_goal;
    RCLCPP_INFO(get_logger(), "Stored start point at (%.2f, %.2f)", initial_goal.x, initial_goal.y);
  }
}

void
MissionControlNode::publish_caution_zones()
{
  for( const auto& [label, polygon] : caution_zones )
  {
    adore_ros2_msgs::msg::CautionZone caution_zone_msg;
    caution_zone_msg.label           = label;
    caution_zone_msg.polygon         = math::conversions::to_ros_msg( polygon );
    caution_zone_msg.header.frame_id = "world";
    publisher_caution_zones->publish( caution_zone_msg );
  }
}

} // namespace adore

int
main( int argc, char* argv[] )
{
  rclcpp::init( argc, argv );
  auto node = std::make_shared<adore::MissionControlNode>( rclcpp::NodeOptions{} );
  rclcpp::spin( node );
  rclcpp::shutdown();
  return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE( adore::MissionControlNode )
