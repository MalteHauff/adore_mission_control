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

#include "mission_control.hpp"
#include "rclcpp/rclcpp.hpp"

int main( int argc, char* argv[] )
{
  rclcpp::init( argc, argv );
  auto node = std::make_shared<adore::MissionControl>( rclcpp::NodeOptions{} );
  rclcpp::spin( node );
  rclcpp::shutdown();
  return 0;
}

