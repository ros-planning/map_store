/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
behavior:
 - listens on "map" topic.  On each map:
   - id_of_most_recent_map = Collection.publish(map, {session ID, map name})

service calls:
 - name latest map (map name) returns void
   - add_metadata() service call to set the map name of the map with id_of_most_recent_map.
   - store map name in variable.
 */

#include <warehouse/warehouse_client.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_store/NameLatestMap.h>

#include <string>
#include <sstream>

#include <uuid/uuid.h>

namespace wh=warehouse;

std::string map_name;
std::string id_of_most_recent_map;
uint64_t session_id;
wh::Collection<nav_msgs::OccupancyGrid> map_collection;
ros::ServiceClient add_metadata_service_client;

void onMapReceived(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
  ROS_INFO("received map");

  uuid_t uuid;
  uuid_generate(uuid);
  char uuid_string[37]; // UUID prints into 36 bytes + NULL terminator
  uuid_unparse_lower(uuid, uuid_string);

  std::string metadata = wh::MetadataString()
    .add("uuid", std::string(uuid_string))
    .add("session_id", session_id)
    .add("name", map_name);

  id_of_most_recent_map = map_collection.publish(*map_msg, metadata);

  ROS_INFO("saved map");
}

bool nameLatestMap(map_store::NameLatestMap::Request &req,
                   map_store::NameLatestMap::Response &res)
{
  map_name = req.map_name;

  // If there is no latest map, fail.
  if( id_of_most_recent_map == std::string() )
  {
    ROS_ERROR("nameLatestMap(%s): There is no latest map.", map_name.c_str());
    return false;
  }

  wh::Metadata srv;
  srv.request.db_name = "my_app";
  srv.request.collection_name = "maps";
  srv.request.id = id_of_most_recent_map;
  srv.request.metadata = wh::MetadataString().add("name", map_name.c_str());

  ROS_INFO("trying to give name '%s' to map id '%s'.", map_name.c_str(), id_of_most_recent_map.c_str());

  if( add_metadata_service_client.call(srv) )
  {
    ROS_INFO("named a map '%s'", map_name.c_str());
    return true;
  }
  else
  {
    ROS_ERROR("metadata service returned error code %d: '%s'", srv.response.error_code, srv.response.error_msg.c_str());
    ROS_ERROR("failed to name a map '%s'", map_name.c_str());
    return false;
  }
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "map_saver");
  ros::NodeHandle nh;

  map_name = "";
  id_of_most_recent_map = "";

  // Use the current ROS time in seconds as the session id.
  session_id = (uint64_t) ros::Time::now().toSec();

  wh::WarehouseClient client("my_app");

  map_collection = client.setupCollection<nav_msgs::OccupancyGrid>("maps");

  ros::Subscriber map_subscriber = nh.subscribe("map", 1, onMapReceived);

  ros::ServiceServer name_latest_map_service = nh.advertiseService("name_latest_map", nameLatestMap);

  add_metadata_service_client = nh.serviceClient<wh::Metadata>("add_metadata");

  ROS_INFO("spinning.");

  ros::spin();
  return 0;
}
