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

#include <mongo_ros/message_collection.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_store/NameLatestMap.h>

#include <string>
#include <sstream>

#include <uuid/uuid.h>

namespace mr=mongo_ros;

std::string map_name;
std::string id_of_most_recent_map;
std::string session_id;
mr::MessageCollection<nav_msgs::OccupancyGrid> *map_collection;
ros::ServiceClient add_metadata_service_client;

std::string uuidGenerate() {
  uuid_t uuid;
  uuid_generate(uuid);
  char uuid_string[37]; // UUID prints into 36 bytes + NULL terminator
  uuid_unparse_lower(uuid, uuid_string);
  return std::string(uuid_string);
}

void onMapReceived(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
  ROS_DEBUG("received map");
  std::string uuid_string = uuidGenerate();
  mr::Metadata metadata
    = mr::Metadata("uuid", uuid_string,
		   "session_id", session_id);
  if (map_name != "") {
    metadata.append("name", map_name);
  }

  id_of_most_recent_map = uuid_string;
  map_collection->insert(*map_msg, metadata);

  ROS_DEBUG("saved map");
}

bool nameLatestMap(map_store::NameLatestMap::Request &req,
                   map_store::NameLatestMap::Response &res)
{
  map_name = req.map_name;
  std::vector<mr::MessageWithMetadata<nav_msgs::OccupancyGrid>::ConstPtr> 
    all_maps = map_collection->pullAllResults( mr::Query("session_id", session_id), false, "creation_time", false );

  ROS_DEBUG("%u maps in current session", (unsigned int)all_maps.size());
  
  // Loop over all_maps to get the first of each session.
  std::vector<mr::MessageWithMetadata<nav_msgs::OccupancyGrid>::ConstPtr>::const_iterator map_iter = all_maps.begin();
  if (map_iter == all_maps.end()) {
    ROS_ERROR("No maps to name");
    return false;
  }
  ROS_DEBUG("nameLastestMaps() reading a map");

  if ((*map_iter)->lookupString("name") == req.map_name) {
    ROS_DEBUG("Already named properly");
    return true;
  }

  //For now, this creates a new map of a given name by copying the old one.
  //It is a bit of a hack that will last until the ablity to update metadata
  //is implemented for the c++ client. See ticket 6 on the warehouse trac.
  //Ideally, we'd search for id_of_most_recent_map and set it to map_name
  std::string session_id = (*map_iter)->lookupString("session_id");
  std::string uuid_string = uuidGenerate();
  mr::Metadata metadata
    = mr::Metadata("uuid", uuid_string,
                   "session_id", session_id,
		   "name", map_name);

  id_of_most_recent_map = uuid_string;
  nav_msgs::OccupancyGridConstPtr map( all_maps[0] );
  ROS_DEBUG("Copy from %s", all_maps[0]->metadata.toString().c_str());
  ROS_DEBUG("Save map %d by %d @ %f as %s", map->info.width, map->info.height, map->info.resolution, map_name.c_str());
  map_collection->insert(*map, metadata);
  
  ROS_DEBUG("nameLastestMaps() service call done");
  return true;
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "map_saver");
  ros::NodeHandle nh;

  map_name = "";
  id_of_most_recent_map = "";

  // Use the current ROS time in seconds as the session id.
  char buff[256]; 
  snprintf(buff, 256, "%f", ros::Time::now().toSec());
  session_id = std::string(buff);

  map_collection = new mr::MessageCollection<nav_msgs::OccupancyGrid>("map_store", "maps");

  ros::Subscriber map_subscriber = nh.subscribe("map", 1, onMapReceived);

  ros::ServiceServer name_latest_map_service = nh.advertiseService("name_latest_map", nameLatestMap);

  ROS_DEBUG("spinning.");

  ros::spin();
  return 0;
}
