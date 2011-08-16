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
 - sets up connection to warehouse
 - tells warehouse to publish latest map of any session (or default map?  or nothing?)
 - spins, handling service calls

service calls:
 - list_last_map_of_every_session() returns list of map metadata: {id, name, timestamp, maybe thumbnail}
   - query for all maps.
   - return list of latest metadata for each session id. (which may or may not have a name)
 - publish_map(map id) returns void
   - queries warehouse for map of given id
   - listens for resulting map and republishes it (?) on appropriate topic name (like "/map")
 */

#include <mongo_ros/message_collection.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_store/ListLastMaps.h>
#include <map_store/PublishMap.h>
#include <map_store/DeleteMap.h>
#include <map_store/RenameMap.h>
#include <map_store/MapListEntry.h>
#include <nav_msgs/GetMap.h>

#include <string>
#include <sstream>
#include <exception>
namespace mr=mongo_ros;

mr::MessageCollection<nav_msgs::OccupancyGrid> *map_collection;
ros::Publisher map_publisher;
std::string last_map;

typedef std::vector<mr::MessageWithMetadata<nav_msgs::OccupancyGrid>::ConstPtr> MapVector;

bool listLastMaps(map_store::ListLastMaps::Request &request,
                  map_store::ListLastMaps::Response &response)
{
  ROS_DEBUG("listLastMaps() service call");

  MapVector all_maps;
  all_maps = map_collection->pullAllResults( mr::Query(), true, "creation_time", false );

  // Keep a std::map of session IDs to remember which sessions we have seen before.
  std::map<std::string, bool> sessions_seen;

  // Loop over all_maps to get the first of each session.
  for(MapVector::const_iterator map_iter = all_maps.begin(); map_iter != all_maps.end(); map_iter++)
  {
    ROS_DEBUG("listLastMaps() reading a map");

    std::string session_id = (*map_iter)->lookupString("session_id");
    // If we haven't seen this session ID before,
    ROS_DEBUG("Session: %s", session_id.c_str());
    if( sessions_seen.find( session_id ) == sessions_seen.end()
	|| (*map_iter)->lookupString("name") != "" || true)
    {
      ROS_DEBUG("listLastMaps() adding a map to the result list.");
      ROS_DEBUG("listLastMaps() metadata is: '%s'", (*map_iter)->metadata.toString().c_str());
      
      // Mark that we've seen this session.
      sessions_seen[ session_id ] = true;
      
      // Add the map info to our result list.
      map_store::MapListEntry new_entry;
      new_entry.name = (*map_iter)->lookupString("name");
      new_entry.date = (int64_t)(*map_iter)->lookupDouble("creation_time");
      new_entry.session_id = session_id;
      new_entry.map_id = (*map_iter)->lookupString("uuid");
      
      response.map_list.push_back(new_entry);
    } else {
      ROS_WARN("Duplicate session map rejected: %s", (*map_iter)->lookupString("name").c_str());
    }
  }

  ROS_DEBUG("listLastMaps() service call done");
  return true;
}

bool lookupMap(std::string name, nav_msgs::OccupancyGridConstPtr &ptr) {
  MapVector matching_maps;
  try {
    matching_maps = map_collection->pullAllResults(mr::Query("uuid", name), false );
  } catch(const std::exception &e) {
    ROS_ERROR("Error during query: %s", e.what());
    return false;
  }

  if( matching_maps.size() != 1 ) {
    ROS_ERROR("publishMap() found %d matching maps instead of 1.  Failing.", (int) matching_maps.size());
    return false;
  }
  ptr = nav_msgs::OccupancyGridConstPtr( matching_maps[0] );
  return true;
}

bool publishMap(map_store::PublishMap::Request &request,
                map_store::PublishMap::Response &response)
{
  ROS_DEBUG("publishMap() service call");
  ROS_DEBUG("Searching for '%s'", request.map_id.c_str());

  last_map = request.map_id;
  nav_msgs::OccupancyGridConstPtr map;
  if (lookupMap(request.map_id, map))
  {
    try {
      map_publisher.publish(map);
    } catch(...) {
      ROS_ERROR("Error publishing map");
    }
  }
  else
  {
    return false;
  }

  return true;
}

bool deleteMap(map_store::DeleteMap::Request &request,
	       map_store::DeleteMap::Response &response)
{
  return map_collection->removeMessages(mr::Query("uuid", request.map_id)) == 1;
}

bool renameMap(map_store::RenameMap::Request &request,
               map_store::RenameMap::Response &response)
{
  map_collection->modifyMetadata(mr::Query("uuid", request.map_id), mr::Metadata("name", request.new_name));
  return true;
}


bool dynamicMap(nav_msgs::GetMap::Request &request,
		nav_msgs::GetMap::Response &response) {
  if (last_map == "") {
    return false;
  }
  nav_msgs::OccupancyGridConstPtr map;
  if (lookupMap(last_map, map))
  {
    response.map = *map;
  }
  else 
  {
    return false;
  }  
  return true;
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "map_loader");
  ros::NodeHandle nh;

  map_collection = new mr::MessageCollection<nav_msgs::OccupancyGrid>("map_store", "maps");
  map_collection->ensureIndex("uuid");

  ros::ServiceServer list_last_maps_service = nh.advertiseService("list_last_maps", listLastMaps);
  ros::ServiceServer publish_map_service = nh.advertiseService("publish_map", publishMap);
  ros::ServiceServer delete_map_service = nh.advertiseService("delete_map", deleteMap);
  ros::ServiceServer rename_map_service = nh.advertiseService("rename_map", renameMap);
  ros::ServiceServer dynamic_map = nh.advertiseService("dynamic_map", dynamicMap);

  map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1);

  ROS_DEBUG("spinning.");

  ros::spin();

  delete map_collection;

  return 0;
}
