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

#include <warehouse/warehouse_client.h>
#include <warehouse/collection.h>
#include <warehouse/Condition.h>
#include <warehouse/OrderingCriterion.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_store/ListLastMaps.h>
#include <map_store/PublishMap.h>
#include <map_store/MapListEntry.h>

#include <string>
#include <sstream>
namespace wh=warehouse;

wh::Collection<nav_msgs::OccupancyGrid> map_collection;
ros::Publisher map_publisher;

typedef std::vector<wh::MessageWithMetadata<nav_msgs::OccupancyGrid>::ConstPtr> MapVector;

bool listLastMaps(map_store::ListLastMaps::Request &request,
                  map_store::ListLastMaps::Response &response)
{
  ROS_INFO("listLastMaps() service call");

  // No conditions: find all maps in the warehouse.
  std::vector<wh::Condition> empty_conditions;

  // Order maps with most recent first, then take the first in each session.
  wh::OrderingCriterion most_recent_first_ordering;
  most_recent_first_ordering.field = "_creation_time";
  most_recent_first_ordering.reverse = true;

  MapVector all_maps;
  all_maps = map_collection.pullAllResults( empty_conditions, true, most_recent_first_ordering );

  // Keep a std::map of session IDs to remember which sessions we have seen before.
  std::map<std::string, bool> sessions_seen;

  // Loop over all_maps to get the first of each session.
  for(MapVector::const_iterator map_iter = all_maps.begin(); map_iter != all_maps.end(); map_iter++)
  {
    ROS_INFO("listLastMaps() reading a map");

    try {
      std::string session_id = wh::getMetadataVal<std::string>((*map_iter)->metadata, "session_id");
      // If we haven't seen this session ID before,
      if( sessions_seen.find( session_id ) == sessions_seen.end() )
      {
        ROS_INFO("listLastMaps() adding a map to the result list.");
        ROS_INFO("listLastMaps() metadata is: '%s'", (*map_iter)->metadata.c_str());

        // Mark that we've seen this session.
        sessions_seen[ session_id ] = true;

        // Add the map info to our result list.
        map_store::MapListEntry new_entry;
        new_entry.name = wh::getMetadataVal<std::string>((*map_iter)->metadata, "name");
        new_entry.date = (int64_t) wh::getMetadataVal<double>((*map_iter)->metadata, "_creation_time");
        new_entry.session_id = wh::getMetadataVal<std::string>((*map_iter)->metadata, "session_id");
        new_entry.map_id = wh::getMetadataVal<std::string>((*map_iter)->metadata, "uuid");

        response.map_list.push_back(new_entry);
      }
    }
    catch(wh::MissingKeyException ex)
    {
      ROS_WARN("listLastMaps() found invalid map metadata, ignoring. %s", ex.what());
    }
  }

  ROS_INFO("listLastMaps() service call done");
  return true;
}

bool publishMap(map_store::PublishMap::Request &request,
                map_store::PublishMap::Response &response)
{
  ROS_INFO("publishMap() service call");

  std::vector<wh::Condition> correct_id(1);
  correct_id[0].field_name = "uuid";
  correct_id[0].predicate = wh::Condition::EQUALS;
  correct_id[0].args.push_back(request.map_id);

  wh::OrderingCriterion most_recent_first_ordering;
  most_recent_first_ordering.field = "_creation_time";
  most_recent_first_ordering.reverse = true;

  MapVector matching_maps;
  matching_maps = map_collection.pullAllResults( correct_id, false, most_recent_first_ordering );

  if( matching_maps.size() != 1 ) {
    ROS_ERROR("publishMap() found %d matching maps instead of 1.  Failing.", (int) matching_maps.size());
  }
  else
  {
    nav_msgs::OccupancyGridConstPtr map( matching_maps[0] );
    map_publisher.publish( map );
  }

  return true;
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "map_loader");
  ros::NodeHandle nh;

  wh::WarehouseClient client("my_app");

  map_collection = client.setupCollection<nav_msgs::OccupancyGrid>("maps");

  ros::ServiceServer list_last_maps_service = nh.advertiseService("list_last_maps", listLastMaps);
  ros::ServiceServer publish_map_service = nh.advertiseService("publish_map", publishMap);

  map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1);

  ROS_INFO("spinning.");

  ros::spin();
  return 0;
}
