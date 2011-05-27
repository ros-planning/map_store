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

#include <warehouse/warehouse_client.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

namespace wh=warehouse;

int main (int argc, char** argv)
{
  ros::init(argc, argv, "map_store");
  wh::WarehouseClient client("my_app");

  wh::Collection<nav_msgs::OccupancyGrid> coll;
  coll = client.setupCollection<nav_msgs::OccupancyGrid>("maps");
}

/*
behavior:
 - setups up connection to warehouse
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
