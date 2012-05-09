/*
 * Copyright (c) 2011, Maxim Likhachev
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
 *     * Neither the name of the University of Pennsylvania nor the names of its
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
 /** \author Benjamin Cohen (modified by Mike Phillips)*/

#include <uav_collision_checking/uav_collision_space.h>

UAVCollisionSpace::UAVCollisionSpace(OccupancyGrid* grid) : grid_(grid)
{
}

bool UAVCollisionSpace::getSphereGroups()
{
  XmlRpc::XmlRpcValue all_groups;
  Sphere s;
  Group g;
  std::string groups_name = "groups";

  ros::NodeHandle ph("~");
  ph.param<std::string>("kinematics_chain/root_frame", full_body_chain_root_name_, "base_link");

  if(!ph.hasParam(groups_name))
  {
    ROS_WARN_STREAM("No groups for planning specified in " << groups_name);
    return false;
  }
  ph.getParam(groups_name, all_groups);

  if(all_groups.getType() != XmlRpc::XmlRpcValue::TypeArray) 
  {
    ROS_WARN("Groups is not an array.");
    return false;
  }

  if(all_groups.size() == 0) 
  {
    ROS_WARN("No groups in groups");
    return false;
  }

  for(int i = 0; i < all_groups.size(); i++) 
  {
    if(!all_groups[i].hasMember("name"))
    {
      ROS_WARN("All groups must have a name.");
      return false;
    } 
    g.name = std::string(all_groups[i]["name"]);

    if(!all_groups[i].hasMember("frame"))
    {
      ROS_WARN("All groups must have a frame.");
      return false;
    }
    g.root_frame = std::string(all_groups[i]["frame"]);

    std::stringstream ss(all_groups[i]["spheres"]);
    double x,y,z;
    g.spheres.clear();
    std::string sphere_name;
    while(ss >> sphere_name)
    {
      ros::NodeHandle s_nh(ph, sphere_name);
      s.name = sphere_name;
      s_nh.param("x", x, 0.0);
      s_nh.param("y", y, 0.0);
      s_nh.param("z", z, 0.0);
      s_nh.param("radius", s.radius, 0.0);
      s_nh.param("priority", s.priority, 1);
      s.v.x(x);
      s.v.y(y);
      s.v.z(z);
      s.radius_c = s.radius / grid_->getResolution() + 0.5;
      g.spheres.push_back(s);
    }
    
    if(g.name.compare("base") == 0)
      base_g_ = g;
    else
    {
      ROS_ERROR("The list of spheres contains a group with an unrecognized name, '%s'. Temporarily, only 'base' is supported.Exiting.", g.name.c_str());
      return false;
    }
  }

  // make a master list of sphere groups
  all_g_.push_back(base_g_);
  
  ROS_INFO("Successfully parsed collision groups.");
  return true;
}

void UAVCollisionSpace::printSphereGroups()
{
  for(size_t i = 0; i < all_g_.size(); ++i)
  {
    ROS_INFO("----------------[%d]-------------------",int(i));
    ROS_INFO("group: %s", all_g_[i].name.c_str());
    ROS_INFO("frame: %s", all_g_[i].root_frame.c_str());

    if(all_g_[i].spheres.size() == 0)
      ROS_WARN("(no spheres)"); 
    for(size_t j = 0; j < all_g_[i].spheres.size(); ++j)
      ROS_INFO("[%s] x: %0.3f  y: %0.3f  z: %0.3f  radius: %0.3fm  priority: %d", all_g_[i].spheres[j].name.c_str(), all_g_[i].spheres[j].v.x(), all_g_[i].spheres[j].v.y(), all_g_[i].spheres[j].v.z(), all_g_[i].spheres[j].radius, all_g_[i].spheres[j].priority);
    
    if(i == all_g_.size()-1)
      ROS_INFO("-------------------------------------");   
  }
}

void UAVCollisionSpace::getMaptoRobotTransform(double x, double y, double z, double theta, KDL::Frame &frame)
{
  KDL::Rotation r1;
  r1.DoRotZ(theta);
  KDL::Vector t1(x,y,z);
  KDL::Frame base_footprint_in_map(r1,t1);
  frame = base_footprint_in_map;
}

void UAVCollisionSpace::getVoxelsInGroup(KDL::Frame &frame, Group &group)
{
  KDL::Vector v;
  for(size_t i = 0; i < group.spheres.size(); ++i)
  {
    v = frame*group.spheres[i].v;
    grid_->worldToGrid(v.data[0],v.data[1],v.data[2],group.spheres[i].voxel[0], group.spheres[i].voxel[1], group.spheres[i].voxel[2]);
  }
}

bool UAVCollisionSpace::isUAVValid(double x, double y, double z, double theta, unsigned char &dist)
{
  unsigned char dist_temp;

  ROS_DEBUG("[cspace] Checking UAV. x: %0.3f y: %0.3f z: %0.3f theta: %0.3f",x,y,z,theta);
  KDL::Frame map_to_robot;
  getMaptoRobotTransform(x,y,z,theta,map_to_robot);

  int sx,sy,sz;
  grid_->getGridSize(sx, sy, sz);
  getVoxelsInGroup(map_to_robot, base_g_);
  for(size_t i = 0; i < base_g_.spheres.size(); ++i)
  {
    if(base_g_.spheres[i].voxel[0] < 0 || base_g_.spheres[i].voxel[0] >= sx || 
       base_g_.spheres[i].voxel[1] < 0 || base_g_.spheres[i].voxel[1] >= sy || 
       base_g_.spheres[i].voxel[2] < 0 || base_g_.spheres[i].voxel[2] >= sz)
      return false;
    if((dist_temp = grid_->getCell(base_g_.spheres[i].voxel[0], base_g_.spheres[i].voxel[1], base_g_.spheres[i].voxel[2])) <= base_g_.spheres[i].radius_c)
    {
      dist = dist_temp;
      return false;
    }
    
    if(dist > dist_temp)
      dist = dist_temp;
  }
  return true;
}

void UAVCollisionSpace::setGrid(OccupancyGrid* g){
  grid_ = g;
}

