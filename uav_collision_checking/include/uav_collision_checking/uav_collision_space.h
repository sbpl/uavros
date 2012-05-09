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

/* /author Benjamin Cohen (modified by Mike Phillips)*/

#ifndef _UAV_COLLISION_SPACE_
#define _UAV_COLLISION_SPACE_

#include <ros/ros.h>
#include <vector>
//#include <sbpl_arm_planner/sbpl_arm_planning_error_codes.h>
#include <uav_collision_checking/occupancy_grid.h>
//#include <sbpl_arm_planner/sbpl_geometry_utils.h>
//#include <tf_conversions/tf_kdl.h>
//#include <tf/tf.h>

/* added for full body planning */
#include <kdl/frames.hpp>

using namespace std;

typedef struct
{
  std::string name;
  KDL::Vector v;
  double radius;
  int radius_c;
  int priority;
  int voxel[3]; // temp variable
} Sphere;

typedef struct
{
  std::string name;
  std::string root_frame;
  std::vector<Sphere> spheres;
  int kdl_chain;
  int kdl_segment;
  KDL::Frame f; // temp variable
} Group;

class UAVCollisionSpace
{
  public:
    /* constructors */
    UAVCollisionSpace(OccupancyGrid* grid);

    ~UAVCollisionSpace(){};

    /* full body planning */
    bool getSphereGroups();
    void printSphereGroups();
    void getMaptoRobotTransform(double x, double y, double z, double theta, KDL::Frame &frame);
    void getVoxelsInGroup(KDL::Frame &frame, Group &group);

    bool isUAVValid(double x, double y, double z, double theta, unsigned char &dist);
    void setGrid(OccupancyGrid* g);

  private:

    /** @brief occupancy grid used by planner */
    OccupancyGrid* grid_;

    Group base_g_;
    std::vector<Group> all_g_;

    std::string full_body_chain_root_name_;
};

#endif

