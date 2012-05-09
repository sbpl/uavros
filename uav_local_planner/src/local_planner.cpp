
#include<uav_local_planner/local_planner.h>

UAVLocalPlanner::UAVLocalPlanner()
  : tf_(ros::NodeHandle(), ros::Duration(10), true){
  ros::NodeHandle nh;
  ros::NodeHandle ph("~");

  ph.param("size_x",sizex_,4.0);
  ph.param("size_y",sizey_,4.0);
  ph.param("size_z",sizez_,4.0);
  ph.param("resolution",resolution_,0.1);
  //TODO: add params for controller gains and frequency
  //TODO: add params for landing height and nominal height
  //TODO: add params for nominal linear and angular velocity

  //publish UAV commands and goals (in case we detect a collision up ahead we publish the same goal state to engage the planner)
  waypoint_vis_pub_ = nh.advertise<visualization_msgs::Marker>("/controller/next_waypoint",1);
  command_pub_ = nh.advertise<uav_msgs::ControllerCommand>("/controller_cmd",1);
  goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/goal",1);

  //set up a occupancy grid triple buffer
  controller_grid_ = new OccupancyGrid(sizex_,sizey_,sizez_,resolution_,sizex_/2,sizey_/2,sizez_/2);
  latest_grid_ = new OccupancyGrid(sizex_,sizey_,sizez_,resolution_,sizex_/2,sizey_/2,sizez_/2);
  callback_grid_ = new OccupancyGrid(sizex_,sizey_,sizez_,resolution_,sizex_/2,sizey_/2,sizez_/2);
  new_grid_ = false;
  cspace_ = new UAVCollisionSpace(controller_grid_);

  //set up a path triple buffer
  controller_path_ = new nav_msgs::Path();
  latest_path_ = new nav_msgs::Path();
  callback_path_ = new nav_msgs::Path();
  new_path_ = false;

  //spawn the controller thread
  controller_thread_ = new boost::thread(boost::bind(&UAVLocalPlanner::controllerThread, this));

  //subscribe to the collision map, tf, path, goal, and flight mode
  collision_map_sub_ = nh.subscribe("local_collision_map", 1, &UAVLocalPlanner::collisionMapCallback,this);
  path_sub_ = nh.subscribe("path", 1, &UAVLocalPlanner::pathCallback,this);
  goal_sub_ = nh.subscribe("goal", 1, &UAVLocalPlanner::goalCallback,this);
  flight_mode_sub_ = nh.subscribe("flight_mode_request", 1, &UAVLocalPlanner::flightModeCallback,this);
}

UAVLocalPlanner::~UAVLocalPlanner(){
  delete controller_grid_;
  delete latest_grid_;
  delete callback_grid_;
  delete controller_path_;
  delete latest_path_;
  delete callback_path_;
  delete cspace_;

  controller_thread_->join();
}

/***************** MAIN LOOP *****************/

//only the controller thread can update the UAV's state. callbacks must set flags to request change in state.
void UAVLocalPlanner::controllerThread(){
  ROS_DEBUG("[controller] Starting controller thread...");
  ros::NodeHandle n;
  ros::Rate r(controller_frequency_);
  double last_collision_map_update = ros::Time::now().toSec();
  UAVControllerState state = LANDED;
  while(n.ok()){
    //if the robot is hovering and we get a new path, switch to following.
    //conversely, if we get a new goal but don't have a fresh path yet, go back to hover
    bool isNewPath = updatePath(state);

    //try to update the collision map. if the map we have is too old and we are following a path, switch to hover.
    if(updateCollisionMap())
      last_collision_map_update = ros::Time::now().toSec();
    else if(ros::Time::now().toSec()-last_collision_map_update > collision_map_tolerance_ && state==FOLLOWING){
      ROS_ERROR("[controller] collision map is out of date. Will hover instead of following.");
      state = HOVER;
    }

    //check if we should be landing or taking off...
    getFlightMode(state);

    //get the robot's pose. if it is out of date....complain?
    geometry_msgs::PoseStamped pose;
    geometry_msgs::TwistStamped velocity;
    getRobotPose(pose,velocity);
    if(ros::Time::now().toSec()-pose.header.stamp.toSec() > pose_tolerance_){
      ROS_ERROR("[controller] UAV pose is old...hit the deck!");
    }

    uav_msgs::ControllerCommand c;
    switch(state){
      case LANDED:
        ROS_DEBUG("[controller] state: LANDED");
        break;
      case LANDING:
        ROS_DEBUG("[controller] state: LANDING");
        c = land(pose,velocity);
        break;
      case TAKE_OFF:
        ROS_DEBUG("[controller] state: TAKE_OFF");
        c = takeOff(pose,velocity);
        break;
      case HOVER:
        ROS_DEBUG("[controller] state: HOVER");
        c = hover(pose,velocity);
        break;
      case FOLLOWING:
        ROS_DEBUG("[controller] state: FOLLOWING");
        c = followPath(pose,velocity,isNewPath);
        break;
      default:
        ROS_ERROR("[controller] In an invalid state! Landing...");
        state = LANDING;
        break;
    }
    if(!LANDED)
      command_pub_.publish(c);
  }
}

/***************** STATE FUNCTIONS *****************/

uav_msgs::ControllerCommand UAVLocalPlanner::land(geometry_msgs::PoseStamped, geometry_msgs::TwistStamped){
  uav_msgs::ControllerCommand c;
  return c;
}

uav_msgs::ControllerCommand UAVLocalPlanner::takeOff(geometry_msgs::PoseStamped, geometry_msgs::TwistStamped){
  uav_msgs::ControllerCommand c;
  return c;
}

uav_msgs::ControllerCommand UAVLocalPlanner::hover(geometry_msgs::PoseStamped, geometry_msgs::TwistStamped){
  uav_msgs::ControllerCommand c;
  return c;
}

uav_msgs::ControllerCommand UAVLocalPlanner::followPath(geometry_msgs::PoseStamped, geometry_msgs::TwistStamped, bool isNewPath){
  uav_msgs::ControllerCommand c;
  return c;
}

/***************** UPDATE FUNCTIONS *****************/

bool UAVLocalPlanner::updateCollisionMap(){
  //if there is a new occupancy grid in latest then swap into the controller occupancy grid
  bool updated = false;
  boost::unique_lock<boost::mutex> lock(grid_mutex_);
  if(new_grid_){
    OccupancyGrid* temp = latest_grid_;
    latest_grid_ = controller_grid_;
    controller_grid_ = temp;
    new_grid_ = false;
    updated = true;
  }
  lock.unlock();
  
  //if we did a swap, then update the collision space to use the new occupancy grid
  if(updated)
    cspace_->setGrid(controller_grid_);//TODO: this function doesn't exist yet...
  return updated;
}

bool UAVLocalPlanner::updatePath(UAVControllerState& state){
  //if there is a new path in latest then swap into the controller path
  bool ret = false;
  boost::unique_lock<boost::mutex> lock(path_mutex_);
  if(new_path_){
    nav_msgs::Path* temp = latest_path_;
    latest_path_ = controller_path_;
    controller_path_ = temp;
    new_path_ = false;
    ret = true;

    if(latest_goal_.header.stamp.toSec()>controller_path_->header.stamp.toSec()){
      //switch from following to hover if the new path is old
      if(state==FOLLOWING)
        state = HOVER;
    }
    else{
      //switch from hover to following if the new path is up to date
      if(state==HOVER)
        state = FOLLOWING;
    }
  }
  lock.unlock();

  return ret;
}

void UAVLocalPlanner::getFlightMode(UAVControllerState& state){
  boost::unique_lock<boost::mutex> lock(flight_mode_mutex_);
  uav_msgs::FlightModeRequest flightMode = flight_mode_;
  flight_mode_.mode = uav_msgs::FlightModeRequest::NONE;
  lock.unlock();

  if(flightMode.mode==uav_msgs::FlightModeRequest::LAND){
    if(state==TAKE_OFF || state==HOVER || state==FOLLOWING)
      state = LANDING;
    else
      ROS_WARN("[controller] Asked to land, but UAV is landed or already landing.");
  }
  else if(flightMode.mode==uav_msgs::FlightModeRequest::TAKE_OFF){
    if(state==LANDED || state==LANDING)
      state = TAKE_OFF;
    else
      ROS_WARN("[controller] Asked to take off, but UAV is already in the air (or working on it).");
  }
  else if(flightMode.mode==uav_msgs::FlightModeRequest::HOVER){
    if(state==FOLLOWING)
      state = HOVER;
    else
      ROS_WARN("[controller] Asked to hover, but the UAV can only go to hover from path following.");
  }
}

void UAVLocalPlanner::getRobotPose(geometry_msgs::PoseStamped& pose, geometry_msgs::TwistStamped& velocity){
  tf::StampedTransform transform;
  try{
    tf_.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }

  geometry_msgs::TransformStamped geo_pose;
  transformStampedTFToMsg(transform, geo_pose);
  pose.pose.position.x = geo_pose.transform.translation.x;
  pose.pose.position.y = geo_pose.transform.translation.y;
  pose.pose.position.z = geo_pose.transform.translation.z;
  pose.pose.orientation.w = geo_pose.transform.rotation.w;
  pose.pose.orientation.x = geo_pose.transform.rotation.x;
  pose.pose.orientation.y = geo_pose.transform.rotation.y;
  pose.pose.orientation.z = geo_pose.transform.rotation.z;
  pose.header = geo_pose.header;

  //TODO: get the velocity information from.....somewhere?
}

/***************** CALLBACKS *****************/

void UAVLocalPlanner::collisionMapCallback(arm_navigation_msgs::CollisionMapConstPtr cm){
  //compute distance field and load it into the callback occupancy grid
  callback_grid_->updateFromCollisionMap(*cm);
  callback_grid_->visualize();

  //take the grid mutex and swap into the latest occupancy grid 
  boost::unique_lock<boost::mutex> lock(path_mutex_);
  OccupancyGrid* temp = latest_grid_;
  latest_grid_ = callback_grid_;
  callback_grid_ = temp;
  new_grid_ = true;
  lock.unlock();
}

void UAVLocalPlanner::pathCallback(nav_msgs::PathConstPtr path){
  *callback_path_ = *path;
  boost::unique_lock<boost::mutex> lock(path_mutex_);
  nav_msgs::Path* temp = latest_path_;
  latest_path_ = callback_path_;
  callback_path_ = temp;
  new_path_ = true;
  lock.unlock();
}

void UAVLocalPlanner::goalCallback(geometry_msgs::PoseStampedConstPtr goal){
  latest_goal_ = *goal;
}

void UAVLocalPlanner::flightModeCallback(uav_msgs::FlightModeRequestConstPtr req){
  boost::unique_lock<boost::mutex> lock(flight_mode_mutex_);
  flight_mode_ = *req;
  lock.unlock();
}

/***************** MAIN *****************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "uav_local_planner");
  UAVLocalPlanner local_planner;

  ros::spin();

  return 0;
}

