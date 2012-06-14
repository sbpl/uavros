#include<uav_local_planner/local_planner.h>

UAVLocalPlanner::UAVLocalPlanner()
{
  ros::NodeHandle nh;
  ros::NodeHandle ph("~");

  ph.param("size_x",sizex_,4.0);
  ph.param("size_y",sizey_,4.0);
  ph.param("size_z",sizez_,4.0);
  ph.param("resolution",resolution_,0.1);

  ph.param("controller_frequency",controller_frequency_,50.0);
  ph.param("collision_map_tolerance",collision_map_tolerance_,0.5);
  ph.param("pose_tolerance",pose_tolerance_,0.05);

  ph.param("landing_height",landing_height_,0.3);
  ph.param("nominal_height",nominal_height_,1.0);
  ph.param("nominal_linear_velocity",nominal_linear_velocity_,0.3);
  ph.param("nominal_angular_velocity",nominal_angular_velocity_,M_PI/2);

  path_idx_ = 0;

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
  state_sub_ = nh.subscribe("uav_state", 1, &UAVLocalPlanner::stateCallback,this);
  flight_mode_sub_ = nh.subscribe("flight_mode_request", 1, &UAVLocalPlanner::flightModeCallback,this);

//   dynamic_reconfigure::Server<uav_local_planner::UAVControllerConfig>::CallbackType f;
//   f = boost::bind(&HexaController::dynamic_reconfigure_callback, controller, _1, _2);
//   dynamic_reconfigure_server_.setCallback(f);
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
  last_state_ = LANDED;
  while(n.ok()){
    ROS_INFO("[controller] annoying print at the top of the main loop!");
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

    if(state==HOVER && last_state_!=HOVER){
      ROS_INFO("[controller] transitioning to hover (setting the hover pose)");
      hover_pose_ = pose;
    }

    last_state_ = state;
    uav_msgs::ControllerCommand u;
    switch(state){
      case LANDED:
        ROS_DEBUG("[controller] state: LANDED");
        break;
      case LANDING:
        ROS_DEBUG("[controller] state: LANDING");
        u = land(pose,velocity,state);
        break;
      case TAKE_OFF:
        ROS_DEBUG("[controller] state: TAKE_OFF");
        u = takeOff(pose,velocity,state);
        break;
      case HOVER:
        ROS_DEBUG("[controller] state: HOVER");
        u = hover(pose,velocity);
        break;
      case FOLLOWING:
        ROS_DEBUG("[controller] state: FOLLOWING");
        u = followPath(pose,velocity,state,isNewPath);
        break;
      default:
        ROS_ERROR("[controller] In an invalid state! Landing...");
        state = LANDING;
        break;
    }
    last_u_ = u;
    ROS_ERROR("beavis\n");
//     printf("beavis f\n");
    if(!LANDED)
      command_pub_.publish(u);  //TODO: empty message being published....
    r.sleep();
  }
}

/***************** STATE FUNCTIONS *****************/

uav_msgs::ControllerCommand UAVLocalPlanner::land(geometry_msgs::PoseStamped pose, geometry_msgs::TwistStamped vel, UAVControllerState& state){
  uav_msgs::ControllerCommand u;

  if(pose.pose.position.z <= landing_height_){
    u = last_u_;
    u.roll = 0;
    u.pitch = 0;
    u.yaw = 0;
    if(u.thrust==0)
      state = LANDED;
    else
      u.thrust -= 1;
  }
  else{
    geometry_msgs::PoseStamped target = pose;
    target.pose.position.z -= 0.4;
    //if(target.pose.position.z < landing_height_)
      //target.pose.position.z = landing_height_ - 0.1;
    visualizeTargetPose(target);
    u = controller.Controller(pose, vel, target);
  }
  return u;
}

uav_msgs::ControllerCommand UAVLocalPlanner::takeOff(geometry_msgs::PoseStamped pose, geometry_msgs::TwistStamped vel, UAVControllerState& state){
  geometry_msgs::PoseStamped target = pose;
  if(pose.pose.position.z >= nominal_height_)
    state = HOVER;
  else
    target.pose.position.z += 0.4;
  visualizeTargetPose(target);
  return controller.Controller(pose, vel, target);
}

uav_msgs::ControllerCommand UAVLocalPlanner::hover(geometry_msgs::PoseStamped pose, geometry_msgs::TwistStamped vel){
  visualizeTargetPose(hover_pose_);
  return controller.Controller(pose, vel, hover_pose_);
}

uav_msgs::ControllerCommand UAVLocalPlanner::followPath(geometry_msgs::PoseStamped pose, geometry_msgs::TwistStamped vel, UAVControllerState& state, bool isNewPath){
  if(isNewPath)
    path_idx_ = 0;
  double dx = pose.pose.position.x - controller_path_->poses[path_idx_].pose.position.x;
  double dy = pose.pose.position.y - controller_path_->poses[path_idx_].pose.position.y;
  double dz = pose.pose.position.z - controller_path_->poses[path_idx_].pose.position.z;
  double dist = sqrt(dx*dx + dy*dy + dz*dz);
  unsigned int i;
  for(i=path_idx_+1; i<controller_path_->poses.size(); i++){
    dx = pose.pose.position.x - controller_path_->poses[i].pose.position.x;
    dy = pose.pose.position.y - controller_path_->poses[i].pose.position.y;
    dz = pose.pose.position.z - controller_path_->poses[i].pose.position.z;
    double temp = sqrt(dx*dx + dy*dy + dz*dz);
    if(temp > dist)
      break;
    dist = temp;
  }
  path_idx_ = i-1;
  if(i == controller_path_->poses.size())
    i--;

  //TODO: collision check the path from our pose to the target pose (just check the straight line)
  //TODO: collision check the path from the target to the next few points (use a time horizon)
  geometry_msgs::PoseStamped target = controller_path_->poses[i];
  visualizeTargetPose(target);
  uav_msgs::ControllerCommand u = controller.Controller(pose, vel, target);
  //TODO: collision check the controls for some very short period of time
  return u;
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
    cspace_->setGrid(controller_grid_);
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

    boost::unique_lock<boost::mutex> goal_lock(goal_mutex_);
    if(controller_path_->poses.empty() || latest_goal_.header.stamp.toSec()>controller_path_->header.stamp.toSec()){
      //switch from following to hover if the new path is old (or empty)
      if(state==FOLLOWING)
        state = HOVER;
    }
    else{
      //switch from hover to following if the new path is up to date
      if(state==HOVER)
        state = FOLLOWING;
    }
    goal_lock.unlock();
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
  boost::unique_lock<boost::mutex> lock(state_mutex_);
  pose.header = latest_state_.header;
  pose.pose = latest_state_.pose.pose;
  velocity.header = latest_state_.header;
  velocity.twist = latest_state_.twist.twist;
  lock.unlock();
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
  boost::unique_lock<boost::mutex> lock(goal_mutex_);
  latest_goal_ = *goal;
  lock.unlock();
}

void UAVLocalPlanner::stateCallback(nav_msgs::OdometryConstPtr state){
  boost::unique_lock<boost::mutex> lock(state_mutex_);
  latest_state_ = *state;
  lock.unlock();
}

void UAVLocalPlanner::flightModeCallback(uav_msgs::FlightModeRequestConstPtr req){
  boost::unique_lock<boost::mutex> lock(flight_mode_mutex_);
  flight_mode_ = *req;
  lock.unlock();
}

/***************** VISUALIZATION *****************/

void UAVLocalPlanner::visualizeTargetPose(geometry_msgs::PoseStamped p){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time(0);
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = p.pose;
  marker.scale.z = 0.3;
  marker.scale.y = 0.05;
  marker.scale.x = 0.05;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.ns = "controller target";
  marker.id = 0;
  marker.lifetime = ros::Duration(0);
  waypoint_vis_pub_.publish(marker);
}

/***************** MAIN *****************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "uav_local_planner");
  UAVLocalPlanner local_planner;

  ros::spin();

  return 0;
}

