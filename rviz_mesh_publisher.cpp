#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <stdio.h>



int main( int argc, char** argv )
{
  ros::init(argc, argv, "mesh_grid");
  ros::NodeHandle n;
  ros::Rate r(10000);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10000, true);
  //ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1, true);



  //while (ros::ok())
  {
  visualization_msgs::Marker marker;
  //visualization_msgs::MarkerArray ma;
    

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
   // marker.header.frame_id = "/my_frame";
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "Meshgrid";
    marker.id = 100;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 35; //416.4
    marker.pose.position.y = -2; //315.4
    marker.pose.position.z = -12;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0; //1
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1; //0

    // Set the scale of the marker -- 1x1x1 here means 1m on a side

   // marker.scale.x = 511.75;  // This is z in mesh !
   // marker.scale.y =392;
   // marker.scale.z = 500;

    marker.scale.x = 0.35;  // This is z in mesh ! 0.5
    marker.scale.y =1;
    marker.scale.z = 1;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.847f;
    marker.color.g = 0.164f;
    marker.color.b = 0.164f;
    marker.color.a = 1;

    marker.mesh_resource="package://sbpl_lattice_planner/worlds/latest2blend.dae"; // Your path to mesh here
    marker.mesh_use_embedded_materials=1;
   // marker.frame_locked="true";


    marker.lifetime = ros::Duration(10000);

    // Publish the marker
  // ma.markers.push_back(marker);
   marker_pub.publish(marker); 
  // r.sleep();
    
 while(ros::ok())
{
}
  }

}
