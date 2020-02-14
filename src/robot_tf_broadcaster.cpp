// #include <ros/ros.h>
// #include <tf/transform_broadcaster.h>
// #include <gazebo_msgs/ModelState.h>

// std::string turtle_name;



// void poseCallback(const gazebo_msgs::ModelStateConstPtr& msg){
//   static tf::TransformBroadcaster br;
//   tf::Transform transform;
//   transform.setOrigin( tf::Vector3(msg->pose.position.x, msg->pose.position.y, 0.0) );
//   tf::Quaternion q;
//   q.setRPY(0, 0, msg->pose.orientation.z);
//   transform.setRotation(q);
//   br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "ground_plane", "robot2"));
// }

// int main(int argc, char** argv){
//   ros::init(argc, argv, "my_tf_broadcaster");
//   if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
//   turtle_name = argv[1];

//   ros::NodeHandle node;
//   ros::Subscriber sub = node.subscribe("robot2/pose", 10, &poseCallback);

//   ros::spin();
//   return 0;
// };

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, msg->theta);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/worlds/empty.world", turtle_name));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
  turtle_name = argv[1];

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);

  ros::spin();
  return 0;
}