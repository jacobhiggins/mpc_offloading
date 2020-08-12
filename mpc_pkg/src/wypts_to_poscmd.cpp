#include <ros/ros.h>
#include <std_msgs/String.h>
#include <asctec_msgs/PositionCmd.h>
#include <asctec_msgs/WaypointCmd.h>

static ros::Publisher pos_cmd_pub;
asctec_msgs::PositionCmd pos_cmd; 

void wypt_cb(const asctec_msgs::WaypointCmd::ConstPtr &wypt_cmd){
  pos_cmd.position.x = wypt_cmd->position.x;
  pos_cmd.position.y = wypt_cmd->position.y;
  pos_cmd.position.z = wypt_cmd->position.z;

  ROS_INFO("New waypoint: %f, %f, %f",pos_cmd.position.x,pos_cmd.position.y,pos_cmd.position.z);

  //pos_cmd_pub.publish(pos_cmd);
}

int main(int argc, char **argv){

  ros::init(argc, argv, "wypt_to_poscmd");
  //ros::Rate r(100);
  ros::NodeHandle nh;
  ros::Subscriber wypt_sub = nh.subscribe(ros::this_node::getNamespace()+"/waypoints",10, &wypt_cb);
  pos_cmd_pub = nh.advertise<asctec_msgs::PositionCmd>(ros::this_node::getNamespace()+"/position_cmd",1);

  ros::Rate r(100);

  pos_cmd.position.x = 0.0;
  pos_cmd.position.y = 0.0;
  pos_cmd.position.z = 0.0;

  while(ros::ok()){
    ros::spinOnce();
    pos_cmd_pub.publish(pos_cmd);
    r.sleep();
  }
}
