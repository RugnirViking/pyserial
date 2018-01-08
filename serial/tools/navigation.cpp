#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>       
#include <locale>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/lexical_cast.hpp>
#include <map>
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
using namespace std;
using boost::lexical_cast;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
ros::Publisher out_pub;
vector<string> explode(string s, string c){
	string buff="";
  vector<string> v;
  size_t pos = 0;
  std::string token;
  while ((pos = s.find(c)) != std::string::npos) {
    token = s.substr(0, pos);
    v.push_back(token);
    s.erase(0, pos + c.length());
  }
  v.push_back(s);
	return v;
}
void move(float goalXResPos, float goalYResPos, float goalFinalOrientation){

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
    if (!ros::ok()){
      exit(0);
    }
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = goalXResPos;
  goal.target_pose.pose.position.y = goalYResPos;
  goal.target_pose.pose.orientation.z = goalFinalOrientation;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Hooray, the base moved 1 meter forward");
    std_msgs::String msg;
    std::stringstream ss;
    ss << "Hooray, the base moved 1 meter forward";
    msg.data = ss.str();
    out_pub.publish(msg);
  }
  else{
    ROS_INFO("The base failed to move forward 1 meter for some reason");
    std_msgs::String msg;
    std::stringstream ss;
    ss << "The base failed to move forward 1 meter for some reason";
    msg.data = ss.str();
    out_pub.publish(msg);
  }
  return;
}
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  std::locale loc;
  std::string msgRecieved = msg->data;
  if (msgRecieved.find("say") == 0){
    msgRecieved.erase(0,4);
    ROS_INFO("Soviet Deep Space Transmission Recieved: [%s]", msgRecieved.c_str());
  } 
  if (msgRecieved.find("boop") == 0){
    //msgRecieved.erase(0,4);
    ROS_INFO("beep eggs");
    std_msgs::String msg;
    std::stringstream ss;
    ss << "beep";
    msg.data = ss.str();
    out_pub.publish(msg);
  } 
  if (msgRecieved.find("beep") == 0){
    //msgRecieved.erase(0,4);
    ROS_INFO("boop");
    std_msgs::String msg;
    std::stringstream ss;
    ss << "boop";
    msg.data = ss.str();
    out_pub.publish(msg);
  } 
  if (msgRecieved.find("ping") == 0){
    //msgRecieved.erase(0,4);
    ROS_INFO("pong");
    std_msgs::String msg;
    std::stringstream ss;
    ss << "pong";
    msg.data = ss.str();
    out_pub.publish(msg);
  } 
  if (msgRecieved.find("pong") == 0){
    //msgRecieved.erase(0,4);
    ROS_INFO("ping");
    std_msgs::String msg;
    std::stringstream ss;
    ss << "ping";
    msg.data = ss.str();
    out_pub.publish(msg);
  } 
  if (msgRecieved.find("go") == 0){
    ROS_INFO("Trying to go forward");
    string nothing = " ";
    vector<string> v = explode(msgRecieved, nothing.c_str());
    
    if (v.size()<4){
      ROS_INFO("Bad \"go\" command");
      std_msgs::String msg;
      std::stringstream ss;
      ss << "Bad \"go\" command";
      msg.data = ss.str();
      out_pub.publish(msg);
    } else {
      ROS_INFO("XPos recieved: %s",v[1].c_str());
      ROS_INFO("YPos recieved: %s",v[2].c_str());
      ROS_INFO("Angle recieved: %s",v[3].c_str());

      std_msgs::String msg;
      std::stringstream ss;
      ss << "XPos recieved: "<<v[1].c_str();
      msg.data = ss.str();
      out_pub.publish(msg);

      std_msgs::String msg2;
      std::stringstream ss2;
      ss2 << "YPos recieved: "<<v[2].c_str();
      msg2.data = ss2.str();
      out_pub.publish(msg2);

      std_msgs::String msg3;
      std::stringstream ss3;
      ss3 << "Angle recieved: "<<v[3].c_str();
      msg3.data = ss3.str();
      out_pub.publish(msg3);

      move(lexical_cast<double>(v[1]),lexical_cast<double>(v[2]),lexical_cast<double>(v[3]));
    }
  }
}
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;
  ROS_INFO("::----Soviet Deep Space Transmission Robot Starting Up...---::");
  ROS_INFO("::-------Soviet Deep Space Transmission Robot Online--------::");
  out_pub = n.advertise<std_msgs::String>("navigation_output_msgs", 1000);
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  ros::init(argc, argv, "simple_navigation_goals");

  ros::spin();

  return 0;
}
