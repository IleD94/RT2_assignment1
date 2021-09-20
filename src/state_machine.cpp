#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/RandomPosition.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rt2_assignment1/PlanningAction.h>
#include "geometry_msgs/Twist.h"

bool start = false;
bool stop = false;
bool OnMyWay = true;
bool result = false;
bool FirstRound = true;
bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
    
    if (req.command == "start"){
    	start = true;
    	OnMyWay= false;
    }
    else {
    	start = false;
    	stop = true;
    }
    return true;
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   actionlib::SimpleActionClient<rt2_assignment1::PlanningAction> ac("/reaching_goal", true);
   ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

   rt2_assignment1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;
   rt2_assignment1::PlanningGoal goal;
   geometry_msgs::Twist twist_msg;
   
   while(ros::ok()){
   	ros::spinOnce();
   	if (start){
   		stop = false;
   		FirstRound=true;
   		if (not OnMyWay) {
   		client_rp.call(rp);
   		goal.target_pose.header.frame_id = "base_link";
  		goal.target_pose.header.stamp = ros::Time::now();
   		goal.target_pose.pose.position.x = rp.response.x;
   		goal.target_pose.pose.position.y = rp.response.y;
   		goal.target_pose.pose.orientation.w  = rp.response.theta;
   		std::cout << "\nGoing to the position: x= " << goal.target_pose.pose.position.x << " y= " << goal.target_pose.pose.position.y << " theta = " << goal.target_pose.pose.orientation.w << std::endl;
   		ac.sendGoal(goal);
   		OnMyWay= true;
   		}
   		
   		else {
   		FirstRound=true;
		rt2_assignment1::PlanningResultConstPtr Result;
  		Result = ac.getResult();
   		result = (bool) Result->result;
   		  if (result) {
   		    OnMyWay = false;
   		    ROS_INFO ("Hooray, goal reached!");
   		  }
   		}
    }

   	if (stop and FirstRound) {
   		ac.cancelGoal();
		twist_msg.linear.x = 0;
		//twist_msg.linear.y = 0; not necessary
		twist_msg.angular.z = 0;
		pub.publish(twist_msg);
		OnMyWay = false;
		FirstRound = false;
   		}

   }
   return 0;
}
