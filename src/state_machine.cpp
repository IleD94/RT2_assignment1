/**
* \file state_machine.cpp
* \brief This files implements a state machine in order to command a robot that moves to reach random goal
* \author Ilenia D'Angelo
* \version 1.0
* \date 10/09/2021
*
*
* \details
*
* Subscribes to: <BR>
* ° None
*
* Publishes to: <BR>
* ° /cmd_vel
*
* Services : <BR>
* ° /position_server
* ° /user_interface
* ° /position_server
* Actions : <BR>
* ° /reaching_goal

* Description :
*
* Using a user interface, the robot can be activated by the user, who can choose between 1 (to start the robot) and 0 (to stop the robot).
* This change the value of some boolean variables, which allow the access to two different part of the code:
* -with 'start' the program :sends a request to the /position_server, sets the action goal /reaching_goal with /position_server's response
* and waits the achievement of the goal
* -with 'stop' the program: cancels the action goal, sets to zero the linear and angular velocities in twist_msg and publishes them in the topic /cmd_vel 
*/
#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/RandomPosition.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rt2_assignment1/PlanningAction.h>
#include "geometry_msgs/Twist.h"

bool start = false;  /**< used as condition to start the robot */
bool stop = false;   /**< used as condition to stop the robot */
bool OnMyWay = true; /**< used as condition to control if the robot is moving or not */
bool result = false; /**< the outcome of the action */
bool FirstRound = true; /**< used to avoid useless loops of publishing in cmd_vel when the robot is stopped  */

/**
 *@brief This function is the callback function of the service server /user_interface.
 *@param req  the request received from the client of the user_interface.py. 
 *@param res  it does not use any response 
 * 
 *@retval the true boolen value
 * 
 * This function allows to initialize the global variables *"start"* to true and *"OnMyWay"* to false
 * wether the command received consists in a "start" string. Otherwise, it initializes *"start"* to false and the global variable *"stop"* to true
 * 
 */
 
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

/**
 * @brief The main function
 * 
 * @retval 0
 * 
 * This function implements:
 * -# the state_machine node
 * -# the service server /user_interface
 * -# the service client /position_server
 * -# the action client /reaching_goal
 * -# the publisher for /cmd_vel
 * -# the two custom messages RandomPosition and PlanningAction
 * If start value is set as true it sends a request to the /position_server and 
 * uses the response as a goal for the /reaching_goal action, then it waits for the result
 * If stop value is set as true it cancels the goal of the /reaching_goal action, fills the field 
 * of the Twist_msg, needed to control this kind of robot, with 0 and publishes in /cmd_vel 
 */

int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   actionlib::SimpleActionClient<rt2_assignment1::PlanningAction> ac("/reaching_goal", true);
   ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

   rt2_assignment1::RandomPosition rp;
   /* filling of the RandomPosition fields */
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
   		if (not OnMyWay) { /**< if the robot is not still on its way to reach the goal  */
   		/* call of the random_position service */
   		client_rp.call(rp);
   		/* filling of the PlanningGoal fields */
   		goal.target_pose.header.frame_id = "base_link";
  		goal.target_pose.header.stamp = ros::Time::now();
   		goal.target_pose.pose.position.x = rp.response.x;
   		goal.target_pose.pose.position.y = rp.response.y;
   		goal.target_pose.pose.orientation.w  = rp.response.theta;
   		std::cout << "\nGoing to the position: x= " << goal.target_pose.pose.position.x << " y= " << goal.target_pose.pose.position.y << " theta = " << goal.target_pose.pose.orientation.w << std::endl;
   		/* sending the goal to the action server */
   		ac.sendGoal(goal);
   		OnMyWay= true; /**< the robot is moving to reach the goal */
   		}
   		
   		else {
   		FirstRound=true; 
		rt2_assignment1::PlanningResultConstPtr Result;
  		Result = ac.getResult();
   		result = (bool) Result->result; /**< we need to explicitly transform in bool Result->result in order to avoid bugs */
   		  if (result) {
   		    OnMyWay = false; /**< the robot has reached the goal  */
   		    ROS_INFO ("Hooray, goal reached!");
   		  }
   		}
    }

   	if (stop and FirstRound) { /**< double condion in order to avoid useless loops of cancelling goals and publishing in cmd_vel  */
   		ac.cancelGoal();
		twist_msg.linear.x = 0;
		twist_msg.linear.y = 0;
		twist_msg.angular.z = 0;
		pub.publish(twist_msg);
		OnMyWay = false; /**< the robot is not moving to reach a goal  */
		FirstRound = false;

   }
   return 0;
}
