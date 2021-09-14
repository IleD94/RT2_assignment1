#! /usr/bin/env python
"""
.. module:: go_to_point_action
    :platform: Unix
    :synopsis: Python module for piloting the robot to the target

.. moduleauthor:: Ilenia D'Angelo

ROS node for driving a robot to a specific point within a simulated
environment, given a certain orientation corresponding to direction of the goal.

Subscribes to:
/odom topic to know the robot's position from the simulator

Publishes to:
/cmd_vel the desired robot velocities

Service :
/go_to_point to start the robot motion.

Action :
/reaching_goal to know the position of the goal, if the goal is preempted and the feedback of the action
"""
# import ros stuff
import rospy
#from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from tf import transformations
import math
import actionlib
import actionlib.msg
import rt2_assignment1.msg

# robot state variables
"""Point: current robot position
"""
position_ = Point()
"""Pose: current robot orientation
"""
pose_ = Pose()
"""float: current angle of the robot
"""
yaw_ = 0
# machine state
"""Int: current state of the server
"""
state_ = 0
# goal
desired_position_ = Point()
desired_position_.z = 0
# parameters
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0  
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6

# publisher
"""None: It publish a twist message on the */cmd_vel* topic
"""
pub = None

# action_server
act_s = None

# callbacks


def clbk_odom(msg):
    '''
    Description of the callback:

    This function retrieves the current robot position for saving
    it within the *position_* global variable and is responsible for
    transforming the orientation from quaternion angles to Euler ones

    Args:
      msg(Odometry): data retrieved by */odom* topic

    Returns:
      None

    '''
    global position_
    global pose_
    global yaw_

    # position
    position_ = msg.pose.pose.position
    pose_ = msg.pose.pose

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def change_state(state):
    '''
    Description of the change_state function:

    This function change the value of the global variable (*state_*) and print out the change of the state 

    Args:
      state(int): the state of the robot

    Returns:
      None

    '''
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)


def normalize_angle(angle):
    '''
    Description of the normalize_angle function:

    This function normalize the value of the variable angle between -pi and pi
    Args:
      angle(float): The angle not normalized

    Returns:
      angle(float): The angle normalized

    '''
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def fix_yaw(des_pos):
    '''
    Description of the fix_yaw function:

    This function fix the orientation of the robot using x,y
    coordinates and sets the angular velocity in order to put 
    the robot in the same direction of the goal
		   
    Args:
      des_pos(Point):  the expected x and y coordinates

    Returns:
      None

   '''
    global yaw_, pub, yaw_precision_2_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)

    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a

    pub.publish(twist_msg)

    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(1)


def go_straight_ahead(des_pos):
    '''
    Description of the go_straight_ahead function:

    This function computes the robot trajectory error in the orientation and
    in the position in order to set, using a threshold, the most suitable angular velocity and 
    the linear velocity, which is already set.
		   
    Args:
      des_pos(Point): the expected x and y coordinates
    Returns:
      None

    '''
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw
        pub.publish(twist_msg)
    else:
        print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(0)


def done():
    """
    Description of done function:
		    
    This function set the linear and angular velocities as 0
    
    Args :
      None
    
    Returns :
      None
          
    """
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)

    
def planning(goal):
    """
    Description of planning function:
		    
    This function store as desired position (in coordinates x and y) the goal set by the client and control if the goal is preempted and
    fill feedback of the action and call the suitable function, based on the value of the variable (*state_*). If it succeded it set the result to true.
    
    Args :
      goal (Float) = robot position goal 
    
    
    Returns :
      None
          
    """
    global state_, desired_position_
    global act_s

    desired_position_.x = goal.target_pose.pose.position.x
    desired_position_.y = goal.target_pose.pose.position.y

    state_ = 0
    rate = rospy.Rate(20)
    feedback = rt2_assignment1.msg.PlanningFeedback()
    result = rt2_assignment1.msg.PlanningResult()
    
    while not rospy.is_shutdown():
        if act_s.is_preempt_requested():
            rospy.loginfo('Goal was preempted')
            act_s.set_preempted()
            success = False
            break
        elif state_ == 0:
            feedback.stat = "Fixing the yaw"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            fix_yaw(desired_position_)
        elif state_ == 1:
            feedback.stat = "Angle aligned"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            go_straight_ahead(desired_position_)

        elif state_ == 2:
            feedback.stat = "Target reached!"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            done()
            result.result = True
            success = True
            break
        else:
            rospy.logerr('Unknown state!')

        rate.sleep()
    if success:
       rospy.loginfo('Goal: Succeeded!')
       act_s.set_succeeded(result)


def main():
    """
    Description of the main function:
           
    It initializes the node 'go_to_point', publishes into '/cmd_vel', and subscribes to '/odom' and
    it initializes the action server /reaching_goal 
           
    Args :
      None
    
    Returns :
      None
             
    """
    global pub, active_, act_s
    rospy.init_node('go_to_point')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    act_s = actionlib.SimpleActionServer(
        '/reaching_goal', rt2_assignment1.msg.PlanningAction, planning, auto_start=False)
    act_s.start()

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()
